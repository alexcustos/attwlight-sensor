#include <RF24Network.h>
#include <RF24.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "attwlight_sensor.h"

#define CE_PIN 3
#define CSN_PIN 3 // 3 pin configuration CE == CSN
#define RF24_CHANEL 62 // uint8_t
#define TMP36_PIN 2 // Analog (ADC2)
#define LIGHT_PIN 3 // Digital (PB3) & Analog (ADC3) & Interrupt (PCINT3)
#define RESET_PIN 5 // Digital (PB5)

#define WDTO_INFINITE 255
#define FUZZY_VOLTAGE 0.4 // ширина зоны не приемлемого напряжения в середине, включая концы
//#define STAT_LEN 11

#define SENSOR_ID 101
#define SLEEP_PERIOD WDTO_8S
#define SKIP_WDT_WAKEUPS 14 // x SLEEP_PERIOD + SLEEP_PERIOD (14 ~= 120 sec)

const uint16_t sensor_node = 1;
const uint16_t rx_node = 0;

volatile bool light_pcint = false;
volatile byte wakeup_counter = 0;
const unsigned int fuzzy_start = int(1024.0 * (1.0 - FUZZY_VOLTAGE) / 2.0);
const unsigned int fuzzy_stop = 1023 - fuzzy_start;

RF24 radio(CE_PIN, CSN_PIN);
RF24Network network(radio);
RF24NetworkHeader header(rx_node);

struct payload_t
{
  unsigned int id;
  unsigned long uptime;
  byte light;
  long vcc;
  float tmp36;
  //char stat[STAT_LEN];
  byte stat;
};
payload_t payload;
bool is_payload_sent = false;

void setup() {
  payload.id = SENSOR_ID;

  pinMode(LIGHT_PIN, INPUT); // PB3
  digitalWrite(LIGHT_PIN, LOW); // отключить встроенный pullup резистор ~30kΩ

  buildStat(STAT_BOOT); // сообщить о загрузке

  // Инициализация радиомодуля
  radio.begin();
  network.begin(RF24_CHANEL, sensor_node);
}

void loop()
{
  stat_t stat = STAT_UNKNOWN; // заглушка на случай ошибок в коде
  byte prev_light;

  if (!is_payload_sent) {
    // Отправить статус если есть свежий
    radio.powerUp();
    network.write(header,&payload,sizeof(payload));
    radio.powerDown();
    is_payload_sent = true;
  }

  sleep();

  if (light_pcint) { // пробуждение от прерывания
    light_pcint = false;
    stat = STAT_LIGHT;
  } else if (wakeup_counter > SKIP_WDT_WAKEUPS) { // время отправить статус
    stat = STAT_OK;
  } else {
    // Периодическая оценка проблем с освещением если SKIP_WDT_WAKEUPS > 0
    prev_light = payload.light;
    payload.light = readLightStatus(0);
    // Вариант с FUZZY -> ON/OFF
    if (prev_light == payload.light || payload.light != LIGHT_FUZZY)
      return;
    stat = STAT_FUZZY;
  }

  buildStat(stat); // формирование статуса

  wakeup_counter = 0;
}

bool buildStat(stat_t stat)
{
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH); // подача питания на TMP36

  payload.uptime = millis();
  payload.vcc = readVcc(); // 2ms delay
  payload.tmp36 = readTmp36(payload.vcc); // требуется 1ms для включения

  pinMode(RESET_PIN, INPUT); // отключение питания и перевод в INPUT для уменьшения энергопотребления

  if (stat == STAT_LIGHT)
      payload.light = readLightStatus(10); // 10ms - 1/2 периода 50Hz, для разгореться/погаснуть
  else if (stat != STAT_FUZZY) // если не было измерено раньше
      payload.light = readLightStatus(0);
  payload.stat = stat;
  //strncpy(payload.stat, stat, STAT_LEN);
  //payload.stat[STAT_LEN-1] = '\0';

  is_payload_sent = false; // статус обновлён
}

// Вектор (функция) прерывания PCINT, у ATtiny85 один порт (B), следовательно только один вектор PCINT
ISR(PCINT0_vect)
{
  light_pcint = true;
}

// Вектор прерывания таймера WDT
ISR(WDT_vect)
{
  wakeup_counter++;
}

void sleep()
{
  GIMSK = _BV(PCIE); // Включить Pin Change прерывания
  if (SLEEP_PERIOD == WDTO_INFINITE || payload.light != LIGHT_FUZZY)
    PCMSK |= _BV(LIGHT_PIN); // PCINT3; включить если нет проблем с освещением и есть шанс проснуться
  ADCSRA &= ~_BV(ADEN); // отключить ADC; уменьшает энергопотребление

  if (SLEEP_PERIOD != WDTO_INFINITE) {
    wdt_enable(SLEEP_PERIOD); // установить таймер
    WDTCR |= _BV(WDIE); // включить прерывания от таймера; фикс для ATtiny85
  }

  //MCUSR &= ~_BV(SM0);
  //MCUSR |= _BV(SM1);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // установить режим сна Power-down
  //MCUSR |= _BV(SE);
  sleep_enable(); // разрешить режим сна
  sei(); // включить прерывания

  sleep_cpu(); // заснуть

  cli(); // отключить прерывания; для безопасного отключения PCINT3
  PCMSK &= ~_BV(LIGHT_PIN); // PCINT3; отключить
  //MCUSR &= ~_BV(SE);
  sleep_disable(); // запретить режим сна
  ADCSRA |= _BV(ADEN); // включить ADC
  sei(); // включить прерывания; иначе таймеры не будут работать
}

long readVcc() {
  // Установка референса в Vcc и измерение внутреннего Vbg == 1.1V
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // требуется минимум 1ms для стабилизации
  ADCSRA |= _BV(ADSC); // ADC Start conversion - начать опрос
  while (bit_is_set(ADCSRA, ADSC)); // ADSC == 1 пока опрос активен

  // Чтение ADCL блокирует ADCH, чтение ADCH разблокирует оба регистра => ADCL читать первым
  uint8_t low  = ADCL; // младший регистр данных ADC
  uint8_t high = ADCH; // старший регистр данных ADC

  long result = (high << 8) | low;

  result = 1125300L / result; // 1125300 = 1.1*1023*1000
  return result; // Vcc в милливольтах
}

float readTmp36(long vcc)
{
  int input = analogRead(TMP36_PIN); // напряжение относительно refVolts
  float refVolts = float(vcc) / 1000.0;
  float voltage = float(input) * refVolts / 1024.0; // фактическое напряжение
  float temperatureC = (voltage - 0.5) * 100.0; // смещение 500mV & 10mV == 1°C
  return temperatureC;
}

light_t readLightStatus(unsigned int ms)
{
  delay(ms); // если нужно время на стабилизацию
  int input = analogRead(LIGHT_PIN); // ADC3

  // оценка надёжности результата
  if (input >= fuzzy_start && input <= fuzzy_stop)
    return LIGHT_FUZZY;
  if (input < fuzzy_start)
    return LIGHT_OFF;
  if (input > fuzzy_stop)
    return LIGHT_ON;

  // заглушка; на случай ошибки в коде
  return LIGHT_UNKNOWN;
}
