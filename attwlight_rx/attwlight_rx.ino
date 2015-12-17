/*
 * Подключение радиомодуля NRF24L01+:
 * GND  (1) - GND
 * VCC  (2) - 3.3V
 * CE   (3) - D9
 * CSN  (4) - D10
 * SCK  (5) - D13
 * MOSI (6) - D11
 * MISO (7) - D12
 * IRQ  (8) - -
 */
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "attwlight_sensor.h"

#define CE_PIN 9
#define CSN_PIN 10
#define RF24_CHANEL 62 // uint8_t

const uint16_t sensor_node = 1;
const uint16_t rx_node = 0;

RF24 radio(CE_PIN, CSN_PIN);
RF24Network network(radio);
RF24NetworkHeader header;

struct payload_t
{
  unsigned int id;
  unsigned long uptime;
  byte light;
  long vcc;
  float tmp36;
  //char stat[11];
  byte stat;
};
payload_t payload;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("* RF24Network RX *");

  SPI.begin();
  radio.begin();
  network.begin(RF24_CHANEL, rx_node);
}

void loop(void)
{
  network.update();

  while ( network.available() ) {
    network.read(header, &payload, sizeof(payload));

    Serial.print("SENSOR #");
    Serial.print(payload.id);
    Serial.print("; UPTIME: ");
    Serial.print(payload.uptime);

    Serial.print("; LIGHT: ");
    if (payload.light == LIGHT_ON)
      Serial.print("ON");
    else if (payload.light == LIGHT_OFF)
      Serial.print("OFF");
    else if (payload.light == LIGHT_FUZZY)
      Serial.print("FUZZY");
    else
      Serial.print("UNKNOWN");

    Serial.print("; VCC: ");
    Serial.print(float(payload.vcc) / 1000.0);
    Serial.print("; TMP36: ");
    Serial.print(payload.tmp36);

    Serial.print("; STATUS: ");
    if (payload.stat == STAT_OK)
      Serial.print("OK");
    else if (payload.stat == STAT_BOOT)
      Serial.print("BOOT");
    else if (payload.stat == STAT_LIGHT)
      Serial.print("LIGHT");
    else if (payload.stat == STAT_FUZZY)
      Serial.print("FUZZY");
    else
      Serial.print("UNKNOWN");

    Serial.println("");
  }
}
