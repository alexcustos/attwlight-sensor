#ifndef ATTWLIGHT_SENSOR_H_
#define ATTWLIGHT_SENSOR_H_

#include <inttypes.h>

struct payload_t
{
  uint16_t id;
  uint32_t uptime;
  uint8_t light;
  int32_t vcc;
  int32_t tmp36;
  uint8_t stat;
};

enum light_t {
  LIGHT_ON,
  LIGHT_OFF,
  LIGHT_FUZZY,
  LIGHT_UNKNOWN
};

enum stat_t {
  STAT_OK,
  STAT_BOOT,
  STAT_LIGHT,
  STAT_FUZZY,
  STAT_UNKNOWN
};

#endif /* ATTWLIGHT_SENSOR_H_ */


