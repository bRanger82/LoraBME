#ifndef TCS34725_SETTINGS_H
#define TCS34725_SETTINGS_H

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

#define TCS34725_ADDR   0x29
bool tsc34725_sensor_found = false;
uint16_t r = 0;
uint16_t g = 0;
uint16_t b = 0;
uint16_t c = 0;
uint16_t colorTemp= 0;
uint16_t lux = 0;

#define UNIT_RGB "hex"
#define UNIT_LUX "lx"

#endif
