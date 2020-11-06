#ifndef BME280_SETTINGS_H
#define BME280_SETTINGS_H

#define  SEALEVELPRESSURE_HPA (1013.25)  

#define  BME280_I2C_ADDR  0x76
Adafruit_BME280           bme;

#define UNIT_TEMP     "*C"
#define UNIT_PRESSURE "hPa"
#define UNIT_HUMIDITY "%"
#define UNIT_ALTITUDE "m"

float Temperature = 0;
float Pressure =    0;
float Humidity =    0;
float Altitude =    0;
bool bme_valid_data = false;
bool bme_sensor_found = false;



#endif
