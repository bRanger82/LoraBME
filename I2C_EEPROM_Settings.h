#ifndef I2C_EEPROM_SETTINGS_H
#define I2C_EEPROM_SETTINGS_H

#define I2C_EEPROM_ADDR 0x50
const byte writeDelay = 5; // time required for writing the byte [ms]

bool checkEEPROMExists(void)
{
  Wire.beginTransmission(I2C_EEPROM_ADDR);
  return (Wire.endTransmission() == 0);
}

void writeByte(long eeAddress, byte value)
{
  if (!checkEEPROMExists())
  {
    return;
  }
  delay(writeDelay);
  long _eeAddress = eeAddress;
  byte _value = value;

  Wire.beginTransmission(I2C_EEPROM_ADDR);
  Wire.write(_eeAddress >> 8);
  Wire.write(_eeAddress);
  Wire.write(_value);
  Wire.endTransmission();
  delay(writeDelay);
}

byte readByte(long eeAddress)
{
  if (!checkEEPROMExists())
  {
    return 0xFF;
  }
  delay(writeDelay);
  long _eeAddress = eeAddress;
  byte value = 0;
  
  Wire.beginTransmission(I2C_EEPROM_ADDR);
  Wire.write(_eeAddress >> 8);
  Wire.write(_eeAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) I2C_EEPROM_ADDR, (uint8_t) 1);
  delay(writeDelay);
  if (Wire.available()) value = (Wire.read());
  return value;
} 

#endif
