#ifndef I2C_EEPROM_SETTINGS_H
#define I2C_EEPROM_SETTINGS_H

#define I2C_EEPROM_ADDR 0x50
const byte writeDelay = 5; // time required for writing the byte [ms]

bool checkEEPROMExists(void)
{
  Wire1.beginTransmission(I2C_EEPROM_ADDR);
  return (Wire1.endTransmission() == 0);
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

  Wire1.beginTransmission(I2C_EEPROM_ADDR);
  Wire1.write(_eeAddress >> 8);
  Wire1.write(_eeAddress);
  Wire1.write(_value);
  Wire1.endTransmission();
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
  
  Wire1.beginTransmission(I2C_EEPROM_ADDR);
  Wire1.write(_eeAddress >> 8);
  Wire1.write(_eeAddress);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t) I2C_EEPROM_ADDR, (uint8_t) 1);
  delay(writeDelay);
  if (Wire1.available()) value = (Wire1.read());
  return value;
} 

#endif
