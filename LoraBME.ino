/*
  Heltec.LoRa Multiple Communication
  Referenced/Based on https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "heltec.h"
#include "Adafruit_TCS34725.h"
#include "TCS34725_Settings.h"
#include "BME280_Settings.h"
#include "I2C_EEPROM_Settings.h"
#include "LoRa_Settings.h"


#define LED    LED_BUILTIN
#define BTN_TRIG_SEND_MSG_TEMP 12 // trigger send msg (temperature) via button press
bool trig_send_msg_temp_flag = false;
#define BTN_SHOW_STATUS_SCREEN 39 // switch display to the status screen

String incoming = "";

volatile bool data_received = false;
volatile bool timer_flag = false;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

String outgoing;              // outgoing message

byte localAddress = 0x2F;     // address of this device
byte destination = 0x2E;      // destination to send to

byte msgCount = 0;            // count of outgoing messages

void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  timer_flag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void enableTimer(void)
{
  Serial.print("[STARTUP] Enable Timer ...");
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 30000000, true);
  timerAlarmEnable(timer);
  Serial.println(" done");
}

void Read_BME_Values(void) 
{
  if (!bme_sensor_found)
    return;
    
  Temperature = bme.readTemperature();
  Pressure = bme.readPressure() / 100.0F;
  Humidity = bme.readHumidity();
  Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if ((Temperature == 0 && Pressure == 0) || Humidity == 0 || Altitude == 0)
  {
    bme_valid_data = false;
  } else
  {
    bme_valid_data = true;
  }
}

void Read_TCS_Values(void) 
{
  if (!tsc34725_sensor_found)
    return;
    
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
}

/*
 * Method sends out all available commands to the lora receiver and serial monitor.
 */
void Send_Answer_Available_Commands(void)
{
  String reply = F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  reply += "<reply>\n";
  reply += "\t<commands>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_TEMPERATURE) + "\" remark=\"Temperature\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_PRESSURE) + "\" remark=\"Pressure\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_HUMIDITY) + "\" remark=\"Humidity\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_ALTITUDE) + "\" remark=\"Altitude\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_TCS_R) + "\" remark=\"TSC_Light_Red_Part\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_TCS_G) + "\" remark=\"TSC_Light_Green_Part\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_TCS_B) + "\" remark=\"TSC_Light_Blue_Part\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_GET_TCS_LUX) + "\" remark=\"TSC_Light_Calculated_Lux\" parameterId =\"0\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_READ_EEPROM) + "\" remark=\"Get_EEPROM_value\" parameterId =\"1\"/>\n";
  reply += "\t\t<command value=\"" + String(CMD_WRITE_EEPROM) + "\" remark=\"Set_EEPROM_value\" parameterId =\"2\"/>\n";
  reply += "\t</commands>\n";
  reply += "\t<params>\n";
  reply += "\t\t<id index=\"0\">\n";
  reply += "\t\t\t<param pos=\"0\" name=\"NO_PARAM\" type=\"\" />\n";  
  reply += "\t\t\t<param pos=\"1\" name=\"NO_PARAM\" type=\"\" />\n";  
  reply += "\t\t</id>\n";  
  reply += "\t\t<id index=\"1\">\n";
  reply += "\t\t\t<param pos=\"0\" name=\"EEPROM_ADDR\" type=\"LONG\" />\n";  
  reply += "\t\t\t<param pos=\"1\" name=\"NO_PARAM\" type=\"\" />\n";  
  reply += "\t\t</id>\n"; 
  reply += "\t\t<id index=\"2\">\n";
  reply += "\t\t\t<param pos=\"0\" name=\"EEPROM_ADDR\" type=\"LONG\" />\n";  
  reply += "\t\t\t<param pos=\"1\" name=\"EEPROM_VALUE\" type=\"BYTE\" />\n";  
  reply += "\t\t</id>\n";  
  reply += "\t</params>\n";
  reply += "</reply>";
  Serial.println(reply);   
  sendMessage(reply);
}

/*
 * Method sends out a notification, that no data is currently available, to the lora receiver and serial monitor.
 */
void Send_Answer_No_Data(void)
{
  String reply = F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  reply += F("<reply>\n");
  reply += F("\t<valid>false</valid>\n");
  reply += F("\t<value>0</value>\n");
  reply += F("\t<unit>X</unit>\n");
  reply += F("\t<remark>NO_DATA_AVAILABLE</remark>\n");
  reply += F("</reply>");
  Serial.println(reply); 
  sendMessage(reply); 
}

/*
 * Method answer message in case and unknown command was received, is sent to the lora receiver and serial monitor.
 */
void Send_Answer_Unknown(void)
{
  String reply = F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  reply += F("<reply>\n");
  reply += F("\t<valid>false</valid>\n");
  reply += F("\t<value>0</value>\n");
  reply += F("\t<unit>X</unit>\n");
  reply += F("\t<remark>ERROR_REQUEST_CODE_UNKNOWN</remark>\n");
  reply += F("</reply>");
  Serial.println(reply);  
  sendMessage(reply);
}

/*
 * Method which creates a response message and sends to the lora receiver and serial monitor.
 */
void Send_Answer(String s_value, String s_unit)
{
  String reply = F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  reply += F("<reply>\n");
  reply += F("\t<valid>true</valid>\n");
  reply += F("\t<value>");
  reply += s_value;
  reply += F("</value>\n");
  reply += F("\t<unit>");
  reply += s_unit;
  reply += F("</unit>\n");
  reply += F("\t<remark>REQEUEST_OK</remark>\n");
  reply += F("</reply>");
  Serial.println(reply);
  sendMessage(reply);
}

void Send_Answer_EEPROM_Read(String s_value)
{
  String reply = F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  reply += F("<reply>\n");
  reply += F("\t<valid>true</valid>\n");
  reply += F("\t<value>");
  reply += s_value;
  reply += F("</value>\n");
  reply += F("\t<unit>");
  reply += F("</unit>\n");
  reply += F("\t<remark>REQEUEST_EEPROM_READ_OK</remark>\n");
  reply += F("</reply>");
  Serial.println(reply);
  sendMessage(reply);
}

void Send_Answer_EEPROM_Write(void)
{
  String reply = F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  reply += F("<reply>\n");
  reply += F("\t<valid>true</valid>\n");
  reply += F("\t<value>");
  reply += F("</value>\n");
  reply += F("\t<unit>");
  reply += F("</unit>\n");
  reply += F("\t<remark>REQEUEST_EEPROM_WRITE_OK</remark>\n");
  reply += F("</reply>");
  Serial.println(reply);
  sendMessage(reply);
}

/*
 * Returns a substring based on a seperator char in a string. 
 */
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) 
  {
    if (data.charAt(i) == separator || i == maxIndex) 
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void ProcessCmdEEPROM_Get(String serialLine)
{
  String str_addr = getValue(serialLine, '|', 2);
  Serial.print("[EEPROM READ] ");
  Serial.print("Address: ");
  Serial.println(str_addr);
  if (str_addr.length() < 1)
  {
    Send_Answer_No_Data();
  } else
  {
    char arr_eeprom_addr[str_addr.length() + 1];
    str_addr.toCharArray(arr_eeprom_addr, sizeof(arr_eeprom_addr));
    long addr = atol(arr_eeprom_addr);     
    Send_Answer_EEPROM_Read("0x" + String(readByte(addr), HEX));
  }
}

void ProcessCmdEEPROM_Set(String serialLine)
{
  String str_addr = getValue(serialLine, '|', 2);
  String str_value = getValue(serialLine, '|', 3);
  Serial.print("[EEPROM WRITE] ");
  Serial.print("Address: ");
  Serial.print(str_addr);
  Serial.print(" - Value: ");
  Serial.println(str_value);
  if (str_addr.length() < 1 || str_value.length() < 1)
  {
    Send_Answer_No_Data();
  } else
  {
    char arr_eeprom_addr[str_addr.length() + 1];
    str_addr.toCharArray(arr_eeprom_addr, sizeof(arr_eeprom_addr));
    long addr = atol(arr_eeprom_addr);  

    char arr_eeprom_value[str_value.length() + 1];
    str_value.toCharArray(arr_eeprom_value, sizeof(arr_eeprom_value));
    byte value = (byte)atoi(arr_eeprom_value);
    writeByte(addr, value);
    Send_Answer_EEPROM_Write();
  }
}

void ProcessCmdPrintEEPROMDebug(void)
{
  for (long addr = 1; addr <= 255; addr++)
  {
    if (addr % 8 == 0)
    {
      Serial.println("");
    }
    Serial.print(addr, HEX);
    Serial.print(":");
    Serial.print(readByte(addr), HEX);
    Serial.print("\t");  
  }
  Serial.println("");
}

void ProcessSerialCmd(void)
{
  if (Serial.available() > 0 || data_received || trig_send_msg_temp_flag)
  {
    String serialLine = "";
    if (data_received)
    {
      Serial.println("[ProcessSerialCmd] data_received path");
      serialLine = incoming;
      data_received = false;
    } else if (trig_send_msg_temp_flag)
    {
      Serial.println("[ProcessSerialCmd] BTN_TRIG_SEND_MSG_TEMP pressed, sending data Temperature");
    } else
    {
      Serial.println("[ProcessSerialCmd] Serial.readString path");
      serialLine = Serial.readString();  
    }
    
    Serial.print("[ProcessSerialCmd] Processing command: ");
    Serial.println(serialLine);
    
    if (serialLine.startsWith(CMD_GET_TEMPERATURE) || trig_send_msg_temp_flag)
    {
      (bme_valid_data) ? Send_Answer(String(Temperature), UNIT_TEMP) : Send_Answer_No_Data();
      trig_send_msg_temp_flag = false;
    } else if (serialLine.startsWith(CMD_GET_PRESSURE))
    {
      (bme_valid_data) ? Send_Answer(String(Pressure), UNIT_PRESSURE) : Send_Answer_No_Data();
    } else if (serialLine.startsWith(CMD_GET_HUMIDITY))
    {
      (bme_valid_data) ? Send_Answer(String(Humidity), UNIT_HUMIDITY) : Send_Answer_No_Data();
    } else if (serialLine.startsWith(CMD_GET_ALTITUDE))
    {
      (bme_valid_data) ? Send_Answer(String(Altitude), UNIT_ALTITUDE) : Send_Answer_No_Data();
    } else if (serialLine.startsWith(CMD_GET_TCS_R))
    {
      Send_Answer(String(r), UNIT_RGB);
    } else if (serialLine.startsWith(CMD_GET_TCS_G))
    {
      Send_Answer(String(g), UNIT_RGB);
    } else if (serialLine.startsWith(CMD_GET_TCS_B))
    {
      Send_Answer(String(b), UNIT_RGB);
    } else if (serialLine.startsWith(CMD_GET_TCS_LUX))
    {
      Send_Answer(String(lux), UNIT_LUX);
    } else if (serialLine.startsWith(CMD_READ_EEPROM))
    {
      ProcessCmdEEPROM_Get(serialLine);
    } else if (serialLine.startsWith(CMD_WRITE_EEPROM))
    {
      ProcessCmdEEPROM_Set(serialLine);
    } else if (serialLine.startsWith(CMD_GET_HELP))
    {
      Send_Answer_Available_Commands();
    } else if (serialLine.startsWith("DEBUG_READ_EEPROM"))
    {
      ProcessCmdPrintEEPROMDebug();  // debug only
    } else
    {
      Send_Answer_Unknown();
    }
    Serial.flush();
    delay(10);
  }
}

void SetupInOutputs(void)
{
  Serial.print("[STARTUP] Setup In/Output ...");
  pinMode(LED, OUTPUT);
  pinMode(BTN_TRIG_SEND_MSG_TEMP, INPUT);  
  pinMode(BTN_SHOW_STATUS_SCREEN, INPUT);
  Serial.println("done");
}

void SetupLoRa(void)
{
  //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  delay(50);
  Serial.print("[STARTUP] LoRa initialization ...");
  LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST); //20dB output must via PABOOST
  //LoRa.enableCrc();
  LoRa.setSpreadingFactor(DEFAULT_LORA_SPREADING_FACTOR);
  LoRa.setPreambleLength(DEFAULT_LORA_PREAMBLE_LENGTH);
  LoRa.setFrequency(DEFAULT_LORA_FREQUENCY);
  LoRa.setSignalBandwidth(DEFAULT_LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(DEFAULT_LORA_CODING_RATE);
  // register the receive callback
  LoRa.onReceive(onReceive);
  // put the radio into receive mode
  LoRa.receive();
  Serial.println("done");
}

void Setup_BME_Sensor(void)
{
  Serial.println("[STARTUP] BME280 initialization started");
  // Display is using the I2C interface as well, so for the BME sensor the Wire1 has to be defined.

  unsigned status = bme.begin(BME280_I2C_ADDR, &Wire1);  
  if (status) 
  {
    Serial.println("[STARTUP] BME280 Sensor found and sucessfully initialized");
    Serial.print("[STARTUP] Waiting 2 seconds for reading first BME280 values ...");
    delay(2000);
    Serial.println(" done");
    bme_sensor_found = true;
    Read_BME_Values();
  } else
  {
    Serial.println("[STARTUP] Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("[STARTUP] BME280 SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
  }
  
  Serial.println("[STARTUP] BME280 initialization done");
}

void Setup_TSC_Sensor(void)
{
  Serial.println("[STARTUP] TCS34725 initialization started");
  if (tcs.begin(TCS34725_ADDR, &Wire1)) 
  {
    Serial.println("[STARTUP] TCS34725 Sensor found and sucessfully initialized");
    Serial.print("[STARTUP] Waiting 2 seconds for reading first TCS34725 values ...");
    delay(2000);
    Serial.println(" done");
    tsc34725_sensor_found = true;
    Read_TCS_Values();
  } else 
  {
    Serial.println("[STARTUP] Could not find a valid TCS34725 Sensor!");
  }  
  Serial.println("[STARTUP] TCS34725 initialization done");
}

void DisplayLoRaReadyMessage(void)
{
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 5 , "LoRa transceiver ready ...");
  Heltec.display->drawString(0 , 16 , "Frequency: " + String(LoRa.getFrequency() / 1000000) + "MHz");
  Heltec.display->drawString(0 , 27 , "Sig. bandw.: " + String(LoRa.getSignalBandwidth() / 1000) + "kHz");
  Heltec.display->drawString(0 , 38 , "Dest. Addr. 0x" + String(destination, HEX));
  Heltec.display->drawString(0 , 49 , "Local Addr. 0x" + String(localAddress, HEX));
  Heltec.display->display(); 
}

void DisplayLoRaDisplaySendingMessage(void)
{
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 30 , "Sending message ...");
  Heltec.display->display(); 
}

void DisplayLoRaDisplayReceivingMessage(void)
{
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 30 , "Receiving message ...");
  Heltec.display->display(); 
}

void sendMessage(String outgoing)
{
  digitalWrite(LED, HIGH);
  
  DisplayLoRaDisplaySendingMessage();
  
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
                             
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 4  , "Message sent:");
  Heltec.display->drawString(0 , 15 , "Msg Num     " + String(msgCount));
  Heltec.display->drawString(0 , 26 , "Dest. Addr. 0x" + String(destination, HEX));
  Heltec.display->drawString(0 , 37 , "Local Addr. 0x" + String(localAddress, HEX));
  Heltec.display->drawString(0 , 48 , "Length      " + String(outgoing.length()));
  Heltec.display->display();    

  msgCount++;                         // increment message ID
  
  digitalWrite(LED, LOW);
}

void DisplayErrorMessage(String message)
{
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 4  , "Error occurred, message:");
  
  // handling of multi-lines, as no wordwrap is done automatically
  int chars_per_line = 15;
  int cnt = message.length() / chars_per_line;
  for (int idx = 0; idx <= cnt; idx++)
  {
      int y_pos = idx * 11 + 15;
      int current_char_start_pos = idx * chars_per_line;
      
      if (y_pos > 48)
        break; // cannot display anymore lines as end of height was reached

      // has to be handled differently, as substring throws an exception if current_char_start_pos + chars_per_line > message.length()
      if (message.length() < idx * chars_per_line + chars_per_line)
      {
          Heltec.display->drawString(0 , y_pos , message.substring(current_char_start_pos));
          break;
      }
      Heltec.display->drawString(0 , y_pos , message.substring(current_char_start_pos, chars_per_line));
  }
  Heltec.display->display();    
}

void onReceive(int packetSize)
{
  data_received = false;
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  incoming = "";
  
  DisplayLoRaDisplayReceivingMessage();
  
  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }
  
  // check length for error
  if (incomingLength != incoming.length())
  {   
    DisplayErrorMessage("Message length does not match length");
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) 
  {
    Serial.println("This message is not for me.");
    DisplayLoRaReadyMessage();
    return;
  }

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 4  , "Message received:");
  Heltec.display->drawString(0 , 15 , "Msg Num     " + String(LoRa.packetSnr()));
  Heltec.display->drawString(0 , 26 , "Received " + String(incomingLength) + " bytes");
  Heltec.display->drawString(0 , 37 , "RSSI: " + String(LoRa.packetRssi()));  
  Heltec.display->drawString(0 , 48 , "Received from: 0x" + String(recipient, HEX));
  Heltec.display->display();    

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  
  data_received = true;
}

/*
 * Reads the LoRa settings from the EEPROM.
 * If no eeprom is available or no settings are saved yet -> default values are used.
 */
void ReadLoRaAdressSettings(void)
{
  byte destinationAdressFromEEPROM = readByte(EEPROM_POS_LORA_LOCAL_ADDR);
  byte localAdressFromEEPROM = readByte(EEPROM_POS_LORA_DESTINATION_ADDR);
  
  localAddress = DEFAULT_LORA_LOCAL_ADDR;    
  destination = DEFAULT_LORA_DESTINATION_ADDR; 

  if (localAdressFromEEPROM == 0xFF || destinationAdressFromEEPROM == 0xFF)
  {
    Serial.print("[SETUP] EEPROM setup NOT available, using default values, for local address: ");
    Serial.print(localAddress, HEX);
    Serial.print(" - destination address: ");
    Serial.println(destination, HEX);
  } else if (localAdressFromEEPROM == destinationAdressFromEEPROM)
  {
    Serial.print("[STARTUP] EEPROM available, but have the same local and destination address - using default values, for local address: ");
    Serial.print(localAddress, HEX);
    Serial.print(" - destination address: ");
    Serial.println(destination, HEX);
  } else
  {
    localAddress = localAdressFromEEPROM;
    destination = destinationAdressFromEEPROM;
    Serial.print("[STARTUP] EEPROM setup IS available, reading from EEPROM, for local address: ");
    Serial.print(localAddress, HEX);
    Serial.print(" - destination address: ");
    Serial.println(destination, HEX);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.flush();
  delay(50);
    
  Wire1.begin(21,22);
  delay(50);
  ReadLoRaAdressSettings();
  delay(50);
  SetupInOutputs();
  delay(50);
  SetupLoRa();
  delay(50);
  Setup_BME_Sensor();
  delay(50);
  Setup_TSC_Sensor();
  delay(50);
  enableTimer();
  delay(50);
  DisplayLoRaReadyMessage();
}

void loop()
{
  if (timer_flag)
  {
    Read_BME_Values();
    Read_TCS_Values();
    timer_flag = false;  
    Serial.println("[LOOP] TMR EVENT");
  }
  
  if (digitalRead(BTN_TRIG_SEND_MSG_TEMP) == HIGH)
  {
    Serial.println("[LOOP] Button-Pressed: manual trigger for sending temperature data via LoRa set");
    trig_send_msg_temp_flag = true;     
  }
  
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

  ProcessSerialCmd();
}
