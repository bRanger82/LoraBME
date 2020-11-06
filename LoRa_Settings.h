#ifndef LORA_SETTINGS_H
#define LORA_SETTINGS_H

#define CMD_GET_TEMPERATURE  "OPT|1|"
#define CMD_GET_PRESSURE     "OPT|2|"
#define CMD_GET_HUMIDITY     "OPT|3|"
#define CMD_GET_ALTITUDE     "OPT|4|"
#define CMD_GET_TCS_R        "OPT|5|"
#define CMD_GET_TCS_G        "OPT|6|"
#define CMD_GET_TCS_B        "OPT|7|"
#define CMD_GET_TCS_LUX      "OPT|8|"
#define CMD_READ_EEPROM      "OPT|9|"
#define CMD_WRITE_EEPROM     "OPT|10|"
#define CMD_GET_HELP         "HELP"

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
#define DEFAULT_LORA_SPREADING_FACTOR  7
#define DEFAULT_LORA_PREAMBLE_LENGTH   8
#define DEFAULT_LORA_FREQUENCY         433E6
#define DEFAULT_LORA_SIGNAL_BANDWIDTH  125E3
#define DEFAULT_LORA_CODING_RATE       5

#define EEPROM_POS_LORA_LOCAL_ADDR        0x30
#define EEPROM_POS_LORA_DESTINATION_ADDR  0x40
#define DEFAULT_LORA_LOCAL_ADDR           0x2F
#define DEFAULT_LORA_DESTINATION_ADDR     0x2E
#endif
