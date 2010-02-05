#ifndef EEPROM_H
#define EEPROM_H

#define SAVE_EEPROM_ATONCE    1

//
// EEPROM memorized data for board configuration
//
typedef struct s_eeprom
{
  // configrazioni relative alla scheda
  unsigned EE_B_EEErased  :1;  // if 1 the ee has been erased (hopefully ;-)
  unsigned EE_B_EnableWD  :1;
  unsigned EE_B_EnableBOR :1;
  
  // configurazioni relative al CAN
  unsigned char EE_CAN_BoardAddress;  
  unsigned char EE_CAN_MessageDataRate;    // framerate of outgoing messages
  unsigned char EE_CAN_Speed;

  // additional info
  unsigned char EE_AdditionalInfo[32]; 

  unsigned int  EE_ChkSum;                 // data validation checksum

} s_eeprom;


extern struct s_eeprom BoardConfig; 
extern struct s_eeprom _EEDATA(1) ee_data;


void SaveEepromBoardConfig();
void RecoverConfigurationFromEEprom();

#endif
