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
  
  // configurazioni relative ai canali analogici
  unsigned char EE_AN_ActiveChannels[13];  // sequenza di acquisizione canali 0 se canale non usato
  unsigned char EE_AN_SelectedChannel;     // canale attualmente attivo
  unsigned int  EE_AN_ChannelScanningTime; // tempo di scansione dei canali in 100aia di usec
  unsigned int  EE_AN_ChannelOffset[6];    // DAC generated offset
  unsigned int  EE_AN_ChannelValue[13];    // ADC values

  // torque/force
  int  EE_TF_TorqueValue[3];      // Torque values
  int  EE_TF_ForceValue[3];       // Force values

  int           EE_TF_TMatrix[6][6];       // SG2TF Transformation Matrix

  // additional info
  unsigned char EE_AdditionalInfo[32]; 

  // matrix calibration gain
  unsigned char EE_MatrixGain;
  int           EE_CalibrationTare[6];
  int           EE_FullScales[6];
  int           EE_debug2[6];
  int           EE_debug3[6];
  int           EE_debug4[6];
  int           EE_debug5[6];
  int           EE_debug6[6];
  int           EE_debug7[6];
  int           EE_debug8[6];
  unsigned char EE_SerialNumber[8]; 
  unsigned int  EE_ChkSum;                 // data validation checksum

} s_eeprom;


extern struct s_eeprom BoardConfig; 
extern struct s_eeprom _EEDATA(1) ee_data;


void SaveEepromBoardConfig();
void RecoverConfigurationFromEEprom();

#endif
