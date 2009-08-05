#include "dsp56f807.h"


# define WREN  0b00000110
# define WRDI  0b00000100
# define RDSR  0b00000101
# define WRSR  0b00000001
# define READ  0b00000011
# define WRITE 0b00000010

void EEPROM_Init();
byte EEPROM_WREN();
byte EEPROM_WRDI();
byte EEPROM_RDSR(byte *);
byte EEPROM_WRSR(byte);
byte EEPROM_READ(byte *,UInt16,UInt16);
byte EEPROM_WRITE(byte *,UInt16,UInt16);
