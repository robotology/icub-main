#ifndef UTILS_H
#define UTILS_H

void __delay32(unsigned long);

// define the HES data size for MAIS
#define HESDATA_IS_16_BIT    0
#define HESDATA_IS_12_BIT    1
#define HESDATA_IS_8_BIT     2

extern unsigned char HESDATA_RESOLUTION; 

typedef struct CANin12bit
{
  union
  {
    struct 
    {
	   unsigned data0          :12;
	   unsigned data1          :12;
	   unsigned data2          :12;
	   unsigned data3          :12;
	   unsigned data4          :12;
	   unsigned NU             :4 ;
	};
	unsigned char data[8]      ;
  };
} CANin12bit;  

void from16to12bitsviaCAN(unsigned char data[8]);


#endif

