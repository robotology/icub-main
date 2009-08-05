#define PM_ROWSIZE                32         //Row size for program memory: 32 instructions (24+8 bits)


typedef unsigned long  UWord32;
typedef unsigned short UWord16;

typedef union tuPMBuf
{
  unsigned char asByte[PM_ROWSIZE*4];
  int asWord16[PM_ROWSIZE*2];
  long asWord32[PM_ROWSIZE];
} uBuf;


typedef union tuReg32
{
	UWord32 Val32;

	struct
	{
		UWord16 LW;
		UWord16 HW;
	} Word;

	unsigned char Val[4];
} uReg32;


typedef struct
{
  uReg32 Addr;
  uReg32 Data;
} tConfReg;


void WriteLatchCM(UWord16 Adr_HW,UWord16 Adr_LW, UWord16 Data);
void WriteMem(UWord16 ConfigWord);
void ResetDevice(void);
UWord16 ReadLatchCM(UWord16 Adr_HW,UWord16 Adr_LW);

