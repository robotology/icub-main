/*****************************************************************************
* File Name:         prog.h
*
* Description:       Global parameters and functions for programming module
*
*****************************************************************************/
#if !defined(__PROG_H)
#define __PROG_H

#include "PE_types.h"

/*****************************************************************************/

#define  AREA_DISABLED     0x0001
#define  AREA_FLASH        0x0002
#define  AREA_RAM          0x0000
#define  AREA_DELAYED      0x0004
#define  AREA_ERROR        0x0008
#define  AREA_P_MEMORY     0x0040
#define  AREA_X_MEMORY     0x0000

typedef volatile struct{
    UWord16 ControlReg;
	UWord16 ProgramReg;
	UWord16 EraseReg;
	UWord16 AddressReg;
	UWord16 DataReg;
	UWord16 IntReg;
	UWord16 IntSourceReg;
	UWord16 IntPendingReg;
	UWord16 DivisorReg;
	UWord16 TimerEraseReg;
	UWord16 TimerMassEraseReg;
	UWord16 TimerNVStorageReg;
	UWord16 TimerProgramSetupReg;
	UWord16 TimerProgramReg;
	UWord16 TimerNVHoldReg;
	UWord16 TimerNVHold1Reg;
	UWord16 TimerRecoveryReg;
	UWord16 Reserved[15];
} arch_sFlash;


typedef struct {
   UWord16        StartAddress;
   UWord16        EndAddress;
   UWord16        Description;
   arch_sFlash  * FIUBaseAddress;
} Area;



extern void progSaveData   ( UWord16 * pData, UWord16 length, UWord16 Address, mem_eMemoryType MemoryType);
extern void progPlaceData  ( bool );
extern void progPlaceDelayValue ( void );

extern void progPlaceFlash(void);





#define progFlush()        {  progPlaceData(TRUE);                \
                              progPlaceDelayValue();              \
                           }

#define programFlash()	   {
#define progInit() {/* all zero initialization is done in booArchStart() */}

extern UWord16 progProgCounter;
extern UWord16 progDataCounter;
extern UWord16 progIndicatorCounter;

typedef void * (*tpMemCopy)( void *, const void *, size_t  );
typedef int    (*tpMemCmp)( const void *, const void *, size_t );


#endif /* !defined(__PROG_H) */
