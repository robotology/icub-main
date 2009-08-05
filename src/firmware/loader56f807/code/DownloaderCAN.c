/* Including used modules for compilling procedure */
#include "Cpu.h"
#include "Timer.h"
#include "Can1.h"
/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#include "bootloader.h"
#include "controller.h"
#include "flashprog.h"
#include "comCAN.h"
#include "sparser.h"
#include "prog.h"

#define CMD_DONWL	0x7    //broadcast class 
#define DEST		0x0    //destination (PC=> id=0)

#define readflash(address) (*(word *)(address)) 

/****************************************************************************
*        Local function prototypes
****************************************************************************/
static void userDelay         (UWord16 Counter );
void mySleep(void);

/****************************************************************************
* Variables from linker.cmd file used to copy initialization data into .data
****************************************************************************/

extern void * _Ldata_RAM_addr;
extern void * _Ldata_size;
extern void * _Ldata_ROM_addr;
extern void * _Ldata_ROM_addr;
extern void * _Ldata_RAM_addr;
extern void * _Ldata_size;

extern bool downloadEn;
//bool        passed_through_sleep=FALSE;

void * archStartDelayAddress;
int get_flash_addr (void); 
/*
** ===================================================================
**     Method      :  PE_low_level_init (bean 56F807)
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/
void PE_low_level_init(void);

unsigned int TmpXdataVar;

/* CAN bus communication global vars */
byte 	CAN_data[8];					/* CAN bus message */
dword 	CAN_messID = 0;					/* arbitration */
byte 	CAN_frameType;
byte 	CAN_frameFormat;
byte 	CAN_length;		

// Change the BootLoader Version here
char _add_info [4]={'B','L','0','6'};
Int16 _flash_addr = 0;  

byte	_board_ID;	/* */ 
byte	_board_TYPE = 0;
word	_board_FW_VER; 
 

/*****************************************************************************
*
* Module:         main()
*
* Description:    Initialize used hardware, Start S-Record loading.
*                 After end disable used hardware and exit.
*
* Returns:        None
*
* Arguments:      None
*
*****************************************************************************/

void main(void)
{
	Int32 acceptance_code;
	/*** Initialization **/

	PE_low_level_init();
	CAN1_init ();  // CAN bus init

	//led init
//	setRegBits(GPIO_A_DDR,0xF0);   
//	clrRegBits(GPIO_A_PER,0xF0); 
//	clrRegBits(GPIO_A_DR, 0xF0);

	//firstBoot=1;
//	setRegBits(GPIO_A_DR, 0xF0);	
/*****************************************************************************/
/* Initialize all downloader and bootloader subsystems                                      */
/*****************************************************************************/

   /* Get delay value. This value specifies the amount of time that the boot
      loader will wait to begin receiving code to be programmed. If no code
      is received before this timeout, the bootloader will begin execution
      of the application already loaded in Program Flash. */
//	archStartDelayAddress = (void *)0x0083;
//	flashmemCopyPtoX(&TmpXdataVar, archStartDelayAddress, sizeof(unsigned int));
	TmpXdataVar = 5;
	
	/* gets the address of flash memory from the linker */
	_flash_addr = get_flash_addr();	 
	_board_ID = readflash(_flash_addr);
	_board_ID &= 0xF;
	_board_FW_VER = readflash(_flash_addr+2);	
		
	// CAN masks/filters init 
	acceptance_code = (_board_ID>>3) | ((_board_ID & 0x7)<<13) | 0x00e000e0;
	CAN1_setFilter (0x1f1e1f1e,0x1f1e1f1e,acceptance_code,0xe0e1e0e1,TWO_16_FILTER);
	CAN_messID = (CMD_DONWL << 8) | (_board_ID << 4) | DEST;
   
	// TmpXdataVar is the downloader start delay
	// TmpXdataVar = 0 => Jumps immediately to the application's Start Address; 
	//					 downloader disabled
	// TmpXdataVar = 1-254 => Waits a specified number of seconds before the 
	//						 S-Record begins to download
	// TmpXdataVar = 255 => Waits forever before the S-Record begins downloading 
	if(TmpXdataVar != 0x0000)
	{
		if(TmpXdataVar != 0x00FF)
		{
			setReg(TMRC0_CNTR,0);              // Reset counter /
			setReg(TMRC1_CNTR,0);
			setRegBitGroup(TMRC0_CTRL,CM,1);   // Run counter /
		}
		
		// send ACK packet for CMD_BOARD command
		comACK(CMD_BOARD);
		
		downloadEn = FALSE;
//		if (passed_through_sleep==FALSE) downloadEn=TRUE;
		// Start communication loop. Wait for S-Record file /
		comMainLoop();
	}	

	userDelay(TERMINAL_OUTPUT_DELAY);

	/* User application will be started from _EntryPoint() file after exit from main() */
	asm(jmp archStart);
}

void mySleep(void)
{
	//enable init
	
    setRegBits(GPIO_D_DDR,0xF);   
	clrRegBits(GPIO_D_PER,0xF); 
	clrRegBits(GPIO_D_DR, 0xF); 
		
	//led init
	setRegBits(GPIO_A_DDR,0xF0);   
	clrRegBits(GPIO_A_PER,0xF0); 
//	clrRegBits(GPIO_A_DR, 0xF0);
 // setRegBits(GPIO_A_DR, 0xF0);
//	passed_through_sleep = TRUE;
	
	
	PE_low_level_init();
	CAN1_init ();  // CAN bus init
	

	//wait time in order to guarantee that
	//all devices have been initialized
	//Removing this sleep time might cause
	//problems if the power supply ramp 
	//is slow: CAN bus driver might 
	//not be ready.
	// (1)
	setRegBits(GPIO_A_DR, 0xF0);
	userDelay(30000);
	// (2)

//	setRegBits(GPIO_A_DR, 0x30);
//	userDelay(30000);
	// (3)
//	setRegBits(GPIO_A_DR, 0x70);
	clrRegBits(GPIO_A_DR, 0xF0);
	userDelay(20000);  
	main();
}


/* defined by the linker */
extern _data_ROM2_addr;
asm int get_flash_addr (void)
{
	move #_data_ROM2_addr, y0
	rts
}

/*****************************************************************************
*
* Module:         userError()
*
* Description:    Send packet error and wait the end of trasmission of S-Record.
*                 Perform processor reset.
*
* Returns:        Function does not return control.
*
* Arguments:      Error number
*
*****************************************************************************/
void userError(int Error)
{
	CAN_data[0] = CMD_ERR;
	CAN_data[1] = Error;
	CAN1_sendFrame (CAN_messID, 2, CAN_data);	
	comWaitEnd();
	_downloadEntryPoint();
}

/*****************************************************************************
*
* Module:         userDelay()
*
* Description:    Delay program execution aproximately on (Counter * 8191)
*                 instruction clock cycles
*
* Returns:        None
*
* Arguments:      Counter - programmable delay
*
*****************************************************************************/
asm void userDelay(UWord16 Counter)
{
		do      y0,userDelay1
		movei   #0xffff, y0
		rep     y0             ;// bootloader does not serve any interrupts
		nop
userDelay1:
	    rts
}

/*****************************************************************************
*
* Module:         bootmemCopyXtoX()
*
* Description:    Copy src words from X:src to X:dest memory location
*                 Register usage:
*                    R2 - dest
*                    R3 - src
*                    Y0 - count, tmp variable
*
* Returns:        (void *)(dest + count)
*
* Arguments:      dest - data destination
*                 scr  - data source
*                 count - data length in words
*
*****************************************************************************/
asm void *  bootmemCopyXtoX(void *dest, const void *src, size_t count)
{
		tstw    Y0
		beq     EndDo
		do      Y0,EndDo
		move    X:(R3)+,Y0
		move    Y0,X:(R2)+
EndDo:
        ; R2 - Contains *dest return value
        rts
}

/*
** ===================================================================
**     Method      :  PE_low_level_init (bean 56F807)
**
**     Description :
**         This method is internal. It is used by Processor Expert
**         only.
** ===================================================================
*/
void PE_low_level_init(void)
{
  /* SYS_CNTL: ??=0,??=0,??=0,??=0,TMR_PD=0,CTRL_PD=0,ADR_PD=0,DATA_PD=0,??=0,??=0,??=0,BOOTMAP_B=0,LVIE27=0,LVIE22=0,PD=0,RPD=0 */
  setReg16(SYS_CNTL, 0);               /* Enable/Disable pullup registers for the given devices */ 
  /* Common initialization of the CPU registers */
  /* GPIO_E_PER: PE|=3 */
  setReg16Bits(GPIO_E_PER, 3);          
  /* ### FreeCntr "Timer" init code ... */
  Timer_Init();
  __DI();                              /* Disable interrupts. Only level 1 is allowed */
}