/*****************************************************************************
*
* Freescale Inc.
* (c) Copyright 2000,2003 Freescale, Inc.
* ALL RIGHTS RESERVED.
*
******************************************************************************
*
* File Name:         bootloader.h
*
* Description:       Bootloader main header file with configuration parameters
*
*****************************************************************************/

#if !defined(__BOOTLOADER_H)
#define __BOOTLOADER_H

#include "Cpu.h"


/*****************************************************************************
*
* Configuration parameters for bootloader application
*
*****************************************************************************/

/* To use word address define SRECORD_WORD_ADDRESS, for byte address mode
/* in S-Record this symbol must be undefined. */
#define SRECORD_WORD_ADDRESS

/* SCI configuration: 115200u, 8N1, No parity. */
#define SCI_BAUD_RATE            115200u
/* To change SCI mode refer to SCI_SCICR_* constant in com.c file */
#define SCI_MODE                 0

/* PLL configuration */
/* On board oscillator frequency. Used to calculate SCI speed */
#if defined(PEcfg_56F826BootLoader)|| defined(PEcfg_56F827BootLoader)
/* 826 and 827 are designed for 4MHz oscillator */
  #define OSCILLATOR_FREQUENCY     4000000ul
#else
/* 80x parts are designed for 8MHz oscillator */
  #define OSCILLATOR_FREQUENCY     8000000ul
#endif

/* PLL multiplier */
#if defined(PEcfg_56F826BootLoader)|| defined(PEcfg_56F827BootLoader) /* run 826 at 80MHz */
  #define PLL_MUL                  40u
#elif defined(DSP56F800_DEMO_BOARD)
  #define PLL_MUL                  15u
#else /* run 80x at 72Mhz */
  #define PLL_MUL                  18u
#endif

/* Bus clock calculation (based on below PLL additional settings) */
#define ZCLOCK_FREQUENCY         (PLL_MUL*(OSCILLATOR_FREQUENCY/2))

/* Saved words per one dot in progress indicator */
/* To disable the progress indication set value of it to 0 */
/* #define PROG_INDICATION_UNIT     0xff */
#define PROG_INDICATION_UNIT     0

/* NB: Start point address (archStart) and COP vector reset address for */
/* users application defined in linker.cmd file                         */

/* Errors codes */
#define INDICATE_ERROR_RECEIVE   1
#define INDICATE_ERROR_CHARACTER 2
#define INDICATE_ERROR_FORMAT    3
#define INDICATE_ERROR_CHECKSUM  4
#define INDICATE_ERROR_OVERRUN   5
#define INDICATE_ERROR_FLASH     6
#define INDICATE_ERROR_INTERNAL  7
#define INDICATE_ERROR_PARITY    8
#define INDICATE_ERROR_PROTECTED_BOOT_SECTION 9

/* Small delay after application start message displayed and real application started.    */
/* Needs for MS Hyperterminal to correct displaying last message in case of application   */
/* changes SCI parametres or work incorrectly. If set to zero, works without delay.       */

#define TERMINAL_OUTPUT_DELAY    ((UWord16)(ZCLOCK_FREQUENCY / 800000ul) * 5u)

/* Small delay after bootloader error message displayed and actual bootloader restart. */
/* Needs for skip end of curently transfered S-Record file to reduce  future stream of */
/* INDICATE_ERROR_FORMAT or INDICATE_ERROR_CHARACTER errors displayed bootloader after */
/* restart.  If set to zero, works without delay.                                      */

#define TERMINAL_ERROR_DELAY    ((UWord16)(ZCLOCK_FREQUENCY / 800000ul) * 50u)
/*****************************************************************************
*
* End of configuration parameters
*
*****************************************************************************/

/* Additional PLL settings */
/* PLL CONTROL REGISTER FLAGS */

#define PLL_LOCK_DETECTOR               0x0080
#define PLL_ZCLOCK_PRESCALER            0x0001
#define PLL_ZCLOCK_POSTSCALER           0x0002

/* PLL CLKO SELECT REGISTER FLAGS */

#define PLL_CLKO_SELECT_ZCLK            0x0000
#define PLL_CLKO_SELECT_NO_CLK          0x0010

/* PLL STATUS REGISTER FLAGS */

#define PLL_STATUS_LOCK_0               0x0020

#define PLL_DIVIDE_BY_REG               (PLL_MUL - 1)
#define PLL_CONTROL_REG                 (PLL_LOCK_DETECTOR | PLL_ZCLOCK_POSTSCALER)
#define PLL_TEST_REG                    0
#define PLL_SELECT_REG                  (PLL_CLKO_SELECT_ZCLK)

/* For DSP56F801 Internal/external clock specific defines */
#define PLL_PRESCALER_EXTERNAL_CLK_SELECT	0x0004	/* 1 for Crystal Oscillator */


/* Disable Pull up for EXTAL & XTAL pins in PUR(b3,b2 for GPIO B)*/
#define PLL_DISABLE_PULLUP_EXTAL_XTAL   0x00F3


/*****************************************************************************/

#if !defined(NULL)
#define NULL (0)
#endif

typedef unsigned int size_t;

typedef enum
{
	XData,
	PData
} mem_eMemoryType;

/*****************************************************************************/

void * bootmemCopyXtoX  ( void *dest, const void *src, size_t count );
void * bootmemCopyPtoX  ( void *dest, const void *src, size_t count );


extern void userError(int Error);
//extern void bootArchStart  ( void );


/*****************************************************************************/

/* Define registers and bits to work with COP */
#define CEN         $0002

/* Signals bootloader to start application immediately */
#define NO_DELAY	$fe00

extern void archStart(void);
extern byte	_board_TYPE;
extern byte	_board_FW_VER; 
extern byte	_board_FW_REV; 

	
#endif /* !defined(__BOOTLOADER_H) */
