/*********************************************************************
 *                Microchip USB C18 Firmware Version 1.0
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18f2550.h>
#include "system\typedefs.h"                        // Required
#include "system\usb\usb.h"                         // Required
#include "io_cfg.h"                                 // Required

#include "system\usb\usb_compile_time_validation.h" // Optional

/** Configuração *************************************************************/
#pragma CONFIG DEBUG = OFF


/** V A R I A B L E S ********************************************************/
#pragma udata




/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void 		USBTasks(void);

/** D E C L A R A T I O N S **************************************************/
#pragma code

void main(void)
{
//	letras=0;
    InitializeSystem();
    while(1)
    {
       USBTasks();         // USB Tasks
       ProcessIO();        // See user.c
    }//end while
}//end main



static void InitializeSystem(void)
{
	INTCON = 0;
        
    mInitializeUSBDriver();         // See usbdrv.h
    
    UserInit();                     // See user.c & .h

}//end InitializeSystem


void USBTasks(void)
{
    /* Servicing Hardware */
    USBCheckBusStatus();                    // Must use polling method
    if(UCFGbits.UTEYE!=1)
        USBDriverService();                 // Interrupt or polling method
    #if defined(USB_USE_CDC)
    CDCTxService();
    #endif

}// end USBTasks



/** EOF main.c ***************************************************************/
