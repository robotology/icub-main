
/* @file       test-eogpio.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

// embobj
#include "EoCommon.h"
#include "EOaction.h"

#include "eOcfg_GPIO_MCBSTM32c.h"
#include "EOtheGPIO.h"
#include "EOioPinOutput.h"
#include "EOtheGPIOManager.h"
#include "EOtheGPIOManager_hid.h"
#include "EOioPinOutputManaged.h"
#include "EOioPinInputManaged.h"
#include "EOMtheCallbackManager.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-eogpio.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

void s_cback_sendrequest(uint32_t cbk);
static void s_callback(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static EOtheGPIO *s_thegpio = NULL;

static EOioPinOutput *s_iopinLED10 = NULL;
static EOioPinOutputManaged *s_iopinwaveLED15 = NULL;
static EOioPinInputManaged *s_iopintrigBUTTONWKUP = NULL;



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

// configure gpios, starts the gpio manager, use a waveforme on led15 and a callback on button wakeup pressed 3 secs
// which toggles led 10
void test_eogpio_EOgpioInit(void)
{
    eOiopinVal_t val = eo_iopinvalNONE;
    EOaction * acton;
   
    val = val;

   
    s_thegpio = eo_gpio_Initialise(eo_cfg_gpio_mcbstm32c_Get());
    s_thegpio = s_thegpio;
    
    s_iopinLED10 = eo_iopinout_GetHandle(iopinID_Out_mcbstm32c_LED_010);
    
    // ok, operate manually with pinout functions
    eo_iopinout_SetVal(s_iopinLED10, eo_iopinvalHIGH);
    eo_iopinout_ToggleVal(s_iopinLED10);
    // but also can operate with the base iopin functions.
    eo_iopin_derived_ToggleVal(s_iopinLED10);
    val = eo_iopin_derived_GetVal(s_iopinLED10);


    // now we initialise the gpio manager, so that it can manage waveforms and triggers.
    eo_gpioman_Initialise(s_thegpio, NULL);

    // and we initialise a waveform on led 15
    s_iopinwaveLED15 = eo_iopinoutman_GetHandle(iopinID_Oman_mcbstm32c_LED_015);
    eo_iopinoutman_Waveform_Start(s_iopinwaveLED15, eo_iopinvalHIGH, 1*1000*1000, 2*1000*1000, eok_reltimeINFINITE);

    eo_iopin_derived_ToggleVal(s_iopinwaveLED15);

    eo_iopin_derived_ToggleVal(s_iopinwaveLED15);

    // and also a callback on button wakeup
    s_iopintrigBUTTONWKUP = eo_iopininpman_GetHandle(iopinID_Iman_mcbstm32c_BUTTON_WKUP);
    acton = eo_action_New();
    eo_action_SetCallback(acton, s_callback, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle())); 
    
    eo_iopininpman_ActionOn_Register(s_iopintrigBUTTONWKUP, acton, eo_iopinTrig_OnRiseStay, 3*1000*1000);




}

void test_eogpio_EOgpioTick(uint32_t deltatime)
{

    eo_gpioman_Tick(eo_gpioman_GetHandle(), deltatime);    

}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

void s_cback_sendrequest(uint32_t cbk)
{
    // dont send.... just execute it.
    void (*cbk_fn)(void) = (void (*)(void))cbk;
    cbk_fn();
}


static void s_callback(void *p)
{
    eo_iopinout_ToggleVal(s_iopinLED10);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



