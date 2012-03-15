/**************************************************************************//**
 * @file     main.c
 * @brief    CMSIS Cortex-M3 Blinky example
 *           Blink a LED using CM3 SysTick
 * @version  V1.03
 * @date     24. September 2009
 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#include "stm32f10x.h"
#include "misc.h"

#if USESHALPART
#include "shalPART.h"
#endif

#include "shalBASE.h"
#include "string.h"


volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
/*__INLINE*/ static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

/*------------------------------------------------------------------------------
  configer LED pins
 *------------------------------------------------------------------------------*/
__INLINE static void LED_Config(void) {

  RCC->APB2ENR |=  1 <<  6;                     /* Enable GPIOE clock          */
  GPIOE->CRH    = 0x33333333;                   /* Configure the GPIO for LEDs */
}

/*------------------------------------------------------------------------------
  Switch on LEDs
 *------------------------------------------------------------------------------*/
__INLINE static void LED_On (uint32_t led) {

  GPIOE->BSRR = (led);                          /* Turn On  LED */
}


/*------------------------------------------------------------------------------
  Switch off LEDs
 *------------------------------------------------------------------------------*/
__INLINE static void LED_Off (uint32_t led) {

  GPIOE->BRR  = (led);                          /* Turn Off LED */
}


    eEprocess_t pr;

    eEresult_t res;

    eEversion_t v;

    uint8_t dd[32];

    uint8_t size = 0;
    uint8_t chr;

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/


void goto_error(void)
{
    for(;;)
    {
        LED_On (0x200);                            
        Delay (100);                                
        LED_Off (0x200);                            
        Delay (100);
    }
}


static const eEmoduleInfo_t s_upper_info __attribute__((at(0x8020000+EENV_MODULEINFO_OFFSET))) = 
{
    .type               = ee_process,
    .signature          = ee_procApplication,
    .version            = {1, 0},
    .builddate          = {2010, 3, 11, 16, 57},
    .name               = "eUpper",

    .rom                = 
    {
        .addr           = 0x8020000,
        .size           = 1024*8
    },
    .ram                = 
    {
        .addr           = 0x20000000,
        .size           = 1024*4
    },
    .storage            = {ee_strg_none, 0, 0}
};

int main (void) 
{
  uint32_t countdown = 10;
  static eEmoduleInfo_t *pv = NULL;
  uint8_t data[2];
  static volatile eEresult_t res = ee_res_OK;

  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x20000);

  if (SysTick_Config(SystemCoreClock / 1000)) { /* Setup SysTick Timer for 1 msec interrupts  */
    while (1);                                  /* Capture error */
  }



  pv = shalbase_moduleinfo_get();

  res = shalbase_isvalid();

    if(ee_res_OK == res)
    {
        shalbase_init(1);

#if USESHALPART
        if(ee_res_OK == shalpart_isvalid())
        {
            shalpart_init();
           // put signature in partition table
            shalpart_proc_synchronise(ee_procApplication, &s_upper_info);
            shalpart_proc_def2run_set(ee_procApplication);
        }
#endif
    }
    else
    {
        goto_error();
    }


 

  LED_Config(); 
  LED_On (0x200);
  Delay (2000); 



    


 


//  Delay (5000); 
  
  

  while(1) {
    
    if(0 == countdown)
    {

//        if(ee_res_NOK_generic != shalbase_isvalid())
        {
            // set two bytes: b0 is time on and b1 is time off, and b0+b1=10
            data[0] = 8; data[1] = 2;
            shalbase_ipc_volatiledata_set(data, 2);
            // ask to go to the loader
            shalbase_ipc_gotoproc_set(ee_procLoader);
            // restart
            shalbase_system_restart();
        }
//        else
//        {
//            goto_error();
//        }
    }

    countdown --;

    LED_On (0x200);                             /* Turn on the LED. */
    Delay (300);                                /* delay  300 Msec */
    LED_Off (0x200);                            /* Turn off the LED. */
    Delay (700);                                /* delay  1700 Msec */
  }
  
}

