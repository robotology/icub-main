
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_H_
#define _HAL_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal.h
    @brief      This header file implements public interface to the hal modules.
    @author     valentina.gaggero@iit.it, marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup hal HAL

    The Hardware Abstraction Layer offers service to the system which are independent from the underlying
    HW platform.
 
    @todo acemor-facenda: review documentation
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "emBODYporting.h"

#include "hal_base.h"
#include "hal_can.h"
#include "hal_crc.h"
#include "hal_display.h"
#include "hal_eeprom.h"
#include "hal_encoder.h"
#include "hal_eth.h"
#include "hal_flash.h"
#include "hal_gpio.h"
#include "hal_led.h"
#include "hal_sys.h"
#include "hal_timer.h"
#include "hal_trace.h"
#include "hal_watchdog.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section


/** @}            
    end of group hal  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



