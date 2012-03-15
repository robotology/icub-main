
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPININPUT_H_
#define _EOIOPININPUT_H_


/** @file       EOioPinInput.h
    @brief      This header file implements public interface to an input pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/

/** @defgroup eo_iopininp Object EOioPinInput
    The object EOioPinInput is derived from EOioPin and is simply a GPIO configured as input.
    The EOioPinInput can be instantiated and configured only by means of EOtheGPIO singleton. 
    For this reason, the constructor of the object is not public and the final user can 
    only get a handle of the object and operate to get the input value of the pin.

    The EOioPinInput does dot have the more advanced features such as performing actions upon 
    special conditions which are offered only by the managed input pin, the EoMngInpPin. 
    
    The EOioPinInput is simpler and costs less in terms of RAM and CPU. The choice is up to the
    user.
 
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

/** @typedef    typedef struct EOiopinInp_hid EOioPinInput
    @brief      EOioPinInput is an opaque struct. It is used to implement data abstraction for the input pin 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOioPinInput_hid EOioPinInput;

/** @typedef    typedef void EOiopinInpDerived
    @brief      EoInpPinDerived used to is used to implement polymorphism in the objects derived from EOioPinInput
 **/
typedef void EOioPinInputDerived;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

 
/** @fn         extern EOioPinInput * eo_iopininp_GetHandle(eOid08_t id)
    @brief      Gets the handle of the EOioPinInput identified by its @e id.
    @param      id              The identifier of the EOioPinInput as defined in the public interface of the 
                                configuration object used to initialise EOtheGPIO.
    @return     The handle to the requested EOioPinInput object or NULL if the identifier is invalid.
 **/ 
extern EOioPinInput * eo_iopininp_GetHandle(eOid08_t id);
 
 
/** @fn         extern eOiopinVal_t eo_iopininp_GetVal(EOioPinInput *const p)
    @brief      Gets the value of the pin.
    @param      p               Handle to EOioPinInput object.
    @return     The current value or eo_iopinvalNONE if EOioPinInput is not yet configured or if p is NULL.
 **/
extern eOiopinVal_t eo_iopininp_GetVal(EOioPinInput *const p);


/** @}            
    end of group eo_iopininp  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


