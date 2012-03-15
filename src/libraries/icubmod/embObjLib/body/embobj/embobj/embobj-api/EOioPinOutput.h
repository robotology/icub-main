
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPINOUTPUT_H_
#define _EOIOPINOUTPUT_H_


/** @file       EOioPinOutput.h
    @brief      This header file implements public interface to an outut pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/

/** @defgroup eo_iopinout Object EOioPinOutput
    The object EOioPinOutput is derived from EoIOPin and is simply a GPIO configures as outut.
    The EOioPinOutput can be instantiated and configured only by means of EoTheGPIO singleton. 
    For this reason, the constructor of the object is not public and the final user can 
    only get a handle of the object and operate to get the outut value of the pin.

    The EOioPinOutput does dot have the more advanced features such as performing actions upon 
    special conditions which are offered only by the managed outut pin, the EoMngOutPin. 
    
    The EOioPinOutput is simpler and costs less in terms of RAM and CPU. The choice is up to the
    user.
 
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

/** @typedef    typedef struct EOioPinOutput_hid EOioPinOutput
    @brief      EOioPinOutput is an opaque struct. It is used to implement data abstraction for the outut pin 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOioPinOutput_hid EOioPinOutput;

/** @typedef    typedef void EOioPinOutputDerived
    @brief      EOioPinOutputDerived used to is used to implement polymorphism in the objects derived from EOioPinOutput
 **/
typedef void EOioPinOutputDerived;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

 
/** @fn         extern EOioPinOutput * eo_iopinout_GetHandle(eOid08_t id)
    @brief      Gets the handle of the EOioPinOutput identified by its @e id.
    @param      id              The identifier of the EOioPinOutput as defined in the public interface of the 
                                EoGPIOConfigDerived object used to initialise EoTheGPIO.
    @return     The handle to the requested EOioPinOutput object or NULL if the identifier is invalid.
 **/ 
extern EOioPinOutput * eo_iopinout_GetHandle(eOid08_t id);
 
 
/** @fn         extern eOiopinVal_t eo_iopinout_GetVal(EOioPinOutput *const p)
    @brief      Gets the value of the pin.
    @param      p               Handle to EOioPinOutput object.
    @return     The current value or eo_iopinvalNONE if EOioPinOutput is not yet configured or if p is NULL.
 **/
extern eOiopinVal_t eo_iopinout_GetVal(EOioPinOutput *const p);


/** @fn         extern eOresult_t eo_iopinout_SetVal(EOioPinOutput *const p, eOiopinVal_t val)
    @brief      Sets the passed value to the EOioPinOutput object.
    @param      p               The pointer to the EOioPinOutput object.
    @param      val             The target value
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL.
 **/
extern eOresult_t eo_iopinout_SetVal(EOioPinOutput *const p, eOiopinVal_t val);


/** @fn         extern eOresult_t eo_iopinout_ToggleVal(EOioPinOutput *const p)
    @brief      Toggles the value of the EOioPinOutput object only if the pin has value either ioValHIGH or ioValLOW.
    @param      p               The pointer to the EOioPinOutput object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the value is eo_iopinvalNONE.
 **/
extern eOresult_t eo_iopinout_ToggleVal(EOioPinOutput *const p);

/** @}            
    end of group eo_iopinout  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


