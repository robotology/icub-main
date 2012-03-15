
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPININPUTMANAGED_H_
#define _EOIOPININPUTMANAGED_H_


/** @file       EOioPinInputManaged.h
    @brief      This header file implements public interface to a managed input pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/

/** @defgroup eo_mnginppin Object EOioPinInputManaged
    The object EOioPinInputManaged is derived from EoIOPin and is simply a GPIO configured as input.
    The EOioPinInputManaged can be instantiated and configured only by means of EoTheGPIO singleton. 
    For this reason, the constructor of the object is not public and the final user can 
    only get a handle of the object and operate to get the input value of the pin.

    Also, the EOioPinInputManaged requires the activation of a EOVtheGPIOManager derived object, such as
    EOMtheGPIOManager, which helps to offer its services.
    
    The EOioPinInputManaged adds some capabilities to the ones that EOioPinInput offers. Apart being able to read
    the value of the input pin, the user can also register some actions upon triggers on variation of the input 
    signal.
    Supported actions are those specified in EOaction type, that are: to send an event or a message to a task
    or also to have a callback function executed.
    Supported triggers are those defined in eOiopinTrigger_t type, that are: change level to High, changed level to Low,
    stay in High state for at least some time, stay in Low state for at least some time. 
    Each EOioPinInputManaged can support all the four defined triggers and can have a different action for each of them. 
 
    The EOioPinInputManaged offers more than the EoInpPin and thus costs sligthly more in terms of RAM and CPU. 
    The choice what to use is up to the user.

  
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOaction.h"
#include "EOioPin.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef enum eOiopinTrigger_t
    @brief      eOiopinTrigger_t contains the trigger modes with which a EOioPinInputManaged can associated.
 **/
typedef enum  
{ 
    eo_iopinTrig_NONE                 = 0x00, 
    eo_iopinTrig_OnRiseHit            = 0x01, 
    eo_iopinTrig_OnFallHit            = 0x02,
    eo_iopinTrig_OnRiseStay           = 0x04,
    eo_iopinTrig_OnFallStay           = 0x08, 
    eo_iopinTrig_ALL                  = 0x0F 
} eOiopinTrigger_t; 


/** @typedef    typedef struct EOioPinInputManaged_hid EOioPinInputManaged
    @brief      EOioPinInputManaged is an opaque struct. It is used to implement data abstraction for the managed input pin 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOioPinInputManaged_hid EOioPinInputManaged;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

 
/** @fn         extern EOioPinInputManaged * eo_iopininpman_GetHandle(eOid08_t id)
    @brief      Gets the handle of the EOioPinInputManaged identified by its @e id.
    @param      id              The identifier of the EOioPinInputManaged as defined in the public interface of the 
                                EoGPIOConfigDerived object used to initialise EoTheGPIO.
    @return     The handle to the requested EOioPinInputManaged object or NULL if the identifier is invalid.
 **/ 
extern EOioPinInputManaged * eo_iopininpman_GetHandle(eOid08_t id);
 
 
/** @fn         extern eOiopinVal_t eo_iopininpman_GetVal(EOioPinInputManaged *const p)
    @brief      Gets the value of the pin.
    @param      p               Handle to EOioPinInputManaged object.
    @return     The current value or eo_iopinvalNONE if EOioPinInputManaged is not yet configured or if p is NULL.
 **/
extern eOiopinVal_t eo_iopininpman_GetVal(EOioPinInputManaged *const p);


/** @fn         extern eOiopinStatus_t eo_iopininpman_GetStatus(EOioPinInputManaged *const p)
    @brief      Gets the status of the pin.
    @param      p               Handle to EOioPinInputManaged object.
    @return     The current status or eo_iopinStatUNDEF if p is NULL;
 **/
extern eOiopinStatus_t eo_iopininpman_GetStatus(EOioPinInputManaged *const p);



/** @fn         extern eOresult_t eo_iopininpman_ActionOn_Register(EOioPinInputManaged *mi, EOaction *acton, 
                                                              eOiopinTrigger_t trigger, eOreltime_t after)
    @brief      Instructs the EoTheGPIOManager to perform an action @e acton upon the verification on the managed input pin of a  
                condition specified by the couple of parameters @e trigger and @e after.
                On the same pin, it is possible to register up-to four actions, one for each triggering condition. In such a way
                a pin can send a message when it goes low, another message when it stays low for at least 100k microseconds,
                send an event when it goes high and execute a callback after 50k microseconds it stays high.
    @param      p               Handle to EOioPinInputManaged object.
    @param      acton           Action to be performed at the happening of the condition on the pin.
    @param      trigger         The condition to be verified on the input pin in order to do the action.
    @param      after           Parameters used when trigger is eo_iopinTrig_OnRiseStay or eo_iopinTrig_OnFallStay. It is the duration in 
                                micro-seconds of continuous permanence of the input pin level on high or on low.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_busy if the manager is busy and cannot 
                register the action.
 **/
extern eOresult_t eo_iopininpman_ActionOn_Register(EOioPinInputManaged *mi, EOaction *acton, 
                                              eOiopinTrigger_t trigger, eOreltime_t after);


/** @fn         extern eOresult_t eo_iopininpman_ActionOn_Unregister(EOioPinInputManaged *p, eOiopinTrigger_t trigger)
    @brief      Instructs the EoTheGPIOManager to remove the action associated to @e trigger. 
    @param      p               Handle to EOioPinInputManaged object.
    @param      trigger         The condition to unregister.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_busy if the manager is busy and cannot
                unregister the action.
    @todo       Now the function removes every action. Make this function work to remove a single trigger and also _All.
 **/
extern eOresult_t eo_iopininpman_ActionOn_Unregister(EOioPinInputManaged *p, eOiopinTrigger_t trigger);


/** @}            
    end of group eo_mnginppin  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


