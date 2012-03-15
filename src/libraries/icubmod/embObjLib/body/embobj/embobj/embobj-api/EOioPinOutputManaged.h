
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPINOUTPUTMANAGED_H_
#define _EOIOPINOUTPUTMANAGED_H_


/** @file       EOioPinOutputManaged.h
    @brief      This header file implements public interface to a managed output pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/

/** @defgroup eo_pinwave Object EOioPinOutputManaged
    The object EOioPinOutputManaged is derived from EoIOPin and is simply a GPIO configures as output.
    The EOioPinOutputManaged can be instantiated and configured only by means of EoTheGPIO singleton. 
    For this reason, the constructor of the object is not public and the final user can 
    only get a handle of the object and operate to get the input value of the pin.

    Also, the EOioPinOutputManaged requires the activation of a EOVtheGPIOManager derived object, such as
    EOMtheGPIOManager, which helps to offer its services.
    
    The EOioPinOutputManaged adds some capabilities to the ones that EoOutPin offers. 
    
    Apart being able to read, write and toggle the value of the pin, the user can also start/stop a square waveform
    with a given duration and duty cycle. The transitions of the waveform are precise to the extent of how is frequent
    the tick of the EOVtheGPIOManager derived object (typical is order of millisec or tens of) and on its execution
    priority.  That is teh reason for which a very precise and fast waveform should be implemented with a dedicated 
    HAL entity (e.g., a PWM waveform).
 
    The EOioPinOutputManaged offers more than the EoOutPin and thus costs sligthly more in terms of RAM and CPU. 
    The choice what to use is up to the user.
 
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOioPin.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



/** @typedef    typedef struct EoMngOutin_hid EOioPinOutputManaged
    @brief      EOioPinOutputManaged is an opaque struct. It is used to implement data abstraction for the managed ouput pin 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOioPinOutputManaged_hid EOioPinOutputManaged;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

 
/** @fn         extern EOioPinOutputManaged * eo_iopinoutman_GetHandle(eOid08_t id)
    @brief      Gets the handle of the EOioPinOutputManaged identified by its @e id.
    @param      id              The identifier of the EOioPinOutputManaged as defined in the public interface of the 
                                EoGPIOConfigDerived object used to initialise EoTheGPIO.
    @return     The handle to the requested EOioPinOutputManaged object or NULL if the identifier is invalid.
 **/ 
extern EOioPinOutputManaged * eo_iopinoutman_GetHandle(eOid08_t id);

 
/** @fn         extern eOiopinVal_t eo_iopinoutman_GetVal(EOioPinOutputManaged *const p)
    @brief      Gets the value of the pin.
    @param      p               Handle to EOioPinOutputManaged object.
    @return     The current value or hal_gpio_valNONE if EOioPinOutputManaged is not yet configured or if p is NULL.
 **/
extern eOiopinVal_t eo_iopinoutman_GetVal(EOioPinOutputManaged *const p);


/** @fn         extern eOresult_t eo_iopinoutman_SetVal(EOioPinOutputManaged *p, eOiopinVal_t val)
    @brief      Sets the value of the pin. 
    @param      p               Handle to EOioPinOutputManaged object.
    @param      val             T target value.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_busy if a waveform is 
                active on the EOioPinOutputManaged object or eores_NOK_generic if the EOioPinOutputManaged is not configured.
 **/
extern eOresult_t eo_iopinoutman_SetVal(EOioPinOutputManaged *const p, eOiopinVal_t val); 


/** @fn         extern eOiopinStatus_t eo_iopinoutman_GetStatus(EOioPinOutputManaged *const p)
    @brief      Gets the status of the pin.
    @param      p               Handle to EOioPinOutputManaged object.
    @return     The current status or eo_iopinStatUNDEF if p is NULL;
 **/
extern eOiopinStatus_t eo_iopinoutman_GetStatus(EOioPinOutputManaged *const p);



/** @fn         extern eOresult_t eo_iopinoutman_Waveform_Start(EOioPinOutputManaged *const p, eOiopinVal_t val1st, 
                                                           eOreltime_t ustime1st, eOreltime_t ustime2nd, uint32_t numwaves)
    @brief      Starts a square waveform on the EOioPinOutputManaged. The duration of the waveform is numwaves * (ustime1st + ustime2nd) 
                micro-seconds or infinite if numwaves is equal to timeINFINITE.
    @param      p               Handle to EOioPinOutputManaged object.
    @param      val1st          The value of the first half of the wave.
    @param      ustime1st       The duration in micro-seconds of the first half of the wave.
    @param      ustime2nd       The duration in micro-seconds of the second half of the wave.
    @param      numwaves        The number of the waves (infinite if eok_reltimeINFINITE).
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_busy if the manager was busy and 
                could not start the waveform.
 **/
extern eOresult_t eo_iopinoutman_Waveform_Start(EOioPinOutputManaged *const p, eOiopinVal_t val1st, eOreltime_t ustime1st, 
                                           eOreltime_t ustime2nd, uint32_t numwaves);


/** @fn         extern eOresult_t eo_iopinoutman_Waveform_Stop(EOioPinOutputManaged *const p)
    @brief      Stops a square waveform running on the EOioPinOutputManaged. If the waveform is not running, the call of this function 
                does not produce any harm.
    @param      p               Handle to EOioPinOutputManaged object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_busy if the manager was busy and 
                could not stop the waveform.
 **/
extern eOresult_t eo_iopinoutman_Waveform_Stop(EOioPinOutputManaged *const p);


/** @}            
    end of group eo_pinwave  
 **/

#endif  // include-guard


