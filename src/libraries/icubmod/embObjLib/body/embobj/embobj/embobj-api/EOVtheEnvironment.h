
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHEENVIRONMENT_H_
#define _EOVTHEENVIRONMENT_H_


/** @file       EOVtheEnvironment.h
    @brief      This header file implements public interface to the virtual environment singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eov_EOVtheEnvironment Singleton EOVtheEnvironment
    The EOVtheEnvironment is an abstract object used to derive a callback manager object for the single-task or multi-task
    execution environment: EOStheCallbackManager or EOMtheCallbackManager.
    The EOVtheEnvironment exposes only pure virtual functions which have to be defined inside the derived object.
 
    The normal user shall typically use only the methods of the derived object, i.e. only eom_env_Initialise().

    However, if an object cannot be tied to a particular execution environment, then it can use some methods of the 
    EOVtheEnvironment, provided that a properly derived object has been initialised for the correct execution enviromnent.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "eEcommon.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOVtheEnvironment_hid EOVtheEnvironment
    @brief      EOVtheEnvironment is an opaque struct. It is used to implement data abstraction for the timer manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOVtheEnvironment_hid EOVtheEnvironment;


/** @typedef    typedef void EOVtheEnvironmentDerived
    @brief      EOVtheEnvironmentDerived is used to implement polymorphism in the objects derived from EOVtheEnvironment
 **/
typedef void EOVtheEnvironmentDerived;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOVtheEnvironment* eov_env_GetHandle(void)
    @brief      Returns an handle to the singleton EOVtheEnvironment.  The singleton must have been initialised by
                the _Initialise() method of a derived type, otherwise this function call will return NULL.
    @return     The pointer to the required EOVtheEnvironment (or NULL upon in-initialised singleton).
 **/
//extern EOVtheEnvironment* eov_env_GetHandle(void);


/** @fn         extern eOresult_t eov_env_SharedData_Synchronise(EOVtheEnvironment *p)
    @brief      Allows a caller to specify a callback to be directly executed by the object derived from EOVtheEnvironment
    @param      p               Pointer to the object
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic on error
 **/
extern eOresult_t eov_env_SharedData_Synchronise(EOVtheEnvironmentDerived *p);


/** @fn         extern EOVtaskDerived * eov_env_GetTask(EOVtheEnvironment *p)
    @brief      Retrieves the working task of the object derived from EOVtheEnvironment for use in the object EOaction
    @param      p               Pointer to the object
    @return     The pointer to the EOVtaskDerived.
 **/
extern eOresult_t eov_env_ProcessOffset_Get(EOVtheEnvironmentDerived *p, uint32_t *offset);

extern eOresult_t eov_env_RunningEprocess_Get(EOVtheEnvironmentDerived *p, eEprocess_t *eproc);

extern eOresult_t eov_env_IPnetwork_Get(EOVtheEnvironmentDerived *p, const eEipnetwork_t **ipnet);

extern eOresult_t eov_env_CANnetworks_Get(EOVtheEnvironmentDerived *p, const eEcannetwork_t **cannets, uint8_t *numnets);



/** @}            
    end of group eov_EOVtheEnvironment  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



