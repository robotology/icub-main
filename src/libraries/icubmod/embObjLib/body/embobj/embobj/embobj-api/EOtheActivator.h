
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEACTIVATOR_H_
#define _EOTHEACTIVATOR_H_

/** @file       EOtheActivator.h
	@brief      This header file contains public interfaces for the activator singleton object.
	@author     marco.accame@iit.it
	@date       10/01/2012
**/


/** @defgroup eo_theactivator Object EOtheActivator
    The EOtheActivator is a fcrefvregfvregfregvfergfver 

    
    @{		
 */
 

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"
#include "EOaction.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/**	@typedef    EOtheActivator_hid EOtheActivator
 	@brief 		EOtheActivator is an opaque struct. It is used to implement ferferferfver
                object so that the user cannot see its private fields so that he/she is forced to manipulate the 
                object only with the proper public functions. 
 **/  
typedef struct EOtheActivator_hid EOtheActivator;


/**	@typedef    typedef struct eOactivator_cfg_t 
 	@brief      Contains the configuration for the EOtheErrorManager. 
 **/
typedef struct
{
    EOVmutex*       mutex;
    eOsizecntnr_t   capacity;  
} eOactivator_cfg_t;



/**	@typedef    typedef struct eOactivator_objectinfo_t 
 	@brief      Contains the info used by the EOtheActivator to activate/deactivate the object. 
 **/ 
typedef struct
{
    eOvoid_fp_voidp_t           activate;
    eOvoid_fp_voidp_t           deactivate;
    void*                       objptr;
} eOactivator_objectinfo_t;
   
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOactivator_cfg_t eo_activator_DefaultCfg; // = {NULL, 4};


// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern EOtheActivator * eo_activator_Initialise(const eOactivator_cfg_t *cfg)
    @brief      Initialise the singleton EOtheActivator. 
    @param      cfg             It specifies the working mode of the allocator. A NULL value for cfg forces
                                use of eo_activator_DefaultCfg.
    @return     Pointer to the required EOtheActivator singleton (or NULL upon un-initialised singleton).
 **/
extern EOtheActivator * eo_activator_Initialise(const eOactivator_cfg_t *cfg);


/** @fn         extern EOtheActivator* eo_activator_GetHandle(void)
    @brief      Returns a handle to the singleton EOtheActivator. The singleton must have been initialised
                with mempool_Initialise(), otherwise this function call will return NULL.
    @return     Pointer to the required EOtheActivator singleton (or NULL upon un-initialised singleton).
 **/
extern EOtheActivator * eo_activator_GetHandle(void);


/** @fn         xtern eOresult_t eo_activator_Register(EOtheActivator *p, eOactivator_objectinfo_t* objinfo)
    @brief      Registers an object inside the activator.
    @param      p               The  singleton                  
    @param      objinfo         The info required to activate or deactivate the object.
    @return     eores_OK upon success.
 **/
extern eOresult_t eo_activator_Register(EOtheActivator *p, eOactivator_objectinfo_t* objinfo);


/** @fn         extern eOresult_t eo_activator_SetActionOnNumItems(EOtheActivator *p, EOaction *act, eOsizecntnr_t onnumitems)
    @brief      sets an action to be executed when an object is registered. the action is done on teh first onnumitems items
    @param      p               The singleton                
    @return     eores_OK upon success. 
 **/ 
extern eOresult_t eo_activator_SetActionOnNumItems(EOtheActivator *p, EOaction *act, eOsizecntnr_t onnumitems);

 
/** @fn         extern eOresult_t eo_activator_ActivateAll(EOtheActivator *p)
    @brief      Activates all registered objects.
    @param      p               The singleton                
    @return     eores_OK upon success. 
 **/ 
extern eOresult_t eo_activator_ActivateAll(EOtheActivator *p);


/** @fn         extern eOresult_t eo_activator_DeactivateAll(EOtheActivator *p)
    @brief      Deactivates all registered objects.
    @param      p               The singleton                
    @return     eores_OK upon success. 
 **/ 
extern eOresult_t eo_activator_DeactivateAll(EOtheActivator *p);



/** @}            
    end of group eo_theactivator  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

