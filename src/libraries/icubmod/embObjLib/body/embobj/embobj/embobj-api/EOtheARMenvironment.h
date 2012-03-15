
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEARMENVIRONMENT_H_
#define _EOTHEARMENVIRONMENT_H_

/** @file       EOtheARMenvironment.h
	@brief      This header file contains public interfaces for the ARM environment singleton.
	@author     marco.accame@iit.it
	@date       10/01/2012
**/


/** @defgroup eo_thearmenv Object EOtheARMenvironment
    The EOtheARMenvironment is a fcrefvregfvregfregvfergfver 

    
    @{		
 */
 

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "eEcommon.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/**	@typedef    EOtheARMenvironment_hid EOtheARMenvironment
 	@brief 		EOtheARMenvironment is an opaque struct. It is used to implement ferferferfver
                object so that the user cannot see its private fields so that he/she is forced to manipulate the 
                object only with the proper public functions. 
 **/  
typedef struct EOtheARMenvironment_hid EOtheARMenvironment;


/**	@typedef    typedef struct eOarmenv_cfg_t 
 	@brief      Contains the configuration for the EOtheARMenvironment. 
 **/
typedef struct
{
    uint8_t         dummy;
} eOarmenv_cfg_t;

   
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOarmenv_cfg_t eo_armenv_DefaultCfg; // = {0};


// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern EOtheARMenvironment * eo_armenv_Initialise(const eEmoduleInfo_t *modinfo, const eEboardInfo_t *brdinfo)
    @brief      Initialise the singleton EOtheARMenvironment. 
    @param      cfg            cervrevrevre
    @return     Pointer to the required EOtheARMenvironment singleton (or NULL upon un-initialised singleton).
 **/
extern EOtheARMenvironment * eo_armenv_Initialise(const eEmoduleInfo_t *modinfo, const eEboardInfo_t *brdinfo);


/** @fn         extern EOtheARMenvironment* eo_armenv_GetHandle(void)
    @brief      Returns a handle to the singleton EOtheARMenvironment. The singleton must have been initialised
                with eo_armenv_Initialise(), otherwise this function call will return NULL.
    @return     Pointer to the required EOtheARMenvironment singleton (or NULL upon un-initialised singleton).
 **/
extern EOtheARMenvironment * eo_armenv_GetHandle(void);


// resetta solo la eeprom, mette tutto ai valori di default ma non la riempie .... con nulla
extern eOresult_t eo_armenv_SharedStorage_Clear(EOtheARMenvironment *p);

// verifica se nelle posizioni standard ci sono processi e .... li mette nella partable
extern eOresult_t eo_armenv_SharedStorage_ScanProcesses(EOtheARMenvironment *p);



/** @}            
    end of group eo_thearmenv  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

