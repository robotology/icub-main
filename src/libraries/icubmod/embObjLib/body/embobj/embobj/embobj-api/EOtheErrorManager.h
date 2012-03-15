
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEERRORMANAGER_H_
#define _EOTHEERRORMANAGER_H_


/** @file       EOtheErrorManager.h
	@brief      This header file implements public interface to the error manager singleton.
 	@author     marco.accame@iit.it
	@date       08/03/2011
 **/

/** @defgroup eo_theerrormanager Singleton EOtheErrorManager 
    
    The error manager singleton is used by the embOBJ to report errors and to enter in the appropriate error mode.
    This singleton can work in the SEE or MEE by means of some virtual objects: the EOVtheSystem and the EOVtask.  
  
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef struct EOtheErrorManager_hid EOtheErrorManager
    @brief      EOtheErrorManager is an opaque struct. It is used to implement data abstraction for the error manager 
                object so that the user cannot see its private fields so that he/she is forced to manipulate the
                object only with the proper public functions
 **/  
typedef struct EOtheErrorManager_hid EOtheErrorManager;


/**	@typedef    typedef enum eOerrmanErrorType_t 
 	@brief      Contains the error types managed by the EOtheErrorManager 
 **/  
typedef enum  
{
    eo_errortype_info    = 0,       /**< used to communicate some innocent situation */
    eo_errortype_warning = 1,       /**< used to communicate some strange situation */
    eo_errortype_weak    = 2,       /**< used to communicate a weak error which could be recovered by user intervention */
    eo_errortype_fatal   = 3        /**< used to communicate a fatal error which requires stopping the system */
} eOerrmanErrorType_t;


/** @typedef    typedef struct eOerrman_fn_cfg_t
    @brief      eOerrman_fn_cfg_t keeps pointers to functions of EOtheErrorManager whcih can be redefined by the user.
 **/
typedef struct
{
    /** When an error is detected, the error manager calls this function and if errtype is eo_errortype_weak or lower
        it returns control to the environment. Otherwise if error is eo_errortype_fatal it enters in a forever loop.
        Parameters are: the error type, the id of the calling task, the name of the calling embOBJ, and a string with a
        more detailed info */ 
    void            (*usr_on_error)(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);
} eOerrman_fn_cfg_t;

/**	@typedef    typedef struct eOerrman_cfg_t 
 	@brief      Contains the configuration for the EOtheErrorManager. 
 **/
typedef struct
{
    eOerrman_fn_cfg_t    extfn;
} eOerrman_cfg_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOerrman_cfg_t eom_errman_DefaultCfg; // = {.extfn = { .usr_on_error = NULL}};


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOtheErrorManager * eo_errman_Initialise(const eOerrman_cfg_t *errmancfg)
    @brief      Initialise the EOtheErrorManager singleton 
    @arg        errmancfg       The configuration. NULL is OK.
    @return     The handle to the EOtheErrorManager
 **/

extern EOtheErrorManager * eo_errman_Initialise(const eOerrman_cfg_t *errmancfg);


/** @fn         extern EOtheErrorManager * eo_errman_GetHandle(void)
    @brief      Retrieve the EOtheErrorManager singleton 
    @return     The handle to the EOtheErrorManager
 **/
extern EOtheErrorManager * eo_errman_GetHandle(void);
 
 
/** @fn         extern void eo_errman_Assert(EOtheErrorManager *p, uint32_t cond, const char *eobjstr,
                                            const char *info)
    @brief      If the value of @e cond is 0, it calls the configured usr_on_error() with eo_errortype_fatal, 
                then stops the system using the eov_sys_Stop() function and finally enters in a forever loop.
    @param      p               The singleton
    @param      condition       The condition to be tested
    @param      eobjstr         A string containing the name of the calling object. 
    @param      info            A string containing a specific message from the calling object.
 **/
extern void eo_errman_Assert(EOtheErrorManager *p, uint32_t cond, const char *eobjstr, const char *info);


/** @fn         extern void eo_errman_Error(EOtheErrorManager *p, eOerrmanErrorType_t errtype, const char *eobjstr, 
                                            const char *info) 
    @brief      It calls the configured usr_on_error() with the passed errtype, and if it is a fatal error it also
                 stops the system using the eov_sys_Stop() function and finally enters in a forever loop. 
    @param      p               The singleton
    @param      errtype         The error type.
    @param      eobjstr         A string containing the name of the calling object. 
    @param      info            A string containing a specific message from the calling object.
 **/

extern void eo_errman_Error(EOtheErrorManager *p, eOerrmanErrorType_t errtype, const char *eobjstr, const char *info);


/** @fn         extern void eo_errman_Info(EOtheErrorManager *p, const char *eobjstr, const char *info) 
    @brief      It calls eo_errman_Error(p, eo_errortype_info, eobjstr, info). 
    @param      p               The singleton
    @param      eobjstr         A string containing the name of the calling object. 
    @param      info            A string containing a specific message from the calling object.
 **/
extern void eo_errman_Info(EOtheErrorManager *p, const char *eobjstr, const char *info);







/** @}            
    end of group eo_theerrormanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

