
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSM_H_
#define _EOSM_H_


/** @file       EOsm.h
	@brief      This header file implements public interface to a state machine object.
	@author     marco.accame@iit.it
	@date       09/01/2011
**/

/** @defgroup eo_sm Object EOsm
    The EOsm is a state machine with following limitations:
    - up to 256 states, up to 256 total transitions, up to 32 different events.
    - callback functions on exit from state, on transition, on entry in a new state)
    - init function executed on creation of the object (but also on reset)
    - manipulation of dedicated ram
    The EOsm is less complex than its friend the EOumlsm (which is fully UML2.2-compliant),
    but processes events with a guaranteed time (except the time required for on-transition callback).

    @warning    The EOsm must be used by a single task because it does not have protection
                versus concurrency.
    
    @{		
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------

#define EOSM_STATENAMESIZE  8

 

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef struct EOsm_hid EOsm
 	@brief 		EOsm is an opaque struct. It is used to implement data abstraction for the state machine 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOsm_hid EOsm;



/** @typedef    typedef void (*eOsm_void_fp_smp_t)(EOsm *)
    @brief      Function type used for callbacks inside the EOsm object
 **/ 
typedef void (*eOsm_void_fp_smp_t)(EOsm *);


/** @typedef    typedef uint8_t eOsmEvent_t
    @brief      eOsmEvent_t is the basic type for the events that can be fed into the EOsm.
 **/
typedef uint8_t eOsmEvent_t;


/** @typedef    typedef enum eOsmBasicEvents_t 
    @brief      eOsmBasicEvents_t contains the basic events that are common to every SM.
 **/
typedef enum  
{
    eo_sm_evNONE           = (eOsmEvent_t) EOK_uint08dummy     /**< the null event */
} eOsmBasicEvents_t;


/** @typedef    typedef struct eOsmState_t
    @brief      State of a EOsm.
 **/ 
typedef const struct
{
    char                    name[EOSM_STATENAMESIZE];       /**< Name of the state  */
    eOsm_void_fp_smp_t      on_entry_fn;                    /**< Action on entry. It accepts the EOsm pointer as argument.  */
    eOsm_void_fp_smp_t      on_exit_fn;                     /**< Action on exit. It accepts the EOsm pointer as argument     */
} eOsmState_t;


/** @typedef    typedef struct eOsmTransition_t
    @brief      Transition of a EOsm.
 **/ 
typedef const struct
{
    uint8_t                 curr;                           /**< Index of current state     */
    uint8_t                 next;                           /**< Index of next state        */
    uint8_t                 evt;                            /**< Triggering event           */
    eOsm_void_fp_smp_t      on_transition_fn;               /**< Action on transition. It accepts the EOsm pointer as argument  */
} eOsmTransition_t;


/** @typedef    typedef const struct eOsm_cfg_t
    @brief      Contains the configuration for the EOsm object
 **/ 
typedef const struct  
{
    uint8_t                 nstates;                /**< Total number of states. Up to 255 */  
    uint8_t                 ntrans;                 /**< Total number of transitions. Up to to 255 */
    uint8_t                 maxevts;                /**< Total number of events. Up to 32  */
    uint8_t                 initstate;              /**< Initial state expressed as index of the states array */
    uint8_t                 sizeofdynamicdata;      /**< Total size of dynamic data expressed in bytes  */
    eOsmState_t*            states;                 /**< Array containing all the @e nstates states  */
    eOsmTransition_t*       transitions;            /**< Array containing all the @e ntrans transitions  */
    eOsm_void_fp_smp_t      init_fn;                /**< Called on creation of the EOsm. It accepts the EOsm pointer as argument  */                 
    eOsm_void_fp_smp_t      resetdynamicdata_fn;    /**< Resets the dynamic data of the EOsm. It accepts the EOsm pointer as argument  */
} eOsm_cfg_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern EOsm* eo_sm_New(const eOsm_cfg_t *cfg) 
    @brief      Creates a new EOsm object using a given configuration whcih must be provided by the user. 
                Multiple _New(cfg) calls with the same cfg generates multiple independent instances of state machines
                which share the same behaviour but have separate data structures.
    @param      cfg             The configuration of the state machine.
    @return     Pointer to the required EOsm object. The pointer is guaranteed to be always valid and will 
                never be NULL, because failure is managed by the memory pool.
 **/
extern EOsm * eo_sm_New(const eOsm_cfg_t *cfg);



/** @fn         extern eOresult_t eo_sm_Start(EOsm *p)
    @brief      Start the machine by sending it into its initial state and by executing its on-entry function.
    @param      p               The pointer to the state machine.
    @return     null or ok
 **/ 
extern eOresult_t eo_sm_Start(EOsm *p);


/** @fn         extern uint8_t eo_sm_ProcessEvent(EOsm *p, eOsmEvent_t ev)
    @brief      Processes one incoming event and depending on the @e consume mode either terminates or recursively 
                process the internal events triggered by the incoming @e event.
    @param      ptr             The pointer to the state machine.
    @param      ev              The incoming event.
    @return     ok or failure or null.
 **/     
extern eOresult_t eo_sm_ProcessEvent(EOsm *p, eOsmEvent_t ev);


/** @fn         extern void eo_sm_Reset(EOsm *p)
    @brief      Resets the state machine.  It clears dynamic data of the state machine in the way defined by
                the configuation struct used by eo_sm_New(), then it clears every pending event,
                and finally calls eo_sm_Start().
    @param      p               The pointer to the state machine.
 **/ 
extern void eo_sm_Reset(EOsm *p);


/** @fn         extern void* eo_sm_hid_GetDynamicData(EOsm *p)
    @brief      Gets the user-defined dynamic data which is configured by means of a proper eOsm_cfg_t struct.
    @param      p               The pointer to the state machine.
    @return     A pointer to the dynamic data of the state machine (or NULL if use of dynamic data is not configured).
 **/ 
extern void* eo_sm_GetDynamicData(EOsm *p);


/** @fn         extern uint8_t eo_sm_GetActiveState(EOsm *p)
    @brief      Gets the index inside eOsm_cfg_T::states of the active state.
    @param      p               The pointer to the state machine.
    @return     The index to the active state.
 **/ 
extern uint8_t eo_sm_GetActiveState(EOsm *p);

/** @fn         extern eOsmEvent_t eo_sm_GetLatestEvent(EOsm *p)
    @brief      Gets the latest event received by the EOsm.
    @param      p               The pointer to the state machine.
    @return     The lastest event.
 **/ 
extern eOsmEvent_t eo_sm_GetLatestEvent(EOsm *p);



/** @}            
    end of group eo_sm  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




