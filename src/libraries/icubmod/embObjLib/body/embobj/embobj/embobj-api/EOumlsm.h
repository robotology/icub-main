
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOUMLSM_H_
#define _EOUMLSM_H_


/** @file       EOumlsm.h
	@brief      This header file implements public interface to a uml state machine object.
	@author     marco.accame@iit.it
	@date       09/02/2011
**/

/** @defgroup eo_umlsm Object EOumlsm
    The EOumlsm allows to use a state machine defined according to UML 2.2 standard,
    i.e., with states and sub-states with on entry and on exit actions, with transitions
    conditioned by guards, with actions on transition, with default entry states.
    See http://www.omg.org/spec/UML/2.2/Superstructure/ for more details.
    In addition to UML 2.2 specifications, the EOumlsm also contains an internal 
    fifo queue of events that can be filled with events coming from inner actions (on-entry,
    on-exit, on-transition). The state machine executes first the events contained in such
    a fifo queue. By means of this mechanism, it is possible to trigger multiple state migrations 
    using a single external event.

    @warning    The EOumlsm must be used by a single task because it does not have protection
                versus concurrency.
    
    @{		
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------

#define EOUMLSM_STATENAMESIZE 8
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef struct EOumlsm_hid EOumlsm
 	@brief 		EOumlsm is an opaque struct. It is used to implement data abstraction for the state machine 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOumlsm_hid EOumlsm;


/** @typedef    typedef void (*eOumlsm_void_fp_umlsmp_t)(EOumlsm *)
    @brief      Function type used for callbacks inside the EOumlsm object
 **/ 
typedef void (*eOumlsm_void_fp_umlsmp_t)(EOumlsm *);


/** @typedef    typedef eObool_t (*eOumlm_bool_fp_umlsmp_t)(EOumlsm *)
    @brief      Function type used for guards inside the EOumlsm object
 **/ 
typedef eObool_t (*eOumlsm_bool_fp_umlsmp_t)(EOumlsm *);

/** @typedef    typedef enum eOumlsmConsumeMode_t
    @brief      eOumlsmConsumeMode_t contains the consume modes with which eo_umlsm_ProcessEvent can be
                called.
 **/
typedef enum  
{
    eo_umlsm_consume_ONE      = 1,    /**< The SM consumes only one event, be it external or internal */
    eo_umlsm_consume_UPTO08   = 8     /**< The SM consumes up to eight events, be them external or internal */
} eOumlsmConsumeMode_t;


/** @typedef    typedef uint8_t eOumlsmEvent_t
    @brief      eOumlsmEvent_t is the basic type for the events that can be fed into the SM.
 **/
typedef uint8_t eOumlsmEvent_t;


/** @typedef    typedef enum eOumlsmBasicEvents_t 
    @brief      eOumlsmBasicEvents_t contains the basic events that are common to every SM.
 **/
typedef enum  
{
    eo_umlsm_evNONE           = (eOumlsmEvent_t) EOK_uint08dummy     /**< the null event */
} eOumlsmBasicEvents_t;


/** @typedef    typedef enum eOumlsmConstants_t
    @brief      eOumlsmConstants_t contains some constants used by the EOumlsm object.
 **/
typedef enum 
{
    /** Maximum level of ownerhip for a state. It can be set to any value in [1, 254]. Example: if 
        equal to 3 state1 can be contained recursively by state2 and state3, but not by state4.         **/
    eov_umlsm_OWNERS_maxnumber      = 3  
} eOumlsmConstants_t;


/** @typedef    typedef struct eOumlsmTransition_t
    @brief      Transition of a EOumlsm.
 **/ 
typedef const struct  
{
    eOumlsmEvent_t            trigger;                  /**< Event which triggers the transition. Use a proper enum. */               
    uint8_t                   next;                     /**< Index of eOumlsm_cfg_t.states_table which gives the next state of this transition.  */
    eOumlsm_bool_fp_umlsmp_t  guard_fn;                 /**< Guard condition function which must return eobool_true in order to fire the transition. */
    eOumlsm_void_fp_umlsmp_t  on_transition_fn;         /**< Function to execute on transition. Set to NULL, no action is performed.  **/ 
} eOumlsmTransition_t;


/** @typedef    typedef struct eOumlsm_state_t
    @brief      State of a EOumlsm.
 **/ 
typedef const struct 
{
    char                        name[EOUMLSM_STATENAMESIZE];/**< Name of the state  */
    uint8_t                     initial_substate;           /**< Initial substate of the current state as index eOumlsm_cfg_t::states_table. Set to EOK_uint08dummy if there is none */ 
    uint8_t                     owners_number;              /**< Number of elements in the owners_table. Set to 1 if there are no states upon this one.  */
    const uint8_t*              owners_table;               /**< Owners of current state, expressed as indices to eOumlsm_cfg_t.states_table. In pos 0-th there is the state itself, in pos 1-st its dad, etc   */       
    uint8_t                     transitions_number;         /**< Number or transitions in current state */
    eOumlsmTransition_t*        transitions_table;          /**< Table of transitions in current state, organised as an array of transitions_number elements     */
    eOumlsm_void_fp_umlsmp_t    on_entry_fn;                /**< Entry function for this state. Set to NULL if no action is required.   */
    eOumlsm_void_fp_umlsmp_t    on_exit_fn;                 /**< Exit function for this state. Set to NULL if no action is required.           */
} eOumlsmState_t;


/** @typedef    typedef struct eOumlsm_cfg_t
    @brief      The configuration of a EOumlsm.
 **/ 
typedef const struct 
{
    uint8_t                     sizeofdynamicdata;          /**<  Total size of dynamic data expressed in bytes  */
    uint8_t                     initial_state;              /**< initial state expressed as an index of states_table. */
    uint8_t                     internal_event_fifo_size;   /**< Size of the fifo of internal events. Set to 0 if you dont use the queue. */    
    uint8_t                     states_number;              /**< Number of states contained in states_table   */
    eOumlsmState_t*             states_table;               /**< Table of states of the state machine   **/
    eOumlsm_void_fp_umlsmp_t    resetdynamicdata_fn;        /**< Reset function for dynamic data   */
} eOumlsm_cfg_t;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern EOumlsm* eo_umlsm_New(const eOumlsm_cfg_t *cfg) 
    @brief      Creates a new EOumlsm object using a pointer to an user-defined eOumlsm_cfg_t struct which
                contains ROM and RAM configuration for the state machine.
                Multiple _New(cfg) calls with the same cfg generates multiple independent instances of state machines
                which share the same behaviour but have separate data structures.
    @param      cfg             The configuration of the state machine.
    @return     Pointer to the required EOumlsm object. The pointer is guaranteed to be always valid and will 
                never be NULL, because failure is managed by the memory pool.
 **/
extern EOumlsm * eo_umlsm_New(const eOumlsm_cfg_t * cfg);


/** @fn         extern void eo_umlsm_Start(EOumlsm *p)
    @brief      Performs the first steps of the state machine entering in the initial state. The user should call 
                this function only once before processing any event. If the user does not call it directly, however, 
                this function will be called inside the first call of eo_umlsm_ProcessEvent(). 
    @param      ptr             The pointer to the state machine.
 **/     
extern void eo_umlsm_Start(EOumlsm *p);


/** @fn         extern uint8_t eo_umlsm_ProcessEvent(EOumlsm *p, eOumlsmEvent_t event, 
                                                         eOumlsmConsumeMode_t consume)
    @brief      Processes one incoming event and depending on the @e consume mode either terminates or recursively 
                process the internal events triggered by the incoming @e event.
    @param      ptr             The pointer to the state machine.
    @param      event           The incoming event.
    @param      consume         The consume mode. See eOumlsmConsumeMode_t for details.
    @return     The number of transitions triggered by the event. =0 if event was not associated to any transition
                or if it was associated but the guard did not allow firing. =1 if one transition was triggered, >1 
                if there was one triggered transition plus other internal events which trigegred other transitions.
 **/     
extern uint8_t eo_umlsm_ProcessEvent(EOumlsm *p, eOumlsmEvent_t event, eOumlsmConsumeMode_t consume);


/** @fn         extern void eo_umlsm_Reset(EOumlsm *p)
    @brief      Resets the state machine.  It clears dynamic data of the state machine in the way defined by
                the EOVumlsmCfgDerived object used by eo_umlsm_New(), then it clears every pending event,
                and finally calls eo_umlsm_Start().
    @param      p               The pointer to the state machine.
 **/ 
extern void eo_umlsm_Reset(EOumlsm *p);


/** @fn         extern eOumlsmEvent_t eo_umlsm_GetInternalEvent(EOumlsm *p)
    @brief      Retrieves the next valid internal event inside the internal queue or eo_umlsm_evNONE if no valid event 
                is present. Each call of the function will consume one internal event. This function is to be used 
                together with eo_umlsm_ProcessEvent(ptr, ev, eo_umlsm_consume_ONE) in a do{} while() loop, where ev
                is the value returned by this function.
    @param      p               The pointer to the state machine.
    @return     The event in the internal queue or eo_umlsm_evNONE if queue is empty or not defined.
 **/ 
extern eOumlsmEvent_t eo_umlsm_GetInternalEvent(EOumlsm *p);



/** @fn         extern eOresult_t eo_umlsm_PutInternalEvent(EOumlsm *p, eOumlsmEvent_t event)
    @brief      Puts an event inside the internal FIFO queue. This event will have precedence upon
                any event passed with eo_umlsm_ProcessEvent().

    @param      p               The pointer to the state machine.
    @param      event           The event. It must be different from eo_umlsm_evNONE.
    @return     .....
 **/ 
extern eOresult_t eo_umlsm_PutInternalEvent(EOumlsm *p, eOumlsmEvent_t event);

/** @fn         extern void* eo_umlsm_GetDynamicData(EOumlsm *p)
    @brief      Gets the dynamic data which is internally used by the state machine. 
    @param      p               The pointer to the state machine.
    @return     A pointer to the dynamic data of the state machine (or NULL if dynamic data is not used).
 **/ 
extern void* eo_umlsm_GetDynamicData(EOumlsm *p);





                                       
/** @}            
    end of group eo_umlsm  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




