// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHEENTITIESENV_H_
#define _EOMTHEENTITIESENV_H_


// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       object.h
    @brief      This header file implements public interface to an object SW entity.
    @author     marco.accame@iit.it
    @date       11/12/2009
**/

/** @defgroup object A service SW entity: object
    The SW entity service is an example of an Object. It offers a filtering block, whose output depends on its state.
    
    
    @todo put documentation proper to the entity.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------
//#include "stdint.h"     // contains uint32_t etc.
#include "EOMtask.h"
#include "EOtimer.h"
#include "EOaction.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



typedef struct
{
    EOMtask         *task_ptr;
    eOmtaskType_t   type;
} EOMtheEntitiesEnv_task_info_t;

typedef struct
{
    EOtimer  *timer_ptr;
    eOabstime_t startat; //Probabilmente per l'uso che ne facciamo noi nonserve, ma lo metto per completezza
    eOreltime_t countdown; 
    eOtimerMode_t mode;
    EOaction *action;
} EOMtheEntitiesEnv_eotimer_info_t;


typedef struct
{
    uint8_t dummy; //TODO
} EOMtheEntitiesEnv_haltimer_info_t;


typedef struct
{
    void *entity;
    eOres_fp_voidp_t funcStartEntity; //void (*funcStartEntity)(void*);
    void *argStart;
    eOres_fp_voidp_t funcStopEntity; //void (*funcStopEntity)(void*);
    void *argStop;    
} EOMtheEntitiesEnv_module_info_t;

/** @typedef    typedef struct EOMtheEntitiesEnv_hid Object
    @brief      Object is an opaque struct. It is used to implement data abstraction for the generic
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMtheEntitiesEnv_hid                    EOMtheEntitiesEnv;



/** @typedef    typedef struct EOMtheEntitiesEnv_signal_cfg_t
    @brief      This struct contains all "signal" to send to entities to start or stop.
                Some entities, like timers, do not need a particular signal, but it is sufficient 
                invoke their apropriate operations to start and stop them. 
 **/  
typedef struct
{
    struct
    {
        eOmessage_t     startMsg;      /**< msg to send to all entity-tasks message based to start (running state) */
        eOmessage_t     stopMsg;       /**< msg to send to all entity-tasks message based to stop (idle state) */
        eOevent_t       startEvt;      /**< evt to send to all entity-tasks event based to start (running state) */
        eOevent_t       stopEvt;       /**< evt to send to all entity-tasks event based to stop (idle state) */
    }task;

} EOMtheEntitiesEnv_signals_cfg_t;



typedef enum
{
    entity_task = 0,
    entity_eotimer = 1,
    entity_haltimer = 2,
    entity_module = 3
} EOMtheEntitiesEnv_type_t;

typedef struct
{
    EOMtask         *task;
    void (*callback_allEntities_registered)(void);
    void (*callback_etity_regitered)(EOMtheEntitiesEnv_type_t type, void*arg);
} EOMtheEntitiesEnv_systemControllercfg_t;
 
typedef struct
{
    uint8_t                                 task_num;
    uint8_t                                 eotimer_num;
    uint8_t                                 haltimer_num;
    uint8_t                                 module_num;
    EOMtheEntitiesEnv_signals_cfg_t         signals;
    EOMtheEntitiesEnv_systemControllercfg_t syscntl_cfg;
} EOMtheEntitiesEnv_cfg_t;




    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         
    @brief      
    @return 
    NOTE MIE: LA DESCRIZIONE DEVE ESSERE DIVERSA DA NULL (se NULL chiama error manager)
     MENTRE IL MANAGER PUO' ESSERE NULL PERCHE' CONFIG DOPO.    
 **/
extern eOresult_t EOMtheEntitiesEnv_Initialise(EOMtheEntitiesEnv_cfg_t *env_cfg, EOMtask *sysController_task);

 
/** @fn         extern obResult_t obj_Filter(Object *p, float x, float *y)
    @brief      Executes a filtering on @e x and retrieves result @e y. 
    @param      p               Pointer to the object.
    @param      x               The input signal to be filtered. It will not be changed by this function.
    @param      y               The filtered output.
    @return     The success or failure of the filtering: ob_Result_OK if OK, ob_Result_NOK_GENERIC or 
                ob_Result_NOK_NULLPOINTER elsewhere.
 **/
extern eOresult_t EOMtheEntitiesEnv_register_task(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_task_info_t *taskentity_info);

/** @fn         extern obResult_t obj_Filter(Object *p, float x, float *y)
    @brief      Executes a filtering on @e x and retrieves result @e y. 
    @param      p               Pointer to the object.
    @param      x               The input signal to be filtered. It will not be changed by this function.
    @param      y               The filtered output.
    @return     The success or failure of the filtering: ob_Result_OK if OK, ob_Result_NOK_GENERIC or 
                ob_Result_NOK_NULLPOINTER elsewhere.
 **/
extern eOresult_t EOMtheEntitiesEnv_register_eotimer(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_eotimer_info_t *eotimerentity_info);



extern eOresult_t EOMtheEntitiesEnv_register_module(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_module_info_t *moduleentity_info);

/** @fn         
    @brief      
    @param      
    @return     
 **/
extern eOresult_t EOMtheEntitiesEnv_register_sysController(EOMtheEntitiesEnv *handler, EOMtheEntitiesEnv_systemControllercfg_t *sysCntl_cfg);



extern EOMtheEntitiesEnv * EOMtheEntitiesEnv_GetHandle(void);

extern eOresult_t EOMtheEntitiesEnv_Deinitialise(void);

extern eOresult_t EOMtheEntitiesEnv_start_all(EOMtheEntitiesEnv *handler);

extern eOresult_t EOMtheEntitiesEnv_stop_all(EOMtheEntitiesEnv *handler);

// - doxy end ---------------------------------------------------------------------------------------------------------

/** @}            
    end of group object  
 **/

 
#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


