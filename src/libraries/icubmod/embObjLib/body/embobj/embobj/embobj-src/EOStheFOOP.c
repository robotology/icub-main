
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"


#include "EOtheErrorManager.h"
#include "EOVtheSystem_hid.h" 
#include "EOSmutex.h"
#include "EOStheTimerManager_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheFOOP.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheFOOP_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define PROCESS_ONLY_FIRST_MESSAGE

#define EOS_FOOP_FLAG_SYSTICK_AVAIL  0x00000001
#define EOS_FOOP_FLAG_MESSAGE_AVAIL  0x00000002
#define EOS_FOOP_FLAG_CALLBACK_AVAIL 0x00000004


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOsfoop_cfg_t eos_foop_DefaultCfg = 
{
    .messagefifosize    = 0,
    .callbackfifosize   = 0, 
    .usrfn              =
    {
        .on_startup     = NULL, 
        .on_event       = NULL, 
        .on_message     = NULL
    }
};



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eos_foop_isr_set_evt(EOVtaskDerived *tsk, eOevent_t evt);
static eOresult_t s_eos_foop_tsk_set_evt(EOVtaskDerived *tsk, eOevent_t evt);
static eOresult_t s_eos_foop_isr_send_msg(EOVtaskDerived *tsk, eOmessage_t msg);
static eOresult_t s_eos_foop_tsk_send_msg(EOVtaskDerived *tsk, eOmessage_t msg, eOreltime_t tim);
static eOresult_t s_eos_foop_isr_exec_cbk(EOVtaskDerived *tsk, eOcallback_t cbk, void *arg);
static eOresult_t s_eos_foop_tsk_exec_cbk(EOVtaskDerived *tsk, eOcallback_t cbk, void *arg, eOreltime_t tim);
static eOid08_t s_eos_foop_get_id(EOVtaskDerived *tsk);

static eOresult_t s_eos_foop_start(void);


static void s_eos_foop_foreverloop(void *p, uint32_t u);
static void s_eos_foop_process_tick(void);
static void s_eos_foop_process_every_event(void);
static void s_eos_foop_process_message(void);
static void s_eos_foop_process_callback(void);

static void s_eos_foop_dummy_ontick(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static const char s_eobj_ownname[] = "EOStheFOOP";

static EOStheFOOP s_eos_the_foop = 
{
    .tsk            = NULL,
    .cfg            = {0},
    .hfn            = {0},
    .ontick         = NULL,
    .flags          = 0,
    .events_mask    = 0,
    .message_fifo   = NULL,
    .callback_fifo  = NULL,
    .argument_fifo  = NULL
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOStheFOOP * eos_foop_Initialise(eOsfoop_cfg_t *cfg, eObasicabstr_hal_sys_fn_t *hfn) 
{
    if(NULL != s_eos_the_foop.tsk) 
    {
        // already initialised
        return(&s_eos_the_foop);
    }
    
    
    if(NULL == cfg)
    {
        cfg = (eOsfoop_cfg_t*)&eos_foop_DefaultCfg;
    }
        
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != hfn), s_eobj_ownname, "eos_foop_Initialise(): invalid basic hal sys functions");
    
    s_eos_the_foop.tsk = eov_task_hid_New();    
    memcpy(&s_eos_the_foop.cfg, cfg, sizeof(eOsfoop_cfg_t));
    memcpy(&s_eos_the_foop.hfn, hfn, sizeof(eObasicabstr_hal_sys_fn_t));

    s_eos_the_foop.ontick       = s_eos_foop_dummy_ontick;
    s_eos_the_foop.flags        = 0;
    s_eos_the_foop.events_mask  = 0;
    
    if(0 != cfg->messagefifosize)
    {
        s_eos_the_foop.message_fifo = eo_fifoword_New(cfg->messagefifosize, 
                                                        eos_mutex_New(s_eos_the_foop.hfn.hal_sys_criticalsection_take,
                                                                      s_eos_the_foop.hfn.hal_sys_criticalsection_release)
                                                        );  
    }

    if(0 != cfg->callbackfifosize)
    {       
        s_eos_the_foop.callback_fifo = eo_fifo_New(sizeof(eOcallback_t), cfg->callbackfifosize, NULL, 0, NULL, NULL, 
                                                     eos_mutex_New(s_eos_the_foop.hfn.hal_sys_criticalsection_take,
                                                                   s_eos_the_foop.hfn.hal_sys_criticalsection_release)
                                                     );  
        s_eos_the_foop.argument_fifo = eo_fifo_New(sizeof(void*), cfg->callbackfifosize, NULL, 0, NULL, NULL, 
                                                     eos_mutex_New(s_eos_the_foop.hfn.hal_sys_criticalsection_take,
                                                                   s_eos_the_foop.hfn.hal_sys_criticalsection_release)
                                                     );  
    }   
   
   
    // init vtable of tsk
    eov_task_hid_SetVTABLE(s_eos_the_foop.tsk, 
                           cfg->usrfn.on_startup,  s_eos_foop_foreverloop,
                           s_eos_foop_isr_set_evt, s_eos_foop_tsk_set_evt,
                           s_eos_foop_isr_send_msg, s_eos_foop_tsk_send_msg,
                           s_eos_foop_isr_exec_cbk, s_eos_foop_tsk_exec_cbk,
                           s_eos_foop_get_id
                          ); 
                             
    return(&s_eos_the_foop);
   
}


extern EOStheFOOP* eos_foop_GetHandle(void)
{
    if(NULL == s_eos_the_foop.tsk)
    {
        return(NULL);
    }

    return(&s_eos_the_foop);
}    


extern void eos_foop_Start(EOStheFOOP *p)
{
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != p), s_eobj_ownname, "eos_foop_Start() uses a NULL handle");

    s_eos_foop_start();
} 


extern eOresult_t eos_foop_SetEvent(EOStheFOOP *p, eOevent_t evt)
{
    return(s_eos_foop_isr_set_evt(&s_eos_the_foop, evt));
} 


extern eOresult_t eos_foop_SendMessage(EOStheFOOP *p, eOmessage_t msg)
{
    return(s_eos_foop_isr_send_msg(&s_eos_the_foop, msg));
}


extern eOresult_t eos_foop_ExecCallback(EOStheFOOP *p, eOcallback_t cbk, void *arg)
{
    return(s_eos_foop_isr_exec_cbk(&s_eos_the_foop, cbk, arg));
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern eOresult_t eos_foop_hid_SetOnTick(EOStheFOOP *p, eOvoid_fp_void_t ontick)
{
    s_eos_the_foop.ontick = (NULL != ontick) ? (ontick) : (s_eos_foop_dummy_ontick);
    return(eores_OK);
}


extern eOresult_t eos_foop_hid_Tick(EOStheFOOP *p)
{
    s_eos_the_foop.hfn.hal_sys_atomic_bitwiseOR(&s_eos_the_foop.flags, EOS_FOOP_FLAG_SYSTICK_AVAIL);
    return(eores_OK);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eos_foop_isr_set_evt(EOVtaskDerived *tsk, eOevent_t evt)
{
    // set a bit in atomic way
    
    // the evt is a mask of a single bit or of some bits.
    // better to do the following operation atomic, in case the isr is interrupted
    // by another one. 

    if(0 == evt)
    {
        return(eores_NOK_generic);
    }

    s_eos_the_foop.hfn.hal_sys_atomic_bitwiseOR(&s_eos_the_foop.events_mask, evt);
        
    return(eores_OK);
}


static eOresult_t s_eos_foop_tsk_set_evt(EOVtaskDerived *tsk, eOevent_t evt)
{
    return(s_eos_foop_isr_set_evt(tsk, evt));
}


static eOresult_t s_eos_foop_isr_send_msg(EOVtaskDerived *tsk, eOmessage_t msg)
{
    // add the msg inside the fifo in atomic way
    
    if(NULL == s_eos_the_foop.message_fifo) 
    {
        return(eores_NOK_generic);
    }

    // ok, we send the message to the task by simply inserting it inside the fifo.
    eo_fifoword_Put(s_eos_the_foop.message_fifo, msg, eok_reltimeZERO);

    // and we set the first bit
    s_eos_the_foop.hfn.hal_sys_atomic_bitwiseOR(&s_eos_the_foop.flags, EOS_FOOP_FLAG_MESSAGE_AVAIL);
    
    return(eores_OK);
}


static eOresult_t s_eos_foop_tsk_send_msg(EOVtaskDerived *tsk, eOmessage_t msg, eOreltime_t tim)
{
    return(s_eos_foop_isr_send_msg(tsk, msg));
}


static eOresult_t s_eos_foop_isr_exec_cbk(EOVtaskDerived *tsk, eOcallback_t cbk, void *arg)
{
    // add the msg inside the fifo in atomic way
    
    if(NULL == s_eos_the_foop.callback_fifo) 
    {
        return(eores_NOK_generic);
    }

    if(NULL == cbk)
    {
        return(eores_NOK_generic);
    }

    // ok, we send the callback request to the task by simply inserting it inside the fifo.
    eo_fifo_Put(s_eos_the_foop.argument_fifo, &arg, eok_reltimeZERO);
    eo_fifo_Put(s_eos_the_foop.callback_fifo, &cbk, eok_reltimeZERO);

    // and we set the first bit
    s_eos_the_foop.hfn.hal_sys_atomic_bitwiseOR(&s_eos_the_foop.flags, EOS_FOOP_FLAG_CALLBACK_AVAIL);
    
    
    return(eores_OK);
}


static eOresult_t s_eos_foop_tsk_exec_cbk(EOVtaskDerived *tsk, eOcallback_t cbk, void *arg, eOreltime_t tim)
{
    return(s_eos_foop_isr_exec_cbk(tsk, cbk, arg));
}


static eOid08_t s_eos_foop_get_id(EOVtaskDerived *tsk)
{
    return(1);
}


static eOresult_t s_eos_foop_start(void)
{
    // exec the foop on-start-up
    eov_task_hid_StartUp(&s_eos_the_foop, 0);

    
    // exec the foop run
    eov_task_hid_Run(&s_eos_the_foop, 0);
 
    return(eores_OK);
}


static void s_eos_foop_foreverloop(void *p, uint32_t u)
{
    // EOStheFOOP *foop = (EOStheFOOP *)p;
    // u is 0

    volatile uint8_t tck_avail = 0;
    volatile uint8_t evt_avail = 0;
    volatile uint8_t msg_avail = 0;
    volatile uint8_t cbk_avail = 0;


    // this function gives priority to events. messages and callbacks are managed 
    // only after every event is processed. and also, messages and callbacks are 
    // managed one at a time.
    // if more fairness is required, then it could be that the message/callback 
    // queue is set empty completely when it is the time for processing events.
    // to do so, just undefine PROCESS_ONLY_FIRST_MESSAGE



    for(;;) 
    {  
    
        //--- keep on checking upon arrive of a new tick, event, message, callback
        while((0 == tck_avail) && (0 == evt_avail) && (0 == msg_avail) && (0 == cbk_avail)) 
        {

            if(EOS_FOOP_FLAG_SYSTICK_AVAIL == (s_eos_the_foop.flags & EOS_FOOP_FLAG_SYSTICK_AVAIL))
            {
                tck_avail = 1;
            }

            if(0 != s_eos_the_foop.events_mask) 
            {
                evt_avail = 1;
            }

            if(EOS_FOOP_FLAG_MESSAGE_AVAIL == (s_eos_the_foop.flags & EOS_FOOP_FLAG_MESSAGE_AVAIL))
            {
                msg_avail = 1;
            }

            if(EOS_FOOP_FLAG_CALLBACK_AVAIL == (s_eos_the_foop.flags & EOS_FOOP_FLAG_CALLBACK_AVAIL))
            {
                cbk_avail = 1;
            }

        }


        //--- verify again so that we can catch any *_avail until the last time

        if(EOS_FOOP_FLAG_SYSTICK_AVAIL == (s_eos_the_foop.flags & EOS_FOOP_FLAG_SYSTICK_AVAIL))
        {
            tck_avail = 1;
        }

        if(0 != s_eos_the_foop.events_mask) 
        {
            evt_avail = 1;
        }

        if(EOS_FOOP_FLAG_MESSAGE_AVAIL == (s_eos_the_foop.flags & EOS_FOOP_FLAG_MESSAGE_AVAIL))
        {
            msg_avail = 1;
        }

        if(EOS_FOOP_FLAG_CALLBACK_AVAIL == (s_eos_the_foop.flags & EOS_FOOP_FLAG_CALLBACK_AVAIL))
        {
            cbk_avail = 1;
        }


        //--- now we process the *_avail

        // if we have a tick we process it
        if(1 == tck_avail) 
        {
            tck_avail = 0;
            s_eos_foop_process_tick();
        }

        // if we have a new event we process all the events
        if(1 == evt_avail) 
        {
            evt_avail = 0;
            s_eos_foop_process_every_event();
        }
        
        // if we have at least a new message we process the message
        // if you want to process them all, undef PROCESS_ONLY_FIRST_MESSAGE
        if(1 == msg_avail) 
        {
            msg_avail = 0;
            s_eos_foop_process_message();
        }

        // if we have at least a new callback we process the callback
        // if you want to process them all, undef PROCESS_ONLY_FIRST_MESSAGE
        if(1 == cbk_avail) 
        {
            cbk_avail = 0;
            s_eos_foop_process_callback();
        }        

        //--- ok, i go back to wait for a new message / event
    }

}


static void s_eos_foop_process_tick(void)
{
    // reset it
    s_eos_the_foop.hfn.hal_sys_atomic_bitwiseAND(&s_eos_the_foop.flags, ~EOS_FOOP_FLAG_SYSTICK_AVAIL);
    // execute the ontick
    s_eos_the_foop.ontick();
}


// if hal_sys_atomic_bitwiseAND() IS atomic, then the loop inside s_eos_foop_process_every_event() is 
// safe because we remove only the events that we have just served. however, remember 
// that a new event of the same type can come back after one was already processed.
// it will be processed at next call of the function.


static void s_eos_foop_process_every_event(void) 
{

    uint32_t bit_equal_one   = 0;
    uint32_t bits_equal_zero = 0;
    uint8_t i = 0;
    eOvoid_fp_vuint32p_uint32_t bitwise_and = s_eos_the_foop.hfn.hal_sys_atomic_bitwiseAND;

    // lowest bits have highest priority
    for(i=0; i<31; i++) 
    {    
    
        bit_equal_one = (0x00000001) << i; 
        bits_equal_zero = ~bit_equal_one;
            
        // i work directly on the event mask, as it is volatile and as it can be
        // changed in any moment by an isr. by using it i dont lose any event. 

        if(0x00000000 != (s_eos_the_foop.events_mask & bit_equal_one)) 
        {
            // ok we have got one. 
                
            // erase the bit..... operation MUST be atomic.
            bitwise_and(&s_eos_the_foop.events_mask, bits_equal_zero);
        
            // call the user-defined function on received event
            if(NULL != s_eos_the_foop.cfg.usrfn.on_event) 
            {
                s_eos_the_foop.cfg.usrfn.on_event(i, s_eos_the_foop.events_mask);
            }
        
        }

        // we dont stay in here if we have served the events
        if(0 == s_eos_the_foop.events_mask)
        {
            return;
        }
    }    

}


static void s_eos_foop_process_message(void) 
{ 
    eOmessage_t message;
    eOsizecntnr_t size = 0;
    eOvoid_fp_vuint32p_uint32_t bitwise_and = s_eos_the_foop.hfn.hal_sys_atomic_bitwiseAND;
    eOvoid_fp_vuint32p_uint32_t bitwise_or = s_eos_the_foop.hfn.hal_sys_atomic_bitwiseOR;
    
    if(NULL == s_eos_the_foop.message_fifo)
    {   // dont have a message fifo ... why are we in here?
        eo_errman_Assert(eo_errman_GetHandle(), (NULL == s_eos_the_foop.message_fifo), s_eobj_ownname, "s_eos_foop_process_message() does not have a msg fifo"); 
        // we reset
        bitwise_and(&s_eos_the_foop.flags, ~EOS_FOOP_FLAG_MESSAGE_AVAIL); 
        return;
    }


#ifdef PROCESS_ONLY_FIRST_MESSAGE
    // retrieve only the first message from fifo
    if(eores_OK == eo_fifoword_Get(s_eos_the_foop.message_fifo, &message, eok_reltimeZERO))
#else 
    // retrieve messages until fifo is empty
    while(eores_OK == eo_fifoword_Get(s_eos_the_foop.message_fifo, &message, eok_reltimeZERO))
#endif

    {    
        // remove the message from queue
        eo_fifoword_Rem(s_eos_the_foop.message_fifo, eok_reltimeZERO);
    
        // execute the user-defined function on received message
        if(NULL != s_eos_the_foop.cfg.usrfn.on_message) 
        {
            s_eos_the_foop.cfg.usrfn.on_message(message);
        }
    
    }

    // important: acemor says: dont change the order of AND and OR otherwise there may be a bug
    // we reset
    bitwise_and(&s_eos_the_foop.flags, ~EOS_FOOP_FLAG_MESSAGE_AVAIL);  
    
    // set only if there is a msg in fifo  
    eo_fifoword_Size(s_eos_the_foop.message_fifo, &size, eok_reltimeZERO);
    if(0 != size)
    {
         bitwise_or(&s_eos_the_foop.flags, EOS_FOOP_FLAG_MESSAGE_AVAIL);
    }

} 


static void s_eos_foop_process_callback(void) 
{
    const void *pitem = NULL; 
    eOcallback_t callback = NULL;
    void *arg = NULL;
    eOsizecntnr_t size = 0;
    eOvoid_fp_vuint32p_uint32_t bitwise_and = s_eos_the_foop.hfn.hal_sys_atomic_bitwiseAND;
    eOvoid_fp_vuint32p_uint32_t bitwise_or = s_eos_the_foop.hfn.hal_sys_atomic_bitwiseOR;
    
    if(NULL == s_eos_the_foop.callback_fifo)
    {   // dont have a callback fifo ... why are we in here?
        eo_errman_Assert(eo_errman_GetHandle(), (NULL == s_eos_the_foop.callback_fifo), s_eobj_ownname, "s_eos_foop_process_callback() does not have a msg fifo");
        // we reset
        bitwise_and(&s_eos_the_foop.flags, ~EOS_FOOP_FLAG_CALLBACK_AVAIL);  
        return;
    }    


#ifdef PROCESS_ONLY_FIRST_MESSAGE
    // retrieve only the first callback from fifo
    if(eores_OK == eo_fifo_Get(s_eos_the_foop.callback_fifo, &pitem, eok_reltimeZERO))
#else 
    // retrieve callabcks until fifo is empty
    while(eores_OK == eo_fifo_Get(s_eos_the_foop.callback_fifo, &pitem, eok_reltimeZERO))
#endif

    {
        // in pitem i have the address of the copy inside the fifo of the item i want.
        // i copy it into callback.
        memcpy(&callback, pitem, sizeof(eOcallback_t)); 
           
        // remove the object from queue
        eo_fifo_Rem(s_eos_the_foop.callback_fifo, eok_reltimeZERO);


        eo_fifo_Get(s_eos_the_foop.argument_fifo, &pitem, eok_reltimeZERO);
        memcpy(&arg, pitem, sizeof(void*));
        eo_fifo_Rem(s_eos_the_foop.argument_fifo, eok_reltimeZERO);

    
        // execute directly the user-defined callback
        if(NULL != callback) 
        {
            callback(arg);
        }
    
    }

    // important: acemor says: dont change the order of AND and OR otherwise there may be a bug
    // we reset
    bitwise_and(&s_eos_the_foop.flags, ~EOS_FOOP_FLAG_CALLBACK_AVAIL);  
    
    // set only if there is a cbk in fifo  
    eo_fifo_Size(s_eos_the_foop.callback_fifo, &size, eok_reltimeZERO);
    if(0 != size)
    {
         bitwise_or(&s_eos_the_foop.flags, EOS_FOOP_FLAG_CALLBACK_AVAIL);
    }

}


static void s_eos_foop_dummy_ontick(void)
{

} 



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




