// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------
/* @file       CANmng.c
	@brief      This file implements functionto manage can peripherals.
	@author     valentina.gaggero@iit.it
    @date       12/12/2010
**/



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "string.h"
#include "hal.h"

#include "eEcommon.h"
#include "EOMtheEntitiesEnv.h"
#include "EOtheErrorManager.h"
#include "EOtimer.h"
#include "EOMdataContainer.h"

#include "appl_common.h"

#include "EOaction.h"
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "CANmng_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define EVT_CANMNG_STOP         EVT_TASK_STOP
#define EVT_CANMNG_START        EVT_TASK_START
#define EVT_CANMNG_REC_CANMSG   (1 << 2)



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------
extern EOMdataContainer               *commonData;


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_canmng_callbkp_onrec(void *arg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
#ifdef _DEBUG_PRINT_ 
static char my_deb_string[50];
#endif
//questi due frame sono definiti solo per test
static hal_can_frame_t frame_start = 
    {
        .id = 0,
        .id_type = hal_can_frameID_std,
        .frame_type = hal_can_frame_data,
        .size = 5,
        .unused = 0,
        .data = {'S', 'T', 'A', 'R', 'T'}
    };

static hal_can_frame_t frame_stop = 
    {
        .id = 0,
        .id_type = hal_can_frameID_std,
        .frame_type = hal_can_frame_data,
        .size = 4,
        .unused = 0,
        .data = {'S', 'T', 'O', 'P'}
    };



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern void CANmng_startup(hal_can_port_t port, EOMtask *p, uint32_t *idcan)
{
    eOresult_t res;
    hal_result_t hal_res;
    EOMtheEntitiesEnv_task_info_t my_task_info;
    
    const hal_can_cfg_t can_cfg =  
    {
        .runmode = hal_can_runmode_isr_1txq1rxq,
        .baudrate = hal_can_baudrate_1mbps, 
        .priorx = hal_int_priority04, //TODO: messa a caso!!!!
        .priotx = hal_int_priority04, //TODO: messa a caso!!!!
        .callback_on_rx = s_canmng_callbkp_onrec, 
        .arg = p
    }; 



    //int CAN peripherals
    hal_res = hal_can_init(port, &can_cfg);
    if(hal_res_OK != hal_res)
    {
       eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, 
                       ((port == hal_can_port1) ? "can1mng" : "can2mng"),
                       "error in init CAN");
    }


    hal_res = hal_can_enable(port);
    if(hal_res_OK != hal_res)
    {
       eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, 
                       ((port == hal_can_port1) ? "can1mng" : "can2mng"),
                       "error in enable CAN");
    }



    #warning: per ora non leggo dalle shared lib ind CAN
    //get can address (called also id)
#ifndef _WITHOUT_SHALIB_
    res = shalinfo_deviceinfo_get(&deviceinfo_ptr);

    if(ee_res_OK != res)
    {
        ;//uso id di default????
    }
    *idcan = ((port == hal_can_port1) ? deviceinfo_ptr->can1network : deviceinfo_ptr->can2network);

#else
    *idcan = ((port == hal_can_port1) ? 1 : 2);
#endif




    //Regitser me to EOMtheEntitiesEnv
    my_task_info.task_ptr = p;
    my_task_info.type = eom_mtask_EventDriven;

    res = EOMtheEntitiesEnv_register_task(EOMtheEntitiesEnv_GetHandle(), &my_task_info);

    if(eores_OK != res)
    {
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, 
                       ((port == hal_can_port1) ? "can1mng" : "can2mng"),
                       "I can NOT register");
    }

}


extern void CANmng_task(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}



extern void CANmng_run(hal_can_port_t port, EOMtask *tsk, eOevent_t evt, uint32_t idcan)
{
//    hal_can_frame_t frame_rec;
//    uint8_t remaining_recframe;
//    hal_result_t hal_res;
//    eOresult_t res;
//
// 
//
//#ifdef _DEBUG_PRINT_ 
//    sprintf(my_deb_string, "CANmng: Evt received!");
//    hal_trace_puts(my_deb_string);  
//#endif
//
////all actions on all evt are for test!!!
//
//    frame_stop.id = idcan;
//    frame_start.id = idcan;
//
//
//    if(EVT_CHECK(evt, EVT_CANMNG_STOP)) 
//    {
//        hal_can_put(port, &frame_stop, hal_can_send_normprio_now);
//        res = eo_timer_Stop(s_can1mng_timer);
//        /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
//        APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, "CANMng", "I can't stop timer"); 
//
//    }
//
//
//    if(EVT_CHECK(evt, EVT_CANMNG_START))
//    {
//        hal_can_put(port, &frame_start, hal_can_send_normprio_now);
//        res = eo_timer_Start(s_can1mng_timer, eok_abstimeNOW, 1000, /*eo_tmrmode_ONESHOT*/ eo_tmrmode_FOREVER, s_CANmng_action_onTimeout);
//        /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
//        APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, "CANMng", "I can't start timer"); 
//
//    }
//    
//
//    if(EVT_CHECK(evt, EVT_CANMNG_REC_CANMSG))
//    {
//        hal_res = hal_can_get(port, &frame_rec, &remaining_recframe);
//        frame_rec.id = idcan;
//        hal_res = hal_can_put(port, &frame_rec, hal_can_send_normprio_now);
//        if(hal_res_OK != hal_res)
//        {
//            ;//error
//        }
//    }
//
//


}

/*
Questa funzione e' usata solo per test. Essa dovrebbe parserizzare i dati arrivati da ethernet e iviarli
col formato opportuno alle schede su can.
*/
extern void CANmng_parserAndSend_TEST(hal_can_port_t port, uint8_t *buff, uint32_t idcan)
{
    hal_can_frame_t frameTX = 
    {
        .id = idcan,
        .id_type = hal_can_frameID_std,
        .frame_type = hal_can_frame_data,
        .unused = 0,
        .size = 8
    };

    memcpy(&frameTX.data, buff, 8); 
    hal_can_put(port, &frameTX, hal_can_send_normprio_now);
}
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
static void s_canmng_callbkp_onrec(void *arg)
{
    eom_task_isrSetEvent((EOMtask*)arg, EVT_CANMNG_REC_CANMSG); 
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



