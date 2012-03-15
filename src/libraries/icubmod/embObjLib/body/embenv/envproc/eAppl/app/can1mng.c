// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------
/* @file       can1mng.c
	@brief      This file implements the CAN1 manager task.
	@author     valentina.gaggero@iit.it
    @date       12/13/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"

//embObj
#include "eEcommon.h"
#include "EOtheErrorManager.h"
#include "EOMprodConsSharedData.h"
#include "EOMtask.h"
#include "EOtimer.h"
#include "EOMdataContainer.h"

// application
#include "appl_common.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "can1mng.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "CANmng_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define EVT_CANMNG_REC_ETHMSG   (1 << 3)
#define EVT_CANMNG_TIMEOUT   (1 << 4)
#define EVT_CANMNG_STOP         EVT_TASK_STOP
#define EVT_CANMNG_START        EVT_TASK_START
#define EVT_CANMNG_REC_CANMSG   (1 << 2)




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------
extern EOMprodConsSharedData          *ethCan1_shData;
extern EOMdataContainer               *commonData;


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_can1mng_callbkp_newDataFromEth(void *arg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static const char s_task_name[] = "can1task";

static uint8_t                  my_memorybuff[ETHCAN1_SHDATA_ITEMSIZE];
static uint32_t                 my_idcan;
static EOtimer                  *s_can1mng_timer;
static EOaction                 *s_can1mng_action_onTimeout;
static uint32_t                 result[6]= {1,1,1,1,1,1};

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
extern void can1mng_startup(EOMtask *p, uint32_t t)
{

    EOMprodConsSharedData_ConsumerCallbackSet(ethCan1_shData, s_can1mng_callbkp_newDataFromEth, (void*)p);
	
    s_can1mng_timer = eo_timer_New();
    s_can1mng_action_onTimeout = eo_action_New();
    eo_action_SetEvent(s_can1mng_action_onTimeout, (eOevent_t)EVT_CANMNG_TIMEOUT , p);



    CANmng_startup(hal_can_port1, p, &my_idcan);
 
}


extern inline void can1mng_task(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    CANmng_task(p);
}

extern void can1mng_run(EOMtask *tsk, uint32_t evtmsgper)
{
    eOresult_t res;
    eOevent_t evt = (eOevent_t)evtmsgper;
        hal_result_t hal_res;
        hal_can_frame_t frame_rec;
        uint8_t remaining_recframe;

    hal_can_frame_t frame_periodic = 
    {
        .id = 0,
        .id_type = hal_can_frameID_std,
        .frame_type = hal_can_frame_data,
        .size = 8,
        .unused = 0,
        .data = {2, 2, 2, 2, 2, 2, 2, 2}
    };



    if(EVT_CHECK(evt, EVT_CANMNG_STOP)) 
    {
        hal_can_put(hal_can_port1, &frame_stop, hal_can_send_normprio_now);
        res = eo_timer_Stop(s_can1mng_timer);
        /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
        APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, "CANMng", "I can't stop timer"); 

    }


    if(EVT_CHECK(evt, EVT_CANMNG_START))
    {
        hal_can_put(hal_can_port1, &frame_start, hal_can_send_normprio_now);
        res = eo_timer_Start(s_can1mng_timer, eok_abstimeNOW, 1000, /*eo_tmrmode_ONESHOT*/ eo_tmrmode_FOREVER, s_can1mng_action_onTimeout);
        /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
        APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, "CANMng", "I can't start timer"); 

    }
    

    if(EVT_CHECK(evt, EVT_CANMNG_REC_CANMSG))
    {
        hal_res = hal_can_get(hal_can_port1, &frame_rec, &remaining_recframe);
        frame_rec.id = 1;
        hal_res = hal_can_put(hal_can_port1, &frame_rec, hal_can_send_normprio_now);
        if(hal_res_OK != hal_res)
        {
            ;//error
        }
    }

    if(EVT_CHECK(evt, EVT_CANMNG_REC_ETHMSG))
    {
        res = EOMprodConsSharedData_Get(ethCan1_shData, my_memorybuff, 100); //TODO: stai attenta!!!! non devi bloccare troppo il task qui se no poi non riceve piu' msg!!!!!!
        if(eores_OK != res )
        {
            if(eores_NOK_timeout == res)
            {
                //try later....
                eom_task_SetEvent(tsk, EVT_CANMNG_REC_ETHMSG);     
            }
            APPL_ERRMAN_ERROR(eo_errortype_warning, s_task_name, "error in gettig data from ethshData")
        }
        CANmng_parserAndSend_TEST(hal_can_port1, my_memorybuff, my_idcan);

    }

    if(EVT_CHECK(evt, EVT_CANMNG_TIMEOUT))
    {

        eom_dataContainer_Read(commonData, result);
        memcpy(frame_periodic.data, result, 8);
        result[0]=11;
  
        frame_periodic.id = 1;
        hal_res = hal_can_put(hal_can_port1, &frame_periodic, hal_can_send_normprio_now);
        if(hal_res_OK != hal_res)
        {
            hal_res = hal_res;//error
        }
    }
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
static void s_can1mng_callbkp_newDataFromEth(void *arg)
{
    eom_task_SetEvent(arg, EVT_CANMNG_REC_ETHMSG ); 
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



