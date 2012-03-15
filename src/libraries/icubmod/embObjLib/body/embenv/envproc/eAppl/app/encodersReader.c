// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------
// empty section


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "string.h"

//abslayer
#include "hal.h"

//embobj
#include "EOMtheEntitiesEnv.h"
#include "EOtheErrorManager.h"
#include "EOtimer.h"
#include "EOMdatacontainer.h"

//appl
#include "appl_common.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "encodersReader.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
//#include "entity_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define EVT_ENCREADER_STOP                 EVT_TASK_STOP
#define EVT_ENCREADER_START                EVT_TASK_START
#define EVT_ENCREADER_STARTREAD               (1 << 2)
#define EVT_ENCREADER_NEXTREAD               (1 << 3)
#define EVT_ENCREADER_FINISHREAD               (1 << 4)

#define ENCODER_NULL                        255

#define CHECK_ENC_IS_CONNECTED(enc)   ((1<<enc) == ((1<<enc)&(deviceCfg_connectedEncodersMask)))



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
static void s_encReader_timeout_callbkp(void *arg);


static void s_encoder_callbkp(void *arg);
static void s_encoder_callbkpLast(void *arg);

static void s_readDeviceEncConfg(hal_encoder_t startEnc, hal_encoder_t endEnc, uint8_t *encList, hal_encoder_t *firstEnc);
static void s_configureEncoders(hal_encoder_t startEnc, hal_encoder_t endEnc, uint8_t *encList, hal_encoder_t firstEnc,EOMtask *p);
// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static char s_task_name[] = "encReader";

static EOtimer                  *s_encReader_timer_startRead;
static EOaction                 *s_encReader_action_onTimeout;
static uint8_t                  s_encReader_firstCfgEnc;

static uint32_t                 result[6] = {ENCODER_NULL, ENCODER_NULL, ENCODER_NULL, ENCODER_NULL, ENCODER_NULL, ENCODER_NULL};
static uint8_t                  s_configuredEncSPI1_list[3]= {ENCODER_NULL, ENCODER_NULL, ENCODER_NULL};
static uint8_t                  s_configuredEncSPI3_list[3]= {ENCODER_NULL, ENCODER_NULL, ENCODER_NULL};
static hal_encoder_t            s_firstEncSPI1 = ENCODER_NULL;
static hal_encoder_t            s_firstEncSPI3 = ENCODER_NULL;
static uint8_t                  s_numRead = 0;

//thi is a maskbits tat indicats which encoders are connected to ems board. Each ems has its device config 
//connected encoders mask
static uint16_t deviceCfg_connectedEncodersMask;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern void encodersReader_startup(EOMtask *p, uint32_t t)
{
    EOMtheEntitiesEnv_task_info_t my_task_info;
    eOresult_t res;


#ifndef _WITHOUT_SHALIB_
    res = shalinfo_deviceinfo_get(&deviceinfo_ptr);

    if(ee_res_OK != res)
    {
        ;//configuro tutti/nessun encoder???
    }
    deviceCfg_connectedEncodersMask = deviceinfo_ptr->onnectedEncodersMask;
#else
    deviceCfg_connectedEncodersMask = ((1 <<hal_encoder1) /*|| (1<<hal_encoder2)*/ | (1 <<hal_encoder3) | /*(1 <<hal_encoder7)|*/ (1 <<hal_encoder8) /*| (1 <<hal_encoder9)*/);
#endif

    s_readDeviceEncConfg(hal_encoder1, hal_encoder3, s_configuredEncSPI1_list, &s_firstEncSPI1);
    s_readDeviceEncConfg(hal_encoder7, hal_encoder9, s_configuredEncSPI3_list, &s_firstEncSPI3);
    s_configureEncoders(hal_encoder1, hal_encoder3, s_configuredEncSPI1_list, s_firstEncSPI1, p);
    s_configureEncoders(hal_encoder7, hal_encoder9, s_configuredEncSPI3_list, s_firstEncSPI3, p);

    //if at least 1 encoder is connected
    if( (ENCODER_NULL !=s_firstEncSPI1) ||  (ENCODER_NULL !=s_firstEncSPI3)) 
    {
        s_encReader_timer_startRead = eo_timer_New();
        s_encReader_action_onTimeout = eo_action_New();
        eo_action_SetEvent(s_encReader_action_onTimeout, (eOevent_t)EVT_ENCREADER_STARTREAD , p);
    }


    my_task_info.task_ptr = p;
    my_task_info.type = eom_mtask_EventDriven;

    
    res = EOMtheEntitiesEnv_register_task(EOMtheEntitiesEnv_GetHandle(), &my_task_info);
    if(eores_OK != res)
    {
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal , s_task_name, "I can NOT register");
    }

}



extern void encodersReader_run(EOMtask *tsk, uint32_t evtmsgper)
{
    eOresult_t res;
    eOevent_t evt = (eOevent_t)evtmsgper;
    uint8_t i;


    if(EVT_CHECK(evt, EVT_ENCREADER_STOP))
    {
       res = eo_timer_Stop(s_encReader_timer_startRead);
       /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
       APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, s_task_name, "I can't stop timer"); 
    }

    if(EVT_CHECK(evt, EVT_ENCREADER_START))
    {
       res = eo_timer_Start(s_encReader_timer_startRead, eok_abstimeNOW, 1000, /*eo_tmrmode_ONESHOT*/eo_tmrmode_FOREVER, s_encReader_action_onTimeout);
       /* Note: TODO if I can't stop the timer, is it batter i send me the same event..to tray another time??*/  
       APPL_CHECKandPRINT_ERROR(res, eo_errortype_warning, s_task_name, "I can't start timer"); 
    }
    

    if(EVT_CHECK(evt, EVT_ENCREADER_STARTREAD))
    {
       s_encReader_timeout_callbkp(NULL); 
    }

    if(EVT_CHECK(evt, EVT_ENCREADER_FINISHREAD))
    {
        s_numRead++;

        if(s_numRead == 2)
        {
            s_numRead = 0;
            for(i=0; i<3; i++)
            {
                hal_encoder_get_value((hal_encoder_t)i, &result[i]);
            }
        
            for(i=6; i<9; i++)
            {
                hal_encoder_get_value((hal_encoder_t)i, &result[i-3]);
            }

   //         eom_dataContainer_Read(commonData, result);
           eom_dataContainer_Write(commonData, (void*)result);
        }


    }


}

extern void  encodersReader_task(void *p)
{
    eom_task_Start(p);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
static void s_encReader_timeout_callbkp(void *arg)
{
    if(ENCODER_NULL != s_firstEncSPI1)
    {   
        hal_encoder_read_start(s_firstEncSPI1);  
    }

    if(ENCODER_NULL != s_firstEncSPI3)
    {   
        hal_encoder_read_start(s_firstEncSPI3);  
    }
}


static void s_encoder_callbkp(void *arg)
{
    hal_encoder_read_start(*((hal_encoder_t*)(arg)));
}

static void s_encoder_callbkpLast(void *arg)
{
   eom_task_isrSetEvent((EOMtask*)arg, EVT_ENCREADER_FINISHREAD); 
}




static void s_readDeviceEncConfg(hal_encoder_t startEnc, hal_encoder_t endEnc, uint8_t *encList, hal_encoder_t *firstEnc)
{
    uint8_t i = 0, j = 0;

    for(i=startEnc; i<=endEnc; i++)
    {
        if(CHECK_ENC_IS_CONNECTED(i))
        //if((deviceCfg & (1<<i)) == (1<<i))
        {
            if(ENCODER_NULL == *firstEnc)
            {
                *firstEnc = i;    
            }
            for(j =i+1; j<=endEnc; j++)
            {
                if(CHECK_ENC_IS_CONNECTED(j))
                //if((deviceCfg & (1<<j)) == (1<<j))        
                {
                    encList[(i-startEnc)] = j ;
                    break;    
                }
            }
        
        }
    }

}

static void s_configureEncoders(hal_encoder_t startEnc, hal_encoder_t endEnc, uint8_t *encList, hal_encoder_t firstEnc,EOMtask *p)
{
    uint8_t i, j;
    hal_encoder_cfg_t enc_cfg;

    enc_cfg.priority = hal_int_priority06;
    #warning VALE: --> metti le prio in un file di config a compiletime!!!


    for(i=startEnc; i<=endEnc; i++)
    {
        if(CHECK_ENC_IS_CONNECTED(i))
        //if((deviceCfg & (1<<i)) == (1<<i))
        {
            if(ENCODER_NULL != encList[(i-startEnc)])
            {
                enc_cfg.callback_on_rx = s_encoder_callbkp;
                enc_cfg.arg = &(encList[(i-startEnc)]);
                hal_encoder_init(i,  &enc_cfg);
            }
            else
            {
                //is the last
                enc_cfg.callback_on_rx = s_encoder_callbkpLast;
                enc_cfg.arg = p;
                hal_encoder_init(i, &enc_cfg);

            }
        }
    }

}
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



