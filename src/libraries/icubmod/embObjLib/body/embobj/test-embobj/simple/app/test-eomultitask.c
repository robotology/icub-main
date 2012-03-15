
/* @file       test-eomultitask.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#include "osal.h"
#include "hal.h"
#include "ipal.h"

// embobj
#include "EoCommon.h"
#include "EOaction.h"

#include "EOMmutex.h"
#include "EOVmutex.h"

#include "EOMtask.h"
#include "EOVtask.h"

#include "EOMtheSystem.h"
#include "EOVtheSystem.h"

#include "EOtimer.h"
#include "EOMtheTimerManager.h"
#include "EOMtheCallbackManager.h"
#include "EOMtheGPIOManager.h"
#include "EOMtheIPnet.h"
#include "EOsocketDatagram.h"



#include "EOtheGPIO.h"
#include "EOioPinOutput.h"
#include "EOtheGPIOManager.h"
#include "EOioPinOutputManaged.h"
#include "EOioPinInputManaged.h"


#include "test-eonetvar.h"

#include "test-eocontainers.h"

#include "eOcfg_GPIO_MCBSTM32c.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-eomultitask.h"

extern const osal_params_cfg_t *osal_params_cfgMINE;


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

// must be extern to be visible in uv4
extern void task_example00(void *p);
extern void task_example01(void *p);
extern void task_example02(void *p);
extern void task_example03(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


//static void s_test_EOmultitask_startothers(void);

//static void s_test_mutex_step_one(void);
//static void s_test_mutex_step_two(void);


static void s_mytper_startup(EOMtask *p, uint32_t);
static void s_mytper_run(EOMtask *p, uint32_t);

static void s_mytuser_startup(EOMtask *p, uint32_t t);
static void s_mytuser_run(EOMtask *p, uint32_t t);


static void s_mytmes_startup(EOMtask *p, uint32_t t);
static void s_mytmes_run(EOMtask *p, uint32_t t);

static void s_mytmesrx_startup(EOMtask *p, uint32_t t);
static void s_mytmesrx_run(EOMtask *p, uint32_t t);

//static void s_testeom_init(void);

//static void s_testeom_task_test_embobj(void *p);

//static void s_testeom_onbuttonpressedreleased(void);

static void s_testeom_callback(void *p);


static void s_test_init_gpio(void);

static void s_test_dosome_gpio(void);

static void s_callback(void *p);

static eObool_t s_connectedtohost = eobool_false;


static void s_onrxsynskt0(void *p);
static void s_onrxsynskt1(void *p);
static void s_ontxsynskt0(void *p);
static void s_ontxsynskt1(void *p);


     

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


                         //(eOmacaddr_t)0, 
                         //EO_COMMON_MACADDR(0xAA, 0x00, 0x00, 0x01, 0x02, 0x03),
                         //(eOipv4addr_t)0, 
                         //EO_COMMON_IPV4ADDR(255, 255, 252, 0), 
                         //(eOipv4addr_t)0, 

//static const eOmipnet_cfg_addr_t s_differentaddrcfg =
//{
//    EO_COMMON_MACADDR(0xAA, 0x00, 0x00, 0x01, 0x02, 0x03),
//    EO_COMMON_IPV4ADDR(10, 255, 39, 151),
//    EO_COMMON_IPV4ADDR(255, 255, 252, 0)
//};


static EOtimer *s_tmr0 = 0;
static EOtimer *s_tmr1 = 0;

static EOaction * s_action;
//static EOaction * act;

static EOaction * acttx0 = NULL;
static EOaction * acttx1 = NULL;
static EOaction * actrx0 = NULL;
static EOaction * actrx1 = NULL;

EOMmutex *mmutex = NULL;
EOMtask *mytask_periodic = NULL;
EOMtask *mytask_userdef  = NULL;
EOMtask *mytask_message = NULL;
EOMtask *mytask_messagerx = NULL;

eOnanotime_t nanotime;


//static const eOmempool_cfg_t s_mymempool_cfg =
//{
//    eo_mempool_alloc_dynamic,
//    0, NULL,
//    0, NULL,
//    0, NULL,
//    0, NULL
//};


EOsocketDatagram *dgramskt = NULL;
EOsocketDatagram *synchskt0 = NULL;
EOsocketDatagram *synchskt1 = NULL;
EOpacket *pkt = NULL;

EOpacket *rxpkt = NULL;



static EOtheGPIO *s_thegpio = NULL;

static EOioPinOutput *s_iopinLED10 = NULL;
static EOioPinOutputManaged *s_iopinwaveLED15 = NULL;
static EOioPinInputManaged *s_iopintrigBUTTONWKUP = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------





extern void test_EOmultitask_init00(void)
{
    extern const ipal_params_cfg_t *ipal_params_cfgMINE;



    //test_eocont_EOlist();

#ifdef _TEST_ROPS_
    test_eonetvar_Init();
#endif

    // start
    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
                         (ipal_params_cfg_t*)ipal_params_cfgMINE, 
                         NULL,
                         // &s_differentaddrcfg, 
                         &eom_ipnet_dtgskt_DefaultCfg
                         );

    dgramskt = eo_socketdtg_New(    4, 48, eom_mutex_New(), // input
                                    6, 32, eom_mutex_New()  // output
                               );


    synchskt0 = eo_socketdtg_New(2, 48, eom_mutex_New(), // input
                                 2, 48, eom_mutex_New()  // output
                                );
    synchskt1 = eo_socketdtg_New(2, 48, eom_mutex_New(), // input
                                 2, 48, eom_mutex_New()  // output
                                );

    pkt = eo_packet_New(48);



    mytask_periodic = eom_task_New(eom_mtask_Periodic, 66, 4*1024, s_mytper_startup, s_mytper_run, 0, 1*1000*1000, task_example00, "example00");

    mytask_userdef = eom_task_New(eom_mtask_UserDefined, 67, 2*1024, s_mytuser_startup, s_mytuser_run,  0, 0, task_example01, "example01");

    mytask_message = eom_task_New(eom_mtask_MessageDriven, 68, 3*1024, s_mytmes_startup, s_mytmes_run,  4, eok_reltimeINFINITE, task_example02, "example02");

    mytask_messagerx = eom_task_New(eom_mtask_MessageDriven, 69, 3*1024, s_mytmesrx_startup, s_mytmesrx_run,  6, 
    eok_reltimeINFINITE, 
    //0,
    task_example03, "example03");


    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);

    s_test_init_gpio();

    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);

    s_test_dosome_gpio();


    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);


    ///////////////////////////////////////////////////////////////////////////


    s_tmr0 = eo_timer_New();

    if(NULL == s_action)
    {
        s_action = eo_action_New();
    }

    eo_action_SetMessage(s_action, 0x12345678,  mytask_message);

 
    eo_timer_Start(s_tmr0, eok_abstimeNOW, 2*1000*1000, eo_tmrmode_FOREVER, s_action);
 

    s_tmr1 = eo_timer_New();


    eo_action_SetCallback(s_action, s_testeom_callback, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));

    eo_timer_Start(s_tmr1, eok_abstimeNOW, 3*1000*1000, eo_tmrmode_FOREVER, s_action);

}



extern void test_EOmultitask_start(void)
{
    // we have it in 
//    EOMtheSystem *thesys = eom_sys_Initialise(NULL, NULL, &s_mymempool_cfg, NULL, osal_params_cfgMINE, NULL, NULL);

//    eom_sys_Start(eom_sys_GetHandle(), s_testeom_init);

}


//static void s_test_EOmultitask_startothers(void)
//{
//    extern const ipal_params_cfg_t *ipal_params_cfgMINE; 
//    
//    s_test_mutex_step_one();
//    s_test_mutex_step_two();
//
//
//    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
//                         (ipal_params_cfg_t*)ipal_params_cfgMINE, 
//                         NULL,
//                         // &s_differentaddrcfg, 
//                         &eom_ipnet_dtgskt_DefaultCfg,
//                         2   // maxsynchrosocks
//                         );
//
//    dgramskt = eo_socketdtg_New(    4, 48, eom_mutex_New(), // input
//                                    6, 32, eom_mutex_New()  // output
//                                    );
//
//
//    synchskt0 = eo_socketdtg_New(32);
//    synchskt1 = eo_socketdtg_New(32);
//
//    pkt = eo_packet_New(48);
//
//
//
//    mytask_periodic = eom_task_New(eom_mtask_Periodic, 66, 4*1024, s_mytper_startup, s_mytper_run, 0, 1*1000*1000, task_example00, "example00");
//
//    mytask_userdef = eom_task_New(eom_mtask_UserDefined, 67, 2*1024, s_mytuser_startup, s_mytuser_run,  0, 0, task_example01, "example01");
//
//    mytask_message = eom_task_New(eom_mtask_MessageDriven, 68, 3*1024, s_mytmes_startup, s_mytmes_run,  4, eok_reltimeINFINITE, task_example02, "example02");
//
//    mytask_messagerx = eom_task_New(eom_mtask_MessageDriven, 69, 3*1024, s_mytmesrx_startup, s_mytmesrx_run,  6, 
//    eok_reltimeINFINITE, 
//    //0,
//    task_example03, "example03");
//
//
//    eom_callbackman_Initialise(NULL);
//    eom_timerman_Initialise(NULL);
//
//    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);
//
//    s_test_init_gpio();
//
//    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);
//
//    s_test_dosome_gpio();
//
//
//    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);
//
//
////    eom_ipnet_Initialise(209, 1024, 10*1000, 
////                         eobool_false,
////                         (ipal_params_cfg_t*)ipal_params_cfgMINE, 
////                         (eOmacaddr_t)0, 
////                         //EO_COMMON_MACADDR(0xAA, 0x00, 0x00, 0x01, 0x02, 0x03),
////                         (eOipv4addr_t)0, 
////                         //EO_COMMON_IPV4ADDR(255, 255, 252, 0), 
////                         (eOipv4addr_t)0, 
////                         2);
////
////    dgramskt = eo_socketdtg_New(    4, 48, eom_mutex_New(), // input
////                                    6, 32, eom_mutex_New()  // output
////                                    );
////
////
////    if(NULL == s_action)
////    {
////        act = eo_action_New();
////    }
////
////    eo_action_SetMessage(act, 0x00000001, mytask_messagerx);
////
////    eo_socketdtg_Open(dgramskt, 1001, eo_sktdir_TXRX, eo_sktgetmode_bySignalling, act);
////
////    pkt = eo_packet_New(48);
//
//
//
////    for(;;);
//
//}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


//static void s_test_mutex_step_one(void)
//{
//    static eOresult_t res = eores_OK;
//    res = res;
//
//    mmutex = eom_mutex_New();
//
//    res = eom_mutex_Take(mmutex, eok_reltimeINFINITE);
//
//    res = eom_mutex_Release(mmutex);
//    // shall succed when it is the same task which took it.
//
//    res = eom_mutex_Take(mmutex, eok_reltimeINFINITE);
//
//}


static void s_test_mutex_step_two(void)
{
    static eOresult_t res = eores_OK;


    res = eom_mutex_Take(mmutex, 1*1000*1000);
    res = res;
    // shall wait for 1 sec

    res = eom_mutex_Release(mmutex);
    // shall fail to release if another task took it.

    res = eov_mutex_Take(mmutex, 1*1000*1000);
    // shall still wait for 1 sec
 
}


static void s_mytper_startup(EOMtask *p, uint32_t t)
{
    void *tsk = NULL;
    static uint8_t taskid = 0;
    tsk = osal_system_task_get();
    taskid = *((uint8_t*)tsk);
    taskid = taskid;
    s_test_mutex_step_two();
}

static void s_mytper_run(EOMtask *p, uint32_t t)
{
    static uint32_t aaa = 0;

    if(100 == ++aaa)
    {
        aaa = 0;
    }
}


extern void task_example00(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}


static void s_mytuser_startup(EOMtask *p, uint32_t t)
{
    void *tsk = NULL;
    static uint8_t taskid = 0;
    tsk = osal_system_task_get();
    taskid = *((uint8_t*)tsk);
    taskid = taskid;
    s_test_mutex_step_two();
}

static void s_mytuser_run(EOMtask *p, uint32_t t)
{
    static uint32_t aaa = 0;

    for(;;)
    {
        osal_system_task_wait(200*1000);
        if(100 == ++aaa)
        {
            aaa = 0;
        }
    }
}


extern void task_example01(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}

static void s_mytmes_startup(EOMtask *p, uint32_t t)
{
    eOresult_t res;
    eOipv4addr_t ipaddr = IPAL_ipv4addr(10, 255, 37, 204);
    
    res = eo_socketdtg_Connect(dgramskt, ipaddr, 50*1000*1000);

    if(eores_OK == res)
    {
        printf("connection result: OKKEI\n\r");
        s_connectedtohost = eobool_true;
    } 
    else
    {
        printf("connection result: FAILED\n\r"); 
        s_connectedtohost = eobool_false;
    }
       
}

static void s_mytmes_run(EOMtask *p, uint32_t t)
{
    static uint32_t aaa = 0;
    static uint8_t n = 0;
//    static eOresult_t res = eores_NOK_generic;
    //uint8_t ipaddr[4] = {10, 255, 39, 152};
    eOipv4addr_t ipaddr = IPAL_ipv4addr(10, 255, 37, 204);
    uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    static  eOipv4addr_t mmmm = IPAL_ipv4addr(239, 0, 0, 0);
    static eOresult_t rex = eores_NOK_generic;
    rex = rex;
    uint8_t *ipaddrarray = NULL;

    ipaddrarray = (uint8_t*)(&ipaddr);



    rex = eom_ipnet_IGMPgroupJoin(eom_ipnet_GetHandle(), mmmm);


    n++;

    printf("message from EOtimer #%d\n\r", n);

    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nanotime);


    if(n >= 2)
    {

 
        printf("stopped EOtimer\n\r");
        eo_timer_Stop(s_tmr0);
        n = 0;

        printf("but starting it again\n\r");

        if(NULL == s_action)
        {
            s_action = eo_action_New();
        }

        eo_action_SetMessage(s_action, 0x12345678,  mytask_message);

        eo_timer_Start(s_tmr0, eok_abstimeNOW, 1*1000*1000, eo_tmrmode_FOREVER, s_action);

        if(eobool_true == s_connectedtohost)
        {
            printf("and transmitted packet #%d to %d.%d,%d.%d, port %d\n\r", ++aaa, ipaddrarray[0], ipaddrarray[1], ipaddrarray[2], ipaddrarray[3], 1001);
            eo_packet_Destination_Set(pkt, ipaddr, 1001);
            data[0] = aaa & 0xFF;
            data[1] = (aaa>>8) & 0xFF;
            eo_packet_Payload_Set(pkt, data, sizeof(data));
            eo_socketdtg_Put(dgramskt, pkt);
            //eo_socketdtg_Put(dgramskt, pkt);

            eo_socketdtg_Close(dgramskt); 
            eo_action_SetMessage(s_action, 0x00000001, mytask_messagerx);
            eo_socketdtg_Open(dgramskt, 1001, eo_sktdir_TXRX, eobool_false, NULL, s_action, NULL); 

//            eo_socketdtg_Close(synchskt);
//            eo_action_SetMessage(s_action, 0x00000002, mytask_messagerx);
//            eo_socketdtg_Open(synchskt, 1002, 0, 500*1000, s_action, s_action, s_action);  
            eo_packet_Destination_Set(pkt, ipaddr, 1002);
            eo_socketdtg_Put(synchskt0, pkt);
            eo_packet_Destination_Set(pkt, ipaddr, 1003);
            eo_socketdtg_Put(synchskt1, pkt);

  
            //rex = eo_socketdtg_Connect(dgramskt, *((uint32_t*)ipaddr), 500*1000);
        }

    }

 

}


static void s_mytmesrx_startup(EOMtask *p, uint32_t t)
{
    // init the packet 

    eOsktdtgTXmode_t txm;
    
    rxpkt = eo_packet_New(32);  
     
    if(NULL == s_action)
    {
        s_action = eo_action_New();
    }

    eo_action_SetMessage(s_action, 0x00000001, p);

    eo_socketdtg_Open(dgramskt, 1001, eo_sktdir_TXRX, eobool_false, NULL, s_action, NULL); 
    
    
    eo_action_SetMessage(s_action, 0x00000002, p); 

    acttx0 = eo_action_New();
    acttx1 = eo_action_New();
    actrx0 = eo_action_New();
    actrx1 = eo_action_New();

    eo_action_SetCallback(actrx0, s_onrxsynskt0, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    eo_action_SetCallback(actrx1, s_onrxsynskt1, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));

    eo_action_SetCallback(acttx0, s_ontxsynskt0, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    eo_action_SetCallback(acttx1, s_ontxsynskt1, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    
    txm.startat = 5000; txm.after = 500*1000; txm.periodic = eobool_true;
    eo_socketdtg_Open(synchskt0, 1002, eo_sktdir_TXRX, eobool_false, &txm, 
                      actrx0, acttx0); 

    txm.startat = 10000; txm.after = 500*1000; txm.periodic = eobool_true;
    eo_socketdtg_Open(synchskt1, 1003, eo_sktdir_TXRX, eobool_false, &txm, 
                      actrx1, acttx1); 
      
}

static void s_mytmesrx_run(EOMtask *p, uint32_t t)
{
    // read the packet.
    static eOresult_t res;
    eOipv4addr_t remaddr;
    eOipv4port_t remport;
    uint8_t *ipaddrarray;
    uint8_t *data;
    uint16_t size;

    eOmessage_t msg = (eOmessage_t)t;


    if(0x00000001 == msg)
    {

        res = eo_socketdtg_Get(dgramskt, rxpkt, 
                                //1000
                                eok_reltimeZERO
                                //eok_reltimeINFINITE
                                );
    
        if(eores_OK == res)
        {
            eo_packet_Destination_Get(rxpkt, &remaddr, &remport);
            eo_packet_Payload_Get(rxpkt, &data, &size);
            ipaddrarray = ((uint8_t*)&remaddr);
            printf("received %d bytes from %d.%d.%d.%d-%d\n\r", size, ipaddrarray[0], ipaddrarray[1], ipaddrarray[2], ipaddrarray[3], remport);
            printf("first 4 bytes are: %x %x %x %x\n\r", data[0], data[1], data[2], data[3]);
        }

    }

    //print info on screen
}


extern void task_example02(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}



extern void task_example03(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}

//static void s_testeom_init(void)
//{
//    const osal_time_t period  = 500*1000; // 500 msec 
//
//    // create test embobj task, low priority and periodic
//    osal_task_new(s_testeom_task_test_embobj, (void*)period, 10, 4*1024);
//}

//static void s_testeom_task_test_embobj(void *p)
//{
//    osal_time_t per = (uint32_t)p;
//    hal_gpio_val_t curval = hal_gpio_valHIGH;
//    hal_gpio_val_t currinp = hal_gpio_valNONE;
//    hal_gpio_val_t previnp = hal_gpio_valNONE;
//
//
//    s_test_EOmultitask_startothers();
//
//    s_tmr0 = eo_timer_New();
//
//    if(NULL == s_action)
//    {
//        s_action = eo_action_New();
//    }
//
//    eo_action_SetMessage(s_action, 0x12345678,  mytask_message);
//
// 
//    eo_timer_Start(s_tmr0, eok_abstimeNOW, 2*1000*1000, eo_tmrmode_FOREVER, s_action);
//    //eo_timer_Start(s_tmr0, 0, 2*1000*1000, eo_tmrmode_FOREVER, s_action);
//    //eo_timer_Start(s_tmr0, 0, 1000, eo_tmrmode_ONESHOT, s_action);
//
//    s_tmr1 = eo_timer_New();
//
//
//    eo_action_SetCallback(s_action, s_testeom_callback, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
//
//    eo_timer_Start(s_tmr1, eok_abstimeNOW, 3*1000*1000, eo_tmrmode_FOREVER, s_action);
//
//    osal_system_task_period_set(per);
//
//    for(;;)
//    {
//        osal_system_task_period_wait();
//
//       
//        // do action .... toggle a led
//        curval = (hal_gpio_valHIGH == curval) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);
//        hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8, curval);
//
//        // and get value of input button
//        currinp = hal_gpio_getval(hal_gpio_portB, hal_gpio_pin7);
//
//        if(previnp == hal_gpio_valNONE)
//        {
//            previnp = currinp;
//        }
// 
//        if(currinp != previnp)
//        {
//            s_testeom_onbuttonpressedreleased();
//        }
//
//        previnp = currinp;
//        currinp = hal_gpio_valNONE;
//
//    }
//}


//static void s_testeom_onbuttonpressedreleased(void)
//{
//    hal_gpio_val_t curval = hal_gpio_getval(hal_gpio_portE, hal_gpio_pin9);
//    curval = (hal_gpio_valHIGH == curval) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);
//    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin9, curval);
//}


static void s_testeom_callback(void *p)
{
    static uint32_t nn = 0;
    hal_gpio_val_t curval = hal_gpio_getval(hal_gpio_portE, hal_gpio_pin14);
    curval = (hal_gpio_valHIGH == curval) ? (hal_gpio_valLOW) : (hal_gpio_valHIGH);
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin14, curval);

    printf("callback called for the %d-th time\n\r", nn++);
}


static void s_test_init_gpio(void)
{
    hal_gpio_val_t val = hal_gpio_valNONE;
   
    val = val;

 
    s_thegpio = eo_gpio_Initialise(eo_cfg_gpio_mcbstm32c_Get());
    s_thegpio = s_thegpio;

    eom_gpioman_Initialise(s_thegpio, &eom_gpioman_DefaultCfg);
    
}

static void s_test_dosome_gpio(void)
{
    EOaction * acton;
    eOiopinVal_t val;
    s_iopinLED10 = eo_iopinout_GetHandle(iopinID_Out_mcbstm32c_LED_010);
    
    // ok, operate manually with pinout functions
    eo_iopinout_SetVal(s_iopinLED10, eo_iopinvalHIGH);
    eo_iopinout_ToggleVal(s_iopinLED10);
    // but also can operate with the base iopin functions.
    eo_iopin_derived_ToggleVal(s_iopinLED10);
    val = eo_iopin_derived_GetVal(s_iopinLED10);
    val = val;

    // we initialise a waveform on led 15
    s_iopinwaveLED15 = eo_iopinoutman_GetHandle(iopinID_Oman_mcbstm32c_LED_015);
    eo_iopinoutman_Waveform_Start(s_iopinwaveLED15, eo_iopinvalHIGH, 1*1000*1000, 4*1000*1000, eok_reltimeINFINITE);

    eo_iopin_derived_ToggleVal(s_iopinwaveLED15);

    eo_iopin_derived_ToggleVal(s_iopinwaveLED15);

    // and also a callback on button wakeup
    s_iopintrigBUTTONWKUP = eo_iopininpman_GetHandle(iopinID_Iman_mcbstm32c_BUTTON_WKUP);
    acton = eo_action_New();
    eo_action_SetCallback(acton, s_callback, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle())); 
    
    eo_iopininpman_ActionOn_Register(s_iopintrigBUTTONWKUP, acton, eo_iopinTrig_OnRiseStay, 3*1000*1000);
}

static void s_callback(void *p)
{
    static volatile uint32_t aa = 0;

    if(10 < ++aa)
    {
        aa = 0;
    }
    printf("gpio callback on WKUP\n");
}



static void s_onrxsynskt0(void *p)
{  
    static EOpacket *pkt = NULL;
    uint8_t *data = NULL;
    uint16_t size;

    if(NULL == pkt)
    {
        pkt = eo_packet_New(32);
    }

    eo_socketdtg_Get(synchskt0, pkt, eok_reltimeZERO);

    eo_packet_Payload_Get(pkt, &data, &size);

    printf("rx from synsock 0: size is %d and firste byte is %d\n", size, data[0]);
}

static void s_onrxsynskt1(void *p)
{
    static EOpacket *pkt = NULL;
    uint8_t *data = NULL;
    uint16_t size;

    if(NULL == pkt)
    {
        pkt = eo_packet_New(32);
    }

    eo_socketdtg_Get(synchskt1, pkt, eok_reltimeZERO);

    eo_packet_Payload_Get(pkt, &data, &size);

    printf("rx from synsock 1: size is %d and firste byte is %d\n", size, data[0]);
}

static void s_ontxsynskt0(void *p)
{
    printf("tx by synsock 0\n");
}

static void s_ontxsynskt1(void *p)
{
    printf("tx by synsock 1\n");
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



