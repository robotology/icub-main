
/* @file       xboardemo-main.c
	@brief      This file implements a simple application able to run on multiple boards
	@author     marco.accame@iit.it
    @date       03/02/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

// abslayer 
#include "hal.h"
#include "osal.h"
#include "ipal.h"
#include "fsal.h"
#include "dspal.h"

// embobj  
#include "EoCommon.h"
#include "EOVtheSystem.h"
#include "EOMtheSystem.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrormanager.h"
#include "EOMtheIPnet.h"

#include "EOaction.h"
#include "EOpacket.h"
#include "EOMmutex.h"
#include "EOdatagramSocket.h"

#include "EOVtheGPIOCfg.h"
#include "EOtheGPIO.h"
#include "EOioPinOutputManaged.h"
#include "EOioPinOutput.h"
#include "EOMtheGPIOManager.h"

// embobj-cfg
#ifdef MCBSTM32C
#include "EOtheGPIOCfgMCBSTM32x.h"
#endif

#ifdef EMS001
#include "EOtheGPIOCfgEMS001.h"
#endif








// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
extern const hal_params_cfg_t *hal_params_cfgMINE;
extern const fsal_params_cfg_t *fsal_params_cfgMINE;
extern const osal_params_cfg_t *osal_params_cfgMINE;
 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


extern void task_udpserver(void *p);
extern void task_dspexec(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------




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

static void s_xboardemo_init(void);

static void s_xboardemo_gpio_init(void);

static void s_udpserver_startup(EOMtask *p, uint32_t t);
static void s_udpserver_run(EOMtask *p, uint32_t t);

static void s_dspexec_startup(EOMtask *p, uint32_t t);

static void s_dspexec_run(EOMtask *p, uint32_t t);





// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOdatagramSocket*        s_dgramskt          = NULL;
static EOpacket*                s_rxpkt             = NULL;
static EOpacket*                s_txpkt             = NULL;
static EOMtask*                 s_task_udpserver    = NULL;
static EOMtask*                 s_task_dspexec      = NULL;
static EOaction*                s_action            = NULL;


static EOioPinOutputManaged *s_iopinLED_one = NULL;
static EOioPinOutputManaged *s_iopinLED_two = NULL;
static EOioPinOutputManaged *s_iopinLED_three = NULL;
static EOioPinOutputManaged *s_iopinLED_four = NULL;


static const eOmessage_t s_message_from_skt = 0x00000001;

static const eOipv4port_t s_server_port = 3333; 


static dspal_matrix_f32_t matA;
static dspal_matrix_f32_t matB;
static dspal_matrix_f32_t matC;

static dspal_f32_t data_mata[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
static dspal_f32_t data_matb[3] = {1.0, 1.0, 1.0};
static dspal_f32_t data_matc[2] = {0.0, 0.0};


eOreltime_t time1 = 100*1000;
eOreltime_t time2 = 900*1000;






// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{

   eom_sys_Initialise(  hal_params_cfgMINE,
                        osal_params_cfgMINE,
                        fsal_params_cfgMINE,
                        NULL,                   // default mempool cfg
                        NULL,                   // default errman cfg
                        &eom_timerman_DefaultCfg,
                        &eom_callbackman_DefaultCfg
                      );  
    
    eom_sys_Start(eom_sys_GetHandle(), s_xboardemo_init);

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


void osal_on_idle(void)
{
    static uint32_t cnt = 0;

    for(;;)
    {
        cnt++;
    }
}


extern void eo_errman_OnError(eOerrmanErrorType_t errtype, uint32_t taskid, const char *eobjstr, const char *info)
{
    const char err[4][16] = {"info", "warning", "weak error", "fatal error"};

    printf("[eobj: %s, tsk: %d] %s: %s\n\r", eobjstr, taskid, err[(uint8_t)errtype], info);

    if(errtype <= eo_errortype_warning)
    {
        return;
    }

    for(;;);
}


extern void task_udpserver(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}  


extern void task_dspexec(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}  

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
               
static void s_xboardemo_init(void)
{
    extern const ipal_params_cfg_t *ipal_params_cfgMINE;

    // inside here we have LED_on, _two, and _three
    s_xboardemo_gpio_init();
                                                                                  
       
    // init the action used for various tasks
    s_action = eo_action_New();    

    // start the ipnet
    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
                         (ipal_params_cfg_t*)ipal_params_cfgMINE, 
                         NULL,
                         // &s_differentaddrcfg, 
                         &eom_ipnet_dtgskt_DefaultCfg,
                         0   // maxsynchrosocks
                         );

    printf("XBRDEMO: Initted the system service: scheduler, memory manager, error manager, timer manager, callback manager\n\r");
    printf("XBRDEMO: Initted the GPIO manager\n\r");
    printf("XBRDEMO: Initted the IPnet\n\r");


    s_task_udpserver = eom_task_New(eom_mtask_MessageDriven, 69, 3*1024, s_udpserver_startup, s_udpserver_run,  6, 
                                    eok_reltimeINFINITE, 
                                    task_udpserver, "udpserver");

    s_task_dspexec = eom_task_New(eom_mtask_MessageDriven, 60, 3*1024, s_dspexec_startup, s_dspexec_run,  6, 
                                    eok_reltimeINFINITE, 
                                    task_dspexec, "dspexec");



}


static void s_udpserver_startup(EOMtask *p, uint32_t t)
{
    // init the rx packet 
    s_rxpkt = eo_packet_New(32);  
    s_txpkt = eo_packet_New(32);

    // initialise the socket 
    s_dgramskt = eo_dtgsocket_New(  4, 32, eom_mutex_New(), // input queue
                                    4, 32, eom_mutex_New()  // output queue
                                 );
    
    
    // set the rx action on socket to be a message s_message_from_skt to this task object
    eo_action_SetMessage(s_action, s_message_from_skt, p);

    // open the socket on port s_server_port to be tx-rx and and execute s_action upon rx of a datagram
    eo_dtgsocket_Open(s_dgramskt, s_server_port, eo_sktdir_TXRX, eo_sktgetmode_bySignalling, s_action);  

    printf("XBRDEMO: task UDPserver has initted IPnet and a UDP socket on port %d\n\r", s_server_port);
    
}

static void s_udpserver_run(EOMtask *p, uint32_t t)
{
    // read the packet.
    static eOresult_t res;
    eOipv4addr_t remaddr;
    eOipv4port_t remport;
    uint8_t *ipaddr;
    uint8_t *data;
    uint16_t size;

    static eObool_t connected = eobool_false;

    // the message that we have received
    eOmessage_t msg = (eOmessage_t)t;


    if(s_message_from_skt == msg)
    {   // ok, message from the socket

        res = eo_dtgsocket_Get(s_dgramskt, 
                               s_rxpkt, 
                               eok_reltimeZERO //eok_reltimeINFINITE
                               );
    
        if(eores_OK == res)
        {
            // print stats of rx packet
            eo_packet_Destination_Get(s_rxpkt, &remaddr, &remport);
            eo_packet_Payload_Get(s_rxpkt, &data, &size);
            ipaddr = ((uint8_t*)&remaddr);
            printf("XBRDEMO: task UDPserver has received %d bytes from %d.%d.%d.%d-%d\n\r", size, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], remport);
            
            if(eobool_false == connected)
            {
                printf("XBRDEMO: socket not yet connected \n\r");
                res = eo_dtgsocket_Connect(s_dgramskt, remaddr, eok_reltime1sec);

                if(eores_OK == res)
                {
                    connected = eobool_true;
                    printf("XBRDEMO: socket connecetd now\n\r");
                }
                else
                {
                    printf("XBRDEMO: socket not connecetd after %d ms. i shall try again at next reception\n\r", eok_reltime1sec/1000);
                }
            }

            if(eobool_true == connected)
            {
                // prepare tx packet
   
                eo_packet_Full_Set(s_txpkt, remaddr, s_server_port, size, data);
                
                // and put it back into the socket
                eo_dtgsocket_Put(s_dgramskt, s_txpkt);
                
                printf("XBRDEMO: task UDPserver has transmitted a pkt back\n\r");

                eom_task_SendMessage(s_task_dspexec, (eOmessage_t)0x12345678,eok_reltimeINFINITE);
                printf("XBRDEMO: task UDPserver has sent a message to task DSP-EXEC\n\r");
            }
        }

    }

}


static void s_dspexec_startup(EOMtask *p, uint32_t t)
{
    // do nothing
    const dspal_params_cfg_t cfg = {0};
    uint64_t *data08aligned = NULL;
    uint32_t memsize = 0;

    dspal_memory_getsize(&cfg, &memsize);

    if(0 != memsize)
    {
        data08aligned = (uint64_t*) calloc(memsize/8, sizeof(uint64_t));   
    }
    
    dspal_initialise(&cfg, data08aligned);  
    
    
    dspal_matrix_init_f32(&matA, 2, 3, data_mata);
    dspal_matrix_init_f32(&matB, 3, 1, data_matb);
    dspal_matrix_init_f32(&matC, 2, 1, data_matc); 

    printf("XBRDEMO: task DSPexec has initted matrices A, B, C\n\r");
    printf("XBRDEMO: A[2x3] = [[1, 2, 3], [4, 5, 6]]\n\r");
    printf("XBRDEMO: B[3x1] = [[1, 1, 1]]\n\r");
    printf("XBRDEMO: C[2x1] = [[0],[0]]\n\r");

    eo_iopinoutman_Waveform_Start(s_iopinLED_one, hal_gpio_valHIGH, time1, time2, eok_reltimeINFINITE);
    printf("XBRDEMO: it also started a waveform of infinite duration on led_one: %d ms ON and %d ms OFF\n\r", time1/1000, time2/1000);

    eo_iopinoutman_SetVal(s_iopinLED_two, hal_gpio_valHIGH);

    printf("XBRDEMO: and put led_two to ON\n\r");

}

static void s_dspexec_run(EOMtask *p, uint32_t t)
{

    eOmessage_t msg = (eOmessage_t)t;
    dspal_result_t res;
    eOreltime_t tt;

    res = dspal_matrix_mult_f32(&matA, &matB, &matC);

    printf("XBRDEMO: task DSPexec has just multipled A * B = \n\r");
    printf("XBRDEMO: C[2x1] = [[%f],[%f]]\n\r", matC.pData[0], matC.pData[1]);

    eov_iopin_ToggleVal(s_iopinLED_two);

    printf("XBRDEMO: and toggled led_two. now it is %s \n\r", 
            (hal_gpio_valHIGH == eo_iopinoutman_GetVal(s_iopinLED_two)) ? "ON" : "OFF");

    eo_iopinoutman_Waveform_Stop(s_iopinLED_one);
    tt = time1; time1 = time2; time2 = tt;
    eo_iopinoutman_Waveform_Start(s_iopinLED_one, hal_gpio_valHIGH, time1, time2, eok_reltimeINFINITE);
    printf("XBRDEMO: DSPexec has changed the waveform on led_one: %d ms ON and %d ms OFF\n\r", time1/1000, time2/1000);


}


static void s_xboardemo_gpio_init(void)
{
#ifdef MCBSTM32C
    const EOVtheGPIOCfgDerived *gpiocfg = eo_gpiocfg_mcbstm32x_GetHandle();
#endif

#ifdef EMS001
    const EOVtheGPIOCfgDerived *gpiocfg = eo_gpiocfg_ems001_GetHandle();
#endif

    EOtheGPIO *thegpio = NULL;

    eov_gpiocfg_Initialise(gpiocfg);

    thegpio = eo_gpio_Initialise(eov_gpiocfg_GetHandle());

    s_iopinLED_one = eo_iopinoutman_GetHandle(xbrdwaveLED_ONE);
    s_iopinLED_two = eo_iopinoutman_GetHandle(xbrdwaveLED_TWO);
    s_iopinLED_three = eo_iopinoutman_GetHandle(xbrdwaveLED_THREE);


    eom_gpioman_Initialise(thegpio, &eom_gpioman_DefaultCfg);

}







// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



