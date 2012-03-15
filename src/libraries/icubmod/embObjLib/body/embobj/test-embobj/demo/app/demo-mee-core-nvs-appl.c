
/* @file       demo-mee-core-gpio-sm-appl.c
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

#include "hal.h"
#include "osal.h"
#include "fsal.h"


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
#include "EOVtheCallbackManager.h"


#include "demo-info.h"

#include "EOtheARMenvironment.h"
#include "EOVtheEnvironment.h"

#include "EOnvsCfg.h"

#include "EOmatrix3d.h"



//#include "EOtheNVs.h"
//#include "eOcfg_NVs_updater.h"
//#include "eOcfg_nvsEP_base_con.h"

//#include "eOcfg_nvsEP_base_usr_loc_anydev.h"
//#include "eOcfg_nvsEP_mngmnt_usr_loc_board.h"
//
//#include "eOcfg_nvsEP_base_usr_rem_anydev.h"
//#include "eOcfg_nvsEP_mngmnt_usr_rem_board.h"


//
//
//#include "eOcfg_nvsEP_mngmnt_con.h"



#include "EOrop.h"
#include "EOtheAgent.h"
#include "EOtheFormer.h"
#include "EOtheParser.h"

#include "EOropframe.h"

#include "EOtransmitter.h"
#include "EOreceiver.h"

#include "EOtransceiver.h"


// all that is enough for the local board
#include "eOcfg_EPs_loc_board.h"
#include "EOtheBOARDtransceiver.h"

// all that is enough for the remote host to see the board
#include "eOcfg_EPs_rem_board.h"
#include "EOhostTransceiver.h"


//#include "eOcfg_nvsEP_base_usr_rem_anydev.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "demo-mee-core-nvs-appl.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

// must be extern to be visible in uv4
extern void task_example01(void *p);


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


extern EOtransceiver* theems00transceiver = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

extern void demo_nvs_init(void);
extern void demo_nvs_tick(void);

extern void demo_nvs_signal(uint16_t port, uint16_t nvid);



static void s_mytmes_startup(EOMtask *p, uint32_t t);
static void s_mytmes_run(EOMtask *p, uint32_t t);

//static void s_testeom_callback_button_wkup(void *arg);

static void s_testeom_callback_timer(void *arg);





//static void s_test_nvs00(void);
//static void s_test_nvs01(void);
//static void s_test_nvs_ropframe(void);
//static void s_test_nvs_transmitter(void);
//static void s_test_nvs_receiver(void);

static void s_test_nvs_transceiver_init(void);
static void s_test_nvs_transceiver_tick(void);

//static void s_nvs_common_init(void);
//static void s_nvs_pc104_init(void);
//static void s_nvs_ems00_init(void);


//static void s_nvs_pc104_asks(EOrop **nvs_rop2ems00, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr);

//static eObool_t s_nvs_ems00_receive(uint8_t *rxpktdata, uint16_t rxpktsize, uint32_t fromipaddr, 
//                                    EOrop **nvs_rop2pc104, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr);
//
//static eObool_t s_nvs_pc104_receive(uint8_t *rxpktdata, uint16_t rxpktsize, uint32_t fromipaddr, 
//                                    EOrop **nvs_rop2ems00, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr);
//
//
//static void s_nvs_ems00_emits(EOrop **nvs_rop2pc104, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr);

//static void s_nvs_pc104_set(EOrop **nvs_rop2ems00, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr);

//static EOnvsCfg* s_test_nvs_nvscfg_pc104_get(void);
//static EOnvsCfg* s_test_nvs_nvscfg_ems00_get(void);
//static void s_test_nvs_transceiver_pc104_cfg_set(eo_transceiver_cfg_t *cfg);
//static void s_test_nvs_transceiver_ems00_cfg_set(eo_transceiver_cfg_t *cfg);

//static void s_test_nvs_transceiver_ems00_regulars_config(EOtransceiver *txrx);
static void s_test_nvs_transceiver_ems00_occasional_load(EOtransceiver *txrx, uint16_t nvid);
static void s_test_nvs_transceiver_pc104_occasional_load(EOtransceiver *txrx, eOropcode_t opc, uint16_t ep, uint16_t nvid);
static void s_test_nvs_transceiver_pc104_configure_ems(EOtransceiver *txrx);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static EOaction * s_action;
EOMtask *mytask_message = NULL;



// --- nvs ----------------------------------

//static EOnvsCfg *nvs_ems00_nvscfg;
//static EOnvsCfg *nvs_pc104_nvscfg;

static const uint32_t nvs_ems00_ipaddress = 0x01020304;
static const uint32_t nvs_pc104_ipaddress = 0x12345678;

static const uint16_t nvs_base_endpoint_iport = 33333;

//static EOtheFormer* nvs_theformer = NULL;
//static EOtheParser* nvs_theparser = NULL; 
//static EOtheAgent*  nvs_theagent = NULL;

//#define DATACAPACITY 256
//typedef struct
//{
//    uint16_t    size;
//    uint8_t     data[DATACAPACITY];
//} nvs_PKT_t;
//
//static nvs_PKT_t nvs_pkt;


EOrop *nvs_rop2ems00 = NULL;
EOrop *nvs_rop2pc104 = NULL;

EOrop *nvs_rop_rec = NULL;
EOrop *nvs_rop_out = NULL;
EOrop *nvs_rop_tmp = NULL;

EOnv *nvs_nv = NULL;

EOropframe *nvs_pc104_ropframe = NULL;
EOropframe *nvs_ems00_ropframe = NULL;
EOropframe *nvs_recvd_ropframe = NULL;
uint8_t nvs_pc104_ropframe_payload[512];
uint8_t nvs_ems00_ropframe_payload[512];
uint8_t nvs_recvd_ropframe_payload[512];
uint16_t nvs_pc104_ropframe_payloadsize = 0;
uint16_t nvs_ems00_ropframe_payloadsize = 0;

EOpacket *nvs_rx_pkt = NULL;


static EOtransceiver *pc104txrx = NULL;
static EOtransceiver *ems00txrx = NULL;
static EOpacket *transpacket = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------





extern void demo_mee_core_nvs_appl_init00(void)
{

    s_action = eo_action_New(); 

    hal_eeprom_init(hal_eeprom_i2c_01, NULL);
    eo_armenv_Initialise(&demoinfo_modinfo, &demoinfo_boardinfo);
    eov_env_SharedData_Synchronise(eo_armenv_GetHandle());


    hal_trace_puts("start a single user-task: msg-based\n\r");
 
    mytask_message = eom_task_New(eom_mtask_MessageDriven, 68, 4*1024, s_mytmes_startup, s_mytmes_run,  4, eok_reltimeINFINITE, NULL, task_example01, "example01 msg");

    ///////////////////////////////////////////////////////////////////////////


    
 
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

//static EOtheFormer* s_theformer = NULL;
//static EOtheParser* s_theparser = NULL; 
//static EOtheAgent*  s_theagent = NULL;
//static EOrop* s_out_rop = NULL;
//
//static EOrop* s_tx_rop = NULL;
//static EOrop* s_rx_rop = NULL;
//
//static uint8_t s_pktdata[256] = {0};
//static uint16_t s_pktsize = 0;
//static eOipv4addr_t s_ipaddr = 0;


extern void demo_nvs_init(void)
{

    
#if 0    
    {
        typedef struct
        {
            uint8_t     num;
            uint8_t     aaa;
            uint16_t    bbb;
            uint32_t    ccc;
            uint64_t    item;
        } sss_t;
        uint8_t i, j, k;
        
        uint64_t item = 0;
        sss_t *psss;
        char str[128];
        sss_t sss;
        sss.num = 0;
        sss.aaa = 0xaa;
        sss.bbb = 0xbbbb;
        sss.ccc = 0xcccccccc;
        sss.item = 0x1122334455667788;
        EOmatrix3d *m3d;
        
        m3d = eo_matrix3d_New(sizeof(sss_t), 2);

        for(i=0; i<2; i++)
        {
            eo_matrix3d_Level1_PushBack(m3d, 3);
            for(j=0; j<3; j++)
            {
                eo_matrix3d_Level2_PushBack(m3d, i, 4);
                for(k=0; k<4; k++)
                {
                    eo_matrix3d_Level3_PushBack(m3d, i, j, &sss);
                    sss.num++;
                }
            }
        }

        for(i=0; i<2; i++)
        {
            for(j=0; j<3; j++)
            {
                for(k=0; k<4; k++)
                {
                    psss = (sss_t*) eo_matrix3d_At(m3d, i, j, k);
                    snprintf(str, sizeof(str)-1, "md3[%d, %d, %d] = %d is at address %x", i, j, k, (uint32_t) (psss->num), psss);
                    hal_trace_puts(str);
                }
            }
        }

    }
#endif


    s_test_nvs_transceiver_init();



}



extern void demo_nvs_tick(void)
{

    //s_test_nvs01();
    //s_test_nvs_ropframe();


    //nvs_rx_pkt = eo_packet_New(512);
    //s_test_nvs_transmitter();
    //s_test_nvs_receiver();

    {


    }

    s_test_nvs_transceiver_tick();

}

extern void demo_nvs_signal(uint16_t port, uint16_t nvid)
{
//    eOresult_t res;
//
//    res = eo_agent_OutROPinit(s_theagent, eo_common_ipv4addr(10, 0, 0, 100), port, eo_ropcode_sig, nvid, eok_ropconfig_basic, s_out_rop);
//
//    res = eo_former_GetStream(s_theformer, s_out_rop, DATACAPACITY, s_pktdata, &s_pktsize, &s_ipaddr);
//
//    res = res;
 

}

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------




static void s_mytmes_startup(EOMtask *p, uint32_t t)
{
    EOtimer *tmr;

    hal_trace_puts("start a timer which every 5 seconds shall call a callback\n\r");
    tmr = eo_timer_New();
    eo_action_SetCallback(s_action, s_testeom_callback_timer, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));

    eo_timer_Start(tmr, eok_abstimeNOW, 1*eok_reltime1sec, eo_tmrmode_FOREVER, s_action); 

    demo_nvs_init();

    //demo_nvs_signal(EOK_cfg_nvs_devANY_portMNGMNT_port, EOK_cfg_nvs_devANY_portMNGMNT_NVID__boardinfo);

       
}

static void s_mytmes_run(EOMtask *p, uint32_t t)
{
    eOmessage_t msg = (eOmessage_t)t;
    //char str[64];

    //snprintf(str, sizeof(str)-1, "task msg-based has received msg = 0x%x\n\r", msg);
    //hal_trace_puts(str);

    if(0x10001000 == msg)
    {
        //demo_nvs_signal(EOK_cfg_nvs_devANY_portMNGMNT_NVID__boardinfo);
    }


    demo_nvs_tick();

}

extern void task_example01(void *p)
{
    // do here whatever you like before startup() is executed and then forever()
    eom_task_Start(p);
}



//static void s_testeom_callback_button_wkup(void *arg)
//{
//    static eOmessage_t msg = 0x10001000;
//    eom_task_SendMessage(mytask_message, msg, eok_reltimeINFINITE);
//}


static void s_testeom_callback_timer(void *arg)
{
    //static uint32_t nn = 0;
    static eOmessage_t msg = 0x10001001;
    //char str[80];


    //snprintf(str, sizeof(str)-1, "callback registered by timer has been called for the %d-th time\n\r", ++nn);
    //hal_trace_puts(str);

    //snprintf(str, sizeof(str)-1, "the callback has sent msg 0x%x to task msg-based\n\r", msg);
    //hal_trace_puts(str);


    eom_task_SendMessage(mytask_message, msg, eok_reltimeINFINITE);
}



//static void s_test_nvs00(void)
//{
//    static EOnvsCfg *locnvscfg;
//    static EOnvsCfg *remnvscfg;
//    static EOtreenode* treenode;
//    uint16_t portindex = 0;
//    const uint32_t ipaddress_pc104 = 0x01020304;
//    const uint32_t ipaddress_leftarm_ems_a = 0x12345678;
//    const uint16_t iport_base_rops = 33333;
//    EOrop *rop_to_ems = NULL;
//    EOrop *rop_from_pc104 = NULL;
//    EOrop *rop_to_pc104 = NULL;
//    EOrop *rop_from_ems = NULL;
//    EOrop *rop1 = NULL;
//    EOrop *rop2 = NULL;
//
//    // initialise the remote side
//
//    remnvscfg = eo_nvscfg_New(1, NULL);
//    eo_nvscfg_PushBackDevice(remnvscfg, eo_nvscfg_ownership_remote, ipaddress_leftarm_ems_a, NULL, 1);
//    eo_nvscfg_ondevice_PushBackEndpoint(remnvscfg, 0, EOK_cfg_nvsEP_base_endpoint,
//                                    eo_cfg_nvsEP_base_fptr_hashfunction_id2index,
//                                    eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_base_usr_rem_anydev_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_base_t), 
//                                    eo_cfg_nvsEP_base_usr_rem_anydev_initialise,
//                                    NULL);
//
//    eo_nvscfg_data_Initialise(remnvscfg);
//
//
//    // initialise the local side
//
//    locnvscfg = eo_nvscfg_New(1, NULL);
//    eo_nvscfg_PushBackDevice(locnvscfg, eo_nvscfg_ownership_local, ipaddress_leftarm_ems_a, NULL, 1);
//    eo_nvscfg_ondevice_PushBackEndpoint(locnvscfg, 0, EOK_cfg_nvsEP_base_endpoint,
//                                    eo_cfg_nvsEP_base_fptr_hashfunction_id2index,
//                                    eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_base_t), 
//                                    eo_cfg_nvsEP_base_usr_loc_anydev_initialise,
//                                    NULL);
//
//    eo_nvscfg_data_Initialise(locnvscfg);
//
//
//    // initialise the singleton and the rops
//    s_theformer = eo_former_Initialise();
//    s_theparser = eo_parser_Initialise();
//    s_theagent  = eo_agent_Initialise(NULL); 
//    rop1   = eo_rop_New(256); 
//    rop2   = eo_rop_New(256);
//
//
//    // ---- the pc104 forms a rop to ask the boardinfo
//
//    rop_to_ems = rop1;
//    eo_agent_OutROPinit(s_theagent, remnvscfg, 
//                        ipaddress_leftarm_ems_a, iport_base_rops, 
//                        eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__boardinfo, eok_ropconfig_basic, 
//                        rop_to_ems, NULL);
//
//    // ---- the ems has received the rop and pass it to its agent
//
//    rop_from_pc104 = rop_to_ems; // rop1
//    rop_to_pc104 = rop2;
//
//    eo_agent_InpROPprocess(s_theagent, locnvscfg, rop_from_pc104, rop_to_pc104, ipaddress_pc104);
//
//    if(eo_ropcode_none != eo_rop_GetROPcode(rop_to_pc104))
//    {
//        rop_from_ems = rop_to_pc104;   /// rop2
//    }
//    else
//    {
//        rop_from_ems = NULL;
//    }
//
//    // ---- the pc104 has received the answer and pass it to its agent
//
//    rop_to_ems = rop1;
//    //rop_to_pc104->aboutip.ipaddr = ipaddress_leftarm_ems_a;
//    eo_agent_InpROPprocess(s_theagent, remnvscfg, rop_from_ems, rop_to_ems, ipaddress_leftarm_ems_a);
//
//
////    portindex = eo_nvscfg_hid_ondevice_endpoint2index(locnvscfg, 0, EOK_cfg_nvsEP_base_endpoint);
////
////    treenode = eo_nvscfg_hid_ondevice_onendpoint_withID_GetTreeNode(locnvscfg, 0, portindex, EOK_cfg_nvsEP_base_NVID__forcerestart);
////    treenode = treenode;
//
//}

//
//static void s_test_nvs01(void)
//{
//
//    uint32_t toipaddr;
//    uint32_t fromipaddr;
//
//    s_nvs_common_init();
//
//    s_nvs_pc104_init();
//
//    s_nvs_ems00_init();
//
//
//    s_nvs_pc104_asks(&nvs_rop2ems00, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr);
//
//    // the packet is delivered
//
//    if(eobool_true == s_nvs_ems00_receive(&nvs_pkt.data[0], nvs_pkt.size, nvs_pc104_ipaddress, 
//                                &nvs_rop2pc104, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr))
//    {
//
//        // the reply packet is transmitted to teh pc104
//
//        s_nvs_pc104_receive(&nvs_pkt.data[0], nvs_pkt.size, nvs_ems00_ipaddress, 
//                                &nvs_rop2ems00, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr);
//
//    }
//
//
//
//    // the ems emits a sig
//
//
//    s_nvs_ems00_emits(&nvs_rop2pc104, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr);
//
//    // the packet is delivered
//
//    s_nvs_pc104_receive(&nvs_pkt.data[0], nvs_pkt.size, nvs_ems00_ipaddress, 
//                                &nvs_rop2ems00, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr);
//
//
//    // the pc104 sets a value
//
//    s_nvs_pc104_set(&nvs_rop2ems00, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr);
//
//    // the packet is delivered
//
//    if(eobool_true == s_nvs_ems00_receive(&nvs_pkt.data[0], nvs_pkt.size, nvs_pc104_ipaddress, 
//                                &nvs_rop2pc104, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr))
//    {
//
//        // the reply packet is transmitted to teh pc104
//
//        s_nvs_pc104_receive(&nvs_pkt.data[0], nvs_pkt.size, nvs_ems00_ipaddress, 
//                                &nvs_rop2ems00, &nvs_pkt.data[0], &nvs_pkt.size, &toipaddr);
//
//    }
//
//
//}


//static void s_nvs_pc104_init(void)
//{
//    // initialise the remote side
//
//    nvs_pc104_nvscfg = eo_nvscfg_New(1, NULL);
//    eo_nvscfg_PushBackDevice(nvs_pc104_nvscfg, eo_nvscfg_ownership_remote, nvs_ems00_ipaddress, NULL, 1);
//    eo_nvscfg_ondevice_PushBackEndpoint(nvs_pc104_nvscfg, 0, EOK_cfg_nvsEP_base_endpoint,
//                                    eo_cfg_nvsEP_base_fptr_hashfunction_id2index,
//                                    eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_base_usr_rem_anydev_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_base_t), 
//                                    eo_cfg_nvsEP_base_usr_rem_anydev_initialise,
//                                    NULL);
//
//    eo_nvscfg_data_Initialise(nvs_pc104_nvscfg);
//}
//
//
//static void s_nvs_ems00_init(void)
//{  
//
//    // initialise the local side
//
//    nvs_ems00_nvscfg = eo_nvscfg_New(1, NULL);
//    eo_nvscfg_PushBackDevice(nvs_ems00_nvscfg, eo_nvscfg_ownership_local, nvs_ems00_ipaddress, NULL, 1);
//    eo_nvscfg_ondevice_PushBackEndpoint(nvs_ems00_nvscfg, 0, EOK_cfg_nvsEP_base_endpoint,
//                                    eo_cfg_nvsEP_base_fptr_hashfunction_id2index,
//                                    eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_base_t), 
//                                    eo_cfg_nvsEP_base_usr_loc_anydev_initialise,
//                                    NULL);
//
//    eo_nvscfg_data_Initialise(nvs_ems00_nvscfg);
//
//}



//static void s_nvs_common_init(void)
//{  
//    // initialise singletons
//    nvs_theformer = eo_former_Initialise();
//    nvs_theparser = eo_parser_Initialise();
//    nvs_theagent  = eo_agent_Initialise(NULL); 
//
//    nvs_pkt.size = 0;
//
//
//    nvs_rop_rec = eo_rop_New(256);
//    nvs_rop_out = eo_rop_New(256);
//    nvs_rop_tmp = eo_rop_New(256);
//
//    nvs_nv = eo_nv_New();
//}



//static void s_nvs_pc104_asks(EOrop **nvs_rop2ems00, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr)
//{
//    *txpktsize = 0;
//
//    eo_agent_OutROPinit(nvs_theagent, nvs_pc104_nvscfg, 
//                        nvs_ems00_ipaddress, nvs_base_endpoint_iport, 
//                        eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__boardinfo, eok_ropconfig_basic, 
//                        nvs_rop_out, NULL);
//
//    eo_former_GetStream(nvs_theformer, nvs_rop_out, DATACAPACITY, txpktdata, txpktsize, toipaddr);
//    
//    *nvs_rop2ems00 = nvs_rop_out;
//}


//static eObool_t s_nvs_ems00_receive(uint8_t *rxpktdata, uint16_t rxpktsize, uint32_t fromipaddr, 
//                                    EOrop **nvs_rop2pc104, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr)
//{
//    uint16_t consumedbytes = 0;
//    eEboardInfo_t *brd = NULL;
//
//    eo_parser_GetROP(nvs_theparser, rxpktdata, rxpktsize, fromipaddr, nvs_rop_rec, &consumedbytes);
//
//
//    eo_agent_InpROPprocess(nvs_theagent, nvs_ems00_nvscfg, nvs_rop_rec, nvs_rop_out, fromipaddr);
//
//    if(eo_ropcode_none != eo_rop_GetROPcode(nvs_rop_out))
//    {
//        eo_former_GetStream(nvs_theformer, nvs_rop_out, DATACAPACITY, txpktdata, txpktsize, toipaddr);
//        brd = (eEboardInfo_t*)(txpktdata+8);
//        brd =  brd;
//
//        *nvs_rop2pc104 = nvs_rop_out; 
//        return(eobool_true);  
//    }
//    else
//    {
//        *txpktsize = 0;
//        *nvs_rop2pc104 = NULL;
//        return(eobool_false);
//    }
//}

//
//static eObool_t s_nvs_pc104_receive(uint8_t *rxpktdata, uint16_t rxpktsize, uint32_t fromipaddr, 
//                                    EOrop **nvs_rop2ems00, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr)
//{
//    uint16_t consumedbytes = 0;
//
//    eo_parser_GetROP(nvs_theparser, rxpktdata, rxpktsize, fromipaddr, nvs_rop_rec, &consumedbytes);
//
//
//    eo_agent_InpROPprocess(nvs_theagent, nvs_pc104_nvscfg, nvs_rop_rec, nvs_rop_out, fromipaddr);
//
//    if(eo_ropcode_none != eo_rop_GetROPcode(nvs_rop_out))
//    {
//        eo_former_GetStream(nvs_theformer, nvs_rop_out, DATACAPACITY, txpktdata, txpktsize, toipaddr);
//        *nvs_rop2ems00 = nvs_rop_out;
//        return(eobool_true);  
//    }
//    else
//    {
//        *txpktsize = 0;
//        return(eobool_false);
//    }
//
//}

//
//static void s_nvs_ems00_emits(EOrop **nvs_rop2pc104, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr)
//{
//    *txpktsize = 0;
//
//    eo_agent_OutROPinit(nvs_theagent, nvs_ems00_nvscfg, 
//                        nvs_pc104_ipaddress, nvs_base_endpoint_iport, 
//                        eo_ropcode_sig, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__boardinfo, eok_ropconfig_basic, 
//                        nvs_rop_out, NULL);
//
//    eo_former_GetStream(nvs_theformer, nvs_rop_out, DATACAPACITY, txpktdata, txpktsize, toipaddr);
//    
//    *nvs_rop2pc104 = nvs_rop_out;
//}

//
//static void s_nvs_pc104_set(EOrop **nvs_rop2ems00, uint8_t *txpktdata, uint16_t *txpktsize, uint32_t *toipaddr)
//{
//
//    // write a ipmnetwork value inside ....
//    static const eEipnetwork_t ipnet =
//    {
//        .macaddress     = 0x1234567890abcdef,
//        .ipaddress      = 0x01020304,
//        .ipnetmask      = 0xaabbccdd
//    };
//
//
//    eo_nvscfg_GetNV(nvs_pc104_nvscfg, nvs_ems00_ipaddress, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID_ipnetwork, NULL, nvs_nv);
//
//    eo_nv_Set(nvs_nv, &ipnet, eobool_false, eo_nv_upd_dontdo);
//
//    
//    *txpktsize = 0;
//
//    eo_agent_OutROPinit(nvs_theagent, nvs_pc104_nvscfg, 
//                        nvs_ems00_ipaddress, nvs_base_endpoint_iport, 
//                        eo_ropcode_set, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID_ipnetwork, eok_ropconfig_basic, 
//                        nvs_rop_out, NULL);
//
//    eo_former_GetStream(nvs_theformer, nvs_rop_out, DATACAPACITY, txpktdata, txpktsize, toipaddr);
//    
//    *nvs_rop2ems00 = nvs_rop_out;
//}

//static void s_nvs_ropframe_init(void)
//{
//    nvs_ems00_ropframe = eo_ropframe_New();
//    nvs_pc104_ropframe = eo_ropframe_New();
//    nvs_recvd_ropframe = eo_ropframe_New();
//}

//static void s_nvs_ropframe_pc104_prepare(void)
//{
//        // write a ipmnetwork value inside ....
//    static const eEipnetwork_t ipnet =
//    {
//        .macaddress     = 0x1234567890abcdef,
//        .ipaddress      = 0x01020304,
//        .ipnetmask      = 0xaabbccdd
//    };
//
//    uint16_t remainingbytes;
//    uint16_t usedbytes;
//
//    eo_ropframe_Load(nvs_pc104_ropframe, nvs_pc104_ropframe_payload, 0, 512);
//    eo_ropframe_Clear(nvs_pc104_ropframe);
//
//    // prepare the first rop: ask 
//    eo_agent_OutROPinit(nvs_theagent, nvs_pc104_nvscfg, 
//                        nvs_ems00_ipaddress, nvs_base_endpoint_iport, 
//                        eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__boardinfo, eok_ropconfig_basic, 
//                        nvs_rop_out, &usedbytes);
//
//    // put the rop inside the ropframe
//    eo_ropframe_ROP_Set(nvs_pc104_ropframe, nvs_rop_out, NULL, NULL, &remainingbytes);
//
//    // prepare the second rop: set
//    eo_nvscfg_GetNV(nvs_pc104_nvscfg, nvs_ems00_ipaddress, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID_ipnetwork, NULL, nvs_nv);
//    eo_nv_Set(nvs_nv, &ipnet, eobool_false, eo_nv_upd_dontdo);
//    eo_agent_OutROPinit(nvs_theagent, nvs_pc104_nvscfg, 
//                        nvs_ems00_ipaddress, nvs_base_endpoint_iport, 
//                        eo_ropcode_set, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID_ipnetwork, eok_ropconfig_basic, 
//                        nvs_rop_out, &usedbytes);
//
//    // put the rop inside the ropframe
//    eo_ropframe_ROP_Set(nvs_pc104_ropframe, nvs_rop_out, NULL, NULL, &remainingbytes);
//
//    remainingbytes = remainingbytes;
// 
//
//}

//static void s_nvs_ropframe_ems00_receive(void)
//{
//
//    eOresult_t res;
//    uint16_t rxremainingbytes;
//    uint16_t txremainingbytes;
//    uint16_t usedbytes;
//    uint8_t *fdata = NULL;
//    uint16_t fsize = 0;
//    uint16_t fcapa = 0;
//    uint16_t nrops = 0;
//    uint16_t i;
//    uint32_t fromipaddr = nvs_pc104_ipaddress;
//
//    eo_ropframe_Get(nvs_pc104_ropframe, &fdata, &fsize, &fcapa); 
//
//    // copy the payload
//    memcpy(nvs_recvd_ropframe_payload, fdata, fcapa);
//
//    // load it
//    eo_ropframe_Load(nvs_recvd_ropframe, nvs_recvd_ropframe_payload, fsize, fcapa);
//
//    // initialise the frame in output ...
//    eo_ropframe_Load(nvs_ems00_ropframe, nvs_ems00_ropframe_payload, 0, 512);
//    eo_ropframe_Clear(nvs_ems00_ropframe);
//
//
//    if(eobool_false == eo_ropframe_IsValid(nvs_recvd_ropframe))
//    {
//        return;
//    }
//
//    nrops = eo_ropframe_ROP_NumberOf(nvs_recvd_ropframe);
//
//    for(i=0; i<nrops; i++)
//    {
//        res = eo_ropframe_ROP_Get(nvs_recvd_ropframe, nvs_rop_rec, &rxremainingbytes);
//
//        if(eores_OK != res)
//        {
//            break;
//        }
//
//        
//        // now use the rop ...       
//        eo_agent_InpROPprocess(nvs_theagent, nvs_ems00_nvscfg, nvs_rop_rec, nvs_rop_out, fromipaddr); 
//
//        if(eo_ropcode_none != eo_rop_GetROPCode(nvs_rop_out))
//        {
//            // add nvs_rop_out to the outgoing frame.
//            res = eo_ropframe_ROP_Set(nvs_ems00_ropframe, nvs_rop_out, NULL, NULL, &txremainingbytes);
//        }
//
//        if(0 == rxremainingbytes)
//        {
//            break;
//        }
//    }
//
//
//}

//static void s_test_nvs_ropframe(void)
//{
//
//    uint32_t toipaddr;
//    uint32_t fromipaddr;
//
//    s_nvs_common_init();
//
//    s_nvs_pc104_init();
//
//    s_nvs_ems00_init();
//
//    s_nvs_ropframe_init();
//
//
//    s_nvs_ropframe_pc104_prepare();
//
//    s_nvs_ropframe_ems00_receive();
//
//}


//static void s_test_nvs_transmitter(void)
//{
//
//    EOtransmitter *transmitter;
//    EOpacket *pkt;
//    uint16_t numberorops = 0;
//
//    eo_transmitter_cfg_t cfg = 
//    {
//        .capacityoftxpacket             = 512, 
//        .capacityofropframepermanent    = 256, 
//        .capacityofropframetemporary    = 256,
//        .capacityofrop                  = 128, 
//        .maxnumberofpermanentrops       = 16,
//        .nvscfg                         = NULL,
//        .ipv4addr                       = nvs_pc104_ipaddress,
//        .ipv4port                       = 10001
//    };
//
//    s_nvs_common_init();
//
//    s_nvs_pc104_init();
//
//    s_nvs_ems00_init();
//
//    cfg.nvscfg  = nvs_ems00_nvscfg;
//
//    transmitter = eo_transmitter_New(&cfg);
//
//
//    // do whatever you want ...
//
//    eo_transmitter_permanentrops_Load(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__boardinfo, eok_ropconfig_basic);
//
//    eo_transmitter_permanentrops_Load(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID_ipnetwork, eok_ropconfig_basic);
//
//    eo_transmitter_permanentrops_Load(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__bootprocess, eok_ropconfig_basic);
//
//    eo_transmitter_permanentrops_Load(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__localise, eok_ropconfig_basic);
//
////    eo_transmitter_permanentrops_Unload(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_NVID__boardinfo);
////    eo_transmitter_permanentrops_Unload(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_NVID_ipnetwork); 
////    eo_transmitter_permanentrops_Unload(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_NVID__localise);
////    eo_transmitter_permanentrops_Unload(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_NVID__bootprocess);
//
//
//    eo_transmitter_temporaryrop_Load(transmitter, eo_ropcode_sig, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__gotoprocess, eok_ropconfig_basic);
//    
//
//    eo_transmitter_permanentrops_Refresh(transmitter);
//    eo_transmitter_outpacket_Get(transmitter, &pkt, &numberorops);
//
//
//    eo_packet_Copy(nvs_rx_pkt, pkt);
//
//}
//
//static void s_test_nvs_receiver(void)
//{
//    eObool_t thereisareply = eobool_false;
//    EOropframe *ropframereply = NULL;
//    eOipv4addr_t ipv4addr;
//    eOipv4port_t ipv4port;
//    EOreceiver* receiver = NULL;
//    uint16_t numberorops = 0;
//    uint64_t txtime;
//    
//    eo_receiver_cfg_t cfg = 
//    {
//        .capacityofropframereply        = 512, 
//        .capacityofropinput             = 128, 
//        .capacityofropreply             = 128,
//        .nvscfg                         = NULL
//    };
//
//    cfg.nvscfg  = nvs_pc104_nvscfg;
//    
//    receiver = eo_receiver_New(&cfg);
//
//    eo_packet_Addressing_Set(nvs_rx_pkt, nvs_ems00_ipaddress, nvs_base_endpoint_iport);
//
//    eo_receiver_Process(receiver, nvs_rx_pkt, nvs_pc104_nvscfg, &numberorops, &thereisareply, &txtime);
//     
//    if(eobool_true == thereisareply)
//    {
//        eo_receiver_GetReply(receiver, &ropframereply, &ipv4addr, &ipv4port);
//    }
//
//    ropframereply = ropframereply;
//    
//
//}


static void s_test_nvs_transceiver_init(void)
{
//    uint16_t numofrops_pc104_rx;
//    uint16_t numofrops_pc104_tx;
//    uint16_t numofrops_ems00_rx;
//    uint16_t numofrops_ems00_tx;

    uint16_t numofrops;          
//    eo_transceiver_cfg_t pc104cfg = eo_transceiver_cfg_default;
//    eo_transceiver_cfg_t ems00cfg = eo_transceiver_cfg_default;
    eOboardtransceiver_cfg_t boardtxrxcfg = 
    {
        .vectorof_endpoint_cfg          = eo_cfg_EPs_vectorof_loc_board,
        .hashfunction_ep2index          = eo_cfg_nvsEP_loc_board_fptr_hashfunction_ep2index,
        .remotehostipv4addr             = nvs_pc104_ipaddress,
        .remotehostipv4port             = nvs_base_endpoint_iport
    };
    eOhosttransceiver_cfg_t hosttxrxcfg = 
    {
        .vectorof_endpoint_cfg          = eo_cfg_EPs_vectorof_rem_board,
        .hashfunction_ep2index          = eo_cfg_nvsEP_rem_board_fptr_hashfunction_ep2index,
        .remoteboardipv4addr            = nvs_ems00_ipaddress,
        .remoteboardipv4port            = nvs_base_endpoint_iport
    };

    EOhostTransceiver *hosttxrx = NULL;
    eOabstime_t txtime;

//    s_test_nvs_transceiver_pc104_cfg_set(&pc104cfg);
//    pc104txrx = eo_transceiver_New(&pc104cfg);

    hosttxrx = eo_hosttransceiver_New(&hosttxrxcfg);
    pc104txrx = eo_hosttransceiver_Transceiver(hosttxrx);

    
//    s_test_nvs_transceiver_ems00_cfg_set(&ems00cfg);
//    ems00txrx = eo_transceiver_New(&ems00cfg);
    ems00txrx = eo_boardtransceiver_Initialise(&boardtxrxcfg);

    theems00transceiver = ems00txrx;

    // the ems00 is configured at startup
    //s_test_nvs_transceiver_ems00_regulars_config(ems00txrx);


    s_test_nvs_transceiver_pc104_configure_ems(pc104txrx);
 
    // 1. the pc104 adds a command set-EOK_cfg_nvsEP_base_NVID__localise, a ask-EOK_cfg_nvsEP_base_NVID__gotoprocess
    //    and sends the packet
    s_test_nvs_transceiver_pc104_occasional_load(pc104txrx, eo_ropcode_set, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__localise);
    s_test_nvs_transceiver_pc104_occasional_load(pc104txrx, eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__gotoprocess);
    eo_transceiver_Transmit(pc104txrx, &transpacket, &numofrops);


     // 2. the ems receives teh packet, then add a sig-EOK_cfg_nvsEP_base_NVID__boardinfo and transmits
     eo_transceiver_Receive(ems00txrx, transpacket, &numofrops, &txtime);
     s_test_nvs_transceiver_ems00_occasional_load(ems00txrx, EOK_cfg_nvsEP_base_NVID__boardinfo);
     eo_transceiver_Transmit(ems00txrx, &transpacket, &numofrops);

//     // 3. the pc104 receives and transmits, the ems00 receives and transmits
//     for(;;)
//     {
//
//        // -- pc104 
//        eo_transceiver_Receive(pc104txrx, transpacket, &numofrops_pc104_rx); 
//        // may add an extra rop: ask or set or rst
//        eo_transceiver_Transmit(pc104txrx, &transpacket, &numofrops_pc104_tx);
//
//        // -- ems00 
//        eo_transceiver_Receive(ems00txrx, transpacket, &numofrops_ems00_rx); 
//        // may add an extra rop: sig
//        eo_transceiver_Transmit(ems00txrx, &transpacket, &numofrops_ems00_tx);
//     }

}


static void s_test_nvs_transceiver_tick(void)
{
    uint16_t numofrops_pc104_rx;
    uint16_t numofrops_pc104_tx;
    uint16_t numofrops_ems00_rx;
    uint16_t numofrops_ems00_tx;
    char str[128];
    static eObool_t boolval = eobool_false;

    static eObool_t pc104rxtx = eobool_true;
    static uint32_t step = 0;

//    static uint16_t size = 0;
    eOabstime_t txtime;


    if(eobool_true == pc104rxtx)
    {
        uint32_t sz = eo_mempool_SizeOfAllocated(eo_mempool_GetHandle());
        sz = sz;
        // -- pc104 
        snprintf(str, sizeof(str)-1, "step %d: PC104", step);
        hal_trace_puts(str);
        
        // the pkt comes from the ems00
        eo_packet_Addressing_Set(transpacket, nvs_ems00_ipaddress, nvs_base_endpoint_iport);
         
        eo_transceiver_Receive(pc104txrx, transpacket, &numofrops_pc104_rx, &txtime);

        snprintf(str, sizeof(str)-1, "               - rx = %d", numofrops_pc104_rx);
        hal_trace_puts(str); 
        // may add an extra rop: ask or set or rst
        if(4==(step%10))
        {
            s_test_nvs_transceiver_pc104_occasional_load(pc104txrx, eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__gotoprocess);
            snprintf(str, sizeof(str)-1, "               - added a ask<__gotoprocess>");
            hal_trace_puts(str); 

        }
        else if(8==(step%10))
        {
            //boolval = (eobool_false==boolval) ? (eobool_true) : (eobool_false);
            boolval = 0;
            eo_cfg_nvsEP_base_usr_rem_anydev_mem_local->localise = boolval;
            s_test_nvs_transceiver_pc104_occasional_load(pc104txrx, eo_ropcode_set, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__localise);
            snprintf(str, sizeof(str)-1, "               - added a set<__localise, %d>", boolval);
            hal_trace_puts(str); 
        //    s_test_nvs_transceiver_pc104_occasional_load(pc104txrx, eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__localise);
        //    snprintf(str, sizeof(str)-1, "               - added a ask<__localise>");
        //    hal_trace_puts(str); 
        }
        eo_transceiver_Transmit(pc104txrx, &transpacket, &numofrops_pc104_tx);

        snprintf(str, sizeof(str)-1, "               - tx = %d", numofrops_pc104_tx);
        hal_trace_puts(str); 

        pc104rxtx = eobool_false;
        step++;
    }
    else
    {

        // -- ems00 
//        snprintf(str, sizeof(str)-1, "step %d: EMS00", step);
//        hal_trace_puts(str);

        // the pkt comes from the pc104
        eo_packet_Addressing_Set(transpacket, nvs_pc104_ipaddress, nvs_base_endpoint_iport);
        eo_transceiver_Receive(ems00txrx, transpacket, &numofrops_ems00_rx, &txtime); 

//        snprintf(str, sizeof(str)-1, "               - rx = %d", numofrops_ems00_rx);
//        hal_trace_puts(str); 
        // may add an extra rop: sig
        if(3==(step%10))
        {
            s_test_nvs_transceiver_ems00_occasional_load(ems00txrx, EOK_cfg_nvsEP_base_NVID__bootprocess);
//            snprintf(str, sizeof(str)-1, "               - added a sig<__boardinfo>");
//            hal_trace_puts(str); 
        }

        // may add an extra rop: sig
        eo_transceiver_Transmit(ems00txrx, &transpacket, &numofrops_ems00_tx);

//        snprintf(str, sizeof(str)-1, "               - tx = %d", numofrops_ems00_tx);
//        hal_trace_puts(str); 

        pc104rxtx = eobool_true;
        step++;
    }

}



//static EOnvsCfg* s_test_nvs_nvscfg_pc104_get(void)
//{
//    static EOnvsCfg* cfg = NULL;
//
//    if(NULL != cfg)
//    {
//        return(NULL);
//    }
//
//    cfg = eo_nvscfg_New(1, NULL);
//    eo_nvscfg_PushBackDevice(cfg, eo_nvscfg_ownership_remote, nvs_ems00_ipaddress, 2);
//    eo_nvscfg_ondevice_PushBackEndpoint(cfg, 0, EOK_cfg_nvsEP_base_endpoint,
//                                    eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_base_usr_rem_anydev_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_base_t), 
//                                    eo_cfg_nvsEP_base_usr_rem_anydev_initialise,
//                                    NULL);
//
//    eo_nvscfg_ondevice_PushBackEndpoint(cfg, 0, EOK_cfg_nvsEP_mngmnt_endpoint,
//                                    eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_mngmnt_usr_rem_board_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_mngmnt_t), 
//                                    eo_cfg_nvsEP_mngmnt_usr_rem_board_initialise,
//                                    NULL);
//
//
//    eo_nvscfg_data_Initialise(cfg);
//
//    return(cfg);
//}
//
//
//static EOnvsCfg* s_test_nvs_nvscfg_ems00_get(void)
//{
//    static EOnvsCfg* cfg = NULL;
//
//    if(NULL != cfg)
//    {
//        return(NULL);
//    }
//
//    cfg = eo_nvscfg_New(1, NULL);
//    eo_nvscfg_PushBackDevice(cfg, eo_nvscfg_ownership_local, nvs_ems00_ipaddress, 1);
//    eo_nvscfg_ondevice_PushBackEndpoint(cfg, 0, EOK_cfg_nvsEP_base_endpoint,
//                                    eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con,
//                                    eo_cfg_nvsEP_base_usr_loc_anydev_constvector_of_EOnv_usr,
//                                    sizeof(eo_cfg_nvsEP_base_t), 
//                                    eo_cfg_nvsEP_base_usr_loc_anydev_initialise,
//                                    NULL);
//
//    eo_nvscfg_data_Initialise(cfg);
//
//    return(cfg);
//}


//static void s_test_nvs_transceiver_pc104_cfg_set(eo_transceiver_cfg_t *cfg)
//{
//    memcpy(cfg, &eo_transceiver_cfg_default, sizeof(eo_transceiver_cfg_t));
//
//    cfg->capacityofpacket               = 512;
//    cfg->capacityofrop                  = 128;
//    cfg->capacityofropframeregulars     = 256;
//    cfg->capacityofropframeoccasionals  = 128;
//    cfg->capacityofropframereplies      = 128;
//    cfg->maxnumberofregularrops         = 16;
//    cfg->remipv4addr                    = nvs_ems00_ipaddress;
//    cfg->remipv4port                    = nvs_base_endpoint_iport;
//    cfg->nvscfg                         = s_test_nvs_nvscfg_pc104_get();
//}

//static void s_test_nvs_transceiver_ems00_cfg_set(eo_transceiver_cfg_t *cfg)
//{
//    memcpy(cfg, &eo_transceiver_cfg_default, sizeof(eo_transceiver_cfg_t));
//
//    cfg->capacityofpacket               = 512;
//    cfg->capacityofrop                  = 128;
//    cfg->capacityofropframeregulars     = 256;
//    cfg->capacityofropframeoccasionals  = 128;
//    cfg->capacityofropframereplies      = 128;
//    cfg->maxnumberofregularrops         = 16;
//    cfg->remipv4addr                    = nvs_pc104_ipaddress;
//    cfg->remipv4port                    = nvs_base_endpoint_iport;
//    cfg->nvscfg                         = s_test_nvs_nvscfg_ems00_get();
//}

//static void s_test_nvs_transceiver_ems00_regulars_config(EOtransceiver *txrx)
//{
//    // regularly signal ...   ipnetwork__ipaddress and bootprocess
//    eo_transceiver_ropinfo_t ropinfo;
//
//    ropinfo.ropcfg      = eok_ropconfig_basic;
//    ropinfo.ropcode     = eo_ropcode_sig;
//    ropinfo.nvep        = EOK_cfg_nvsEP_base_endpoint;
//
//
//    eo_transceiver_rop_regular_Clear(txrx);
//
//    ropinfo.nvid = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress;
//    eo_transceiver_rop_regular_Load(txrx, &ropinfo);
//
//
//    ropinfo.nvid = EOK_cfg_nvsEP_base_NVID__bootprocess;
//    eo_transceiver_rop_regular_Load(txrx, &ropinfo);
//}


static void s_test_nvs_transceiver_ems00_occasional_load(EOtransceiver *txrx, uint16_t nvid)
{
    eo_transceiver_ropinfo_t ropinfo;

    ropinfo.ropcfg      = eok_ropconfig_basic;
    ropinfo.ropcode     = eo_ropcode_sig;
    ropinfo.nvep        = EOK_cfg_nvsEP_base_endpoint;

    ropinfo.nvid = nvid;
    eo_transceiver_rop_occasional_Load(txrx, &ropinfo);
}


static void s_test_nvs_transceiver_pc104_occasional_load(EOtransceiver *txrx, eOropcode_t opc, uint16_t ep, uint16_t nvid)
{
    eo_transceiver_ropinfo_t ropinfo;

    ropinfo.ropcfg      = eok_ropconfig_basic;
    ropinfo.ropcode     = opc;
    ropinfo.nvep        = ep;

    ropinfo.nvid = nvid;
    eo_transceiver_rop_occasional_Load(txrx, &ropinfo);
}


static void s_test_nvs_transceiver_pc104_configure_ems(EOtransceiver *txrx)
{
    EOarray *upto10 = (EOarray*) & eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_local->upto10rop2signal;
    eOropSIGcfg_t sigcfg;

    eo_array_Reset(upto10);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork;
    sigcfg.plustime = 1;
    eo_array_PushBack(upto10, &sigcfg);


    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID__bootprocess;
    sigcfg.plustime = 1;
    eo_array_PushBack(upto10, &sigcfg);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID__applicationinfo;
    sigcfg.plustime = 0;
    eo_array_PushBack(upto10, &sigcfg);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID__boardinfo;
    sigcfg.plustime = 0;
    eo_array_PushBack(upto10, &sigcfg);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask;
    sigcfg.plustime = 0;
    eo_array_PushBack(upto10, &sigcfg);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress;
    sigcfg.plustime = 0;
    eo_array_PushBack(upto10, &sigcfg);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID__remoteipaddress;
    sigcfg.plustime = 0;
    eo_array_PushBack(upto10, &sigcfg);

    sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
    sigcfg.id = EOK_cfg_nvsEP_base_NVID__remoteipport;
    sigcfg.plustime = 0;
    eo_array_PushBack(upto10, &sigcfg);    
#if 0
#endif

    s_test_nvs_transceiver_pc104_occasional_load(txrx, eo_ropcode_set, EOK_cfg_nvsEP_mngmnt_endpoint, EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal);  
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



