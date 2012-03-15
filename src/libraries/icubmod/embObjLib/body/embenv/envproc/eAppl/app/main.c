
/* @file       main.c
	@brief      This file implements application form ems
	@author     valentina.gaggero@iit.it
    @date       12/05/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

#include "hal.h"
//#include "osal.h"
//#include "ipal.h"
//#include "fsal.h"
//#include "dspal.h"

#include "eEcommon.h"
#include "shalPART.h" 
#include "shalBASE.h" 
#include "shalINFO.h" 


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

//application
#include "appl_common.h"
#include "systemController.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

extern hal_cfg_t *hal_cfgMINE;
extern fsal_params_cfg_t *fsal_params_cfgMINE;
extern osal_cfg_t *osal_cfgMINE;
extern const ipal_params_cfg_t *ipal_params_cfgMINE;
 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
EOMtask                 *sysCntr_task    = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_initialise_shal(void);
static void s_application_init(void);
static void s_appl_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info);
static void s_application_error_init(void);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
#ifdef _DEBUG_PRINT_ 
static char my_deb_string[50];
static char *errtype_str[4] = {"Info", "Warning", "Weak", "Fatal"};
#endif


static const eEmoduleInfo_t s_appl_info __attribute__((at(EENV_MEMMAP_EAPPLICATION_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .info           =
    {
        .entity     =
        {
            .type       = ee_entity_process,
            .signature  = ee_procApplication,
            .version    = 
            { 
                .major = 1, 
                .minor = 0
            },  
            .builddate  = 
            {
                .year  = 2011,
                .month = 12,
                .day   = 5,
                .hour  = 16,
                .min   = 0
            }
        },
        .rom        = 
        {   
            .addr   = EENV_MEMMAP_EAPPLICATION_ROMADDR,
            .size   = EENV_MEMMAP_EAPPLICATION_ROMSIZE
        },
        .ram        = 
        {   
            .addr   = EENV_MEMMAP_EAPPLICATION_RAMADDR,
            .size   = EENV_MEMMAP_EAPPLICATION_RAMSIZE
        },
        .storage    = 
        {
            .type   = ee_strg_eeprom,
            .size   = EENV_MEMMAP_EAPPLICATION_STGSIZE,
            .addr   = EENV_MEMMAP_EAPPLICATION_STGADDR
        },
        .communication  = ee_commtype_eth | ee_commtype_can1 | ee_commtype_can2 | ee_commtype_gtw ,  // later on we may also add can1 and can2
        .name           = "eAppl"
    },
    .protocols  =
    {
        .udpprotversion  = { .major = 0, .minor = 1},
        .can1protversion = { .major = 0, .minor = 0},
        .can2protversion = { .major = 0, .minor = 0},
        .gtwprotversion  = { .major = 0, .minor = 0}
    },
    .extra      = {0}
};


static const eOerrman_cfg_t  errmancfg = 
{
    .extfn.usr_on_error = s_appl_errman_OnError
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

int main(void)
{
    //system configuration
    eOmsystem_cfg_t syscfg =
    {
        .halcfg     = hal_cfgMINE,
        .osalcfg    = osal_cfgMINE,
        .fsalcfg    = NULL, //fsal_params_cfgMINE
    };

#warning: per ora ho commentato l'uso delle shared lib [MAIN]
//    s_initialise_shal();  //inizializzo le shared libs

//hal_sys_vectortable_relocate(EENV_MEMMAP_EUPDATER_ROMADDR-EENV_ROMSTART);


    eom_sys_Initialise( &syscfg,
                        NULL,                           // mempool  is NULL ==> we use dynimic allocation
                        &errmancfg,                     // errman
                        &eom_timerman_DefaultCfg,
                        &eom_callbackman_DefaultCfg
                      );  

    eom_sys_Start(eom_sys_GetHandle(), s_application_init); //faccio partire il sistema

}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_initialise_shal(void)
{

    if(ee_res_OK == shalbase_isvalid())
    {
        shalbase_init(1);
    }
    else
    {
        s_application_error_init();
    }

    // put signature info partition table
    if(ee_res_OK == shalpart_isvalid())
    {
        shalpart_init();
        shalpart_proc_synchronise(ee_procApplication, &s_appl_info);
    }
    else
    {
        // consider to enter in an error mode.
        s_application_error_init();
    }

    
    if(ee_res_OK == shalinfo_isvalid())
    {
        shalinfo_init();
    }
    else
    {
        // consider to enter in an error mode.
        s_application_error_init();
    }
    
    hal_trace_puts("initted sys-shals");      
   
}

static void s_application_error_init(void)
{
    /*questa funzione deve gestire l'errore avvenuto durante la fase di init.
    Ancora priam della partenza del sistema. */
    while(1)
    {;}


}


static void s_appl_errman_OnError(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    /*
        Dato che la funzione del'error manager invocata in caso di errore (eo_errman_Error)
        per prima cosa chiama la callback definita dall'utente e poi se l'errore e' grave
        ferma il sistema ed entra nel forever looop.

        io qui propongo di accendere un led o cmq fare qc di semplice e veloce 
        in modo tale che la funzione chiami al piu' presto la funz che ferma il sistema.
    */

#ifdef _DEBUG_PRINT_ 
    if(eo_errortype_info == errtype)
    {
        return;
    }
    sprintf(my_deb_string, "%s: %s - %s", errtype_str[errtype], eobjstr, info);
    hal_trace_puts(my_deb_string);  
#endif

    ;

}


static void s_application_init(void)
{
    eEresult_t res;
    const shalinfo_deviceinfo_t *deviceinfo_ptr;
    eOmipnet_cfg_addr_t  ipnet_cfg_addr;
    eOmipnet_cfg_dtgskt_t ipnet_cfg_dtgskt;


#ifdef _DEBUG_PRINT_ 
    hal_trace_init();
#endif
    hal_trace_init();

#ifndef _WITHOUT_SHALIB_
   res = shalinfo_deviceinfo_get(&deviceinfo_ptr);

    if(ee_res_OK != res)
    {
        ;//errore grave!!!!!Ferma tutto!!!
    }

    //forse e' suff un memcopy, perche' le due struct hanno la stessa struttura
    // e non dovrebbero contenere buchi, ma per ora non me la rischio!! :)
    ipnet_cfg_addr.macaddr = deviceinfo_ptr->ipnetwork.macaddress;
    ipnet_cfg_addr.ipaddr = deviceinfo_ptr->ipnetwork.ipaddress;
    ipnet_cfg_addr.ipmask = deviceinfo_ptr->ipnetwork.ipnetmask;
#endif
//QUESTO E' DA TOGLIERE QUANDO SI LEGGE DA DEVICE INFO!!!
    ipnet_cfg_addr.macaddr = ipal_params_cfgMINE->eth_mac;
    ipnet_cfg_addr.ipaddr = ipal_params_cfgMINE->eth_ip;
    ipnet_cfg_addr.ipmask = ipal_params_cfgMINE->eth_mask;
    /* The application needs 3 socket:
        - controlmessage (SUPERVISONE)
        - one for "normal communication", that is motor controller message + message for can-board
        - messages that need immediate response
    */
    ipnet_cfg_dtgskt.numberofsockets = 3; 
    ipnet_cfg_dtgskt.maxdatagramenqueuedintx = 8; //da vedere..ho messo default

    // start the ipnet
    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg, //da rivedere per ora ho messo default...
                         (ipal_params_cfg_t*)ipal_params_cfgMINE, 
                         &ipnet_cfg_addr, 
                         &ipnet_cfg_dtgskt
                         );
    //se avviene errore nella creazione di ipnet singleton, e' la funz stessa a chiamare errorManager??

    sysCntr_task = eom_task_New(eom_mtask_EventDriven,
                                  69,//Priorita' del task: da rivedere ora ho messo rpio a caso.
                                  3*1024, //stacksize: da rivedere
                                  sysController_startup, 
                                  sysController_run,  
                                  0, //message queue size. the task is evt based 
                                  eok_reltimeINFINITE, 
                                  sysController_task, 
                                  "sysController");


}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



