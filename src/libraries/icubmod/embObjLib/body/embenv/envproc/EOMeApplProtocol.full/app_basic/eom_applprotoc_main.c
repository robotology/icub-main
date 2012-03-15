
/* @file       eom_applprotoc_main.c
	@brief      This file keeps the main of an application using the embobj
	@author     marco.accame@iit.it
    @date       01/11/2012
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"

// abslayer 
#include "hal.h"
#include "hal_trace.h"
#include "osal.h"
#include "ipal.h"


// embobj  
#include "EoCommon.h"
#include "EOMtheSystem.h"
#include "EOVtheSystem.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrormanager.h"
#include "EOMtheIPnet.h"

#include "EOaction.h"
#include "EOpacket.h"
#include "EOMmutex.h"
#include "EOsocketDatagram.h"


// keeps info on appl
#include "eom_applprotoc_info.h"

// keeps specialisation proper to the board function
#include "eom_applprotoc_specialise.h"


#include "EOtheARMenvironment.h"
#include "EOVtheEnvironment.h"





// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section
 
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



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eom_applprotoc_main_init(void);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


int main(void)
{

    eom_sys_Initialise( &eom_applprotoc_info_syscfg,
                        NULL,                                           // mempool uses calloc
                        &eom_applprotoc_specialise_errcfg,              // errman
                        &eom_timerman_DefaultCfg,
                        &eom_callbackman_DefaultCfg
                      );  
    
    eom_sys_Start(eom_sys_GetHandle(), s_eom_applprotoc_main_init);

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


/** @fn         static void s_eom_applprotoc_main_init(void)
    @brief      It initialises services such arm-environment and tcp/ip. then it calls a specific specialisation 
                function.
    @details    bla bla bla.
 **/

static void s_eom_applprotoc_main_init(void)
{
    //uint8_t *ipaddr = (uint8_t*)&(eom_applprotoc_info_ipal_cfg->eth_ip);
    eOmipnet_cfg_addr_t* eomipnet_addr = NULL;
    const eEipnetwork_t *ipnet = NULL;

    
    // ----------------------------------------------------------------------------------------------------------------
    // 1. initialise eeprom and the arm-environmemnt

    hal_eeprom_init(hal_eeprom_i2c_01, NULL);
    eo_armenv_Initialise(&eom_applprotoc_info_modinfo, NULL);
    eov_env_SharedData_Synchronise(eo_armenv_GetHandle());



    // ----------------------------------------------------------------------------------------------------------------
    // 2. initialise the parameters for ipnet with params taken from the arm-environment (or from ipal-cfg)


    // retrieve the configuration for ipnetwork
#ifndef _FORCE_NETWORK_FROM_IPAL_CFG
    if(eores_OK == eov_env_IPnetwork_Get(eo_armenv_GetHandle(), &ipnet))
    {
        eomipnet_addr = (eOmipnet_cfg_addr_t*)ipnet;   //they have the same memory layout

        //ipaddr = (uint8_t*)&(eomipnet_addr->ipaddr);
    }

    else
#endif
    {
        eomipnet_addr = NULL;
        //ipaddr = (uint8_t*)&(eom_applprotoc_info_ipal_cfg->eth_ip);
    }


    // ----------------------------------------------------------------------------------------------------------------
    // 3. start the ipnet

    eom_ipnet_Initialise(&eom_ipnet_DefaultCfg,
                         eom_applprotoc_specialise_ipal_cfg, 
                         eomipnet_addr,
                         &eom_applprotoc_specialise_dtgskt_cfg
                         );



    // ----------------------------------------------------------------------------------------------------------------
    // 5. start task-upd-server

    eom_applprotoc_specialise_updserver_start();  


    // ----------------------------------------------------------------------------------------------------------------
    // 5. call specialisation function

    eom_applprotoc_specialise_otherthings();

}







// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



