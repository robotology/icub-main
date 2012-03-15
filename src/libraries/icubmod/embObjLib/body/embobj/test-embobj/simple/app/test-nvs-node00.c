
/* @file       test-nvs-node00.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       09/08/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

// embobj
#include "EoCommon.h"
#include "EOnetvar.h"
#include "EOnetvar_hid.h"  // only for advanced operations


#include "EOrop_hid.h"

#include "EOtheNVsCfgNode00.h"


#include "EOVtheNVsCfg.h"
#include "EOtheNVs.h"
#include "EOtheParser.h"
#include "EOtheAgent.h"
#include "EOtheFormer.h"

#include "EOarray.h"

#include "EOvport.h"
#include "EOnetvarNode.h"






// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-nvs-node00.h"


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

static void s_initialise_all(void);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOrop *rop = NULL;
static EOrop *rop0 = NULL;
static EOrop *reply = NULL;

static uint8_t  pkt2tx_data[128] =  {0};
static uint16_t pkt2tx_size = 0;

static volatile uint32_t aaa32;
static volatile uint16_t aaa16;
static volatile uint8_t  aaa08;

static EOnetvar *nvInp08 = NULL;
static EOnetvar *nvInp16 = NULL;

static uint8_t  *pInp08 = NULL;
static uint16_t *pInp16 = NULL;


static EOnetvar *nvTMP = NULL;

static EOvportCfg *vportcfg = NULL;

static eOropconfig_t ropcfg = {.confrqst = eobool_true, .timerqst = eobool_true, .plussign = eobool_true, .plustime = eobool_true};
static uint32_t sign = 0x21000012;
static uint64_t time = 0xaaaabbbbccccdddd;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern void test_nvs_node00_Init(void)
{

//    const eOipv4addr_t ipaddr_smartnode = eo_common_ipv4addr(10, 10, 1, 100);
    const eOipv4addr_t ipaddr_simplenode00 = node00_loc_ipaddr; 


    s_initialise_all(); 
 

    for(;;)
    {
        // try to understand how a smart node process a reception of a say ...
        // answer: the agent copies the received data into ram of remote NVs and a callback function on_say() shoudl be
        // defined and called. the function is responsible to operate the reply.
        ;
    }


}



extern void test_nvs_node00_Do(void)
{

}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_initialise_all(void)
{
    // the network variables ...
    //eo_nvs_Initialise(eov_nvscfg_Initialise(eo_nvscfg_device_GetHandle()));
    eo_nvs_Initialise(eo_nvscfg_node00_GetHandle());
    
    // the parser
    eo_parser_Initialise();

    // the agent
    eo_agent_Initialise(NULL);
    
    // the former
    eo_former_Initialise();

    // some rops
    rop0 = eo_rop_New(128);
    rop = eo_rop_New(128);
    reply = eo_rop_New(128);

    // a vportcfg
    vportcfg = eo_vportcfg_New();

}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



