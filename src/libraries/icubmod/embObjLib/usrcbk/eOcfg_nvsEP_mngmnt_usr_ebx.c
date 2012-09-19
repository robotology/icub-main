/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/* @file       eOcfg_nvsEP_mngmnt_usr_ebx.c
    @brief      This file keeps the user-defined local ...
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"

#include "EoCommon.h"

#include "eOcfg_nvsEP_mn.h"
#include "eOcfg_nvsEP_mn_hid.h"

#include "EOarray.h"
#include "EOnv_hid.h"

#include "EOtheBOARDtransceiver.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

//#include "eOcfg_nvsEP_mngmnt_usr_hid.h"

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_cfg_nvsEP_mngmnt_usr_ebx_generic_ropsigcfgassign(EOarray* array);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern void eo_cfg_nvsEP_mngmnt_usr_hid_INITIALISE(eOnvEP_t ep, void *loc, void *rem)
{
    eObool_t theOwnershipIsRemote = (NULL == rem) ? eobool_true : eobool_false;
    
//    memcpy(loc, eOcfg_nvsEP_mngmnt_defaultvalue, sizeof(eo_cfg_nvsEP_mngmnt_t));
//    if(NULL != rem)
//    {
//        memcpy(rem, eOcfg_nvsEP_mn_defaultvalue, sizeof(eo_cfg_nvsEP_mngmnt_t));
//    }
    
    // nothing else ...

}


extern void eo_cfg_nvsEP_mngmnt_usr_hid_INIT__ropsigcfgassign(uint16_t n, const EOnv* nv)
{
    eObool_t theOwnershipIsRemote = (NULL == nv->rem) ? eobool_true : eobool_false;
    
    EOarray* arrayloc = (EOarray*)nv->loc;
    EOarray* arrayrem = (EOarray*)nv->rem;
    
    if(eobool_false == theOwnershipIsRemote)
    {   // function is called from within the local board
        s_eo_cfg_nvsEP_mngmnt_usr_ebx_generic_ropsigcfgassign(arrayloc);  
    }
    else
    {   // function is called from within the remote host because it has initialised its data
        // it is possible to know which board owns the arrayrem from the ip address
        eOipv4addr_t ipaddress_of_remote_board = nv->ip;

    }    
}

extern void eo_cfg_nvsEP_mngmnt_usr_hid_UPDT__ropsigcfgcommand(uint16_t n, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
    eObool_t theOwnershipIsRemote = (NULL == nv->rem) ? eobool_true : eobool_false;
    
    EOarray* arrayloc = (EOarray*)nv->loc;
    EOarray* arrayrem = (EOarray*)nv->rem;
    
    if(eobool_false == theOwnershipIsRemote)
    {   // function is called from within the local board
        s_eo_cfg_nvsEP_mngmnt_usr_ebx_generic_ropsigcfgassign(arrayloc);  
    }
    else
    {   // function is called from within the remote host because it has received a say or a sig
        // it is possible to know which board has sent the say/sig by the ipaddress
        eOipv4addr_t ipaddress_of_remote_board = nv->ip;

    }    
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_cfg_nvsEP_mngmnt_usr_ebx_generic_ropsigcfgassign(EOarray* array)
{
    uint8_t size, i;
    eOropSIGcfg_t *sigcfg;
    eo_transceiver_ropinfo_t ropinfo;
    EOtransceiver* theems00transceiver; 

    eOresult_t res;
    
    if(NULL == (theems00transceiver = eo_boardtransceiver_GetHandle()))
    {
        return;
    }
    
    if((eo_array_ItemSize(array) != sizeof(eOropSIGcfg_t)) || (NUMOFROPSIGCFG != eo_array_Capacity(array)) || ((size = eo_array_Size(array)) > NUMOFROPSIGCFG))
    {
        return;
    }

    eo_transceiver_rop_regular_Clear(theems00transceiver);

    for(i=0; i<size; i++)
    {
        sigcfg = (eOropSIGcfg_t*)eo_array_At(array, i);
        //#warning --> so far the regular rops are sent with a eok_ropconfig_basic, that is to say: without time64bit
        ropinfo.ropcfg              = eok_ropconfig_basic;
        ropinfo.ropcfg.plustime     = sigcfg->plustime;
        ropinfo.ropcode             = eo_ropcode_sig;
        ropinfo.nvep                = sigcfg->ep;    
        ropinfo.nvid                = sigcfg->id;
        res = eo_transceiver_rop_regular_Load(theems00transceiver, &ropinfo);
        res = res;
    }

}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



