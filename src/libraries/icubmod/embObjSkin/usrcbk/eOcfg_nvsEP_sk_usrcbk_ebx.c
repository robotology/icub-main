/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
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

/* @file       eOcfg_nvsEP_sk_usrcbk_ebx.c
    @brief      This file keeps the user-defined functions used in every ems board ebx for skin endpoint
    @author     valentina.gaggero@iit.it
    @date       05/04/2012
 **/



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EoCommon.h"
#include "EoProtocol.h"
#include "EOnv.h"

#include "FeatureInterface.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EoProtocolSK.h"


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern functions
// --------------------------------------------------------------------------------------------------------------------

extern void eoprot_fun_ONSAY_sk(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 18 mar 2014: this function is called when a say<id32, data> rop is received
    // and the id32 is about the analog sensors endpoint. this function is common to every board.
    // it is used this function and not another one because inside the hostTransceiver object it was called:
    // eoprot_config_onsay_endpoint_set(eoprot_endpoint_analogsensors, eoprot_fun_ONSAY_as);

    // the aim of this function is to wake up a thread which is blocked because it has sent an ask<id32>
    // the wake up funtionality is implemented in one mode only:
    // a. in initialisation, embObjSkin sets some values and then reads them back.
    //    the read back sends an ask<id32, signature=0xaa000000>. in such a case the board sends back
    //    a say<id32, data, signature = 0xaa000000>. thus, if the received signature is 0xaa000000, then
    //    we must unblock using feat_signal_network_reply().

    if(0xaa000000 == rd->signature)
    {   // case a:
        if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            char str[256] = {0};
            char nvinfo[128];
            char ipinfo[2];
            eoprot_ID2information(rd->id32, nvinfo, sizeof(nvinfo));
            eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo);
            snprintf(str, sizeof(str), "eoprot_fun_ONSAY_sk() received an unexpected message w/ 0xaa000000 signature for IP %s and NV", ipinfo, nvinfo);
            embObjPrintWarning(str);
            return;
        }
    }
}



extern void eoprot_fun_UPDT_sk_skin_status_arrayofcandata(const EOnv* nv, const eOropdescriptor_t* rd)
{
    EOarray* arrayof = (EOarray*)rd->data;
    uint8_t sizeofarray = eo_array_Size(arrayof);
    if(0 != sizeofarray)
    {
        feat_manage_skin_data(eo_nv_GetIP(nv), rd->id32, (void *)arrayof);
    }
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------







