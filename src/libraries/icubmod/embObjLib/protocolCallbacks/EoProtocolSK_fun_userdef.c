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


/*  @file       EOProtocolSK_fun_userdef.c
    @brief      This file keeps callbacks used for SK protocol in icub-main
    @author     marco.accame@iit.it
    @date       22 mar 2016
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
    feat_signal_network_onsay(eo_nv_GetIP(nv), rd->id32, rd->signature);
}


extern void eoprot_fun_UPDT_sk_skin_status_arrayofcandata(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        EOarray* arrayof = (EOarray*)rd->data;
        uint8_t sizeofarray = eo_array_Size(arrayof);
        if(0 != sizeofarray)
        {
            feat_manage_skin_data(eo_nv_GetIP(nv), rd->id32, (void *)arrayof);
        }
    }
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------







