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


/*  @file       EOProtocolMC_fun_userdef.c
    @brief      This file keeps callbacks used for MC protocol in icub-main
    @author     marco.accame@iit.it
    @date       22 mar 2016
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "EoCommon.h"
#include "EOnv.h"
#include "EoProtocol.h"

#include <FeatureInterface.h>


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EoProtocolMC.h"



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty section



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void eoprot_fun_ONSAY_mc(const EOnv* nv, const eOropdescriptor_t* rd)
{
    feat_signal_network_onsay(eo_nv_GetIP(nv), rd->id32, rd->signature);
}


extern void eoprot_fun_UPDT_mc_joint_status_core(const EOnv* nv, const eOropdescriptor_t* rd)
{
    feat_manage_motioncontrol_data(eo_nv_GetIP(nv), rd->id32, (void *)rd->data);
}


extern void eoprot_fun_UPDT_mc_motor_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
}


extern void eoprot_fun_UPDT_mc_joint_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    feat_manage_motioncontrol_data(eo_nv_GetIP(nv), rd->id32, (void *)rd->data);
}


extern void eoprot_fun_UPDT_mc_joint_status_addinfo_multienc(const EOnv* nv, const eOropdescriptor_t* rd)
{
    feat_manage_motioncontrol_addinfo_multienc(eo_nv_GetIP(nv), rd->id32, (void *)rd->data);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



