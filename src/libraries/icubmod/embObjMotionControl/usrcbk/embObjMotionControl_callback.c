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

static void wake(const EOnv* nv);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty section



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void eoprot_fun_ONSAY_mc(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 18 mar 2014: this function is called when a say<id32, data> rop is received
    // and the id32 is about the motion control endpoint. this function is common to every board.
    // it is used this function and not another one because inside the hostTransceiver object it was called:
    // eoprot_config_onsay_endpoint_set(eoprot_endpoint_motioncontrol, eoprot_fun_ONSAY_mc);

    // the aim of this function is to wake up a thread which is blocked because it has sent an ask<id32>
    // the wake up funtionality is implemented in two modes, depending on the wait mechanism used:
    // a. in initialisation, embObjMotionControl sets some values and then reads them back.
    //    the read back sends an ask<id32, signature=0xaa000000>. in such a case the board sends back
    //    a say<id32, data, signature = 0xaa000000>. thus, if the received signature is 0xaa000000, then
    //    we must unblock using feat_signal_network_reply().
    // b. during runtime, some methods send a blocking ask<id32> without signature. It is the case of instance
    //    of getPidRaw() which waits with a eoThreadEntry::synch() call. in such a case the board send back a
    //    normal say<id32, data> with nos signature. in this case we unlock with wake().


    if(0xaa000000 == rd->signature)
    {   // case a:
        if(eobool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            char str[256] = {0};
            char nvinfo[128];
            eoprot_ID2information(rd->id32, nvinfo, sizeof(nvinfo));
            snprintf(str, sizeof(str), "eoprot_fun_ONSAY_mc() received an unexpected message w/ 0xaa000000 signature for %s", nvinfo);
            embObjPrintWarning(str);
            return;
        }
    }
    else
    {   //case b:
        wake(nv);
    }
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

extern void eoprot_fun_UPDT_mc_joint_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mc_motor_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
}



extern void eoprot_fun_UPDT_mc_joint_config_limitsofjoint(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mc_motor_config_currentLimits(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mc_motor_config_gearboxratio(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mc_motor_config_rotorencoder(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mc_joint_config_pidposition(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mc_joint_config_pidtorque(const EOnv* nv, const eOropdescriptor_t* rd)
{
}



extern void eoprot_fun_UPDT_mc_joint_config_impedance(const EOnv* nv, const eOropdescriptor_t* rd)
{
}


extern void eoprot_fun_UPDT_mc_joint_status_core_modes_interactionmodestatus(const EOnv* nv, const eOropdescriptor_t* rd)
{
}



extern void eoprot_fun_UPDT_mc_joint_config_motor_params(const EOnv* nv, const eOropdescriptor_t* rd)
{
}


extern void eoprot_fun_UPDT_mc_controller_config_jointcoupling(const EOnv* nv, const eOropdescriptor_t* rd)
{
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------


static void wake(const EOnv* nv)
{
    eOprotID32_t id32 = 0;
    eOprotProgNumber_t prognum = 0 ;
    void *mchandler = (void*) feat_MC_handler_get(eo_nv_GetIP(nv), eo_nv_GetID32(nv));
    if(NULL == mchandler)
    {
        printf("eoMC class not found\n");
        return;
    }

    id32 = eo_nv_GetID32(nv);
    prognum = eoprot_endpoint_id2prognum(eo_nv_GetBRD(nv), id32);
    if(eobool_false == feat_MC_mutex_post(mchandler, prognum) )
    {
        char nvinfo[128];
        char ipinfo[20];
        char str[256] = {0};
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo, sizeof(ipinfo));
        snprintf(str, sizeof(str),"while releasing mutex for IP %s and NV %s", ipinfo, nvinfo);
        embObjPrintWarning(str);
    }

}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



