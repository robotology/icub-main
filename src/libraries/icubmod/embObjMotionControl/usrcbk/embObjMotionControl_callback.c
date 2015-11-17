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

#undef ENABLE_DEBUG_CONTROLMODESTATUS


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void wake(const EOnv* nv);


#if defined(ENABLE_DEBUG_CONTROLMODESTATUS)
static eObool_t s_debug_is_controlmodestatus_tobemonitored(eOnvBRD_t board, uint8_t joint);
static void s_debug_monitor_controlmodestatus(eOnvBRD_t board, uint8_t joint, eOenum08_t ctrlmodestatus);
// max 4 boards: eb1, eb2, eb3, and eb4 (values 1 and 3)
// max 12 joints
// inital value is 0: eomc_controlmode_cmd_idle
// target boards are eb2 and eb4 (index 1 and 3)
// target joint is: 4:
static eOenum08_t s_debug_some_controlmodevalues[4][12] = {0};
#endif

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
        if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
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


extern void eoprot_fun_UPDT_mc_joint_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
#if defined(ENABLE_DEBUG_CONTROLMODESTATUS)
    eOmc_joint_status_basic_t *jsb = (eOmc_joint_status_basic_t*)rd->data;
    s_debug_monitor_controlmodestatus(eo_nv_GetBRD(nv), eoprot_ID2index(rd->id32), jsb->controlmodestatus);
#endif
    feat_manage_motioncontrol_data(nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv)), rd->id32, (void *)rd->data);
}


extern void eoprot_fun_UPDT_mc_joint_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
#if defined(ENABLE_DEBUG_CONTROLMODESTATUS)
    eOmc_joint_status_t *js = (eOmc_joint_status_t*)rd->data;
    s_debug_monitor_controlmodestatus(eo_nv_GetBRD(nv), eoprot_ID2index(rd->id32), js->basic.controlmodestatus);
#endif
    feat_manage_motioncontrol_data(nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv)), rd->id32, (void *)rd->data);
}


extern void eoprot_fun_UPDT_mc_motor_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
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

extern void eoprot_fun_UPDT_mc_motor_config_maxcurrentofmotor(const EOnv* nv, const eOropdescriptor_t* rd)
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

#if 0
#define DEG_2_ICUBDEG  182.04444
#warning --> marco.accame on 16 oct 2014: the reception of a rop containing a setpoint command is not a good practice. remove this mechanism. see comment below
// marco.accame: the pc104 should send set<id32_cmmnd_setpoint, value_cmmnd_setpoint> and never send ask<id32_cmmnd_setpoint>.
//               the cmmnd variables should be write-only ....
//               thus it should never receive say<id32_cmmnd_setpoint, value> or sig<id32_cmmnd_setpoint, value>.
//               the reason is that the ems board manages the cmmd by writing the value_cmmnd_setpoint in its ram, but then it does not guarantee
//               that the value stays unchanged. also, it may be that many sepoints of different kind are sent. in this case the last overwrite the previous,
//               best way to know is to have a status variable whcih keeps the last received setpoint of some kind.
extern void eoprot_fun_UPDT_mc_joint_cmmnds_setpoint(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOmc_setpoint_t *setpoint_got = (eOmc_setpoint_t*) rd->data;
    static int32_t zero = 360;
    static prev = 0;
    int32_t pos;

//	printf("Callback recv setpoint: \n");
    uint16_t *checkProg = (uint16_t*) setpoint_got;
//	printf("Prog Num %d\n", checkProg[1]);
    pos = (int32_t)(setpoint_got->to.position.value / DEG_2_ICUBDEG) + zero;
//	printf("position %d\n", pos);
//	printf("velocity %d\n", setpoint_got->to.position.withvelocity);

    if( (checkProg[1] - prev) != 1)
        printf(">>>>>>Missing packet!! prev was %d, actual is %d (missing 0x%04X) <<<<<\n", prev, checkProg[1], prev+1);

    prev = checkProg[1];
    wake(nv);
}
#endif


extern void eoprot_fun_UPDT_mc_joint_config_impedance(const EOnv* nv, const eOropdescriptor_t* rd)
{
}


extern void eoprot_fun_UPDT_mc_joint_status_interactionmodestatus(const EOnv* nv, const eOropdescriptor_t* rd)
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
    void *mchandler = (void*) feat_MC_handler_get(nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv)), eo_nv_GetID32(nv));
    if(NULL == mchandler)
    {
        printf("eoMC class not found\n");
        return;
    }

    id32 = eo_nv_GetID32(nv);
    prognum = eoprot_endpoint_id2prognum(eo_nv_GetBRD(nv), id32);
    if(fakestdbool_false == feat_MC_mutex_post(mchandler, prognum) )
    {
        char nvinfo[128];
        char str[256] = {0};
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        snprintf(str, sizeof(str),"while releasing mutex in BOARD %d for variable %s", eo_nv_GetBRD(nv)+1, nvinfo); 
        embObjPrintWarning(str);
    }
    else // marco.accame.debug
    {
        uint8_t brd = eo_nv_GetBRD(nv)+1;
        if((2==brd) || (4==brd))
        {
            if((eoprot_tag_mc_joint_status_ismotiondone == eoprot_ID2tag(id32)) && (eoprot_entity_mc_joint == eoprot_ID2entity(id32)))
            {
                eObool_t motiondone = *((eObool_t*)eo_nv_RAM(nv));
                printf("motiondone = %d for joint %d in BOARD %d", motiondone, eoprot_ID2index(id32), brd);
                //yDebug() << "motiondone = " << motiondone << "for joint " << eoprot_ID2index(id32) << "board" << brd;
            }
        }
    }
}



#if defined(ENABLE_DEBUG_CONTROLMODESTATUS)
static eObool_t s_debug_is_controlmodestatus_tobemonitored(eOnvBRD_t board, uint8_t joint)
{

    if((1 == board) && (4 == joint))
    {
        return(eobool_true);
    }
    if((3 == board) && (4 == joint))
    {
        return(eobool_true);
    }

    return(eobool_false);
}

static void s_debug_monitor_controlmodestatus(eOnvBRD_t board, uint8_t joint, eOenum08_t ctrlmodestatus)
{
    eOenum08_t *stored = NULL;

    if(eobool_false == s_debug_is_controlmodestatus_tobemonitored(board, joint))
    {
        return;
    }

    stored = &s_debug_some_controlmodevalues[board][joint];

    if(ctrlmodestatus != *stored)
    {
        // 1. print the values together with the current time
        double time = feat_yarp_time_now();

        printf("DEBUG: controlmodestatus changes %d -> %d at time %f\n", *stored, ctrlmodestatus, time);


        // 2. change the value
        *stored = ctrlmodestatus;
    }

}
#endif



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



