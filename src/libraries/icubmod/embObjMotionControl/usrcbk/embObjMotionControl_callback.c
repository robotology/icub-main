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

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"

#include "EoCommon.h"
#include "EOnv.h"
#include "EoProtocol.h"
#include "EoProtocolMC.h"

#include "EoProtocolMC_overridden_fun.h"
#include <FeatureInterface.h>


#define DEG_2_ICUBDEG  182.04444


static void wake(const EOnv* nv);

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------

static void wake(const EOnv* nv)
{
    eOprotID32_t protoid;
    void *handler = (void*) get_MChandler_fromEP(nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv)), eo_nv_GetEP8(nv));
    if(NULL == handler)
    {
        printf("eoMC class not found\n");
        return;
    }
    
    protoid = eo_nv_GetID32(nv);
    eOprotProgNumber_t  prognum = eoprot_endpoint_id2prognum(eo_nv_GetBRD(nv), protoid);
    MCmutex_post(handler, prognum);
}


extern void eoprot_fun_UPDT_mc_joint_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
    
    if(eo_ropcode_say == rd->ropcode)
    {
        // This is an answer to a specific question, so wake up someone!
        wake(nv);
    }

    // callback - do the update anyway, both if broadcasted and if reply:
    FEAT_ID id;
    id.ep = eo_nv_GetEP8(nv);
    id.type = MotionControl;
    id.boardNum =  nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv));
    int jointNum = (int) xx;
    addEncoderTimeStamp(&id, jointNum);


    // debug:
#undef _debug_jstatus_basic_
#ifdef _debug_jstatus_basic_
    static int i = 0;
    static uint8_t print = 0;

    if( (1 == print) && (i>= 2000) && (xx == 0) )
    {
        i = 0;
        print = 0;
    }

    if(xx == 0)
        i++;

    if(i >= 2000)
    {
        print = 1;
    }

    if(print)
    {
        eOmc_joint_status_basic_t *jstatus_b = (eOmc_joint_status_basic_t*)rd->data;
        printf("\njstatus__basic for Joint num = %d\n", xx);
        printf("ep = 0x%X\n", eo_nv_GetEP8(nv));
        printf("jstatus_b->acceleration = 0x%X\n", jstatus_b->acceleration);
        printf("jstatus_b->controlmodestatus = 0x%X\n", jstatus_b->controlmodestatus);
        printf("jstatus_b->motionmonitorstatus = 0x%X\n", jstatus_b->motionmonitorstatus);
        printf("jstatus_b->position = 0x%X\n", jstatus_b->position);
        printf("jstatus_b->torque = 0x%X\n", jstatus_b->torque);
        printf("jstatus_b->velocity = 0x%X\n", jstatus_b->velocity);
    }
#endif
}

extern void eoprot_fun_UPDT_mc_joint_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);



    // callback:
    FEAT_ID id;
    id.ep = eo_nv_GetEP8(nv);
    id.type = MotionControl;
    id.boardNum =  nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv));
    int jointNum = (int) xx;
    addEncoderTimeStamp(&id,  jointNum);

#ifdef _SETPOINT_TEST_
    // per test
    static setpoint_test_data_t *rec_test_data_ptr;
    eOmc_joint_status_t *jstatus = (eOmc_joint_status_t*)rd->data;
    uint8_t *aux = (uint8_t*)jstatus;
    rec_test_data_ptr = (setpoint_test_data_t *)&aux[16];
    check_received_debug_data(&id, jointNum, rec_test_data_ptr);
#endif
}

extern void eoprot_fun_UPDT_mc_motor_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);

#undef _debug_mstatus_basic_
#ifdef _debug_mstatus_basic_
    static int i = 0;
    static bool print = false;

    if( (print) && (i>= 2000) && (xx == 0) )
    {
        i = 0;
        print = false;
    }

    if(xx == 0)
        i++;

    if(i >= 2000)
    {
        print = true;
    }

    if(print)
    {
        eOmc_motor_status_basic_t *mstatus_b = (eOmc_motor_status_basic_t*)rd->data;
//		printf("mstatus__basic for motor num = %d\n", xx);
//		printf("id32 = 0x%X\n", rd->id32);
//		printf("mstatus_b->current  = 0x%X\n", mstatus_b->current);
//		printf("mstatus_b->filler02 = 0x%X\n", mstatus_b->filler02);
        printf("mstatus_b->position = 0x%X\n", mstatus_b->position);
//		printf("mstatus_b->velocity = 0x%X\n", mstatus_b->velocity);
    }
#endif
}

extern void eoprot_fun_UPDT_mc_joint_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
#ifdef _debug_jxx_jconfig_
    eOmc_joint_config_t *jConfig = (eOmc_joint_config_t*)rd->data;
    printf("\nmaxpositionofjoint for Joint num = %d\n", xx);
    printf("jConfig->pidposition.kp 	= 0x%X\n",	jConfig->pidposition.kp		);
    printf("jConfig->pidposition.ki 	= 0x%X\n",	jConfig->pidposition.ki		);
    printf("jConfig->pidposition.kd 	= 0x%X\n",		jConfig->pidposition.kd);
    printf("jConfig->pidposition.limitonintegral = 0x%X\n",	jConfig->pidposition.limitonintegral);
    printf("jConfig->pidposition.limitonoutput = 0x%X\n",	jConfig->pidposition.limitonoutput);
    printf("jConfig->pidposition.offset 	= 0x%X\n",		jConfig->pidposition.offset);
    printf("jConfig->pidposition.scale 		= 0x%X\n",	jConfig->pidposition.scale);
    printf("jConfig->minpositionofjoint		= 0x%X\n",	jConfig->minpositionofjoint);
    printf("jConfig->maxpositionofjoint		= 0x%X\n",	jConfig->maxpositionofjoint);
    printf("jConfig->controlmode			= 0x%X\n",	jConfig->motionmonitormode);
    printf("ep = 0x%X\n", eoprot_ID2endpoint(rd->id32));
#endif
}

extern void eoprot_fun_UPDT_mc_motor_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);

#undef _debug_mxx_mconfig_
#ifdef _debug_mxx_mconfig_
    eOmc_motor_config_t *jConfig = (eOmc_motor_config_t*)rd->data;
    printf("\nmaxpositionofjoint for Joint num = %d\n", xx);
    printf("mConfig->pidcurrent.kp 	= 0x%X\n",	jConfig->pidcurrent.kp		);
    printf("mConfig->pidcurrent.ki 	= 0x%X\n",	jConfig->pidcurrent.ki		);
    printf("mConfig->pidcurrent.kd 	= 0x%X\n",		jConfig->pidcurrent.kd);
    printf("mConfig->pidcurrent.limitonintegral = 0x%X\n",	jConfig->pidcurrent.limitonintegral);
    printf("mConfig->pidcurrent.limitonoutput = 0x%X\n",	jConfig->pidcurrent.limitonoutput);
    printf("mConfig->pidcurrent.offset 	= 0x%X\n",		jConfig->pidcurrent.offset);
    printf("mConfig->pidcurrent.scale 		= 0x%X\n",	jConfig->pidcurrent.scale);
    printf("mConfig->maxvelocityofmotor		= 0x%X\n",	jConfig->maxvelocityofmotor);
    printf("mConfig->maxcurrentofmotor		= 0x%X\n",	jConfig->maxcurrentofmotor);
    printf("id32 = 0x%X\n", rd->id32);
#endif
}



extern void eoprot_fun_UPDT_mc_joint_config_limitsofjoint(const EOnv* nv, const eOropdescriptor_t* rd)
{
    wake(nv);
}

extern void eoprot_fun_UPDT_mc_motor_config_maxcurrentofmotor(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // eOmeas_current_t *jMaxCurrent_b = (eOmeas_current_t*)rd->data;
}

extern void eoprot_fun_UPDT_mc_joint_config_pidposition(const EOnv* nv, const eOropdescriptor_t* rd)
{
    wake(nv);
}

extern void eoprot_fun_UPDT_mc_joint_config_pidtorque(const EOnv* nv, const eOropdescriptor_t* rd)
{
    wake(nv);
}


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

extern void eoprot_fun_UPDT_mc_joint_config_impedance(const EOnv* nv, const eOropdescriptor_t* rd)
{
    wake(nv);
}


extern void eoprot_fun_UPDT_mc_joint_status_interactionmodestatus(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_say == rd->ropcode)
    {
        // This is an answer to a specific question, so wake up someone!
        wake(nv);
    }
}


// eof

