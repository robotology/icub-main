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

/* @file       eOcfg_nvsEP_mc_usrcbk_pippo.c
    @brief     This file keeps examples for ems / pc104 of the user-defined functions used for all endpoints in mc
    @author     marco.accame@iit.it
    @date       05/02/2012
**/

//#warning --> read this very important comment on the different use of group or general callbacks
// very important comment:
// the EOtheAgent calls a default callback which is specific of a given _usr group (upperarm, lowerarm, torso, upperleg, lowerleg).
// inside this callback it is possible to understand which endpoint is (left or right) by evaluating argument ep.
// the default group callback then calls the default motioncontrol callback. the same mechanism is used to understand which is the relevant 
// endpoint.
// the two default callbacks do nothing else.
// the user can override one of the two by simply defining a new function with the same name and declaration.
// if the more specific group callback is redefined, then the more general is not called anymore for those groups which found this overriden.
// RULE TO USE: override only the general or the only the group ones. If the genral ones are overridden it is possible to write more general
//              code inside the ems.

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"
#include "stdint.h"

#include "EoCommon.h"
#include "EOnv_hid.h"

//#include "eOcfg_nvsEP_mc_overridden.h"
#include "eOcfg_nvsEP_mc_overridden.h"
#include "eOcfg_nvsEP_mc_hid.h"

#ifdef _ICUB_CALLBACK_
//#include "IRobotInterface.h"
#include "FeatureInterface.h"
#endif


//#include "eOcfg_nvsEP_mc_leg_con.h"
//#include "eOcfg_nvsEP_mc_upperleg_con.h"
//#include "eOcfg_nvsEP_mc_lowerleg_con.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



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





// this function is common to all mc endpoint. if we define this one and we DO NOT redefine eo_cfg_nvsEP_mc_XXXX_usr_hid_UPDT_Jxx_jcmmnds__calibration,
// where XXXX = [lowerleg, upperleg, lowerarm, upperarm, torso], then the orginal eo_cfg_nvsEP_mc_XXXX_usr_hid_UPDT_Jxx_jcmmnds__calibration() calls 
// the following eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jcmmnds__calibration() when the nv is changed by a rop.
// by redefining ONLY the more general eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jcmmnds__calibration(), every ems could use the very same code irrespectively of the
// motion control endpoints whcih has loaded.

// unfortunately, as eo_cfg_nvsEP_mc_lowerleg_usr_hid_UPDT_Jxx_jcmmnds__calibration() and eo_cfg_nvsEP_mc_upperleg_usr_hid_UPDT_Jxx_jcmmnds__calibration()
// have been re-defined in files eOcfg_nvsEP_mc_lowerleg_usrcbk_pippo.c and eOcfg_nvsEP_mc_upperleg_usrcbk_pippo.c, ...
// the following eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jcmmnds__calibration() is effectively called only on mc endpoints of arm and torso.
//
//extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jcmmnds__calibration(eo_cfg_nvsEP_mc_jointNumber_t jxx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
//{
    //eObool_t theOwnershipIsLocal = (NULL == nv->rem) ? eobool_true : eobool_false;
    //eOipv4addr_t ip; // = nv->ip;
    //eOnvEP_t ep = nv->ep;
    //
    //
    //jxx = jxx;                                              // the joint number 
    //theOwnershipIsLocal = theOwnershipIsLocal;              // always eobool_true in ems
    //ep = ep; 
    //ip = ip;                                                // the ip is EO_COMMON_IPV4ADDR_LOCALHOST if owneship is local    
    //
    //
    //// the ems processes this callback when it receives a command from the pc104. in nv->loc the ems retrieves the value to use
    //eOmc_calibrator_t *jcal = (eOmc_calibrator_t*)nv->loc;
    //
    //// use jcal
    //jcal = jcal;
    
//}

typedef struct
{
    uint32_t dummy1;
    uint32_t dummy2;
} MYmotionController;

static void motioncontroller_setpoint(MYmotionController *m, eOmc_setpoint_t *setp)
{
    switch(setp->type)
    {
        case eomc_setpoint_position:    
        {
            m->dummy1 = setp->to.position.value;
            m->dummy2 = setp->to.position.withvelocity;
        } break;
        
        default:
        {
            
        } break;
    }
}

static MYmotionController themotioncontrollers[3];

// the following function is called (nad NOT the more specific one) when a setpoint is received in all the mc endpoints of teh whole body.
// this function could be used to make the c code more generic inside the ems.
//extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jcmmnds__setpoint(eo_cfg_nvsEP_mc_jointNumber_t jxx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
//{
    //eObool_t theOwnershipIsLocal = (NULL == nv->rem) ? eobool_true : eobool_false;
    //eOipv4addr_t ip = nv->ip;
    //eOnvEP_t ep = nv->ep;
    //
    //
    //jxx = jxx;                                              // the joint number 
    //theOwnershipIsLocal = theOwnershipIsLocal;              // always eobool_true in ems
    //ep = ep; 
    //ip = ip;                                                // the ip is EO_COMMON_IPV4ADDR_LOCALHOST if owneship is local    
    //
    //
    //// the ems processes this callback when it receives a command from the pc104. in nv->loc the ems retrieves the value to use
    //eOmc_setpoint_t *jsetpoint = (eOmc_setpoint_t*)nv->loc;
    //
    //// use jsetpoint
    //switch(jsetpoint->type)
    //{
    //    case eomc_setpoint_position:    
    //    {
    //        // use with the position motionController on joint number jxx (from 0 to ....)
    //        eOmeas_position_t pos = jsetpoint->to.position.value;
    //        eOmeas_velocity_t vel = jsetpoint->to.position.withvelocity;
    //        pos = pos;
    //        vel  = vel;
    //        motioncontroller_setpoint(&themotioncontrollers[jxx], jsetpoint);
    //    } break;
    //
    //
    //} 
    //
//}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

void jwake(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, eOcfg_nvsEP_mc_jointNVindex_t nv_name)
{
	void *handler = (void*) get_MChandler_fromEP(nv->ep);
	eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get(nv->ep, xx, nv_name);
	uint16_t epindex, nvindex;
	EP_NV_2_index(nv->ep, nvid, &epindex, &nvindex);
	MCmutex_post(handler, epindex, nvindex);
}

void mwake(eOcfg_nvsEP_mc_motorNumber_t xx, const EOnv* nv,  eOcfg_nvsEP_mc_motorNVindex_t nv_name)
{
	void *handler = (void*) get_MChandler_fromEP(nv->ep);
	eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get(nv->ep, xx, nv_name);
	uint16_t epindex, nvindex;
	EP_NV_2_index(nv->ep, nvid, &epindex, &nvindex);
	MCmutex_post(handler, epindex, nvindex);
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jstatus__basic(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#warning "joint status basic strong iCubInterface"
#define _debug_jstatus_basic_
#ifdef _debug_jstatus_basic_
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
		eOmc_joint_status_basic_t *jstatus_b = nv->rem;
		printf("\njstatus__basic for Joint num = %d\n", xx);
		printf("ep = 0x%X\n", nv->ep);
		printf("jstatus_b->acceleration = 0x%X\n", jstatus_b->acceleration);
		printf("jstatus_b->controlmodestatus = 0x%X\n", jstatus_b->controlmodestatus);
		printf("jstatus_b->motionmonitorstatus = 0x%X\n", jstatus_b->motionmonitorstatus);
		printf("jstatus_b->position = 0x%X\n", jstatus_b->position);
		printf("jstatus_b->torque = 0x%X\n", jstatus_b->torque);
		printf("jstatus_b->velocity = 0x%X\n", jstatus_b->velocity);
	}

#endif
}
extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jstatus(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#warning "joint status full strong iCubInterface"
#define _debug_jstatus_full_
#ifdef  _debug_jstatus_full_
#define T 500

//	static int i = 0;
//	static bool print = false;
//	static int  j=-1;
//
//	if(-1 == j)
//		j = xx;
//
////	transceiver_wait(nv->ep);
//	if( (print) && (i>= T) && (xx == j) )
//	{
//		i = 0;
//		print = false;
//		printf("\n");
//	}
//
//	if(xx == j)
//		i++;
//
//	if(i >= T)
//	{
//		print = true;
//	}
//
//	if(print)
//	{
//		eOmc_joint_status_t *jstatus = nv->rem;
//		printf("-->joint status full for joint num = %d\n", xx);
////		printf("ep = 0x%X\n", nv->ep);
//		printf("jStatus->mode     = 0x%X\n", jstatus->basic.controlmodestatus);
////		printf("jStatus->pid->Ref = 0x%X\n", jstatus->ofpid.reference);
//		printf("jStatus->position = 0x%X\n", jstatus->basic.position);
////		printf("jStatus->velocity = 0x%X\n", jstatus->basic.velocity);
//	}
//	transceiver_post(nv->ep);

#endif
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Mxx_mstatus__basic(eOcfg_nvsEP_mc_motorNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#warning "motor status basic strong iCubInterface"
//#define _debug_mstatus_basic_
#ifdef _debug_mstatus_basic_
	static int i = 0;
	static bool print = false;

	transceiver_wait(nv->ep);

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
		eOmc_motor_status_basic_t *mstatus_b = nv->rem;
//		printf("mstatus__basic for motor num = %d\n", xx);
//		printf("ep = 0x%X\n", nv->ep);
//		printf("mstatus_b->current  = 0x%X\n", mstatus_b->current);
//		printf("mstatus_b->filler02 = 0x%X\n", mstatus_b->filler02);
		printf("mstatus_b->position = 0x%X\n", mstatus_b->position);
//		printf("mstatus_b->velocity = 0x%X\n", mstatus_b->velocity);
	}
	transceiver_post(nv->ep);
#endif
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#ifdef _debug_jxx_jconfig_
	transceiver_wait(nv->ep);
	eOmc_joint_config_t *jConfig = nv->rem;
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
	printf("ep = 0x%X\n", nv->ep);
	transceiver_post(nv->ep);
#endif
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Mxx_mconfig(eOcfg_nvsEP_mc_motorNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#ifdef _debug_mxx_mconfig_
	transceiver_wait(nv->ep);
	eOmc_motor_config_t *jConfig = nv->rem;
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
	printf("ep = 0x%X\n", nv->ep);
	transceiver_post(nv->ep);
#endif
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__maxpositionofjoint(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
	transceiver_wait(nv->ep);
	eOmeas_position_t *jMaxPosition_b = nv->rem;
	printf("\nmaxpositionofjoint for Joint num = %d ", xx);
	printf("pos = %d\n", *jMaxPosition_b);
	transceiver_post(nv->ep);
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__minpositionofjoint(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
	transceiver_wait(nv->ep);
	eOmeas_position_t *jMinPosition_b = nv->rem;
	printf("\nminpositionofjoint for Joint num = %d ", xx);
	printf("pos = %d\n", *jMinPosition_b);
	transceiver_post(nv->ep);
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Mxx_mconfig__maxcurrentofmotor(eOcfg_nvsEP_mc_motorNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
	transceiver_wait(nv->ep);
	eOmeas_position_t *jMaxCurrent_b = nv->rem;
	printf("\nmaxcurrentofmotor for Joint num = %d\n", xx);
	printf("ep = 0x%X\n", nv->ep);
	printf("maxcurrentofmotor = 0x%X\n", jMaxCurrent_b);
	transceiver_post(nv->ep);
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__pidposition(eOcfg_nvsEP_mc_motorNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#warning "pidposition strong iCubInterface"
	printf("jconfig__pidposition Callback eoMotionControl, j=%d ne'\n", xx);
	transceiver_wait(nv->ep);
	jwake(xx, nv, jointNVindex_jconfig__pidposition);
	transceiver_post(nv->ep);
}

extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__controlmode(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
	printf("jconfig__controlmode, j=%d\n", xx);
	transceiver_wait(nv->ep);
//	jwake(xx, nv, jointNVindex_jcmmnds__controlmode);	//???
	transceiver_post(nv->ep);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




