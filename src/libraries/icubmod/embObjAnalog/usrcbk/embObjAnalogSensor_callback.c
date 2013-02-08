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

#include "eOcfg_nvsEP_as_overridden.h"
#include "eOcfg_nvsEP_as_hid.h"


#ifdef _ICUB_CALLBACK_
//#include "IRobotInterface.h"
#include "FeatureInterface.h"
#endif


#define DEG_2_ICUBDEG  182.04444

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

// to use in caswe of get function with wait... correct the following wakeup function and build the wait table - it ssems not necessary for now
// void asWake(eOcfg_nvsEP_mc_jointNumber_t xx, const EOnv* nv, eOcfg_nvsEP_mc_jointNVindex_t nv_name)
// {
//   void *handler = (void*) get_MChandler_fromEP(nv->ep);
//   eOnvID_t nvid = eo_cfg_nvsEP_mc_joint_NVID_Get(nv->ep, xx, nv_name);  // ce ne battiamo il belino   // (??)
//   uint16_t epindex, nvindex;
//   EP_NV_2_index(nv->ep, nvid, &epindex, &nvindex);
//   MCmutex_post(handler, epindex, nvindex);
// }

// not necessary if default values are correct. check any-arch/sys/embobj/icub-nvscfg/ep-analogsensor/src/eOcfg_nvsEP_as_any_con_sxxdefault.c
// extern void eo_cfg_nvsEP_as_hid_INIT_Sxx_sstatus__fullscale(eOcfg_nvsEP_as_strainNumber_t sxx, const EOnv* nv)
// {
//     printf("size = %d\n",eo_array_Capacity(EOarray *nv);
// //     eo_array_Reset();
// }

static void handle_data(eOcfg_nvsEP_as_strainNumber_t sxx, const EOnv* nv, const eOabstime_t time, const uint32_t sign);

extern void eo_cfg_nvsEP_as_hid_UPDT_Sxx_sstatus__calibratedvalues(eOcfg_nvsEP_as_strainNumber_t sxx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#warning "strain status calib values update strong iCubInterface"
//     printf("iCub AS status calib values Callback\n");
    handle_data(sxx, nv, time, sign);
}

extern void eo_cfg_nvsEP_as_hid_UPDT_Sxx_sstatus__uncalibratedvalues(eOcfg_nvsEP_as_strainNumber_t sxx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#warning "strain status UNcalib values update strong iCubInterface"
//     printf("iCub AS status UNcalib values Callback\n");
    handle_data(sxx, nv, time, sign);
}

static void handle_data(eOcfg_nvsEP_as_strainNumber_t sxx, const EOnv* nv, const eOabstime_t time, const uint32_t sign)
{
#define _debug_as_data

    eOsnsr_arrayofupto12bytes_t *jstatus_b = nv->rem;

#ifdef _debug_as_data
//     printf("iCub AS status full Callback\n");
#endif
    FEAT_ID id;
    id.type = Skin;
    id.ep = nv->ep;

    int i=0;
/*  printf("0x");
    for( i=0; i<16; i++)
        printf("%02X", ((char*) jstatus_b)[i]);
    printf("\n\n");
*/
    handle_AS_data(&id, (void *)jstatus_b);
}
