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
    @date       04/16/2014
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include <math.h>
#include "FeatureInterface.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EoProtocolMN.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


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
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

void eoprot_fun_UPDT_mn_appl_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    static const char* states[] =
    {
        "applstate_config",
        "applstate_running",
        "applstate_error",
        "not initted"
    };

    char str[256] = {0};

    eOmn_appl_status_t* appstatus = (eOmn_appl_status_t*) rd->data;

    const char* state = (appstatus->currstate > 2) ? (states[3]) : (states[appstatus->currstate]);

    snprintf(str, sizeof(str), "MANAGEMENT-appl-status: sign = 0x%x, board EB%d -> state = %s, msg = %s ", rd->signature, eo_nv_GetBRD(nv)+1, state, appstatus->filler06);

    printf("%s\n", str);
    fflush(stdout);
}


extern void eoprot_fun_UPDT_mn_info_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    char str[256] = {0};

    eOmn_info_status_t* infostatus = (eOmn_info_status_t*) rd->data;

    uint64_t sec = rd->time / 1000000;
    uint64_t msec = (rd->time % 1000000) / 1000;
    uint64_t usec = rd->time % 1000;
    if(1 == rd->control.plustime)
    {
        sec = rd->time / 1000000;
        msec = (rd->time % 1000000) / 1000;
        usec = rd->time % 1000;
    }
    else
    {
        sec =  msec = usec = 0;
    }

    snprintf(str, sizeof(str), "MANAGEMENT-info: sign = 0x%x, time = %04ds+%03dms+%03dus, board EB%d: -> info.status.type = %d, info.status.string = %s", rd->signature, (uint32_t)sec, (uint32_t)msec, (uint32_t)usec, eo_nv_GetBRD(nv)+1, infostatus->type, infostatus->string);

    printf("%s\n", str);
    fflush(stdout);
}



extern void eoprot_fun_UPDT_mn_comm_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    void* sem = NULL;
    //double yt = 0.0;

    sem = feat_GetSemaphore(eo_nv_GetBRD(nv), eoprot_ID2endpoint(rd->id32), rd->signature);

    if(NULL == sem)
    {
        printf("FATAL ERROR: eoprot_fun_UPDT_mn_comm_status() fails in getting a semaphore\n");
        return;
    }

    //yt = feat_yarp_time_now();
    //printf("reply_numof arrived at yarp time %f\n", yt);

    feat_Semaphore_post(sem);
}

extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof(const EOnv* nv, const eOropdescriptor_t* rd)
{
    void* sem = NULL;
    //double yt = 0.0;
    //eOmn_command_t* command = (eOmn_command_t*)rd->data;
    //eOmn_opc_t opc = (eOmn_opc_t)command->cmd.opc;


    sem = feat_GetSemaphore(eo_nv_GetBRD(nv), eoprot_ID2endpoint(rd->id32), rd->signature);
    
    if(NULL == sem)
    {
        printf("FATAL ERROR: eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() fails in getting a semaphore\n");        
        return;
    }


    //yt = feat_yarp_time_now();
    //printf("reply_numof arrived at yarp time %f\n", yt);

    feat_Semaphore_post(sem);
}

extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray(const EOnv* nv, const eOropdescriptor_t* rd)
{
    void* sem = NULL;

    //double yt = 0.0;
    //eOmn_command_t* command = (eOmn_command_t*)rd->data;
    //eOmn_opc_t opc = (eOmn_opc_t)command->cmd.opc;


    sem = feat_GetSemaphore(eo_nv_GetBRD(nv), eoprot_ID2endpoint(rd->id32), rd->signature);
    
    if(NULL == sem)
    {
        printf("FATAL ERROR: eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray() fails in getting a semaphore\n");        
        return;
    }

    //yt = feat_yarp_time_now();
    //printf("reply arrayarrived at yarp time %f\n", yt);

    feat_Semaphore_post(sem);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



