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
#include "EoError.h"


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

#if   defined(EOMANAGEMENT_USE_VER_2_3)
//
#elif   defined(EOMANAGEMENT_USE_VER_2_4)
static void s_eoprot_print_mninfo_status(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);
#else
    #error --> unspecified EOMANAGEMENT_USE_VER_2_x
#endif

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

    snprintf(str, sizeof(str), "MANAGEMENT-appl-status: sign = 0x%x, board EB%d -> name = %s", rd->signature, eo_nv_GetBRD(nv)+1, appstatus->name);
    printf("%s\n", str);

    snprintf(str, sizeof(str), "                        version = %d.%d, built on date %d %d %d, at hour %d:%d",
                                                        appstatus->version.major,
                                                        appstatus->version.minor,
                                                        appstatus->buildate.day,
                                                        appstatus->buildate.month,
                                                        appstatus->buildate.year,
                                                        appstatus->buildate.hour,
                                                        appstatus->buildate.min);

    printf("%s\n", str);

    snprintf(str, sizeof(str), "                        state = %s", state);

    printf("%s\n", str);


    fflush(stdout);

    if((eo_ropcode_say == rd->ropcode) && (0xaa000000 == rd->signature))
    {
        if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
        {
            printf("ERROR: eoprot_fun_UPDT_mn_appl_status() has received an unexpected message\n");
            return;
        }
    }

}


#if     defined(EOMANAGEMENT_USE_VER_2_3)

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


    const char * infotype[] =
    {
        "eomn_info_type_info",
        "eomn_info_type_debug",
        "eomn_info_type_warning",
        "eomn_info_type_error",
        "unknown"
    };

    const char * sss = (infostatus->properties.type > 3) ? (infotype[4]) : (infotype[infostatus->properties.type]);

    snprintf(str, sizeof(str), "[INFO]-> mn-info: ropsign = 0x%x, roptime = %04ds+%03dms+%03dus, BOARD = %d: -> info.status.properties.type = %s, info.status.data = %s", rd->signature, (uint32_t)sec, (uint32_t)msec, (uint32_t)usec, eo_nv_GetBRD(nv)+1, sss, infostatus->data);

    printf("%s\n", str);
    fflush(stdout);
}

#elif   defined(EOMANAGEMENT_USE_VER_2_4)


static void s_eoprot_print_mninfo_status(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{
    char str[256] = {0};

#if 0
    uint64_t txsec = rd->time / 1000000;
    uint64_t txmsec = (rd->time % 1000000) / 1000;
    uint64_t txusec = rd->time % 1000;
    if(1 == rd->control.plustime)
    {
        txsec = rd->time / 1000000;
        txmsec = (rd->time % 1000000) / 1000;
        txusec = rd->time % 1000;
    }
    else
    {
        txsec =  txmsec = txusec = 0;
    }
#endif

    static const char * typestrings[] =
    {
        "[INFO]",
        "[DEBUG]",
        "[WARNING]",
        "[ERROR]",
        "[FATAL]",
        "[UNKNOWN]"
    };

    static const char * sourcestrings[] =
    {
        "LOCAL",
        "CAN1",
        "CAN2",
        "UNKNOWN"
    };

    static const char nullverbalextra[] = "no extra info despite we are in verbal mode";
    static const char emptyextra[] = "extra info";

    uint32_t sec = infobasic->timestamp / 1000000;
    uint32_t msec = (infobasic->timestamp % 1000000) / 1000;
    uint32_t usec = infobasic->timestamp % 1000;

    eOmn_info_type_t    type        = EOMN_INFO_PROPERTIES_FLAGS_get_type(infobasic->properties.flags);
    eOmn_info_source_t  source      = EOMN_INFO_PROPERTIES_FLAGS_get_source(infobasic->properties.flags);
    uint16_t address                = EOMN_INFO_PROPERTIES_FLAGS_get_address(infobasic->properties.flags);
    eOmn_info_extraformat_t extraf  = EOMN_INFO_PROPERTIES_FLAGS_get_extraformat(infobasic->properties.flags);
    uint16_t forfutureuse           = EOMN_INFO_PROPERTIES_FLAGS_get_futureuse(infobasic->properties.flags);

    const char * str_type           = (type > eomn_info_type_fatal) ? (typestrings[5]) : (typestrings[type]);
    const char * str_source         = (source > eomn_info_source_can2) ? (sourcestrings[3]) : (sourcestrings[source]);;
    const char * str_code           = eoerror_code2string(infobasic->properties.code);
    const char * str_extra          = NULL;

    if(eomn_info_extraformat_verbal == extraf)
    {
        str_extra = (NULL == extra) ? (nullverbalextra) : ((const char *)extra);
    }
    else
    {
        str_extra = emptyextra;
    }


    snprintf(str, sizeof(str), "%s from BOARD %d, source %s, address %d, time %ds %dm %du: (code 0x%x, param 0x%x) -> %s + %s",
                                str_type,
                                eo_nv_GetBRD(nv)+1,
                                str_source,
                                address,
                                sec, msec, usec,
                                infobasic->properties.code,
                                infobasic->properties.param,
                                str_code,
                                str_extra
                                );
    printf("%s\n", str);
    fflush(stdout);
}



extern void eoprot_fun_UPDT_mn_info_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOmn_info_status_t* infostatus = (eOmn_info_status_t*) rd->data;

    s_eoprot_print_mninfo_status(&infostatus->basic, infostatus->extra, nv, rd);
}


extern void eoprot_fun_UPDT_mn_info_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOmn_info_basic_t* infostatusbasic = (eOmn_info_basic_t*) rd->data;

    s_eoprot_print_mninfo_status(infostatusbasic, NULL, nv, rd);
}

#else
    #error --> must specify a EOMANAGEMENT_USE_VER_2_x
#endif


extern void eoprot_fun_UPDT_mn_comm_status(const EOnv* nv, const eOropdescriptor_t* rd)
{

    if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
    {
        printf("ERROR: eoprot_fun_UPDT_mn_comm_status() has received an unexpected message\n");
        return;
    }
}

extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
    {
        printf("ERROR: eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() has received an unexpected message\n");
        return;
    }
}

extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
    {
        printf("ERROR: eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray() has received an unexpected message\n");
        return;
    }
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



