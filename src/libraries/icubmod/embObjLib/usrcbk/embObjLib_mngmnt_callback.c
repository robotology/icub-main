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


static void s_eoprot_print_mninfo_status(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);


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



static void s_eoprot_print_mninfo_status(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{

#define DROPCODES_FROM_LIST
#define CAN_PRINT_PARSING

#if defined(DROPCODES_FROM_LIST)
    static const eOerror_code_t codes2drop_value[] =
    {
        EOERRORCODE(eoerror_category_System, eoerror_value_SYS_canservices_parsingfailure)
    };

    static const int codes2drop_number = sizeof(codes2drop_value) / sizeof(eOerror_code_t);

    int i;

    for(i=0; i<codes2drop_number; i++)
    {
        if(codes2drop_value[i] == infobasic->properties.code)
        {
            return;
        }
    }
#endif
    if (infobasic->properties.code == eoerror_value_SYS_canservices_canprint)
    {
#if defined(CAN_PRINT_PARSING)
     // FIRST VERSION
     /*
        static char can_message[11][256];
        //End of the message, I can print
        if(p64[0] == 0x86)
        {
            //Append the last characters
            uint8_t i;
            for (i = 2; i < (infobasic->properties.par16 + 2); i++)
            {
                char *ch = (char*)&p64[i];
                strncat (can_message[eo_nv_GetBRD(nv)], ch, sizeof(char));
            }
            snprintf(str, sizeof(str), " from BOARD %d, src %s, adr %d, time %ds %dm %du: (code 0x%.8x).CAN PRINT MESSAGE: %s",
                                    eo_nv_GetBRD(nv)+1,
                                    str_source,
                                    address,
                                    sec, msec, usec,
                                    infobasic->properties.code,
                                    can_message[eo_nv_GetBRD(nv)]
                                    );
            embObjPrintInfo(str);
            //Clear the string associated to the board
            memset(can_message[eo_nv_GetBRD(nv)], 0, sizeof(can_message[eo_nv_GetBRD(nv)]));
            return;
        }
        // Message in progress, I only append the characters
        else if(p64[0] == 0x06)
        {
            uint8_t i;
            for (i = 2; i < 8; i++)
            {
                char *ch = (char*)&p64[i];
                strncat(can_message[eo_nv_GetBRD(nv)], ch, sizeof(char));
            }
            return;
        }
    */
    feat_embObjCANPrintHandler(eo_nv_GetBRD(nv), infobasic);
    return;
#endif
    }

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

    char str[256] = {0};
    static const char * sourcestrings[] =
    {
        "LOCAL",
        "CAN1",
        "CAN2",
        "UNKNOWN"
    };

    static const char nullverbalextra[] = "no extra info despite we are in verbal mode";
    static const char emptyextra[] = "NO MORE";

    uint32_t sec = infobasic->timestamp / 1000000;
    uint32_t msec = (infobasic->timestamp % 1000000) / 1000;
    uint32_t usec = infobasic->timestamp % 1000;

    eOmn_info_type_t    type        = EOMN_INFO_PROPERTIES_FLAGS_get_type(infobasic->properties.flags);
    eOmn_info_source_t  source      = EOMN_INFO_PROPERTIES_FLAGS_get_source(infobasic->properties.flags);
    uint16_t address                = EOMN_INFO_PROPERTIES_FLAGS_get_address(infobasic->properties.flags);
    eOmn_info_extraformat_t extraf  = EOMN_INFO_PROPERTIES_FLAGS_get_extraformat(infobasic->properties.flags);
    uint16_t forfutureuse           = EOMN_INFO_PROPERTIES_FLAGS_get_futureuse(infobasic->properties.flags);

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


    uint8_t *p64 = (uint8_t*)&(infobasic->properties.par64);

    snprintf(str, sizeof(str), " from BOARD %d, src %s, adr %d, time %ds %dm %du: (code 0x%.8x, par16 0x%.4x par64 0x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x) -> %s + INFO = %s",
                                eo_nv_GetBRD(nv)+1,
                                str_source,
                                address,
                                sec, msec, usec,
                                infobasic->properties.code,
                                infobasic->properties.par16,
                                p64[7], p64[6], p64[5], p64[4], p64[3], p64[2], p64[1], p64[0],
                                str_code,
                                str_extra
                                );


    if(type == eomn_info_type_debug)
    {
        embObjPrintDebug(str);
    }

    if(type == eomn_info_type_info)
    {
        embObjPrintInfo(str);
    }

    if(type == eomn_info_type_warning)
    {
        embObjPrintWarning(str);
    }

    if(type == eomn_info_type_error)
    {
        embObjPrintError(str);
    }

    if(type == eomn_info_type_fatal)
    {
        embObjPrintFatal(str);
    }
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



