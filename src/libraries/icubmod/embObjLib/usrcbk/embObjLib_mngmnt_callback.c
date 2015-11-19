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

extern void eoprot_fun_ONSAY_mn(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 18 mar 2014: this function is called when a say<id32, data> rop is received
    // and the id32 is about the management endpoint. this function is common to every board.
    // it is used this function and not another one because inside the hostTransceiver object it was called:
    // eoprot_config_onsay_endpoint_set(eoprot_endpoint_management, eoprot_fun_ONSAY_mn);

    // the aim of this function is to wake up a thread which is blocked because it has sent an ask<id32>
    // the wake up funtionality is implemented in one mode only:
    // a. in initialisation, someone sets some values and then reads them back.
    //    the read back sends an ask<id32, signature=0xaa000000>. in such a case the board sends back
    //    a say<id32, data, signature = 0xaa000000>. thus, if the received signature is 0xaa000000, then
    //    we must unblock using feat_signal_network_reply().

    if(0xaa000000 == rd->signature)
    {   // case a:
        if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
        {
            char str[256] = {0};
            char nvinfo[128];
            eoprot_ID2information(rd->id32, nvinfo, sizeof(nvinfo));
            snprintf(str, sizeof(str), "eoprot_fun_ONSAY_mn() received an unexpected message w/ 0xaa000000 signature for %s", nvinfo);
            embObjPrintWarning(str);
            return;
       }
    }
}




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

    //if running, print the control loop timings
    if (appstatus->currstate == 1)
    {
        snprintf(str, sizeof(str), "                        control loop timings: RX->%dus, DO->%dus, TX->%dus",
                                                            appstatus->cloop_timings[0],
                                                            appstatus->cloop_timings[1],
                                                            appstatus->cloop_timings[2]);

        printf("%s\n", str);
    }

    snprintf(str, sizeof(str), "                        state = %s", state);

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



extern void eoprot_fun_UPDT_mn_comm_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 19 mar 2014: the ethResource class sends a set<command, value> and it blocks to wait a reply.
    // the reply arrives in the form sig<command, value>. the signature in here does not work, as it works only with
    // ask<> / say<>.

    if(eo_ropcode_sig == rd->ropcode)
    {   // in here we have a sig and we cannot have the 0xaa000000 signature
        if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
        {
            printf("ERROR: eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() has received an unexpected message\n");
            return;
        }
    }

}

extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 19 mar 2014: the ethResource class sends a set<command, value> and it blocks to wait a reply.
    // the reply arrives in the form sig<command, value>. the signature in here does not work, as it works only with
    // ask<> / say<>.

    if(eo_ropcode_sig == rd->ropcode)
    {   // in here we have a sig and we cannot have the 0xaa000000 signature
        if(fakestdbool_false == feat_signal_network_reply(eo_nv_GetBRD(nv), rd->id32, rd->signature))
        {
            printf("ERROR: eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray() has received an unexpected message\n");
            return;
        }
    }

}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eoprot_print_mninfo_status(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{
#undef DROPCODES_FROM_LIST
#define CAN_PRINT_FULL_PARSING

    static const eOerror_code_t codecanprint = EOERRORCODE(eoerror_category_System, eoerror_value_SYS_canservices_canprint);

#if defined(DROPCODES_FROM_LIST)
    static const eOerror_code_t codes2drop_value[] =
    {
        EOERRORCODE(eoerror_category_System, eoerror_value_SYS_canservices_parsingfailure),
        EOERRORCODE(eoerror_category_System, eoerror_value_SYS_canservices_rxmaisbug),
        EOERRORCODE(eoerror_category_System, eoerror_value_SYS_canservices_rxfromwrongboard),
        EOERRORCODE(eoerror_category_System, eoerror_value_SYS_transceiver_rxseqnumber_error)
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

    {
        char str[512] = {0};
        static const char * const sourcestrings[] =
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
        
        const char * str_source = NULL;
        const char * str_code = NULL;
        const char * str_extra = NULL;
        uint8_t *p64 = NULL;
    
        str_source         = (source > eomn_info_source_can2) ? (sourcestrings[3]) : (sourcestrings[source]);
        str_code           = eoerror_code2string(infobasic->properties.code);
        str_extra          = NULL;
    
        if(eomn_info_extraformat_verbal == extraf)
        {
            str_extra = (NULL == extra) ? (nullverbalextra) : ((const char *)extra);
        }
        else
        {
            str_extra = emptyextra;
        }
    
        p64 = (uint8_t*)&(infobasic->properties.par64);

        int boardnum = eo_nv_GetBRD(nv)+1;
        const char *boardstr = feat_embObj_GetBoardName(eo_nv_GetBRD(nv));
    
        if(codecanprint == infobasic->properties.code)
        {
            uint16_t len = 0;
            char canframestring[7] = {0};
    
    #if defined(CAN_PRINT_FULL_PARSING)
            feat_embObjCANPrintHandler(eo_nv_GetBRD(nv), infobasic);
            return;
    #endif
            // it is a canprint: treat it in a particular way.
            // in first step: just print the 6 bytes (at most) of the payload: now on 03/03/15 we have implemented first step
            // in second step: do the same inside ethResources
            // in third step: inside ethResources it is called the proper class can_string_generic with one instance per can board.
            //                maybe to save memory, we can instantiate the can_string_generic in runtime only when the can board sends a canprint.
            //                this third step allows to concatenate the can print frames into a single message as robotInterface
            //                does with can-based robots
    
            len = infobasic->properties.par16;
            if((len > 2) && (len <=8))
            {
                // we have a valid canframe
                memcpy(canframestring, &p64[2], len-2);
                canframestring[len-2] = 0;  // string terminator
                snprintf(str, sizeof(str), " from BOARD 10.0.1.%d (%s), src %s, adr %d, time %ds %dm %du: CANPRINT: %s [size = %d, D0 = 0x%.2x, D1 = 0x%.2x]",
                                            boardnum,
                                            boardstr,
                                            str_source,
                                            address,
                                            sec, msec, usec,
                                            canframestring,
                                            len, p64[0], p64[1]
                                            );
            }
            else
            {
                snprintf(str, sizeof(str), " from BOARD 10.0.1.%d (%s), src %s, adr %d, time %ds %dm %du: CANPRINT is malformed (code 0x%.8x, par16 0x%.4x par64 0x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x) -> %s + INFO = %s",
                                            boardnum,
                                            boardstr,
                                            str_source,
                                            address,
                                            sec, msec, usec,
                                            infobasic->properties.code,
                                            infobasic->properties.par16,
                                            p64[7], p64[6], p64[5], p64[4], p64[3], p64[2], p64[1], p64[0],
                                            str_code,
                                            str_extra
                                            );
    
            }
    
    
        }
        else
        {   // treat it as the normal case
    
            snprintf(str, sizeof(str), " from BOARD 10.0.1.%d (%s), src %s, adr %d, time %ds %dm %du: (code 0x%.8x, par16 0x%.4x par64 0x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x) -> %s + INFO = %s",
                                        boardnum,
                                        boardstr,
                                        str_source,
                                        address,
                                        sec, msec, usec,
                                        infobasic->properties.code,
                                        infobasic->properties.par16,
                                        p64[7], p64[6], p64[5], p64[4], p64[3], p64[2], p64[1], p64[0],
                                        str_code,
                                        str_extra
                                        );
        }
    
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
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



