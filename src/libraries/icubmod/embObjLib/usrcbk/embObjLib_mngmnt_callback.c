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

#include "EoBoards.h"

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

typedef struct
{
    uint32_t    sec;
    uint32_t    msec;
    uint32_t    usec;
} timeofmessage_t;


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_print_string(char *str, eOmn_info_type_t errortype);

static void s_eoprot_print_mninfo_status(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);

static void s_process_CANPRINT(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);

static void s_process_category_Default(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);

static void s_process_category_Config(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);

static void s_get_timeofmessage(eOmn_info_basic_t* infobasic, timeofmessage_t *t);

static const char * s_get_sourceofmessage(eOmn_info_basic_t* infobasic, uint8_t *address);

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
        if(eobool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            char str[256] = {0};
            char nvinfo[128];
            char ipinfo[20];
            eoprot_ID2information(rd->id32, nvinfo, sizeof(nvinfo));
            eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo, sizeof(ipinfo));
            snprintf(str, sizeof(str), "eoprot_fun_ONSAY_mn() received an unexpected message w/ 0xaa000000 signature for IP %s and NV %s", ipinfo, nvinfo);
            feat_PrintWarning(str);
            return;
       }
    }
}



extern void eoprot_fun_UPDT_mn_info_status(const EOnv* nv, const eOropdescriptor_t* rd)
{   // callback used to print diagnostics sent by eth boards in full form (with strings)
    eOmn_info_status_t* infostatus = (eOmn_info_status_t*) rd->data;
    s_eoprot_print_mninfo_status(&infostatus->basic, infostatus->extra, nv, rd);
}


extern void eoprot_fun_UPDT_mn_info_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{   // callback used to print diagnostics sent by eth boards in compact form
    eOmn_info_basic_t* infostatusbasic = (eOmn_info_basic_t*) rd->data;
    s_eoprot_print_mninfo_status(infostatusbasic, NULL, nv, rd);
}


extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 19 mar 2014: the ethResource class sends a set<command, value> and it blocks to wait a reply.
    // the reply arrives in the form sig<command, value>. the signature in here does not work, as it works only with
    // ask<> / say<>.

    if(eo_ropcode_sig == rd->ropcode)
    {   // in here we have a sig and we cannot have the 0xaa000000 signature
        if(eobool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            feat_PrintError("eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() has received an unexpected message");
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
        if(eobool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            feat_PrintError("eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray() has received an unexpected message");
            return;
        }
    }

}

extern void eoprot_fun_UPDT_mn_service_status_commandresult(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {   // in here we have a sig
        if(eobool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            feat_PrintError("eoprot_fun_UPDT_mn_service_status_commandresult() has received an unexpected message");
            return;
        }
    }
    else
    {
        feat_PrintError("eoprot_fun_UPDT_mn_service_status_commandresult() has received an unexpected opcode");
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
    eOerror_category_t category = eoerror_code2category(infobasic->properties.code);

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

    if(codecanprint == infobasic->properties.code)
    {
        s_process_CANPRINT(infobasic, extra, nv, rd);
    }
    else if(eoerror_category_Config == category)
    {
        s_process_category_Config(infobasic, extra, nv, rd);
    }
    else
    {
        s_process_category_Default(infobasic, extra, nv, rd);
    }

}


static void s_process_category_Default(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{
    char str[512] = {0};

    static const char nullverbalextra[] = "no extra info despite we are in verbal mode";
    static const char emptyextra[] = ".";
    timeofmessage_t tom = {0};
    s_get_timeofmessage(infobasic, &tom);


    eOmn_info_type_t    type        = EOMN_INFO_PROPERTIES_FLAGS_get_type(infobasic->properties.flags);
    uint8_t address = 0;
    eOmn_info_extraformat_t extraf  = EOMN_INFO_PROPERTIES_FLAGS_get_extraformat(infobasic->properties.flags);
    //uint16_t forfutureuse           = EOMN_INFO_PROPERTIES_FLAGS_get_futureuse(infobasic->properties.flags);

    const char * str_source = NULL;
    const char * str_code = NULL;
    const char * str_extra = NULL;
    uint8_t *p64 = NULL;

    str_source         = s_get_sourceofmessage(infobasic, &address);
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

    char ipinfo[20] = {0};
    eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo, sizeof(ipinfo));
    //int boardnum = eo_nv_GetBRD(nv)+1;
    const char *boardstr = feat_GetBoardName(eo_nv_GetIP(nv));


    snprintf(str, sizeof(str), " from BOARD %s (%s), src %s, adr %d, time %ds %dm %du: (code 0x%.8x, par16 0x%.4x par64 0x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x) -> %s + %s",
                                ipinfo,
                                boardstr,
                                str_source,
                                address,
                                tom.sec, tom.msec, tom.usec,
                                infobasic->properties.code,
                                infobasic->properties.par16,
                                p64[7], p64[6], p64[5], p64[4], p64[3], p64[2], p64[1], p64[0],
                                str_code,
                                str_extra
                                );

    s_print_string(str, type);
}


static void s_print_string(char *str, eOmn_info_type_t errortype)
{
    switch(errortype)
    {
        case eomn_info_type_info:
        {
            feat_PrintInfo(str);
        } break;

        case eomn_info_type_debug:
        {
            feat_PrintDebug(str);
        } break;

        case eomn_info_type_warning:
        {
            feat_PrintWarning(str);
        } break;

        case eomn_info_type_error:
        {
            feat_PrintError(str);
        } break;

        case eomn_info_type_fatal:
        {
            feat_PrintFatal(str);
        } break;

        default:
        {
            feat_PrintError(str);
        } break;
    }

}



static void s_process_CANPRINT(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{
    feat_CANprint(eo_nv_GetIP(nv), infobasic);
}

//#warning --> do a proper function in EoBoards.c
const char * eoboard_get_name(eObrd_cantype_t type)
{
    static const char * names[] =
    {
        "UNKNOWN", "UNKNOWN", "UNKNOWN",
        "MC4",
        "UNKNOWN",
        "MTB",
        "STRAIN",
        "MAIS",
        "FOC",
        "ERROR09", "ERROR10"
    };
    static const char *none = "NONE";


    if(type <= eobrd_cantype_1foc)
    {
        return(names[type]);
    }
    else if(eobrd_cantype_none == type)
    {
        return(none);
    }
    else
    {
        return(names[0]);
    }

}

static void s_process_category_Config(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{
    char str[512] = {0};


    // here is the basic oscure print
    //s_process_category_Default(infobasic, extra, nv, rd);

    // but now we do a better parsing

    eOmn_info_type_t type = EOMN_INFO_PROPERTIES_FLAGS_get_type(infobasic->properties.flags);
    //int ethboardnum = eo_nv_GetBRD(nv)+1;
    char ipinfo[20] = {0};
    eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo, sizeof(ipinfo));
    const char *ethboardname = feat_GetBoardName(eo_nv_GetIP(nv));
    timeofmessage_t tom = {0};
    s_get_timeofmessage(infobasic, &tom);

    eOerror_value_t value = eoerror_code2value(infobasic->properties.code);

    switch(value)
    {
        case eoerror_value_CFG_candiscovery_ok:
        {
            uint8_t num = infobasic->properties.par16 & 0x000f;
            const char *canboardname = eoboard_get_name(infobasic->properties.par16 >> 8);
            uint64_t searchtime = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            eOmn_version_t prot = {0};
            eOmn_version_t appl = {0};
            uint64_t reqpr = (infobasic->properties.par64 & 0x00000000ffff0000) >> 16;
            uint64_t reqfw = (infobasic->properties.par64 & 0x000000000000ffff);
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = reqfw >> 8;
            appl.minor = reqfw & 0xff;

            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: successful CAN discovery of %d %s boards with target can protocol ver %d.%d and application ver %d.%d. Search time was %d ms",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        num, canboardname,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor,
                                        (int)searchtime
                                        );

            s_print_string(str, type);

        } break;

        case eoerror_value_CFG_candiscovery_detectedboard:
        {
            const char *canboardname = eoboard_get_name(infobasic->properties.par16 >> 8);
            uint64_t searchtime = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            eOmn_version_t prot = {0};
            eOmn_version_t appl = {0};
            uint64_t reqpr = (infobasic->properties.par64 & 0x00000000ffff0000) >> 16;
            uint64_t reqfw = (infobasic->properties.par64 & 0x000000000000ffff);
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = reqfw >> 8;
            appl.minor = reqfw & 0xff;
            uint8_t address = infobasic->properties.par16 & 0x000f;
            const char *source = s_get_sourceofmessage(infobasic, NULL);


            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery has detected a %s board in %s addr %d with can protocol ver %d.%d and application ver %d.%d. Search time was %d ms",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        canboardname,
                                        source, address,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor,
                                        (int)searchtime
                                        );
            s_print_string(str, type);

        } break;

        case eoerror_value_CFG_candiscovery_boardsmissing:
        {
            uint8_t numofmissing = infobasic->properties.par16 & 0x00ff;
            const char *canboardname = eoboard_get_name(infobasic->properties.par16 >> 8);
            uint64_t searchtime = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            uint16_t maskofmissing = infobasic->properties.par64 & 0x000000000000ffff;
            const char *source = s_get_sourceofmessage(infobasic, NULL);


            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery after %d ms has detected %d missing %s boards in %s:",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        (int)searchtime,
                                        numofmissing,
                                        canboardname,
                                        source
                                        );
            s_print_string(str, type);


            uint8_t n = 1;
            uint8_t i = 0;
            for(i=1; i<15; i++)
            {
                if(eobool_true == eo_common_hlfword_bitcheck(maskofmissing, i))
                {
                    snprintf(str, sizeof(str), "%d of %d: missing %s BOARD %s:%s:%d",
                                                n, numofmissing, canboardname,
                                                ipinfo, source, i
                                                );
                    s_print_string(str, type);
                    n++;

                }
            }

        } break;

        case eoerror_value_CFG_candiscovery_boardsinvalid:
        {
            uint8_t numofinvalid = infobasic->properties.par16 & 0x00ff;
            const char *canboardname = eoboard_get_name(infobasic->properties.par16 >> 8);
            uint64_t invalidmask = infobasic->properties.par64;
            const char *source = s_get_sourceofmessage(infobasic, NULL);


            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery has detected %d invalid %s boards in %s:",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        numofinvalid,
                                        canboardname,
                                        source
                                        );
            s_print_string(str, type);


            uint8_t n = 1;
            uint8_t i = 0;
            const char *empty = "";
            const char *wrongtype = "WRONG BOARD TYPE";
            const char *wrongprot = "WRONG PROTOCOL VERSION";
            const char *wrongappl = "WRONG APPLICATION VERSION";
            for(i=1; i<15; i++)
            {
                uint64_t val = (invalidmask >> (4*i)) & 0x0f;
                if(0 != val)
                {
                    snprintf(str, sizeof(str), "%d of %d: wrong %s BOARD %s:%s:%d because it has: %s %s %s",
                                                n, numofinvalid, canboardname,
                                                ipinfo, source, i,
                                                ((val & 0x1) == 0x1) ? (wrongtype) : (empty),
                                                ((val & 0x2) == 0x2) ? (wrongappl) : (empty),
                                                ((val & 0x4) == 0x4) ? (wrongprot) : (empty)
                                                );
                    s_print_string(str, type);
                    n++;

                }
            }

        } break;

        default:
        {
            s_process_category_Default(infobasic, extra, nv, rd);

        } break;

        case EOERROR_VALUE_DUMMY:
        {
            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: unrecognised eoerror_category_Config error value:",
                                    ipinfo,
                                    ethboardname,
                                    tom.sec, tom.msec, tom.usec
                                    );
            s_print_string(str, type);

        } break;

    }

}


static void s_get_timeofmessage(eOmn_info_basic_t* infobasic, timeofmessage_t *t)
{
    t->sec  = infobasic->timestamp / 1000000;
    t->msec = (infobasic->timestamp % 1000000) / 1000;
    t->usec = infobasic->timestamp % 1000;
}


static const char * s_get_sourceofmessage(eOmn_info_basic_t* infobasic, uint8_t *address)
{
    static const char * const sourcenames[] =
    {
        "LOCAL",
        "CAN1",
        "CAN2",
        "UNKNOWN"
    };

    eOmn_info_source_t source = EOMN_INFO_PROPERTIES_FLAGS_get_source(infobasic->properties.flags);

    if(NULL != address)
    {
        *address = EOMN_INFO_PROPERTIES_FLAGS_get_address(infobasic->properties.flags);
    }

    return((source > eomn_info_source_can2) ? (sourcenames[3]) : (sourcenames[source]));
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



