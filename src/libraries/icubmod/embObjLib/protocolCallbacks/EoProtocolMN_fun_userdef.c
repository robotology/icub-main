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


/*  @file       EOProtocolMN_fun_userdef.c
    @brief      This file keeps callbacks used for MN protocol in icub-main
    @author     marco.accame@iit.it
    @date       22 mar 2016
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
    feat_signal_network_onsay(eo_nv_GetIP(nv), rd->id32, rd->signature);
}



extern void eoprot_fun_UPDT_mn_info_status(const EOnv* nv, const eOropdescriptor_t* rd)
{   // callback used to print diagnostics sent by eth boards in full form (with strings)
    if(eo_ropcode_sig == rd->ropcode)
    {
        eOmn_info_status_t* infostatus = (eOmn_info_status_t*) rd->data;
        s_eoprot_print_mninfo_status(&infostatus->basic, infostatus->extra, nv, rd);
    }
}


extern void eoprot_fun_UPDT_mn_info_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{   // callback used to print diagnostics sent by eth boards in compact form
    if(eo_ropcode_sig == rd->ropcode)
    {
        eOmn_info_basic_t* infostatusbasic = (eOmn_info_basic_t*) rd->data;
        s_eoprot_print_mninfo_status(infostatusbasic, NULL, nv, rd);
    }
}


extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        if(eobool_false == feat_signal_network_onsig(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            feat_PrintError("eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() has received an unexpected message");
            return;
        }
    }
    else
    {
        feat_PrintError("eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() has received an unexpected opcode");
    }
}


extern void eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        if(eobool_false == feat_signal_network_onsig(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            feat_PrintError("eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray() has received an unexpected message");
            return;
        }
    }
    else
    {
        feat_PrintError("eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray() has received an unexpected opcode");
    }
}

extern void eoprot_fun_UPDT_mn_service_status_commandresult(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        if(eobool_false == feat_signal_network_onsig(eo_nv_GetIP(nv), rd->id32, rd->signature))
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
    eOmn_info_type_t    type;
    uint8_t address = 0;
    eOmn_info_extraformat_t extraf;
    const char * str_source = NULL;
    const char * str_code = NULL;
    const char * str_extra = NULL;
    uint8_t *p64 = NULL;
    char ipinfo[20] = {0};
    const char *boardstr = feat_GetBoardName(eo_nv_GetIP(nv));

    static const char nullverbalextra[] = "no extra info despite we are in verbal mode";
    static const char emptyextra[] = ".";
    timeofmessage_t tom = {0};
    s_get_timeofmessage(infobasic, &tom);


    type    = EOMN_INFO_PROPERTIES_FLAGS_get_type(infobasic->properties.flags);
    extraf  = EOMN_INFO_PROPERTIES_FLAGS_get_extraformat(infobasic->properties.flags);
    //uint16_t forfutureuse           = EOMN_INFO_PROPERTIES_FLAGS_get_futureuse(infobasic->properties.flags);

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

    
    eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo, sizeof(ipinfo));
    //int boardnum = eo_nv_GetBRD(nv)+1;
    
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


static void s_process_category_Config(eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd)
{
    char str[512] = {0};
    eOmn_info_type_t type;
    char ipinfo[20] = {0};
    const char *ethboardname = feat_GetBoardName(eo_nv_GetIP(nv));
    timeofmessage_t tom = {0};
    eOerror_value_t value;
    // here is the basic oscure print
    //s_process_category_Default(infobasic, extra, nv, rd);

    // but now we do a better parsing

    type = EOMN_INFO_PROPERTIES_FLAGS_get_type(infobasic->properties.flags);
    //int ethboardnum = eo_nv_GetBRD(nv)+1;
    eo_common_ipv4addr_to_string(eo_nv_GetIP(nv), ipinfo, sizeof(ipinfo));
    s_get_timeofmessage(infobasic, &tom);

    value = eoerror_code2value(infobasic->properties.code);

    switch(value)
    {

        case eoerror_value_CFG_candiscovery_started:
        {
            uint16_t maskcan2 = infobasic->properties.par16;
            uint64_t brdnum =     (infobasic->properties.par64 & 0x0000ff0000000000) >> 40;
            const char *canboardname = eoboards_type2string(brdnum);
            uint16_t maskcan1 = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr =      (infobasic->properties.par64 & 0x000000ffff000000) >> 24;
            uint64_t reqfw =      (infobasic->properties.par64 & 0x0000000000ffffff);
            uint8_t num =0;
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 16) & 0xff;
            appl.minor = (reqfw >> 8)  & 0xff;
            appl.build = reqfw & 0xff;
            num = eo_common_hlfword_bitsetcount(maskcan1)+eo_common_hlfword_bitsetcount(maskcan2);

            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery has started for %d %s boards on (can1map, can2map) = (0x%.4x, 0x%.4x) with target can protocol ver %d.%d and application ver %d.%d.%d.",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        num, canboardname,
                                        maskcan1, maskcan2,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor, appl.build
                                        );

            s_print_string(str, type);

        } break;

        case eoerror_value_CFG_candiscovery_ok:
        {
            uint8_t num = infobasic->properties.par16 & 0x00ff;
            eObool_t fakesearch = (0x0000 == (infobasic->properties.par16 & 0xf000)) ? (eobool_false) : (eobool_true);
            uint64_t brdnum =     (infobasic->properties.par64 & 0x0000ff0000000000) >> 40;
            const char *canboardname = eoboards_type2string(brdnum);
            uint64_t searchtime = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr =      (infobasic->properties.par64 & 0x000000ffff000000) >> 24;
            uint64_t reqfw =      (infobasic->properties.par64 & 0x0000000000ffffff);
            char strOK[80] = "OK";

            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 16) & 0xff;
            appl.minor = (reqfw >> 8)  & 0xff;
            appl.build = reqfw & 0xff;

           
            if(eobool_true == fakesearch)
            {
                snprintf(strOK, sizeof(strOK), "OK but FAKE (without any control on CAN w/ get-fw-version<> message)");
            }

            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery is %s for %d %s boards with target can protocol ver %d.%d and application ver %d.%d.%d. Search time was %d ms",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,
                                        strOK,
                                        num, canboardname,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor, appl.build,
                                        (int)searchtime
                                        );

            s_print_string(str, type);

        } break;

        case eoerror_value_CFG_candiscovery_detectedboard:
        {
            uint64_t brdnum =     (infobasic->properties.par64 & 0x0000ff0000000000) >> 40;
            const char *canboardname = eoboards_type2string(brdnum);
            uint64_t searchtime = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr =      (infobasic->properties.par64 & 0x000000ffff000000) >> 24;
            uint64_t reqfw =      (infobasic->properties.par64 & 0x0000000000ffffff);
            uint8_t address;
            const char *source = s_get_sourceofmessage(infobasic, NULL);
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 16) & 0xff;
            appl.minor = (reqfw >> 8)  & 0xff;
            appl.build = reqfw & 0xff;
            address = infobasic->properties.par16 & 0x000f;


            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery has detected a %s board in %s addr %d with can protocol ver %d.%d and application ver %d.%d.%d Search time was %d ms",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        canboardname,
                                        source, address,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor, appl.build,
                                        (int)searchtime
                                        );
            s_print_string(str, type);

        } break;

        case eoerror_value_CFG_candiscovery_boardsmissing:
        {
            uint8_t numofmissing = infobasic->properties.par16 & 0x00ff;
            const char *canboardname = eoboards_type2string(infobasic->properties.par16 >> 8);
            uint64_t searchtime = (infobasic->properties.par64 & 0xffff000000000000) >> 48;
            uint16_t maskofmissing = infobasic->properties.par64 & 0x000000000000ffff;
            const char *source = s_get_sourceofmessage(infobasic, NULL);
            uint8_t n = 1;
            uint8_t i = 0;

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
            const char *canboardname = eoboards_type2string(infobasic->properties.par16 >> 8);
            uint64_t invalidmask = infobasic->properties.par64;
            const char *source = s_get_sourceofmessage(infobasic, NULL);
            uint8_t n = 1;
            uint8_t i = 0;
            const char *empty = "";
            const char *wrongtype = "WRONG BOARD TYPE";
            const char *wrongprot = "WRONG PROTOCOL VERSION";
            const char *wrongappl = "WRONG APPLICATION VERSION";

            snprintf(str, sizeof(str), " from BOARD %s (%s) @ %ds %dm %du: CAN discovery has detected %d invalid %s boards in %s:",
                                        ipinfo,
                                        ethboardname,
                                        tom.sec, tom.msec, tom.usec,

                                        numofinvalid,
                                        canboardname,
                                        source
                                        );
            s_print_string(str, type);



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



