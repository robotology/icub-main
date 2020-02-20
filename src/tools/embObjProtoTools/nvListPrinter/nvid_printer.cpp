/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
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

/* @file       eOtheEthLowLevelParser.c
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/18/2013
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include <unistd.h>
#include <stdio.h>
#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"


#include"eOcfg_nvsEP_mn.h"
#include"eOcfg_nvsEP_mc.h"
#include"eOcfg_nvsEP_as.h"
#include"eOcfg_nvsEP_sk.h"

#include "eOcfg_nvsEP_mc_any_con_bodypart_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set()
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_print_help(void);
static uint16_t s_getEP_mc(uint8_t board );
static uint16_t s_getEP_as(uint8_t board );
static uint16_t s_getEP_sk(uint8_t board );
static uint16_t s_getEP_mn(uint8_t board );
static uint16_t s_getEP(uint8_t board, char *eptype);
static void s_print_mc_nvid_alljoints(void);
static void s_print_mc_nvid(eOmc_jointId_t j);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    uint8_t board = 0;
    eOmc_jointId_t j = 0xFF;
    char eptype[10];



    //1) init data
    sprintf(eptype, "mc");

    //2) parse arguments
    if(argc>1)
    {
        for(uint8_t i = 0; i<argc; i++)
        {
            if(strcmp("--board", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                    board = atoi(argv[++i]);
                }
                continue;
            }

            if(strcmp("--j", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                    j = atoi(argv[++i]);
                }
                continue;
            }

            if(strcmp("--ep", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                    sprintf(eptype, "%s", argv[++i]);
                }
                continue;
            }

            if(strcmp("--help", argv[i]) == 0)
            {
                s_print_help();
                continue;
            }

        }

    }

    if((board <1) || (board>9))
    {
        printf("ERROR: board has to belong to [1,9]\n");
	    return -1;
    }

    printf("------The following NVID are of baord %d ", board);
    if(j==0xFF)
    {
        printf("for all joint");
    }
    else
    {
        printf("of joint %d ", j);
    }

    printf("about %s-----\n\n", eptype);

    printf("\nENPOINT=0x%x\n\n", s_getEP(board, eptype));

    if(j == 0xFF)
    {
        s_print_mc_nvid_alljoints();
    }
    else
   {
        s_print_mc_nvid(j);
   }
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_print_help(void)
{
    printf("--board <n>: num of board\n");
    printf("--j <n>: num of joint. if not insert, nvid of all joint are printed\n");
    printf("--ep <str>:endpoint. str can be <mc>, or <as>, or <sk> or <mn>\n");
}

static uint16_t s_getEP(uint8_t board, char *eptype)
{
    uint16_t ep = 0;


    if(strcmp(eptype, "mc") == 0)
    {
         ep = s_getEP_mc(board);
    }
   else if(strcmp(eptype, "as") == 0)
    {
         ep = s_getEP_as(board);
    }
   else if(strcmp(eptype, "sk") == 0)
    {
         ep = s_getEP_sk(board);
    }
   else if(strcmp(eptype, "mn") == 0)
    {
         ep = s_getEP_mn(board);
    }
    else
    {
         ep = 0xFFFF;
    }

    return(ep);
}




static uint16_t s_getEP_mc(uint8_t board )
{

   switch(board)
    {
        case 1:
        {
            return(endpoint_mc_leftupperarm);
        }break;

        case 2:
        {
            return(endpoint_mc_leftlowerarm);
        }break;

        case 3:
        {
             return(endpoint_mc_rightupperarm);
        }break;

        case 4:
        {
            return(endpoint_mc_rightlowerarm);
        }break;

        case 5:
        {
             return(endpoint_mc_torso);
        }break;

        case 6:
        {
            return(endpoint_mc_leftupperleg);
        }break;

        case 7:
        {
            return(endpoint_mc_leftlowerleg);
        }break;

        case 8:
        {
            return(endpoint_mc_rightupperleg);
        }break;

        case 9:
        {
            return(endpoint_mc_rightlowerleg);
        }break;

        default:
        {
            return (0xFFFF);
        }
    };

}




static uint16_t s_getEP_as(uint8_t board )
{
   switch(board)
    {
        case 1:
        {
            return(endpoint_as_leftupperarm);
        }break;

        case 2:
        {
            return(endpoint_as_leftlowerarm);
        }break;

        case 3:
        {
             return(endpoint_as_rightupperarm);
        }break;

        case 4:
        {
            return(endpoint_as_rightlowerarm);
        }break;

        case 6:
        {
            return(endpoint_as_leftupperleg);
        }break;

        case 8:
        {
            return(endpoint_as_rightupperleg);
        }break;

        default:
        {
            return (0xFFFF);
        }
    };


}



static uint16_t s_getEP_sk(uint8_t board )
{
   switch(board)
    {

        case 2:
        {
            return(endpoint_sk_emsboard_leftlowerarm);
        }break;

        case 4:
        {
            return(endpoint_sk_emsboard_rightlowerarm);
        }break;

        default:
        {
            return (0xFFFF);
        }
    };
}


static uint16_t s_getEP_mn(uint8_t board )
{
    //used for all board
    return(endpoint_mn_comm);
}


static void s_print_mc_nvid_alljoints(void)
{
//todo: calcola il max num of joint a seconda della board
    for(eOmc_jointId_t j=0; j<12; j++)
    {
        s_print_mc_nvid(j);
    }
}

static void s_print_mc_nvid(eOmc_jointId_t j)
{

    printf("\n---- JOINT %d ----\n", j);
    printf("%s : 0x%x\n", "NVID_jxx_jconfig",                             EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__pidposition",                EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__pidposition(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__pidvelocity",                EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__pidvelocity(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__pidtorque",                  EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__pidtorque(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__impedance",                  EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__impedance(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__minpositionofjoint",         EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__minpositionofjoint(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__maxpositionofjoint",         EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__maxpositionofjoint(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__velocitysetpointtimeout",    EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__velocitysetpointtimeout(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__holder01FFU00",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__holder01FFU00(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__motionmonitormode",          EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__motionmonitormode(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__encoderconversionfactor",    EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__encoderconversionfactor(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__encoderconversionoffset",    EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__encoderconversionoffset(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__des02FORjstatuschamaleon04", EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__des02FORjstatuschamaleon04(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__holder01FFU01",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__holder01FFU01(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__holder02FFU03",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__holder02FFU03(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jconfig__holder02FFU04",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jconfig__holder02FFU04(j) );


    printf("%s : 0x%x\n", "NVID_jxx_jstatus",                             EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jstatus(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jstatus__basic",                      EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jstatus__basic(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jstatus__ofpid",                      EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jstatus__ofpid(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jstatus__chamaleon04",                EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jstatus__chamaleon04(j) );


    printf("%s : 0x%x\n", "NVID_jxx_jinputs",                             EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jinputs(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jinputs__externallymeasuredtorque",   EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jinputs__externallymeasuredtorque(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jinputs__holder02FFU01",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jinputs__holder02FFU01(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jinputs__holder04FFU02",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jinputs__holder04FFU02(j) );


    printf("%s : 0x%x\n", "NVID_jxx_jcmmnds__calibration",                EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jcmmnds__calibration(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jcmmnds__setpoint",                   EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jcmmnds__setpoint(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jcmmnds__stoptrajectory",             EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jcmmnds__stoptrajectory(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jcmmnds__controlmode",                EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jcmmnds__controlmode(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jcmmnds__holder01FFU02",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jcmmnds__holder01FFU02(j) );
    printf("%s : 0x%x\n", "NVID_jxx_jcmmnds__holder01FFU03",              EOK_cfg_nvsEP_mc_any_con_bodypart_NVID_jxx_jcmmnds__holder01FFU03(j) );
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




