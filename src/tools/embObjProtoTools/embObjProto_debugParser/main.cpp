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

/* @file       main.cpp
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/19/2013
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <unistd.h>
#include <stdio.h>

//#include <pcap.h>
//#include "header.h"
//#include "debugFunctions.h"

// ACE stuff
#include <ace/ACE.h>
#include "ace/SOCK_Dgram.h"
#include "ace/Addr.h"

//yarp stuff
//#include <yarp/os/Time.h>
//using namespace yarp::os;

//embody stuff
#include "EoCommon.h"
#include "EoMotionControl.h"
#include "eODeb_eoProtoParser.h"
#include "eOtheEthLowLevelParser.h"
//pcap stuff
#include "pcap_wrapper_linux.h"
#include "stdio.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define MAX_ACQUISITION		1000000000L

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set()
// --------------------------------------------------------------------------------------------------------------------
//extern const eODeb_eoProtoParser_cfg_t  deb_eoParserCfg;
extern const eODeb_eoProtoParser_cfg_t *deb_eoParserCfg_ptr;
FILE *file;



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_process_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet);
static void s_ethLowLevelPraser_configure(void);
static void print_help(void);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	// pcap_t *handle;
	// char 	errbuf[PCAP_ERRBUF_SIZE];
	// struct bpf_program fp;					/* The compiled filter expression */
	char 	filter_exp[260];					/* The filter expression */
	char    filtersrc[20];
	bool    filterOnSrc = false;
	char    filterdst[20];
	bool    filterOnDst = false;
	char 	dev[10];
    uint64_t maxacquisition = MAX_ACQUISITION;
    char default_filter_exp[60];



	struct pcap_pkthdr header;	/* The header that pcap gives us */
	const u_char *packet;		/* The actual packet */


    //1) init data
    sprintf(dev, "eth1");
    sprintf(filter_exp, "src host 10.0.1.104");
//    sprintf(filter_exp, " ");

    //2) parse arguments
    if(argc>1)
    {
        for(uint8_t i = 0; i<argc; i++)
        {
            if(strcmp("--help", argv[i]) == 0)
            {
				print_help();
                return(0);
            }

            if(strcmp("--dev", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                    sprintf(dev, "%s", argv[++i]);
                }
                continue;
            }

            if(strcmp("--filterSrcAddr", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                	filterOnSrc = true;
                    sprintf(filtersrc, "%s", argv[++i]);
                }
                continue;
            }


            if(strcmp("--filterDstAddr", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                	filterOnDst = true;
                	sprintf(filterdst, "%s", argv[++i]);
                }
                continue;
            }

            if(strcmp("--maxacquisition", argv[i]) == 0)
            {
                if(i<(argc-1))
                {
                    maxacquisition = atoll(argv[++i]);
                }
                continue;
            }

        }
    }

    file = fopen("test.txt", "a+");
    if(NULL == file)
    {
    	printf("cannot open file \n");
    	return(0);

    }

    //set filter
    if((filterOnSrc) &&(filterOnDst))
    {
    	sprintf(filter_exp, "src host %s and dst host %s", filtersrc, filterdst);

    }
    else if(filterOnSrc)
    {
    	sprintf(filter_exp, "src host %s ", filtersrc);

    }
    else if(filterOnDst)
    {
    	sprintf(filter_exp, "dst host %s", filterdst);
    }

    sprintf(default_filter_exp, " and not port 4444"); //emsbackdoor
    strcat(filter_exp, default_filter_exp);

    printf("Debug-tool: Ethernet Low Level Praser start on interface %s\n", dev);
    printf("\t\t Used filter: %s\n", filter_exp);


    //3) init pcap
/*
	handle = pcap_open_live(dev, BUFSIZ, 0, TIMEOUT_MS, errbuf);				// 0 = non promiscuo
	if (handle == NULL)
    {
		printf("Sorry, occured problem while opening pcap\n");
        //fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		return(2);
	}

	// make sure we're capturing on an Ethernet device [2]
	if (pcap_datalink(handle) != DLT_EN10MB)
    {
		printf("Sorry, pcap has been open on a NOT ethernet device\n");
        //fprintf(stderr, "%s is not an Ethernet\n", dev);
		exit(EXIT_FAILURE);
	}

	if (pcap_compile(handle, &fp, filter_exp, 0, net) == -1)
    {
        printf("Sorry, pcap can't paser filter%s: %s\n", filter_exp, pcap_geterr(handle));
        //fprintf(stderr, "Couldn't parse filter %s: %s\n", filter_exp, pcap_geterr(handle));
		return(2);
	}

	if (pcap_setfilter(handle, &fp) == -1)
    {
        printf("Sorry, pcap couldn't install filter %s: %s\n", filter_exp, pcap_geterr(handle));
    //		fprintf(stderr, "Couldn't install filter %s: %s\n", filter_exp, pcap_geterr(handle));
		return(2);
	}
*/
    if(wrapperPcap_init(dev, filter_exp)==0)
    {
        printf("error in inti libpacap\n");
        return(1);
    }

    //4)init low level parser objs
    s_ethLowLevelPraser_configure();

    //5)start to acquire packets
/*
	pcap_loop(handle, maxacquisition, s_process_packet, NULL); // Get into the loop

*/
    printf("start\n");
    wrapperPcap_loop(maxacquisition, s_process_packet, NULL);

	printf("\nCapture complete.\n");

	//6) close the session
    wrapperPcap_close();
    fclose(file);

	return(0); // everything ok
}







// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------
static void s_process_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet)
{
//    printf("process pkt\n");
    //eOTheEthLowLevParser_DissectPacket(eo_ethLowLevParser_GetHandle, packet);

    ////currently use thelow level parser and appl paser separately
    eOethLowLevParser_packetInfo_t pktInfo;
    eOresult_t res;

    res  = eOTheEthLowLevParser_GetUDPdatagramPayload(eo_ethLowLevParser_GetHandle(), (uint8_t*)packet, &pktInfo);
    if(eores_OK != res)
    {
        return;
    }
    eODeb_eoProtoParser_RopFrameDissect(eODeb_eoProtoParser_GetHandle(), &pktInfo);
}


//static void my_cbk_onErrorSeqNum(eOethLowLevParser_packetInfo_t *pktInfo_ptr, uint32_t rec_seqNum, uint32_t expected_seqNum)
//{
//
//	printf("ERR in SEQNUM; rec=%d expected=%d\n",rec_seqNum, expected_seqNum );
//return;
//}
//
//static void my_cbk_onNVfound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
//{
//
//	printf("NV found!!: ep=%x id=%x\n", ropAddInfo_ptr->desc.ep, ropAddInfo_ptr->desc.id);
//	return;
//}
//
//static void my_cbk_onNVsetpointFound(eOethLowLevParser_packetInfo_t *pktInfo_ptr, eODeb_eoProtoParser_ropAdditionalInfo_t *ropAddInfo_ptr)
//{
//    eOmc_setpoint_t * setpoint_ptr = (eOmc_setpoint_t *)ropAddInfo_ptr->desc.data;
//    uint8_t board = 0, j;
//    float enc_factor, zero, enc_factor_6=182.044 , enc_factor_8=182.044, zero_6=180, zero_8=-180;
//
//	//printf("NV found!!: ep=%x id=%x\n", ropAddInfo_ptr->desc.ep, ropAddInfo_ptr->desc.id);
//
//    if(ropAddInfo_ptr->desc.ep == 0x18)
//    {
//        board = 8;
//        enc_factor = enc_factor_8;
//        zero = zero_8;
//    }
//    else if(ropAddInfo_ptr->desc.ep == 0x16)
//    {
//        board = 6;
//        enc_factor = enc_factor_6;
//        zero = zero_6;
//    }
//    else
//    {
//        printf("\n\n ERROR: un expected ep!!! %d \n ", ropAddInfo_ptr->desc.ep);
//        return;
//    }
//
//    if(setpoint_ptr->type != eomc_setpoint_position)
//    {
//        printf("ERR: no setpoint position. typse= %d", setpoint_ptr->type);
//        return;
//    }
//
//    switch(ropAddInfo_ptr->desc.id)
//    {
//        case 0xbc1a:
//        {
//            j = 0;
//        }break;
//
//        case 0xbc3a:
//        {
//            j = 1;
//        }break;
//
//        case 0xbc5a:
//        {
//            j = 2;
//        }break;
//
//        case 0xbc7a:
//        {
//            j = 3;
//        }break;
//        default:
//        {
//            printf("ERROR: receiv unexpected nvid %x", ropAddInfo_ptr->desc.id);
//            return;
//        }
//    }
//    float vel, pos;
//
//    pos = (setpoint_ptr->to.position.value/enc_factor)-zero;
//    vel = setpoint_ptr->to.position.withvelocity/fabs(enc_factor);
//
//    printf("board %d  j %d   pos %f (%d)  vel %f (%d)  enc_factor=%f  zero=%f\n", board, j, pos, setpoint_ptr->to.position.value, vel, setpoint_ptr->to.position.withvelocity,enc_factor,zero);
//
//    return;
//}


static void s_ethLowLevelPraser_configure(void)
{

//    //4.1) init application parser: embObjParser
//    const eODeb_eoProtoParser_cfg_t  deb_eoParserCfg =
//    {
//        EO_INIT(.checks)
//        {
//            EO_INIT(.seqNum)
//            {
//                EO_INIT(.cbk_onErrSeqNum)           NULL, //my_cbk_onErrorSeqNum,
//            },
//
//            EO_INIT(.nv)
//            {
//                EO_INIT(.NVs2searchArray)
//                {
//                    EO_INIT(.head)
//                    {
//                        EO_INIT(.capacity)       eODeb_eoProtoParser_maxNV2find,
//                        EO_INIT(.itemsize)       sizeof(eODeb_eoProtoParser_nvidEp_couple_t),
//                        EO_INIT(.size)           8,
//                    },
//                    EO_INIT(.data)
//                    {
//                        {0x18, 0xbc1a},
//                        {0x18, 0xbc3a},
//                        {0x18, 0xbc5a},
//                        {0x18, 0xbc7a},
//                        {0x16, 0xbc1a},
//                        {0x16, 0xbc3a},
//                        {0x16, 0xbc5a},
//                        {0x16, 0xbc7a}
//                    }
//
//                },
//                EO_INIT(.cbk_onNVfound)            my_cbk_onNVsetpointFound, //my_cbk_onNVfound
//            },
//
//            EO_INIT(.invalidRopFrame)               {0}
//        }
//    };

    eODeb_eoProtoParser_Initialise(/*&deb_eoParserCfg*/deb_eoParserCfg_ptr);


    //4.2) init low level parser: eOethLowLevParser
/*    const eOethLowLevParser_cfg_t  ethLowLevParserCfg =
    {
        EO_INIT(.conFiltersData)
        {
            EO_INIT(.filtersEnable)     0,
            EO_INIT(.filters)           {0}, //use pcap filter
        },

        EO_INIT(.appParserData)
        {
            EO_INIT(.func)             eODeb_eoProtoParser_RopFrameDissect
            EO_INIT(.arg)              eODeb_eoProtoParser_GetHandle(),
        }
    };
*/
    //currently use thelow level parser and appl paser separately
    const eOethLowLevParser_cfg_t  ethLowLevParserCfg = {0};
    eo_ethLowLevParser_Initialise(&ethLowLevParserCfg);
}

static void print_help(void)
{

	printf("option --help: print this help\n");
	printf("option --dev: specify device where start to capture (default eth1)\n");
	printf("option --filterSrcAddr: application will capture packet with source address specified here. Not mandatory\n");
	printf("option --filterDstAddr: application will capture packet with dest address specified here. Not mandatory\n");
	printf("option --maxacquisition: max num of captured packets. Not mandatory\n");
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




