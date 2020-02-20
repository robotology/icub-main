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

#include <pcap.h>





//embody stuff
#include "EoCommon.h"






// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define TIMEOUT_MS 1000

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




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static pcap_t *handle = NULL;
static struct bpf_program fp;					/* The compiled filter expression */

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern uint8_t wrapperPcap_init(char* dev, char*filter_expr)
{

	char 			errbuf[PCAP_ERRBUF_SIZE];
	bpf_u_int32 		mask;						/* The netmask of our sniffing device */
	bpf_u_int32 		net;						/* The IP of our sniffing device */

    //3) init pcap

	if (pcap_lookupnet(dev, &net, &mask, errbuf) == -1)
       {
		printf("Sorry, Can't get netmask for device %s\n", dev);
		net = 0;
		mask = 0;
	}

	handle = pcap_open_live(dev, BUFSIZ, 0, TIMEOUT_MS, errbuf);				// 0 = non promiscuo
	if (handle == NULL)
       {
		printf("Sorry, occured problem while opening pcap\n");
        //fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		return(0);
	}

	/* make sure we're capturing on an Ethernet device [2] */
	if (pcap_datalink(handle) != DLT_EN10MB)
    {
		printf("Sorry, pcap has been open on a NOT ethernet device\n");
        //fprintf(stderr, "%s is not an Ethernet\n", dev);
		return(0);
	}


	if (pcap_compile(handle, &fp, filter_expr, 0, net) == -1)
        {
           printf("Sorry, pcap can't paser filter%s: %s\n", filter_expr, pcap_geterr(handle));
           //fprintf(stderr, "Couldn't parse filter %s: %s\n", filter_exp, pcap_geterr(handle));
	   return(0);
	}

	if (pcap_setfilter(handle, &fp) == -1)
    {
        printf("Sorry, pcap couldn't install filter %s: %s\n", filter_expr, pcap_geterr(handle));
    //		fprintf(stderr, "Couldn't install filter %s: %s\n", filter_exp, pcap_geterr(handle));
		return(0);
	}

    return(1); //ok

}

#warning cnt 32 o 64 bit???
extern uint8_t wrapperPcap_loop(int32_t cnt, pcap_handler callback, uint8_t *user)
{
    uint8_t ret;

    ret = pcap_loop(handle, cnt, callback, user); // Get into the loop

    return(ret);
}

extern void wrapperPcap_close(void)
{
	pcap_freecode(&fp);
	pcap_close(handle);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




