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

/* @file       template_buttons.c
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/22/2013
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "template_buttons.hpp"
#include "OPCprotocolManager.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set()
// --------------------------------------------------------------------------------------------------------------------
extern OPCprotocolManager *opcMan_ptr;



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static uint32_t s_getAddressFromUser(void);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialization) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
void commands(void)
{
	printf("\nq: quit\n");
	printf("0: ........\n");
	printf("1: ........\n");
	printf("2: ........\n");
	printf("3: ........\n");
	printf("4: ........\n");
	printf("5: ........\n");
	printf("\n");
}

static uint32_t s_getAddressFromUser(void)
{
    char addrstr[20];
    uint32_t ip1,ip2,ip3,ip4, address=0;

    printf("Insert destination host: x.x.x.x ");
    scanf("%s", &addrstr[0]);
    sscanf(addrstr,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    address = (ip1<<24)|(ip2<<16)|(ip3<<8)|ip4 ;

    return(address);
}


int32_t callback_button_x(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr, int16_t id)
{
	printf("button %d\n", id);

	uint32_t dest_addr = s_getAddressFromUser();
	uint16_t size = 0;
	opcprotman_res_t res;

	addr->set(4444, dest_addr);

	if(opcMan_ptr ==  NULL)
	{
		printf("opcMan_ptr is NULL\n");
		return(0);
	}

	res = opcprotman_Form(opcMan_ptr, opcprotman_opc_ask, id, NULL /*setdata*/, (opcprotman_message_t*)payload_ptr, &size);
	if(opcprotman_OK  != res)
	{

		printf("erron in former\n");
		return(0);
	}
	printf("in button %d : size=%d \n", id, size);

     	return(size);
}




// fill the callback with your code
uint32_t callback_button_0(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr	*addr)
{
	uint32_t dest_addr = s_getAddressFromUser();
	uint16_t size = 0;


//	addr->set(4444, (10<<24)|(255<<16)|(72<<8)|19 );
	addr->set(4444, dest_addr);

//	opcprotman_Form(opcMan_ptr, opcprotman_opc_ask, 1, NULL /*setdata*/, payload_ptr, &size);

	return(size);
}

uint32_t callback_button_1(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(callback_button_x(payload_ptr, payload_size, addr, 1));
}

uint32_t callback_button_2(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(callback_button_x(payload_ptr, payload_size, addr, 2));
}


uint32_t callback_button_3(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

uint32_t callback_button_4(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

uint32_t callback_button_5(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

uint32_t callback_button_6(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

uint32_t callback_button_7(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

uint32_t callback_button_8(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

uint32_t callback_button_9(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}


uint32_t callback_button_10(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	return(0);
}

