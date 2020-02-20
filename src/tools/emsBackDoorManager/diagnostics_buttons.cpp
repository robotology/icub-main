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
#include "OPCprotocolManager_Cfg.h"
#include "EoDiagnostics.h"

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
extern ACE_SOCK_Dgram	*ACE_socket;

eOdgn_commands_t dgnCommands={0};
uint32_t cmdena_rxsetPointCheck = 0;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static uint32_t s_getAddressFromUser(void);
static void enaDisaDiagnostic(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialization) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
void commands(void)
{
	printf("\nq: quit\n");
	printf("0: ena/disa diagnostics on a ems\n");
	printf("1: ena/disa diagnostics on a group of ems (exteranl fault is NOT checked)\n");
	printf("2: ena/disa diagnostics on a group of ems and exteranl fault is checked \n");
	printf("3: advanced configuration....\n");
	printf("4: ena/disa rx setpoint che for red ball demo.\n");
	printf("5: ........\n");
	printf("\n");
}

static uint32_t s_getAddressFromUser(void)
{
    char addrstr[20];
    int ret;
    uint32_t ip1,ip2,ip3,ip4, address=0;

    printf("Insert destination host: x.x.x.x ");
    ret = scanf("%s", &addrstr[0]);
    ret = sscanf(addrstr,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
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
	uint32_t dest_addr;
	uint16_t size = 0;
    uint32_t cmdena=0;
    uint32_t extFaultEna=0;
    uint32_t canStatistics=0;
    uint32_t ethCounters=0;
    opcprotman_res_t res;
    int ret;

    if(opcMan_ptr ==  NULL)
	{
		printf("opcMan_ptr is NULL\n");
		return(0);
	}


    printf("enable/disable diagnostics. Press 1 to enable 0 otherwise\n");
    ret = scanf("%d", &cmdena);

    printf("enable/disable Check external fault. Press 1 to enable 0 otherwise\n");
    ret = scanf("%d", &extFaultEna);

    printf("enable/disable can statistics. Press 1 to enable 0 otherwise\n");
    ret = scanf("%d", &canStatistics);

    printf("enable/disable Check eth counters. Press 0 to disable or 1 for crcError, 2 for num of rxUnicat, 4 for txtUnicast\n");
    ret = scanf("%d", &ethCounters);

    dest_addr = s_getAddressFromUser();
    printf("address is 0x %x \n", dest_addr);

    if(cmdena>1)
    {
        cmdena=1;
    }

    if(extFaultEna>1)
    {
        extFaultEna=1;
    }

    if(canStatistics>1)
	{
    	canStatistics=1;
	}
    dgnCommands.enable = cmdena;
    dgnCommands.signalExtFault = extFaultEna;
    dgnCommands.signalEthCounters = ethCounters;
    dgnCommands.signalCanStatistics = canStatistics;
//	addr->set(4444, (10<<24)|(255<<16)|(72<<8)|19 );
	addr->set(4444, dest_addr);

	res = opcprotman_Form(opcMan_ptr, opcprotman_opc_set, eodgn_nvidbdoor_cmds,  &dgnCommands, (opcprotman_message_t*)payload_ptr, &size);
	if(opcprotman_OK  != res)
	{

		printf("erron in former\n");
		return(0);
	}

    return(size);
}




uint32_t callback_button_1(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	uint32_t cmdena=0;
	int ret;

    if(opcMan_ptr ==  NULL)
	{
		printf("opcMan_ptr is NULL\n");
		return(0);
	}

    printf("enable/disable diagnostics for a set of board. Press 1 to enable 0 otherwise\n");
    ret = scanf("%d", &cmdena);
    dgnCommands.enable = cmdena;

    dgnCommands.signalExtFault = 0;
    dgnCommands.signalEthCounters = 0;
    dgnCommands.signalCanStatistics = 0;

    enaDisaDiagnostic(payload_ptr, payload_size, addr);

    return(0); //no data to send in main loop

}

uint32_t callback_button_2(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	uint32_t cmdena=0;
	int ret;

    if(opcMan_ptr ==  NULL)
	{
		printf("opcMan_ptr is NULL\n");
		return(0);
	}

    printf("enable/disable diagnostics for a set of board. Press 1 to enable 0 otherwise\n");
    ret = scanf("%d", &cmdena);
    dgnCommands.enable = cmdena;

    dgnCommands.signalExtFault = 1;
    dgnCommands.signalEthCounters = 0;
    dgnCommands.signalCanStatistics = 0;

    enaDisaDiagnostic(payload_ptr, payload_size, addr);
    return(0); //no data to send in main loop
}


uint32_t callback_button_3(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
    uint32_t cmdena=0;
    uint32_t extFaultEna=0;
    uint32_t canStatistics=0;
    uint32_t ethCounters=0;
    int ret;

    if(opcMan_ptr ==  NULL)
	{
		printf("opcMan_ptr is NULL\n");
		return(0);
	}

	printf("enable/disable diagnostics for a set of board. Press 1 to enable 0 otherwise\n");
	ret = scanf("%d", &cmdena);
	dgnCommands.enable = cmdena;

	 if(cmdena)
	 {
		 printf("enable/disable Check external fault. Press 1 to enable 0 otherwise\n");
		 ret = scanf("%d", &extFaultEna);

		 printf("enable/disable can statistics. Press 1 to enable 0 otherwise\n");
		 ret = scanf("%d", &canStatistics);

		 printf("enable/disable Check eth counters. Press 0 to disable or 1 for crcError, 2 for num of rxUnicat, 4 for txtUnicast\n");
		 ret = scanf("%d", &ethCounters);

	    dgnCommands.signalExtFault = extFaultEna;
	    dgnCommands.signalEthCounters = ethCounters;
	    dgnCommands.signalCanStatistics = canStatistics;
	 }
	 else
	 {
		 dgnCommands.signalExtFault = 0;
		 dgnCommands.signalEthCounters = 0;
		 dgnCommands.signalCanStatistics = 0;
	 }

	enaDisaDiagnostic(payload_ptr, payload_size, addr);


	return(0);
}

uint32_t callback_button_4(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{
	uint16_t size = 0;
    uint32_t cmdena=0;
    uint32_t min, max, j;
    opcprotman_res_t res;
    uint32_t address=0;
    int ret;


    printf("enable/disable diagnostics about rx setpoint (red ball demo). Press 1 to enable 0 otherwise\n");
    ret = scanf("%d", &cmdena_rxsetPointCheck);

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



static void enaDisaDiagnostic(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr)
{

	uint16_t size = 0;
    uint32_t min, max, j;
    opcprotman_res_t res;
    uint32_t address=0;
    int ret;

    printf("insert renge of board[1,11]: min max\n");
    ret = scanf("%d %d", &min, &max);
    if(min>max)
    {
        printf("error: min > max!\n");
        return;
    }

    if((min == 0) && (max ==0))
    {
    	//use debug board on desk
    	address = (10<<24)|(0<<16)|(1<<8)|99 ;
		addr->set(4444, address);
		res = opcprotman_Form(opcMan_ptr, opcprotman_opc_set, eodgn_nvidbdoor_cmds,  &dgnCommands, (opcprotman_message_t*)payload_ptr, &size);
		if(opcprotman_OK  != res)
		{

			printf("erron in former\n");
			return;
		}
			ssize_t sentBytes = ACE_socket->send(payload_ptr, size, *addr, 0/*flags*/);
			ACE_TCHAR     address[64];
			addr->addr_to_string(address, 64);
			printf("payload of size %d is sent to %s!! (sentbytes=%ld)\n",  size, address, sentBytes);
    }


    if(min<1)
    {
        min = 1;
    }
    if(max>11)
    {
        max = 11;
    }

    for(j=min; j<=max; j++)
    {
    	address = (10<<24)|(0<<16)|(1<<8)|j ;
    	addr->set(4444, address);
    	res = opcprotman_Form(opcMan_ptr, opcprotman_opc_set, eodgn_nvidbdoor_cmds,  &dgnCommands, (opcprotman_message_t*)payload_ptr, &size);
		if(opcprotman_OK  != res)
		{

			printf("erron in former\n");
			return;
		}
    		ssize_t sentBytes = ACE_socket->send(payload_ptr, size, *addr, 0/*flags*/);
			ACE_TCHAR     address[64];
			addr->addr_to_string(address, 64);
			printf("payload of size %d is sent to %s!! (sentbytes=%ld)\n",  size, address, sentBytes);
	}

}
