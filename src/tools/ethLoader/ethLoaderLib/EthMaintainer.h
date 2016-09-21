
/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame, Alessandro Scalzo
 * email:   marco.accame@iit.it, alessandro.scalzo@iit.it
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


#ifndef __ETHMAINTAINER_H__
#define __ETHMAINTAINER_H__


#include "EoUpdaterProtocol.h"


#include "DSocket.h"
#include "EthBoard.h"

#include <vector>
using namespace std;



class EthMaintainer
{

public:

    static const eOipv4addr_t hostIPaddress;        // 10.0.1.104
    static const eOipv4port_t mainIPport;           // 3333

    static const eOipv4addr_t ipv4OfAllSelected;    // 0
    static const eOipv4addr_t ipv4Broadcast;         // 255.255.255.255 ... but it broadcast only int its subnet

    static const int partition_APPLICATION;
    static const int partition_LOADER;
    static const int partition_UPDATER;

    EthMaintainer();

    ~EthMaintainer();

    bool open(eOipv4addr_t ipv4 = hostIPaddress, eOipv4port_t port = mainIPport);

    bool close();

    // verbose is false by default
    void setVerbose(bool on);

    // debug print is false by default
    void setDebugPrint(bool on);

    EthBoardList& getBoards();
    void clearBoards();

    // it adds a dummy board (mac zero etc).
    int addBoard(eOipv4addr_t ipv4);
    int remBoard(eOipv4addr_t ipv4);

    // eOipv4addr_t is in network order, ACE in host order
    eOipv4addr_t ACEtoIPV4(ACE_UINT32 address);
    ACE_UINT32 IPV4toACE(eOipv4addr_t ipv4);

    // - part related to messages sent to the board(s)

    // used to see if a capability is supported by the process running on a board (or by all the selected boards if address is 0)
    // we can use this method to check if a command button can be enabled.
    bool isCommandSupported(eOuprot_proc_capabilities_t capability, eOipv4addr_t ipv4 = ipv4OfAllSelected);


    // used to discover one, a subset or all the boards.
    // if ipv4 is ipv4Broadcast then it sends the discover in broadcast, if ipv4OfAllSelected it sends the discovery to the list of selected,
    // else it sends a discovery only to the specified ip address
    // capabilities to be verified are one or both of the following:
    // - uprot_canDO_reply2discover:    the board sends back full information
    // - uprot_canDO_LEGACY_scan:       the board sends back a limited subset of information
    // it collects info about the boards on data structure available w/ method getBoards().
    // returns the number of discovered boards.


    int commandDiscover(bool clearboardsbeforediscovery = true, eOipv4addr_t ipv4 = ipv4Broadcast);


    // in all following methods: we send the commands in unicast but:
    // if ipv4 = ipv4OfAllSelected we send the command to every selected board, otherwise only to the specified address.


    // this command forces the mainteinance mode for all specified boards by sending a proper message (...).
    // if verify is true then there are other commands sent to all specified boards to verify if they are in maintenance.
    // there are at least retries attempt to verify it and there is a gap of timegap seconds in between them.

    bool commandForceMaintenance(eOipv4addr_t ipv4 = ipv4OfAllSelected, bool verify = true, int retries = 3, double timegap = 0.5);


    // used to refresh the info about a board (or about all the selected boards if address is 0)
    // capabilities to be verified are one or both of the following:
    // - uprot_canDO_reply2moreinfo:    the board sends back full information (as with commandDiscover()).
    // - uprot_canDO_LEGACY_procs:      the board sends back only a string with info about the partition table.

    std::string commandGetMoreInfo(eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // it returns the textual description associated to commandGetMoreInfo() without sending any message.
    // the textual description is filled by commandDiscover() or by commandForceMaintenance() only if the version of protocol supports
    // the flag uprot_canDO_reply2discover (that is: all versions apart old version 0).
    // if the protocol version is 0... we have to use commandGetMoreInfo() to obtain the textual information.
    std::string getTextualDescription(eOipv4addr_t ipv4 = ipv4OfAllSelected);


    // used to clear the 32-bytes field which is stored in EEPROM of each board.
    // capability to be checked is:
    // - uprot_canDO_PAGE_clr
    bool commandInfo32Clear(eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // used to set the 32-bytes field which is stored in EEPROM of each board.
    // capability to be checked is:
    // - uprot_canDO_PAGE_set
    bool commandInfo32Set(const string &info32, eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // used to get the 32-bytes field which is stored in EEPROM of each board.
    // capability to be checked is:
    // - uprot_canDO_PAGE_get
    vector<string> commandInfo32Get(eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // used to force a restart
    // capability to be checked is:
    // - uprot_canDO_restart
    bool commandRestart(eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // used to impose the default to run process (the one after the 5 seconds)
    // capability to be checked is:
    // - uprot_canDO_DEF2RUN_set
    bool commandSetDEF2RUN(eOuprot_process_t process, eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // used to force a jump to the eUpdater (only if the eApplication called eMaintainer is running)
    // capability to be checked is:
    // - uprot_canDO_JUMP2UPDATER
    bool commandJumpUpd(eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // used to force a jump to a given ROM address
    // capability to be checked is:
    // - uprot_canDO_JUMP2ADDRESS
    bool commandJump2ROMaddress(uint32_t romaddress, eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // forces a series of blinks in the LEDs of selected boards
    // capability to be checked is:
    // - uprot_canDO_blink
    bool commandBlink(eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // forces a full erase of the EEPROM. VERY DANGEROUS to do it in multicast because it sets IP addresses to 10.0.1.99
    // capability to be checked is one or both of the following:
    // - uprot_canDO_LEGACY_EEPROM_erase:       it erase the whole eeprom
    // - uprot_canDO_EEPROM_erase:              it can erase the whole eeprom but also a subset.
    bool commandEraseEEPROM(eOipv4addr_t ipv4 = ipv4OfAllSelected);


    // reads the EEPROM of a board. it must be used with care and towards one board at a time. the return value *eeprom is just
    // a pointer to the data stored in the internal reply packet, hence it must be copied elsewhere at reception.
    // capability to be checked is:
    // - uprot_canDO_EEPROM_getMoreInfoText2read:           it reads a portion of eeprom contained in [from, from+size) with size <= uprot_EEPROMmaxsize (1024) bytes.
    bool commandReadEEPROM(uint16_t from, uint16_t size, eOipv4addr_t ipv4, uint8_t **value);

    // changes the IP address of a given board.
    // capability to be checked is one or both of the following:
    // - uprot_canDO_LEGACY_IPaddr_set:         it just changes address.
    // - uprot_canDO_IPaddr_set:                it changes address and it also may force a restart (not requested for now)
    bool commandChangeAddress(eOipv4addr_t ipv4, eOipv4addr_t ipv4new, bool restart = false, bool verify = false);


    // used to program a given partition of a given board
    // capability to be checked is:
    // - uprot_canDO_PROG_loader:       for programming into the loader partition (the first)
    // - uprot_canDO_PROG_updater:      for programming into the updater partition (the second)
    // - uprot_canDO_PROG_application:  for programming into the application partition (the third)
    bool commandProgram(FILE *programFile, int partition, void (*updateProgressBar)(float), string &stringresult, eOipv4addr_t ipv4 = ipv4OfAllSelected);

protected:

    typedef struct
    {
        vector<EthBoard *> selected;
        vector<int> steps;
        int mN2Prog;
        int mNProgSteps;
        int mNChunks;
        void * data;
        int size;
        int answers;
        int retries;
    } progData_t;


    void sendCommand(eOipv4addr_t ipv4, void * cmd, uint16_t len);
    void sendCommandSelected(void * cmd, uint16_t len);
    int sendPROG2(const uint8_t opc, progData_t &progdata);

    bool isInMaintenance(eOipv4addr_t ipv4);

    void processDiscoveryReplies(void);

    std::string processMoreInfoReplies(void);

    // it forms the string returned by cmdGetMoreInfo() by formatting the field
    // eOuprot_cmd_MOREINFO_REPLY_t::discover sent by the remote board. the formatting can hence be defined in here.
    std::string prepareMoreInfoText(eOuprot_cmd_DISCOVER_REPLY_t * disc, char *ipv4string);

    // it forms the string returned by cmdGetMoreInfo() just copying the field
    // eOuprot_cmd_MOREINFO_REPLY_t::description[] inside the UDP packet as filled by the remote board.
    std::string getMoreInfoText(eOuprot_cmd_MOREINFO_REPLY_t *moreinfo, char *ipv4string);

private:

    // in here are methods which we prefer not to be used, but which may be useful in special cases

    // changes the IP mask of a given board, despite default address 0, it does not support it
    // capability to be checked is:
    // - uprot_canDO_LEGACY_IPmask_set
    // NOTE: it is very dangerous to change the IP mask. it is also very improbable that we want to do it. we keep the method however.
    bool cmdChangeMask(eOipv4addr_t newMask, eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // changes the MAC of a given board
    // capability to be checked is:
    // - uprot_canDO_LEGACY_MAC_set
    // NOTE: we may change the MAC only if ... there are two equal mac addresses on two boards ...
    bool cmdChangeMAC(uint64_t newMAC48, eOipv4addr_t ipv4);

    bool cmdPageClr(eOuprot_pagesize_t pagesize, eOipv4addr_t ipv4 = ipv4OfAllSelected);
    bool cmdPageSet(eOuprot_pagesize_t pagesize, uint8_t *data, eOipv4addr_t ipv4 = ipv4OfAllSelected);
    bool cmdPageGet(eOuprot_pagesize_t pagesize, uint8_t **data, eOipv4addr_t ipv4);


protected:

    bool _opened;
    bool _verbose;
    bool _debugprint;

    EthBoardList mEthBoards;

    unsigned char mRxBuffer[uprot_UDPmaxsize];
    unsigned char mTxBuffer[uprot_UDPmaxsize];

    eOipv4addr_t myIPV4addr;
    eOipv4port_t myIPV4port;

    DSocket mSocket;

};

#endif

// eof

