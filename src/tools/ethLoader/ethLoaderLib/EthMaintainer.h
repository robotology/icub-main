
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


string ipv4tostring(eOipv4addr_t ipv4);
bool string2ipv4(const string &ipv4string, eOipv4addr_t &ipv4);

// eOipv4addr_t is in network order, ACE in host order
eOipv4addr_t acetoipv4(ACE_UINT32 address);
ACE_UINT32 ipv4toace(eOipv4addr_t ipv4);


class EthMaintainer
{
public:

    static const eOipv4addr_t hostIPaddress;        // 10.0.1.104
    static const eOipv4port_t mainIPport;           // 3333

    static const eOipv4addr_t ipv4OfAllSelected;    // 0
    static const eOipv4addr_t ipv4Broadcast;         // 255.255.255.255 ... but it broadcast only int its subnet

public:

    EthMaintainer();

    ~EthMaintainer();

    bool open(eOipv4addr_t ipv4 = hostIPaddress, eOipv4port_t port = mainIPport);

    bool close();

    // verbose is false by default
    void verbose(bool on);

    // debug print is false by default
    void debugprint(bool on);


    // by default it is true.
    // if true:     for most complex operations, the results of the queries which go to teh boards are internally stored in permanent
    //              dat structure. As such, if we use an ipv4 address equal to ipv4OfAllSelected, the used IP address are those selected internally.
    //              IN SHORT: this is the method currently used by teh GUI which performs a discovery etc.
    // if false:    the results are not internally stored but where appropriate are returned by the function.
    //              for instance, discover() just returns the found boards. then one must retrieve the relevant ipv4 addresses and use them one by one.
    //              as such one cannot use ipv4 = ipv4OfAllSelected.
    bool boards_useinternal(bool on);


    // loads a complete set of boards to the class.
    bool boards_set(EthBoardList &boards);

    // retrieves the complet set of boards
    EthBoardList& boards_get(void);

    // clear the complete set of boards.
    bool boards_clr(void);

    // select / deselect some board(s). ipv4 can be a single value or ipv4Broadcast. it cannot be ipv4OfAllSelected
    bool boards_select(eOipv4addr_t ipv4, bool on);

    // it adds one board info at a goven address. if force is true it adds it without checking for mac or ip.
    // ipv4 cannot be ipv4OfAllSelected or ipv4Broadcast
    int boards_add(eOipv4addr_t ipv4, boardInfo2_t &info2, bool force);

    // it removes the specified board(s). it can be a single value, ipv4Broadcast or ipv4OfAllSelected
    int boards_rem(eOipv4addr_t ipv4);


    // in here there are complex operations which can group many methods such as discover() or the likes.
    // all these methods are self-organised and can be used as single command without the need to perform any discovery or else before.
    // the only requirement is to have an EthMaintainer object and to have it opened with the open() method.


    // it simply returns a EthBoardList object.
    // it does NOT attempt to send in maintenance, but old versions of eApplication jump to eUpddater when they receive the uprot_OPC_DISCOVER command.
    // if clearbeforediscovery is true, it clears the used boardlist.
    // then the uprot_OPC_DISCOVER command is sent numberofdiscoveries times (1 is enough) with a wait timeout of waittimeout seconds each.
    // the returned EthBoardList is guaranteed to contain unique MAC addresses. if two boards have the same IP address but different MAC they are counted twice.
    EthBoardList discover(bool clearbeforediscovery = true, int numberofdiscoveries = 1, double waittimeout = 1.0);


    // it retrieves information about the board of a given ip address.
    // warning: the same information can be retrieved by discover
    // ipv4 can be a single non zero value (e.g., 10.0.1.3) or the ipv4Broadcast value which means that the request is done in broadcast or ipv4OfAllSelected
    // which means to all the selected boards.
    // the information is returned in object EthBoardList, where there can be more boards with the same ip address.
    // we surely guarantee that the mac is unique.
    EthBoardList information(eOipv4addr_t ipv4, bool ask2board = true, bool forcemaintenance = true, int numberofrequests = 1, double waittimeout = 1.0);


    // it sends the command eOuprot_cmd_MOREINFO_t to query the boards. if forcemaintenance is true it first send the boards in maintenance.
    std::string moreinformation(eOipv4addr_t ipv4, bool forcemaintenance = false);


    // send to maintenance an ipv4 address.
    // if verify is true, then it checks that the IP address(es) are in maintenance with a max number of retries each interleaved by an interval
    // as specified by the other parameters.
    bool go2maintenance(eOipv4addr_t ipv4, bool verify = true, int retries = 6, double timegap = 1.0);


    // send to application an ipv4 address.
    // if checkdef2runapplication is true: it checks if def2run is eApplication.
    // - if it is, then it returns true if boards are already running application
    // - if it is false, it sends the boards in maintenance, it sets def2run = eApplication and then:
    //   a. it restarts teh boards,
    //   b. it waits for bootstraptime seconds,
    //   c. if verify is true it also checks that the eApplication is running
    bool go2application(eOipv4addr_t ipv4, bool checkdef2runapplication = true, double bootstraptime = 10.0, bool verify = true);


    // it programs board(s) w/ ipv4 address which must have:
    // - same type as argument (unless type is eobrd_ethtype_none or eobrd_ethtype_unknown)
    // - same process as argument
    // - same target version as argument (unless targetversion is (0, 0)).
    // if forceaintenance is true, the boards are forced in maintenance (eUpdater or eApplPROGupdater).
    // then some checks are done if the board(s) run a process suitable to program what in argument.
    // if progress is not NULL then it is called to express activity progress
    // finally, if restart2application is true and process is NOT eUpdater, the board(s) are restarted with 10 seconds
    // of bootstrap time and check: go2application(ipv4, true, 10, true)
    bool program(eOipv4addr_t ipv4, eObrd_ethtype_t type, eOuprot_process_t process, eOversion_t targetversion, FILE *fp, bool forcemaintenance = true, void progress(float) = NULL, bool restart2application = true);



    // raw commands.
    bool command_supported(eOipv4addr_t ipv4, eOuprot_proc_capabilities_t capability, bool ask2board = false);

    bool command_def2run(eOipv4addr_t ipv4, eOuprot_process_t process, bool forcemaintenance = true, bool verify = true);

    bool command_restart(eOipv4addr_t ipv4);

    bool command_changeaddress(eOipv4addr_t ipv4, eOipv4addr_t ipv4new, bool checkifnewispresent = true, bool forcemaintenance = true, bool restart = false, bool verify = false);

    bool command_info32_clr(eOipv4addr_t ipv4);
    bool command_info32_set(eOipv4addr_t ipv4, const string &info32);
    vector<string> command_info32_get(eOipv4addr_t ipv4);

    bool command_jump2updater(eOipv4addr_t ipv4);

    bool command_jump2address(eOipv4addr_t ipv4, uint32_t romaddress);

    bool command_blink(eOipv4addr_t ipv4);

    bool command_eeprom_erase(eOipv4addr_t ipv4);

    bool command_eeprom_read(eOipv4addr_t ipv4, uint16_t from, uint16_t size, uint8_t **value);

    // used to program a given partition of a given board
    // capability to be checked is:
    // - uprot_canDO_PROG_loader:       for programming into the loader partition (the first)
    // - uprot_canDO_PROG_updater:      for programming into the updater partition (the second)
    // - uprot_canDO_PROG_application:  for programming into the application partition (the third)
    bool command_program(eOipv4addr_t ipv4, FILE *programFile, eOuprot_partition2prog_t partition, void (*updateProgressBar)(float), EthBoardList *pboardlist, string &stringresult);



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


    bool sendCommand(eOipv4addr_t ipv4, void * cmd, uint16_t len, EthBoardList *boardlist = NULL);

    int sendPROG2(const uint8_t opc, progData_t &progdata);

    bool isInMaintenance(eOipv4addr_t ipv4, EthBoardList &boardlist);
    bool isInApplication(eOipv4addr_t ipv4, EthBoardList &boardlist);

    string processDiscoveryReplies2(EthBoardList &boardlist, double waittimeout = 1.0);

    std::string processMoreInfoReplies(EthBoardList &boardlist);

    // it forms the string returned by cmdGetMoreInfo() by formatting the field
    // eOuprot_cmd_MOREINFO_REPLY_t::discover sent by the remote board. the formatting can hence be defined in here.
    std::string prepareMoreInfoText(eOuprot_cmd_DISCOVER_REPLY_t * disc, const char *ipv4string);

    // it forms the string returned by cmdGetMoreInfo() just copying the field
    // eOuprot_cmd_MOREINFO_REPLY_t::description[] inside the UDP packet as filled by the remote board.
    std::string getMoreInfoText(eOuprot_cmd_MOREINFO_REPLY_t *moreinfo, char *ipv4string);

private:

    // in here are methods which we prefer not to be used, but which may be useful in special cases

    // changes the IP mask of a given board, despite default address 0, it does not support it
    // capability to be checked is:
    // - uprot_canDO_LEGACY_IPmask_set
    // NOTE: it is very dangerous to change the IP mask. it is also very improbable that we want to do it. we keep the method however.
//    bool cmdChangeMask(eOipv4addr_t newMask, eOipv4addr_t ipv4 = ipv4OfAllSelected);

    // changes the MAC of a given board
    // capability to be checked is:
    // - uprot_canDO_LEGACY_MAC_set
    // NOTE: we may change the MAC only if ... there are two equal mac addresses on two boards ...
//    bool cmdChangeMAC(uint64_t newMAC48, eOipv4addr_t ipv4);

//    bool cmdPageClr(eOuprot_pagesize_t pagesize, eOipv4addr_t ipv4 = ipv4OfAllSelected);
//    bool cmdPageSet(eOuprot_pagesize_t pagesize, uint8_t *data, eOipv4addr_t ipv4 = ipv4OfAllSelected);
//    bool cmdPageGet(eOuprot_pagesize_t pagesize, uint8_t **data, eOipv4addr_t ipv4);


protected:

    bool _opened;
    bool _verbose;
    bool _debugprint;

    bool _useofinternalboardlist;
    EthBoardList _internalboardlist;


    unsigned char mRxBuffer[uprot_UDPmaxsize];
    unsigned char mTxBuffer[uprot_UDPmaxsize];

    eOipv4addr_t myIPV4addr;
    eOipv4port_t myIPV4port;

    DSocket mSocket;
};

#endif

// eof

