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


#ifndef __ETHBOARD_H__
#define __ETHBOARD_H__


#include <string>
#include <vector>

using namespace std;

#include "EoCommon.h"
#include "EoUpdaterProtocol.h"
#include "EoBoards.h"

// this struct contains the ultimate source of information for a given board
struct boardInfo2_t
{
    uint8_t             protversion;        // the updater protocol used by the running process which is specified by:
    uint64_t            macaddress;         // the mac address (only 6 bytes out of 8)
    eObrd_ethtype_t     boardtype;          // the type of eth board
    uint32_t            capabilities;       // it contains a mask of eOuprot_proc_capabilities_t which tells what the remote board can do
    eOuprot_proctable_t processes;          // it contains the properties of the processes in the remote board
    uint8_t             boardinfo32[32];    // if boardinfo32[0] is not 0xff then boardinfo32[1] contains a descriptive string of the remote board (30 characters max).
    bool                maintenanceIsActive;// tells if the maintenance is active (hence if processes.runningnow is either eUpdater or eApplPROGupdater).

    eOversion_t         versionOfRunning;   // we keep this info (which for protversion is also inside processes.info[processes.runningnow].version) because ...
                                            // in the case of protversion 0 we dont have any info about which process is running now.
    string              moreinfostring;

    boardInfo2_t()
    {
        reset();
    }

    void reset(void)
    {
        protversion = 0;
        macaddress = 0;
        boardtype = eobrd_ethtype_unknown;
        capabilities = 0;
        memset(&processes, 0, sizeof(processes));
        processes.numberofthem = 0;
        processes.def2run = uprot_proc_None;
        processes.runningnow = uprot_proc_None;
        processes.startup = uprot_proc_None;
        processes.info[0].type = processes.info[1].type = processes.info[2].type = uprot_proc_None;
        memset(boardinfo32, 0, sizeof(boardinfo32));
        boardinfo32[0] = 0xff;
        maintenanceIsActive = false;
        versionOfRunning.major = versionOfRunning.minor = 0;
        moreinfostring.clear();
    }
};


class EthBoard
{

public:

    boardInfo2_t    _info2;
    eOipv4addr_t    _ipv4;
    string          _ipv4string;
    bool            _selected;

public:

    EthBoard(boardInfo2_t &info2, eOipv4addr_t ipv4);

    ~EthBoard();

    void setSelected(bool selected);

    bool isSelected();

    void setIPV4(eOipv4addr_t ipv4);

    eOipv4addr_t getIPV4();

    string getIPV4string();

    boardInfo2_t& getInfo();

    void setInfo(boardInfo2_t &info2);

    bool isInMaintenance();

    bool isInApplication();

    void setMoreInfo(string &moreinfo);

    const string getMoreInfo(void);

    string getInfoOnEEPROM(void);

    string getVersionfRunning(void);

    string getDatefRunning(void);

    string getCompilationDateOfRunning(void);

    // we dont publish any other methods for what is inside _info2 .... use getInfo().

};



class EthBoardList
{
public:
    vector<EthBoard> theboards;

    static const eOipv4addr_t ipv4all;
    static const eOipv4addr_t ipv4selected;

public:
    EthBoardList();

    ~EthBoardList();

    // if force is true: it does not check if the mac or the ipv4 is already inside and just adds a new entry
    // if force is false: it does a number of checks... in order of priority:
    // 1. if it finds the same info2.macaddress inside an item list: it does not add a new item and refreshes the item matching the mac with info2
    // 2. the mac is surely not in the list. if it finds the same ipv4 inside an item list: it adds a new entry (but if the item list has mac = 0,
    //    it refreshes the item).
    int add(boardInfo2_t &info2, eOipv4addr_t ipv4, bool force = false);

    int rem(eOipv4addr_t ipv4);

    int clear();

    int size();

    // it selects on/off the board w/ a given ip address. if ipv4 is 0 the rule applies to all
    void select(bool on, eOipv4addr_t ipv4);

    // tells how many have that ip address. if 0, it tells how many are selected
    int numberof(eOipv4addr_t ipv4);

    // retrieve a vector of pointers with a given ip address. if ipv4 is 0, then it retrieves all the selected
    // if 0xffffffff we retrieve them all
    // we have a vector of pointer so that we can modify the boards (but dont delete them, use rem() or clear() instead).    
    vector<EthBoard *> get(eOipv4addr_t ipv4);

    EthBoard& operator[](int i);
};

#endif


