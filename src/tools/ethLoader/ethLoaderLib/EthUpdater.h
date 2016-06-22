/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_H__
#define __ETHUPDATER_H__


#include "EoUpdaterProtocol.h"


#include "DSocket.h"
#include "BoardList.h"

#include <vector>
using namespace std;

class EthUpdater
{
protected:
    BoardList mBoardList;

    int mN2Prog;
    int mNProgSteps;
    int mNChunks;

    BoardInfo* mBoard2Prog[256];

    unsigned char mRxBuffer[uprot_UDPmaxsize];
    unsigned char mTxBuffer[uprot_UDPmaxsize];

    ACE_UINT16 mPort;
    ACE_UINT32 mBroadcast;
    ACE_UINT32 mMyAddress;
    
    DSocket mSocket;

public:
    static const int partition_APPLICATION;
    static const int partition_LOADER;
    static const int partition_UPDATER;

    EthUpdater()
    {
    }

    ~EthUpdater()
    {
        mSocket.Close();
        mBoardList.empty();
    }

    bool create(ACE_UINT16 port,ACE_UINT32 address)
    {
        mPort=port;
        mMyAddress=address;
        mBroadcast=mMyAddress|0x0000FFFF;

        return mSocket.Create(mPort,mMyAddress);
    }

    bool create(ACE_UINT16 port,std::string& address)
    {
        mPort=port;
        ACE_UINT32 ip1,ip2,ip3,ip4;
        sscanf(address.c_str(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
        mMyAddress=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;
        mBroadcast=mMyAddress|0x0000FFFF;

        return mSocket.Create(mPort,mMyAddress);
    }

    BoardList& getBoardList(){ return mBoardList; }

    // - part related to messages sent to the board(s)


    // used to see if a capability is supported by the process running on a board (or by all the selected boards if address is 0)
    // we can use this method to check if a command button can be enabled.
    bool isCmdSupported(eOuprot_proc_capabilities_t capability, ACE_UINT32 address = 0);


    // used to discover all the available boards.
    // we send the command in broadcast.
    // capabilities to be verified are one or both of teh following:
    // - uprot_canDO_reply2discover:    the board sends back full information
    // - uprot_canDO_LEGACY_scan:       the board sends back a limited subset of information
    // every board replies to this method.
    void cmdDiscover();


    // in all following methods: we send the commands in unicast but:
    // if address = 0 we send the command to every selected board, otherwise only to the specifeid address.


    // used to refresh the info about a board (or about all the selected boards if address is 0)
    // refreshInfo = true allows to update the boardinfo inside mBoardList
    // capabilities to be verified are one or both of the following:
    // - uprot_canDO_reply2moreinfo:    the board sends back full information (as with cmdDiscover()).
    //                                  if plusString = true this methods also requests a string with info about the partition table as old boards do.
    //                                  if refreshInfo = true this methods also updates the boardinfo inside mBoardList
    // - uprot_canDO_LEGACY_procs:      the board sends back only a string with info about the partition table.
    std::string cmdGetMoreInfo(bool refreshInfo = false, ACE_UINT32 address = 0);


    // used to clear the 32-bytes field which is stored in EEPROM of each board.
    // capability to be checked is:
    // - uprot_canDO_PAGE_clr
    void cmdInfo32Clear(ACE_UINT32 address = 0);

    // used to set the 32-bytes field which is stored in EEPROM of each board.
    // capability to be checked is:
    // - uprot_canDO_PAGE_set
    void cmdInfo32Set(const string &info32, ACE_UINT32 address = 0);

    // used to get the 32-bytes field which is stored in EEPROM of each board.
    // capability to be checked is:
    // - uprot_canDO_PAGE_get
    vector<string> cmdInfo32Get(ACE_UINT32 address = 0);

    // used to force a restart
    // capability to be checked is:
    // - uprot_canDO_restart
    void cmdRestart(ACE_UINT32 address = 0);

    // used to impose the default to run process (the one after the 5 seconds)
    // capability to be checked is:
    // - uprot_canDO_DEF2RUN_set
    void cmdSetDEF2RUN(eOuprot_process_t process, ACE_UINT32 address = 0);

    // used to force a jump to the eUpdater (only if the eApplication called eMaintainer is running)
    // capability to be checked is:
    // - uprot_canDO_JUMP2UPDATER
    void cmdJumpUpd(ACE_UINT32 address = 0);

    // forces a series of blinks in the LEDs of selected boards
    // capability to be checked is:
    // - uprot_canDO_blink
    void cmdBlink(ACE_UINT32 address = 0);

    // forces a full erase of the EEPROM. VERY DANGEROUS to do it in multicast because it sets IP addresses to 10.0.1.99
    // capability to be checked is one or both of the following:
    // - uprot_canDO_LEGACY_EEPROM_erase:       it erase the whole eeprom
    // - uprot_canDO_EEPROM_erase:              it can erase the whole eeprom but also a subset.
    void cmdEraseEEPROM(ACE_UINT32 address = 0);


    // reads the EEPROM of a board. it must be used with care and towards one board at a time. the return value *eeprom is just
    // a pointer to the data stored in the internal reply packet, hence it must be copied elsewhere at reception.
    // capability to be checked is:
    // - uprot_canDO_EEPROM_read:           it reads a portion of eeprom contained in [from, from+size) with size <= uprot_EEPROMmaxsize (1024) bytes.
    bool cmdReadEEPROM(uint16_t from, uint16_t size, ACE_UINT32 address, uint8_t **value);

    // changes the IP address of a given board. despite default address 0, it does not support it
    // capability to be checked is one or both of the following:
    // - uprot_canDO_LEGACY_IPaddr_set:         it just changes address.
    // - uprot_canDO_IPaddr_set:                it changes address and it also may force a restart (not requested for now)
    void cmdChangeAddress(ACE_UINT32 newaddress, ACE_UINT32 address = 0);


    // used to program a given partition of a given board
    // capability to be checked is:
    // - uprot_canDO_PROG_loader:       for programming into the loader partition (the first)
    // - uprot_canDO_PROG_updater:      for programming into the updater partition (the second)
    // - uprot_canDO_PROG_application:  for programming into the application partition (the third)
    std::string cmdProgram(FILE *programFile, int partition, void (*updateProgressBar)(float), ACE_UINT32 address = 0);

protected:

    void sendCommandSelected(void * cmd, uint16_t len);
    int sendPROG(const uint8_t opc, void * data, int size, int answers, int retry);

private:

    // in here are methods which we prefer not to be used, but which may be useful in special cases

    // changes the IP mask of a given board, despite default address 0, it does not support it
    // capability to be checked is:
    // - uprot_canDO_LEGACY_IPmask_set
    // NOTE: it is very dangerous to change the IP mask. it is also very improbable that we want to do it. we keep the method however.
    void cmdChangeMask(ACE_UINT32 newMask, ACE_UINT32 address = 0);

    // changes the MAC of a given board
    // capability to be checked is:
    // - uprot_canDO_LEGACY_MAC_set
    // NOTE: we may change the MAC only if ... there are two equal mac addresses on two boards ...
    void cmdChangeMAC(uint64_t newMAC48, ACE_UINT32 address);

    bool cmdPageClr(eOuprot_pagesize_t pagesize, ACE_UINT32 address = 0);
    bool cmdPageSet(eOuprot_pagesize_t pagesize, uint8_t *data, ACE_UINT32 address = 0);
    bool cmdPageGet(eOuprot_pagesize_t pagesize, uint8_t **data, ACE_UINT32 address);

};

#endif

// eof

