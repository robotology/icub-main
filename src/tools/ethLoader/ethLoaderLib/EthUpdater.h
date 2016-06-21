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

    // used to discover all the available boards.
    void cmdDiscover();

    // used to refresh the info about a board (or request more)
    std::string cmdGetMoreInfo(bool refreshInfo = false, ACE_UINT32 address = 0);

    // used to manage the 32-bytes field which is stored in EEPROM of each board
    void cmdInfo32Clear(ACE_UINT32 address = 0);
    void cmdInfo32Set(const string &info32, ACE_UINT32 address = 0);
    vector<string> cmdInfo32Get(ACE_UINT32 address = 0);

    // it imposes the default to run process (the one after teh 5 seconds)
    void cmdSetDEF2RUN(eOuprot_process_t process);

    // forces a restart of selected boards
    void cmdRestart();

    // forces a jump to the eUpdater (only if the eApplication called eMaintainer is running)
    void cmdJumpUpd();

    // forces a series of blinks in the LEDs of selected boards
    void cmdBlink();

    // forces a full erase of the EEPROM
    void cmdEraseEEPROM();

    // changes the IP address of a given board
    void cmdChangeAddress(ACE_UINT32 address, ACE_UINT32 newAddr);

    // changes the IP mask of a given board
    void cmdChangeMask(ACE_UINT32 address, ACE_UINT32 newMask);

    // used to program a given partition of selected boards
    std::string cmdProgram(FILE *programFile, int partition, void (*updateProgressBar)(float));

    std::string cmdProgramOLD(FILE *programFile, int partition, void (*updateProgressBar)(float));

    //void sendCommandSelected(unsigned char command);
    void sendCommandSelected(void * cmd, uint16_t len);

   
protected:
    int sendPROG(const uint8_t opc, void * data, int size, int answers, int retry);
};

#endif

// eof

