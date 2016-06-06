/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_H__
#define __ETHUPDATER_H__

#define CMD_SCAN           0xFF
#define CMD_SCAN2          0x7F

#define CMD_START          0x01
#define CMD_DATA           0x02
#define CMD_JUMP           0x03
#define CMD_END            0x04
#define CMD_BOOT           0x05
#define CMD_RESET          0x06
#define CMD_CHADDR         0x07
#define CMD_CHMASK         0x08
#define CMD_PROCS          0x09
#define CMD_PROCS2         0x79
#define CMD_SYSEEPROMERASE 0x12

#define CMD_INFO_CLR        0x30
#define CMD_INFO_SET        0x31
#define CMD_INFO_GET        0x32

#define CMD_SHALS      0x0A
#define CMD_BLINK      0x0B
#define CMD_JUMP_UPD   0x0C

#define PACKET_SIZE 512

#define BOARD_TYPE_EMS 0x0A

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

    unsigned char mRxBuffer[1024];
    unsigned char mTxBuffer[1024];

    ACE_UINT16 mPort;
    ACE_UINT32 mBroadcast;
    ACE_UINT32 mMyAddress;
    
    DSocket mSocket;

public:
    static const int PROGRAM_APP;
    static const int PROGRAM_LOADER;
    static const int PROGRAM_UPDATER;

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

    std::string cmdGetProcs();
    std::string cmdGetProcs2();
    std::string cmdProgram(FILE *programFile, int partition, void (*updateProgressBar)(float));

    void cmdScan();
    void cmdScan1();
    void cmdScan2();
    void cmdBootSelect(unsigned char sector);
    void sendCommandSelected(unsigned char command);
    void sendCommandSelected(uint8_t * cmd, uint16_t len);
    void cmdReset();
    void cmdJumpUpd();
    void cmdBlink();
    void cmdEraseEprom();
    void cmdChangeAddress(ACE_UINT32 oldAddr,ACE_UINT32 newAddr);
    void cmdChangeMask(ACE_UINT32 oldAddr,ACE_UINT32 newAddr);

    void cmdInfo32Clear();
    void cmdInfo32Set(const string &info32);
    vector<string> cmdInfo32Get();
   
protected:
    int sendDataBroadcast(unsigned char* data,int size,int answers,int retry);
};

#endif
