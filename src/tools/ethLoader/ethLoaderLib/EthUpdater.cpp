/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "EthUpdater.h"

#include <ace/ACE.h>
#include <yarp/os/Time.h>

#include <yarp/os/Log.h>

using namespace yarp::os;

const int EthUpdater::PROGRAM_APP=0x5A;
const int EthUpdater::PROGRAM_LOADER=0x55;
const int EthUpdater::PROGRAM_UPDATER=0xAA;

void EthUpdater::cmdScan()
{
    cmdScan2();
}

void EthUpdater::cmdScan1()
{
    // it sends a can command in the old format.
    // it can be decoded by new boards as well.

    mBoardList.empty();

    mTxBuffer[0]=CMD_SCAN;
    //mSocket.SendTo(mTxBuffer,1,mPort,0x0A000163);
    mSocket.SendBroad(mTxBuffer,1,mPort);

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress=mMyAddress;

    while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,1000)>0)
    {
        if (mRxBuffer[0]==CMD_SCAN)
        {
            if (rxAddress!=mMyAddress)
            {
                printf("ADDRESS=%x",rxAddress);
                fflush(stdout);

                ACE_UINT8 major = mRxBuffer[1];
                ACE_UINT8 minor = mRxBuffer[2];
                ACE_UINT8 typeofboard = mRxBuffer[3];

                ACE_UINT32 mask=*(ACE_UINT32*)(mRxBuffer+4);
                printf(" mask=%x\n",mask);
                ACE_UINT64 mac=0;

                for (int i=13; i>=8; --i)
                {
                    mac=(mac<<8)|mRxBuffer[i];
                }

                BoardInfo *pBoard=new BoardInfo(rxAddress, mask, mac, major, minor);

                mBoardList.addBoard(pBoard);
            }
        }
    }
}

//typedef struct
//{
//    uint32_t            year  : 12;    /**< the year a.d. upto 2047 */
//    uint32_t            month : 4;     /**< the month, where Jan is 1, Dec is 12 */
//    uint32_t            day   : 5;     /**< the day from 1 to 31 */
//    uint32_t            hour  : 5;     /**< the hour from 0 to 23 */
//    uint32_t            min   : 6;     /**< the minute from 0 to 59 */
//} eO_E_date_t;

void eo_E_common_date_to_string(eO_E_date_t date, char *str, uint8_t size)
{
    static const char * months[16] = {"ERR", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec", "ERR", "ERR", "ERR"};
    if(NULL != str)
    {
        snprintf(str, size, "%d %s %.2d %d:%d", date.year, months[date.month], date.day, date.hour, date.min);
    }
}

const char * eo_E_common_proc_to_string(uint8_t proc)
{
    static const char * procs[4] = {"eLoader", "eUpdater", "eApplication", "ERR"};
    if(proc >= 3)
    {
        return(procs[3]);
    }

    return(procs[proc]);
}

//typedef struct
//{
//    uint8_t     type;
//    uint8_t     major;
//    uint8_t     minor;
//    eO_E_date_t builddate;
//} eO_E_procInfo_t;

//typedef struct
//{
//    uint8_t         protversion;
//    uint64_t        macaddress;
//    uint8_t         boardtype;
//    uint8_t         startup;
//    uint8_t         def2run;
//    uint8_t         nprocs;
//    eO_E_procInfo_t procinfo[3];
//    uint8_t         runningproc;
//    uint8_t         info32[32];
//} scan2rxdata_t;

void EthUpdater::cmdScan2()
{
    // it sends the old scan command, ... but it is able to process replies form both old and new boards
    // - the old boards send the old information
    // - the new boards send additional information
    // the receiver knows if we have an old or a new format on the basis of the received OPCODE

    mBoardList.empty();

    mTxBuffer[0] = CMD_SCAN;
    mTxBuffer[1] = CMD_SCAN2;

    mSocket.SendBroad(mTxBuffer, 2, mPort);

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress = mMyAddress;

    while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,1000)>0)
    {
        if (CMD_SCAN2 == mRxBuffer[0])
        {
            // new format

            if (rxAddress!=mMyAddress)
            {
                //printf("ADDRESS=%x",rxAddress);
                fflush(stdout);

                scan2rxdata_t rxdata = {0};

                rxdata.protversion = mRxBuffer[1];

                memcpy(&rxdata.macaddress, &mRxBuffer[2], 6);
                rxdata.boardtype = mRxBuffer[8];
                rxdata.startup = mRxBuffer[9];
                rxdata.def2run = mRxBuffer[10];
                rxdata.nprocs = mRxBuffer[11];


                for(int i=0; i<rxdata.nprocs; i++)
                {
                    rxdata.procinfo[i].type  = mRxBuffer[12+8*i];
                    rxdata.procinfo[i].major = mRxBuffer[13+8*i];
                    rxdata.procinfo[i].minor = mRxBuffer[14+8*i];
                    memcpy(&rxdata.procinfo[i].builddate, &mRxBuffer[15+8*i], 4);
                }

                rxdata.runningproc = mRxBuffer[36];

                // dont use mRxBuffer[37, 8, 9]

                memcpy(rxdata.info32, &mRxBuffer[40], 32);

//#warning -------------------> TODO.marco.accame: use eOboard_t values

                printf("\nBOARD found at address %x:", rxAddress);
                printf("\n prot = %d, boardtype = %s, startup proc = %s, def2run proc = %s. it has %d processes:",
                            rxdata.protversion,
                            (32 == rxdata.boardtype)? "EMS" : "MC4PLUS",
                            eo_E_common_proc_to_string(rxdata.startup),
                            eo_E_common_proc_to_string(rxdata.def2run),
                            rxdata.nprocs
                       );
                for(int n=0; n<rxdata.nprocs; n++)
                {
                    char strdate[24] = {0};
                    eo_E_common_date_to_string(rxdata.procinfo[n].builddate, strdate, sizeof(strdate));
                    printf("\n proc-%d: type %s w/ appl version = (%d, %d), built on %s",
                           n,
                           eo_E_common_proc_to_string(rxdata.procinfo[n].type),
                           rxdata.procinfo[n].major, rxdata.procinfo[n].minor,
                           strdate
                           );

                }

                printf("\n now process %s is running", eo_E_common_proc_to_string(rxdata.runningproc));

                if(0xff == rxdata.info32[0])
                {
                    printf("\n stored info32 is .. ");
                    for(int m=0; m<32; m++)
                    {
                        printf("0x%x ", rxdata.info32[m]);
                    }

                }
                else
                {
                    printf("\n stored info32 is .. ");
                    printf("l = %d, string = %s \n", rxdata.info32[0], &rxdata.info32[1]);
                }

//                printf("\n stored info32 is .. ");
//                for(int m=0; m<32; m++)
//                {
//                    printf("0x%x ", rxdata.info32[m]);
//                }
                printf("\n\n");


                ACE_UINT32 mask = 0xFFFFFF00; // fixed


                ACE_UINT8 major = rxdata.procinfo[rxdata.runningproc].major;
                ACE_UINT8 minor = rxdata.procinfo[rxdata.runningproc].minor;
                ACE_UINT64 mac    = rxdata.macaddress;

//                #warning -------------> TODO.marco.accame: change to add in BoardInfo new information

                //BoardInfo *pBoard = new BoardInfo(rxAddress, mask, mac, major, minor);

                BoardInfo *pBoard = new BoardInfo(rxAddress, rxdata);

                mBoardList.addBoard(pBoard);
            }
        }
        else if (mRxBuffer[0]==CMD_SCAN)
        {
            if (rxAddress!=mMyAddress)
            {
                printf("ADDRESS=%x",rxAddress);
                fflush(stdout);

                ACE_UINT8 major = mRxBuffer[1];
                ACE_UINT8 minor = mRxBuffer[2];
                ACE_UINT8 typeofboard  = mRxBuffer[3];

                ACE_UINT32 mask=*(ACE_UINT32*)(mRxBuffer+4);
                printf(" mask=%x\n",mask);
                ACE_UINT64 mac=0;

                for (int i=13; i>=8; --i)
                {
                    mac=(mac<<8)|mRxBuffer[i];
                }

                BoardInfo *pBoard = new BoardInfo(rxAddress, mask, mac, major, minor);

                mBoardList.addBoard(pBoard);
            }
        }
    }
}

std::string EthUpdater::cmdProgram(FILE *programFile,int partition,void (*updateProgressBar)(float))
{
    updateProgressBar(0.0f);

    const int UPD_OK=0;
    const int HEAD_SIZE=7;

    fseek(programFile,0,SEEK_END);
    float fileSize=(float)(ftell(programFile)/3);
    fseek(programFile,0,SEEK_SET);

    mTxBuffer[0]=CMD_START;
    mTxBuffer[1]=(unsigned char)partition;

    //mTxBuffer[1]=PROGRAM_APP;
    //mTxBuffer[1]=PROGRAM_LOADER;
    //mTxBuffer[1]=PROGRAM_UPDATER;

    mN2Prog=mNProgSteps=0;

    mNChunks=1;

    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mBoard2Prog[mN2Prog++]=&mBoardList[i];
            mBoardList[i].mSuccess=0;
            mSocket.SendTo(mTxBuffer,2,mPort,mBoardList[i].mAddress);
            yarp::os::Time::delay(0.01);
        }
    }

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    ++mNProgSteps;

    int success=0;

    for (int n=0; n<1000; ++n)
    {
        while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,10)>0)
        {
            if (mRxBuffer[0]==CMD_START)
            {
                if (rxAddress!=mMyAddress)
                {
                    for (int i=0; i<mN2Prog; ++i)
                    {
                        if (rxAddress==mBoard2Prog[i]->mAddress)
                        {
                            if (mRxBuffer[1]==UPD_OK)
                            {
                                ++(mBoard2Prog[i]->mSuccess);
                            }

                            if (++success>=mN2Prog) n=1000;
                        }
                    }
                }
            }
        }
    }

    int addrH=0;
    int baseAddress=0;
    int bytesToWrite=0;
    int bytesWritten=0;

    char buffer[1024];

    bool beof=false;

    while (!beof && fgets(buffer,1024,programFile))
    {
        std::string line(buffer);

        int cmd=strtol(line.substr(7,2).c_str(),NULL,16);

        switch (cmd)
        {
        case 0: //standard data record        
            {
                int size =strtol(line.substr(1,2).c_str(),NULL,16);
                int addrL=strtol(line.substr(3,4).c_str(),NULL,16);

                int address=addrH<<16|addrL;

                if (!baseAddress) baseAddress=address;

                if (bytesToWrite+size>PACKET_SIZE || address!=baseAddress+bytesToWrite)
                {
                    if (bytesToWrite)
                    {
                        mTxBuffer[5]= bytesToWrite    &0xFF;
                        mTxBuffer[6]=(bytesToWrite>>8)&0xFF;
                        sendDataBroadcast(mTxBuffer,HEAD_SIZE+bytesToWrite,mN2Prog,1000);
                        updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                        bytesToWrite=0;
                    }
                }

                if (!bytesToWrite)
                {
                    baseAddress=address;
                    mTxBuffer[0]=CMD_DATA;
                    mTxBuffer[1]=addrL&0xFF;
                    mTxBuffer[2]=(addrL>>8)&0xFF;
                    mTxBuffer[3]=addrH&0xFF;
                    mTxBuffer[4]=(addrH>>8)&0xFF;
                }

                for (int i=0; i<size; ++i)
                {
                    mTxBuffer[HEAD_SIZE+bytesToWrite+i]=(unsigned char)strtol(line.substr(i*2+9,2).c_str(),NULL,16);
                }

                bytesToWrite+=size;

                break;
            }
        case 1: //end of file
            if (bytesToWrite) // force write
            {
                beof=true;
                mTxBuffer[5]= bytesToWrite    &0xFF;
                mTxBuffer[6]=(bytesToWrite>>8)&0xFF;
                sendDataBroadcast(mTxBuffer,HEAD_SIZE+bytesToWrite,mN2Prog,1000);
                updateProgressBar(1.0f);
                bytesToWrite=0;
            }
            break;
        case 2:
        case 3:
            //AfxMessageBox("Unsupported hex commad");
            break;
        case 4: //extended linear address record
            {
                if (bytesToWrite) // force write
                {
                    mTxBuffer[5]= bytesToWrite    &0xFF;
                    mTxBuffer[6]=(bytesToWrite>>8)&0xFF;
                    sendDataBroadcast(mTxBuffer,HEAD_SIZE+bytesToWrite,mN2Prog,1000);
                    updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                    bytesToWrite=0;
                }

                addrH=strtol(line.substr(9,4).c_str(),NULL,16);

                break;
            }
        case 5: // jump
            if (bytesToWrite) // force write
            {
                mTxBuffer[5]= bytesToWrite    &0xFF;
                mTxBuffer[6]=(bytesToWrite>>8)&0xFF;
                sendDataBroadcast(mTxBuffer,HEAD_SIZE+bytesToWrite,mN2Prog,1000);
                updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                bytesToWrite=0;
            }

            break;
        }
    }

    mTxBuffer[0]=CMD_END;
    mTxBuffer[1]=mNChunks&0xFF;
    mTxBuffer[2]=(mNChunks>>8)&0xFF;
    sendDataBroadcast(mTxBuffer,3,mN2Prog,1000);

    updateProgressBar(1.0f);

    std::string sOutput;
    char addr[16];

    for (int i=0; i<mN2Prog; ++i)
    {
        ACE_UINT32 ip=mBoard2Prog[i]->mAddress;
        sprintf(addr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
        sOutput+=addr;
        sOutput+=(mBoard2Prog[i]->mSuccess==mNProgSteps)?"\tsuccess\r\n":"\tFAILED\r\n";
    }

    return sOutput;
}

void EthUpdater::cmdBootSelect(unsigned char sector)
{
    mTxBuffer[0]=CMD_BOOT;
    mTxBuffer[1]=sector;

    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mSocket.SendTo(mTxBuffer,2,mPort,mBoardList[i].mAddress);
        }
    }
}


void EthUpdater::cmdEraseEprom()
{
    sendCommandSelected(CMD_SYSEEPROMERASE);
}

void EthUpdater::sendCommandSelected(unsigned char command)
{
    mTxBuffer[0]=command;

    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mSocket.SendTo(mTxBuffer,1,mPort,mBoardList[i].mAddress);
        }
    }
}


void EthUpdater::sendCommandSelected(uint8_t * cmd, uint16_t len)
{
    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mSocket.SendTo(cmd, len, mPort, mBoardList[i].mAddress);
        }
    }
}


void EthUpdater::cmdInfo32Clear()
{
    sendCommandSelected(CMD_INFO_CLR);
}


vector<string> EthUpdater::cmdInfo32Get()
{
    vector<string> thestrings(0);


    uint8_t cmd[2] = { CMD_INFO_GET, 32};

    sendCommandSelected(cmd, sizeof(cmd));

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;


    while(mSocket.ReceiveFrom(mRxBuffer, 1024, rxAddress, rxPort, 500)>0)
    {
        if((CMD_INFO_GET == mRxBuffer[0]) && (32 == mRxBuffer[1]))
        {

            if(rxAddress != mMyAddress)
            {
                string readstring;

                char ip32[20];
                snprintf(ip32, sizeof(ip32), "%d.%d.%d.%d",(rxAddress>>24)&0xFF, (rxAddress>>16)&0xFF, (rxAddress>>8)&0xFF, rxAddress&0xFF);

                printf("\n received info32 from board IP = %s: ", ip32);

                if(0 != mRxBuffer[2])
                {
                    printf("failure w/ value %d", mRxBuffer[2]);
                }
                else
                {
                    if(0xff != mRxBuffer[3])
                    {
                        readstring = std::string((const char*)&mRxBuffer[4]);
                    }

                    uint8_t info32[32] = {0};
                    memcpy(info32, &mRxBuffer[3], 32);

                    if(0xff == info32[0])
                    {
                        printf("\n stored info32 is .. ");
                        for(int m=0; m<32; m++)
                        {
                            printf("0x%x ", info32[m]);
                        }

                    }
                    else
                    {
                        printf("l = %d, string = %s \n", info32[0], &info32[1]);
                    }

                }

                thestrings.push_back(readstring);

                fflush(stdout);

            }
        }
    }

    return thestrings;

}


void EthUpdater::cmdInfo32Set(const string &info32)
{
    uint8_t cmd[2+32] = {0};
    cmd[0] = CMD_INFO_SET;
    cmd[1] = 32;

    const char * str32 = info32.c_str();

    int len = strlen(str32);
    if(len>30)
    {
        len = 30;
    }
    cmd[2] = len;
    memcpy(&cmd[3], str32, len);

    sendCommandSelected(cmd, sizeof(cmd));
}


void EthUpdater::cmdReset()
{
    sendCommandSelected(CMD_RESET);
}

void EthUpdater::cmdJumpUpd()
{
    sendCommandSelected(CMD_JUMP_UPD);
}

void EthUpdater::cmdBlink()
{
    sendCommandSelected(CMD_BLINK);
}

void EthUpdater::cmdChangeAddress(ACE_UINT32 address,ACE_UINT32 newAddr)
{
    mTxBuffer[0]=CMD_CHADDR;

    mTxBuffer[1]=(newAddr>>24)&0xFF;
    mTxBuffer[2]=(newAddr>>16)&0xFF;
    mTxBuffer[3]=(newAddr>>8) &0xFF;
    mTxBuffer[4]= newAddr     &0xFF;

    printf("change unit %x address to %x\n",address,newAddr);

    mSocket.SendTo(mTxBuffer,5,mPort,address);
}

void EthUpdater::cmdChangeMask(ACE_UINT32 address,ACE_UINT32 newMask)
{
    mTxBuffer[0]=CMD_CHMASK;

    mTxBuffer[1]=(newMask>>24)&0xFF;
    mTxBuffer[2]=(newMask>>16)&0xFF;
    mTxBuffer[3]=(newMask>>8) &0xFF;
    mTxBuffer[4]= newMask     &0xFF;

    printf("change unit %x mask to %x\n",address,newMask);

    mSocket.SendTo(mTxBuffer,5,mPort,address);
}

std::string EthUpdater::cmdGetProcs()
{
    sendCommandSelected(CMD_PROCS);

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    std::string info;

    while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,500)>0)
    {
        if (mRxBuffer[0]==CMD_PROCS)
        {
            if (rxAddress!=mMyAddress)
            {
                char buff[16];
                sprintf(buff,"%d.%d.%d.%d",(rxAddress>>24)&0xFF,
                    (rxAddress>>16)&0xFF,
                    (rxAddress>>8)&0xFF,
                    rxAddress&0xFF);

                info+="------------------------------\r\n";
                info+=std::string("Board\t")+std::string(buff);
                info+="\r\n\r\n";
                info+=std::string((char*)mRxBuffer+2);
            }
        }
    }

    return info;
}


std::string EthUpdater::cmdGetProcs2()
{
    // this method sends a PROCS request in such a way that:
    // - if we have an old board, we receive just a string full of pre-formatted info
    // - if we have a new board, we receive the new SCAN data structure and then the string (for backward compatibility).
    // 

    uint8_t cmd[2] = { CMD_PROCS, CMD_PROCS2};

    sendCommandSelected(cmd, sizeof(cmd));

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    std::string info;

    while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,500)>0)
    {
        if (mRxBuffer[0]== CMD_PROCS)
        {   // old boards reply with CMD_PROCS

            printf("\n received a CMD_PROCS\n");
            fflush(stdout);

            if (rxAddress != mMyAddress)
            {
                char buff[16];
                sprintf(buff,"%d.%d.%d.%d",(rxAddress>>24)&0xFF,
                    (rxAddress>>16)&0xFF,
                    (rxAddress>>8)&0xFF,
                    rxAddress&0xFF);

                info+="------------------------------\r\n";
                info+=std::string("Board\t")+std::string(buff);
                info+="\r\n\r\n";
                info+=std::string((char*)mRxBuffer+2);
            }
        }
        else if(mRxBuffer[0] == CMD_PROCS2)
        {   // new boards reply with CMD_PROCS2
            // the format is the same as the reply to CMD_SCAN in the first 40+32 bytes. then we we have ...

            printf("\n received a CMD_PROCS2\n");
            fflush(stdout);

            if (rxAddress != mMyAddress)
            {
                scan2rxdata_t rxdata = {0};
                
                // fill rxdata

                rxdata.protversion = mRxBuffer[1];

                memcpy(&rxdata.macaddress, &mRxBuffer[2], 6);
                rxdata.boardtype = mRxBuffer[8];
                rxdata.startup = mRxBuffer[9];
                rxdata.def2run = mRxBuffer[10];
                rxdata.nprocs = mRxBuffer[11];


                for(int i=0; i<rxdata.nprocs; i++)
                {
                    rxdata.procinfo[i].type  = mRxBuffer[12+8*i];
                    rxdata.procinfo[i].major = mRxBuffer[13+8*i];
                    rxdata.procinfo[i].minor = mRxBuffer[14+8*i];
                    memcpy(&rxdata.procinfo[i].builddate, &mRxBuffer[15+8*i], 4);
                }

                rxdata.runningproc = mRxBuffer[36];

                // dont use mRxBuffer[37, 8, 9]

                memcpy(rxdata.info32, &mRxBuffer[40], 32);

#if 1
                // print rxdata. 
                // it would be much bettwer, however, store it somewhere and made it available
                // through some method
#warning --> TODO.marco.accame: store rxdata somewhere

                printf("\nBOARD at address %x:", rxAddress);
                printf("\n prot = %d, boardtype = %s, startup proc = %s, def2run proc = %s. it has %d processes:",
                            rxdata.protversion,
                            (32 == rxdata.boardtype)? "EMS" : "MC4PLUS",
                            eo_E_common_proc_to_string(rxdata.startup),
                            eo_E_common_proc_to_string(rxdata.def2run),
                            rxdata.nprocs
                       );
                for(int n=0; n<rxdata.nprocs; n++)
                {
                    char strdate[24] = {0};
                    eo_E_common_date_to_string(rxdata.procinfo[n].builddate, strdate, sizeof(strdate));
                    printf("\n proc-%d: type %s w/ appl version = (%d, %d), built on %s",
                           n,
                           eo_E_common_proc_to_string(rxdata.procinfo[n].type),
                           rxdata.procinfo[n].major, rxdata.procinfo[n].minor,
                           strdate
                           );

                }

                printf("\n now process %s is running", eo_E_common_proc_to_string(rxdata.runningproc));

                if(0xff == rxdata.info32[0])
                {
                    printf("\n stored info32 is .. ");
                    for(int m=0; m<32; m++)
                    {
                        printf("0x%x ", rxdata.info32[m]);
                    }

                }
                else
                {
                    printf("\n stored info32 is .. ");
                    printf("l = %d, string = %s \n", rxdata.info32[0], &rxdata.info32[1]);
                }

                printf("\n\n");

#endif
                // now we put into info all what is ..... in mRxBuffer[40+32] and beyond

                {
                    char buff[16];
                    sprintf(buff,"%d.%d.%d.%d",(rxAddress>>24)&0xFF,
                        (rxAddress>>16)&0xFF,
                        (rxAddress>>8)&0xFF,
                        rxAddress&0xFF);

                    info+="------------------------------\r\n";
                    info+=std::string("Board\t")+std::string(buff);
                    info+="\r\n\r\n";
                    info+=std::string((char*)&mRxBuffer[40+32]);
                }
            }
        }
    }

    return info;
}

int EthUpdater::sendDataBroadcast(unsigned char* data,int size,int answers,int retry)
{
    const int UPD_OK=0;

#if 0
    //mSocket.SendTo(data,size,mPort,mBroadcast);
    mSocket.SendBroad(data,size,mPort);
#else
    // use multicast to all selected boards
    for(int k=0;k<mN2Prog; k++)
    {
        mSocket.SendTo(data, size, mPort, mBoard2Prog[k]->mAddress);
    }
#endif
    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    unsigned char cmd=data[0];

    ++mNChunks;

    if (answers)
    {
        ++mNProgSteps;

        for (int r=0; r<retry; ++r)
        {
            for (int a=0; a<answers; ++a)
            {
                if (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,10)>0)
                {
                    if (mRxBuffer[0]==cmd)
                    {
                        if (rxAddress!=mMyAddress)
                        {
                            for (int i=0; i<mN2Prog; ++i)
                            {
                                if (rxAddress==mBoard2Prog[i]->mAddress)
                                {
                                    if (mRxBuffer[1]==UPD_OK)
                                    {
                                        ++(mBoard2Prog[i]->mSuccess);
                                    }
                                    break;
                                }
                            }

                            if (!--answers) return 0;
                        }
                    }
                }
            }
        }
    }

    return answers;
}



/*
void EthUpdater::cmdGetShals()
{
POSITION pos=mListBoards.GetFirstSelectedItemPosition();

if (pos==NULL)
{
AfxMessageBox("No board selected");
return;
}

mTxBuffer[0]=CMD_SHALS;

while (pos)
{
int nItem=mListBoards.GetNextSelectedItem(pos);  
CString address=mListBoards.GetItemText(nItem,0);
mSocket.SendTo(mTxBuffer,1,mPort,address);
}

UINT rxPort;
CString rxAddress;

Sleep(500);

CString info;

int ip1,ip2,ip3,ip4;

while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort)>0)
{
if (mRxBuffer[0]==CMD_SHALS)
{
sscanf_s(rxAddress,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
UINT addr=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;

if (addr!=mMyAddress)
{
mOutputBox+="------------------------------\r\n";
mOutputBox+=CString("Board\t")+rxAddress;
mOutputBox+="\r\n\r\n";
mOutputBox+=CString(mRxBuffer+2);
}
}
}

UpdateData(FALSE);
}
*/
