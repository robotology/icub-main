/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "EthUpdater.h"

void EthUpdater::cmdScan()
{
    mBoardList.empty();

    mTxBuffer[0]=CMD_SCAN;
    mSocket.SendTo(mTxBuffer,1,mPort,mBroadcast);

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    while (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,1000)>0)
    {
        if (mRxBuffer[0]==CMD_SCAN)
        {
            if (rxAddress!=mMyAddress)
            {
                ACE_UINT8 version=mRxBuffer[1];
                ACE_UINT8 release=mRxBuffer[2];
                ACE_UINT8 build  =mRxBuffer[3];

                ACE_UINT32 mask=*(ACE_UINT32*)(mRxBuffer+4);
                ACE_UINT64 mac=0;

                for (int i=8; i<14; ++i)
                {
                    mac=(mac<<8)|mRxBuffer[i];
                }

                BoardInfo *pBoard=new BoardInfo(rxAddress,mask,mac,version,release,build);

                mBoardList.addBoard(pBoard);
            }
        }
    }
}

std::string EthUpdater::cmdProgram(FILE *programFile,void (*updateProgressBar)(float))
{
    updateProgressBar(0.0f);

    const int UPD_OK=0;
    const int HEAD_SIZE=7;

    fseek(programFile,0,SEEK_END);
    float fileSize=(float)(ftell(programFile)/2);
    fseek(programFile,0,SEEK_SET);

    mTxBuffer[0]=CMD_START;
    mTxBuffer[1]=0x5A; // application updater

    mN2Prog=mNProgSteps=0;

    mNChunks=1;

    ACE_Time_Value tv_sleep(0,10000);

    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mBoard2Prog[mN2Prog++]=&mBoardList[i];
            mBoardList[i].mSuccess=0;
            mSocket.SendTo(mTxBuffer,2,mPort,mBoardList[i].mAddress);
            ACE_OS::sleep(tv_sleep);
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
                int size=strtol(line.substr(1,2).c_str(),NULL,16);
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
        sprintf(addr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>24)&0xFF,ip&0xFF);
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

void EthUpdater::cmdChangeAddressAndMask(ACE_UINT32 oldAddr,ACE_UINT32 newAddr,bool bMask)
{
    mTxBuffer[0]=bMask?CMD_CHMASK:CMD_CHADDR;

    mTxBuffer[1]=(newAddr>>24)&0xFF;
    mTxBuffer[2]=(newAddr>>16)&0xFF;
    mTxBuffer[3]=(newAddr>>8) &0xFF;
    mTxBuffer[4]= newAddr     &0xFF;

    mSocket.SendTo(mTxBuffer,5,mPort,oldAddr);
}

std::string EthUpdater::cmdGetProcs()
{
    sendCommandSelected(CMD_PROCS);

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    ACE_OS::sleep(ACE_Time_Value(0,500000));

    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mBoard2Prog[mN2Prog++]=&mBoardList[i];
            mBoardList[i].mSuccess=0;
            mSocket.SendTo(mTxBuffer,2,mPort,mBoardList[i].mAddress);
            
        }
    }

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

int EthUpdater::sendDataBroadcast(unsigned char* data,int size,int answers,int retry)
{
    const int UPD_OK=0;

    mSocket.SendTo(data,size,mPort,mBroadcast);

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
                if (mSocket.ReceiveFrom(mRxBuffer,1024,rxAddress,rxPort,1)>0)
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
