
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


#include "EthMaintainer.h"

#include <ace/ACE.h>
#include <yarp/os/Time.h>

#include <yarp/os/Log.h>

using namespace yarp::os;


const eOipv4addr_t EthMaintainer::hostIPaddress = EO_COMMON_IPV4ADDR(10, 0, 1, 104);
const eOipv4port_t EthMaintainer::mainIPport = 3333;

const eOipv4addr_t EthMaintainer::ipv4OfAllSelected = EO_COMMON_IPV4ADDR(0, 0, 0, 0);

const eOipv4addr_t EthMaintainer::ipv4Broadcast = EO_COMMON_IPV4ADDR(255, 255, 255, 255);

const int EthMaintainer::partition_APPLICATION = uprot_partitionAPPLICATION;
const int EthMaintainer::partition_LOADER = uprot_partitionLOADER;
const int EthMaintainer::partition_UPDATER = uprot_partitionUPDATER;



EthMaintainer::EthMaintainer()
{
    _opened = false;
    _verbose = false;
    _debugprint = true;
}

EthMaintainer::~EthMaintainer()
{
    close();
}



bool EthMaintainer::open(eOipv4addr_t ipv4, eOipv4port_t port)
{
    if(!_opened)
    {
        myIPV4addr = ipv4;
        myIPV4port = port;

        _opened = mSocket.Create(ipv4, port);

        return _opened;
    }
    else if((myIPV4addr == ipv4) && (myIPV4port == port))
    {
        return true;
    }

    return false;
}


bool EthMaintainer::close()
{
    if(_opened)
    {
        mSocket.Close();
        mEthBoards.clear();
        _opened = false;
    }
}


void EthMaintainer::setVerbose(bool on)
{
    _verbose = on;
}


void EthMaintainer::setDebugPrint(bool on)
{
    _debugprint = on;
}

EthBoardList& EthMaintainer::getBoards()
{
    return mEthBoards;
}


void EthMaintainer::clearBoards()
{
    mEthBoards.clear();
}

int EthMaintainer::addBoard(eOipv4addr_t ipv4)
{
    boardInfo2_t info2;

    bool force = true;
    mEthBoards.add(info2, ipv4, force);

    return mEthBoards.size();
}

int EthMaintainer::remBoard(eOipv4addr_t ipv4)
{
    mEthBoards.rem(ipv4);
    return mEthBoards.size();
}


eOipv4addr_t EthMaintainer::ACEtoIPV4(ACE_UINT32 address)
{
    return htonl(address);
}


ACE_UINT32 EthMaintainer::IPV4toACE(eOipv4addr_t ipv4)
{
    return ntohl(ipv4);
}


bool EthMaintainer::isCommandSupported(eOuprot_proc_capabilities_t capability, eOipv4addr_t ipv4)
{
    // search for the BoardInfo with a given address ... if we have a double address then we return a vector
    // we also may return a vector if address is 0 because we search for the selected.
    vector<EthBoard *> boards = mEthBoards.get(ipv4);

    if(0 == boards.size())
    {
        return false;
    }

    // i assume it is true. i return true only if all the boards support it.
    bool ret = true;
    for(int i=0; i<boards.size(); i++)
    {
        uint32_t mask = boards[i]->getInfo().capabilities;
        bool r = ((mask & capability) == capability) ? true : false;
        ret = ret && r;
    }

    return ret;
}


int EthMaintainer::commandDiscover(bool clearboardsbeforediscovery, eOipv4addr_t ipv4)
{

    if(clearboardsbeforediscovery)
    {
        clearBoards();
    }

    const bool forceUpdatingMode = false;

    eOuprot_cmd_DISCOVER_t * cmd = (eOuprot_cmd_DISCOVER_t*) mTxBuffer;

    memset(cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_DISCOVER_t));

    cmd->opc = uprot_OPC_LEGACY_SCAN;
    cmd->opc2 = uprot_OPC_DISCOVER;

    cmd->filler[0] = (true == forceUpdatingMode) ? (1) : (0);


    // we send the discovery in ...
    if(ipv4Broadcast == ipv4)
    {
        mSocket.SendBroad(myIPV4port, cmd, sizeof(eOuprot_cmd_DISCOVER_t));
    }
    else
    {
        sendCommand(ipv4, cmd, sizeof(eOuprot_cmd_DISCOVER_t));
    }

    processDiscoveryReplies();

    return mEthBoards.size();
}



bool EthMaintainer::commandForceMaintenance(eOipv4addr_t ipv4, bool verify, int retries, double timegap)
{
    const bool forceUpdatingMode = true;

    eOuprot_cmd_DISCOVER_t command;

    memset(&command, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_DISCOVER_t));

    command.opc = uprot_OPC_LEGACY_SCAN;
    command.opc2 = uprot_OPC_DISCOVER;

    command.filler[0] = (true == forceUpdatingMode) ? (1) : (0);

    // step 1. we send command and process replies a first time

    sendCommand(ipv4, &command, sizeof(command));
    processDiscoveryReplies();


    // step 2. must verify that all boards to which we sent the command are in maintenance
    if(true == verify)
    {
        for(int iter=0; iter<retries; iter++)
        {
            if(isInMaintenance(ipv4))
            {
                return true;
            }

            yarp::os::Time::delay(timegap);

            // re-send the message and process replies
            sendCommand(ipv4, &command, sizeof(command));
            processDiscoveryReplies();
        }

    }

    return isInMaintenance(ipv4);
}




std::string EthMaintainer::commandGetMoreInfo(eOipv4addr_t ipv4)
{
    eOuprot_cmd_MOREINFO_t command;

    command.opc = uprot_OPC_LEGACY_PROCS;
    command.opc2 = uprot_OPC_MOREINFO;
    command.plusdescription = 1;
    command.filler[0] = EOUPROT_VALUE_OF_UNUSED_BYTE;

    sendCommand(ipv4, &command, sizeof(command));

    return processMoreInfoReplies();
}



std::string EthMaintainer::getTextualDescription(eOipv4addr_t ipv4)
{
    string info;

    vector<EthBoard *> selected = mEthBoards.get(ipv4);
    for(int i=0; i<selected.size(); i++)
    {
        info += selected[i]->getMoreInfo();
    }

    return info;
}




bool EthMaintainer::commandInfo32Clear(eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_PAGE_CLR_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_CLR;
    command.pagesize = 32;

    sendCommand(ipv4, &command, sizeof(command));

    return ret;
}


bool EthMaintainer::commandInfo32Set(const string &info32, eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_PAGE_SET_t *cmd = (eOuprot_cmd_PAGE_SET_t*) mTxBuffer;
    uint16_t sizeofcmd = sizeof(eOuprot_cmd_PAGE_SET_t) - uprot_pagemaxsize + 32;
    // think of the best way to specify the length of themessage in a proper way
    // we could decide to send always a length of sizeof(eOuprot_cmd_PAGE_SET_t) which is 132, or ...

    memset(cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeofcmd);

    cmd->opc = uprot_OPC_PAGE_SET;
    cmd->pagesize = 32;
    memset(cmd->page, 0, 32);


    const char * str32 = info32.c_str();

    int len = strlen(str32);
    if(len>30)
    {
        len = 30;
    }
    cmd->page[0] = len;
    memcpy(&cmd->page[1], str32, len);

    sendCommand(ipv4, cmd, sizeofcmd);

    return ret;
}


vector<string> EthMaintainer::commandInfo32Get(eOipv4addr_t ipv4)
{
    vector<string> thestrings(0);

    eOuprot_cmd_PAGE_GET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_GET;
    command.pagesize = 32;

    sendCommand(ipv4, &command, sizeof(command));

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 500) > 0)
    {
        eOuprot_cmd_PAGE_GET_REPLY_t * pageget = (eOuprot_cmd_PAGE_GET_REPLY_t*) mRxBuffer;

        char ipv4rxaddr_string[20];
        eo_common_ipv4addr_to_string(rxipv4addr, ipv4rxaddr_string, sizeof(ipv4rxaddr_string));

        if(uprot_OPC_PAGE_GET == pageget->reply.opc)
        {
            string readstring;

            if((rxipv4addr != myIPV4addr) && (uprot_RES_OK == pageget->reply.res) && (32 == pageget->pagesize) && (0xff != pageget->page[0]))
            {
                readstring = string((const char*)&pageget->page[1]);
                thestrings.push_back(readstring);
                // and update what in boardinfo32
                vector<EthBoard *> selected = mEthBoards.get(rxipv4addr);
                for(int i=0; i<selected.size(); i++)
                {
                    boardInfo2_t binfo = selected[i]->getInfo();
                    memcpy(binfo.boardinfo32, pageget->page, sizeof(binfo.boardinfo32));
                }
            }

            if(_debugprint)
            {
                if(uprot_RES_OK != pageget->reply.res)
                {
                    printf("\n received a eOuprot_cmd_PAGE_GET_REPLY_t from IP %s with a failure result %d for size %d", ipv4rxaddr_string, pageget->reply.res, pageget->pagesize);
                }
                else if(rxipv4addr != myIPV4addr)
                {
                    printf("\n received a eOuprot_cmd_PAGE_GET_REPLY_t from IP %s with size %d: ", ipv4rxaddr_string, pageget->pagesize);

                    if(32 == pageget->pagesize)
                    {   // the page is formatted so that in position 0 there is 0xff if erased and never programmed or strlen(&page[1])

                        uint8_t info32[32] = {0};
                        memcpy(info32, pageget->page, 32);

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

                    fflush(stdout);
                }
            }

        }
    }

    return thestrings;
}


bool EthMaintainer::commandRestart(eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_RESTART_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_RESTART;

    sendCommand(ipv4, &command, sizeof(command));

    return ret;
}


bool EthMaintainer::commandSetDEF2RUN(eOuprot_process_t process, eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_DEF2RUN_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_DEF2RUN;
    command.proc = process;

    sendCommand(ipv4, &command, sizeof(command));


    return ret;
}


bool EthMaintainer::commandJumpUpd(eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_JUMP2UPDATER_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_JUMP2UPDATER;

    sendCommand(ipv4, &command, sizeof(command));

    return ret;
}



bool EthMaintainer::commandJump2ROMaddress(uint32_t romaddress, eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_JUMP2ADDRESS_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_JUMP2ADDRESS;
    command.address = romaddress;

    sendCommand(ipv4, &command, sizeof(command));

    return ret;
}


bool EthMaintainer::commandBlink(eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_BLINK_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_BLINK;

    sendCommand(ipv4, &command, sizeof(command));

    return ret;
}


bool EthMaintainer::commandEraseEEPROM(eOipv4addr_t ipv4)
{
    bool ret = true;

    eOuprot_cmd_EEPROM_ERASE_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_EEPROM_ERASE;
    command.opc2 = uprot_OPC_EEPROM_ERASE;
    command.sysrestart = 0;
    command.address = 0;
    command.size = 0;

    sendCommand(ipv4, &command, sizeof(command));

    return ret;
}


bool EthMaintainer::commandReadEEPROM(uint16_t from, uint16_t size, eOipv4addr_t ipv4, uint8_t **value)
{
    bool ret = false;

    if(NULL == value)
    {
        return ret;
    }

    if(0 == ipv4)
    {
        return ret;
    }

    if((0 == size) || (size > uprot_EEPROMmaxsize))
    {
        return ret;
    }

    eOuprot_cmd_EEPROM_READ_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_EEPROM_READ;
    command.address = from;
    command.size = size;

    sendCommand(ipv4, &command, sizeof(command));

    // now we wait for the reply ...

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 1000) > 0)
    {
        eOuprot_cmd_EEPROM_READ_REPLY_t * eepromread = (eOuprot_cmd_EEPROM_READ_REPLY_t*) mRxBuffer;

//        char ipv4rxaddr_string[20];
//        eo_common_ipv4addr_to_string(rxipv4addr, ipv4rxaddr_string, sizeof(ipv4rxaddr_string));

        if(uprot_OPC_EEPROM_READ == eepromread->reply.opc)
        {
            // the board has replied.
            if(rxipv4addr == ipv4)
            {
                ret = (uprot_RES_OK == eepromread->reply.res) ? true : false;

                if(ret)
                {
                    *value = eepromread->eeprom;
                }
            }
        }
    }

    return ret;
}


bool EthMaintainer::commandChangeAddress(eOipv4addr_t ipv4, eOipv4addr_t ipv4new, bool restart, bool verify)
{
    bool ret = false;

    eOuprot_cmd_IP_ADDR_SET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_IP_ADDR_SET;
    command.opc2 = uprot_OPC_IP_ADDR_SET;
    command.sysrestart = 0; //(restart) ? (1) : (0);

    command.address[3] = (ipv4new>>24) & 0xFF;
    command.address[2] = (ipv4new>>16) & 0xFF;
    command.address[1] = (ipv4new>>8 ) & 0xFF;
    command.address[0] = (ipv4new    ) & 0xFF;


    char ipaddr[20];
    char newipaddr[20];

    if(_debugprint)
    {
        eo_common_ipv4addr_to_string(ipv4, ipaddr, sizeof(ipaddr));
        eo_common_ipv4addr_to_string(ipv4new, newipaddr, sizeof(newipaddr));
    }


    bool stopit = false;

    if(0 == ipv4)
    {
        stopit = true;
    }

    // we must have 10.0.1.x, where x is not 0 or 255
    if((10 != command.address[0]) || (0 != command.address[1]) || (1 != command.address[2]))
    {
        stopit =  true;
    }

    if((0 == command.address[3]) || (255 == command.address[3]))
    {
        stopit = true;
    }


    if(true == stopit)
    {
        if(_debugprint)
        {
            printf("cannot send command uprot_OPC_IP_ADDR_SET to %s with new address %s because either one or both are not valid\n", ipaddr, newipaddr);
        }

        return ret;
    }

    // search for ipv4new
    if(0 != mEthBoards.numberof(ipv4new))
    {
        if(_debugprint)
        {
            printf("cannot send command uprot_OPC_IP_ADDR_SET to %s with new address %s because the new address is already present\n", ipaddr, newipaddr);
        }

        return ret;
    }


    if(_debugprint)
    {
        printf("sent command uprot_OPC_IP_ADDR_SET to %s, new address is %s. w/%s sysrestart\n", ipaddr, newipaddr, (0 == command.sysrestart) ? "out" : "");
    }

    sendCommand(ipv4, &command, sizeof(command));

    if(true == restart)
    {
        commandRestart(ipv4);
        mEthBoards.rem(ipv4);
        if(_debugprint)
        {
            printf("there are now %d boards in list\n", mEthBoards.size());
        }
    }

    if(true == verify)
    {   // cannot verify if we dont restart.
        verify = restart;
    }


    if(false == verify)
    {
        return true;
    }

    // we verify ....

    // must wait a tick ... maybe 500 ms and then i run a discovery on the new address.
    yarp::os::Time::delay(1.000);


    ret = commandForceMaintenance(ipv4new, true, 5, 0.5);

    if(_debugprint)
    {
        printf("board w/ new address %s is in maintenance: %s\n", newipaddr, ret ? "YES" : "NO!");
    }

    if(false == ret)
    {             
        return false;
    }

    if(_debugprint)
    {
        printf("after commandForceMaintenance() there are now %d boards in list\n", mEthBoards.size());
    }

    if(0 == mEthBoards.numberof(ipv4new))
    {
        if(_debugprint)
        {
            printf("error: we dont have the new address %s ...\n", newipaddr);
        }
        return false;
    }

    if(_debugprint)
    {
        printf("commandChangeAddress(): OK\n");
    }

    return true;
}


bool EthMaintainer::commandProgram(FILE *programFile, int partition, void (*updateProgressBar)(float), string &stringresult, eOipv4addr_t ipv4)
{

    progData_t progdata;
    progdata.mN2Prog = 0;
    progdata.mNProgSteps = 0;
    progdata.mNChunks = 0;

    progdata.selected = mEthBoards.get(ipv4);
    progdata.steps.resize(progdata.selected.size(), 0);

    updateProgressBar(0.0f);

    eOuprot_cmd_PROG_START_t * cmdStart = (eOuprot_cmd_PROG_START_t*) mTxBuffer;
    eOuprot_cmd_PROG_DATA_t *  cmdData  = (eOuprot_cmd_PROG_DATA_t*)  mTxBuffer;
    eOuprot_cmd_PROG_END_t *   cmdEnd   = (eOuprot_cmd_PROG_END_t*)   mTxBuffer;

    const int sizeStart = sizeof(eOuprot_cmd_PROG_START_t);
    const int sizeEnd = sizeof(eOuprot_cmd_PROG_END_t);


    const int HEAD_SIZE = 7;

    fseek(programFile,0,SEEK_END);
    float fileSize=(float)(ftell(programFile)/3);
    fseek(programFile,0,SEEK_SET);

    memset(cmdStart, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_PROG_START_t));
    cmdStart->opc = uprot_OPC_PROG_START;
    cmdStart->partition = partition;

    string partname("UNK");
    eOuprot_proc_capabilities_t capability = uprot_canDO_nothing;
    if(uprot_partitionLOADER == partition)
    {
        partname = string("LDR");
        capability = uprot_canDO_PROG_loader;
    }
    else if(uprot_partitionUPDATER == partition)
    {
        partname = string("UPD");
        capability = uprot_canDO_PROG_updater;
    }
    else if(uprot_partitionAPPLICATION == partition)
    {
        partname = string("APP");
        capability = uprot_canDO_PROG_application;
    }


    // sending the start and preparing the list of boards to program

    // send the start command to all selectde
    sendCommand(ipv4, cmdStart, sizeStart);
    // wait a tick
    yarp::os::Time::delay(0.01);



    if(_debugprint)
    {

        printf("EthMaintainer::cmdProgram() is about to program the %s partition of %d boards:", partname.c_str(), (int)progdata.selected.size());
        for(int j=0; j< progdata.selected.size(); j++)
        {
            char ipv4str[20];
            eOipv4addr_t ipv4 = progdata.selected[j]->getIPV4();
            bool ok = isCommandSupported(capability, ipv4);

            printf(" %s (%s)", progdata.selected[j]->getIPV4string().c_str(), ok ? ("candoit") : ("CANTdoit"));
        }
        printf("\n");
        fflush(stdout);

    }


    // now we start

    progdata.mNProgSteps = 0;
    progdata.mNChunks = 1;

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    ++progdata.mNProgSteps;

    int success=0;

    int numberOfOKreplies = 0;

    // waiting for reply of start
    for(int n=0; n<1000; ++n)
    {
        while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 10) > 0)
        {
            eOuprot_cmdREPLY_t * reply = (eOuprot_cmdREPLY_t*) mRxBuffer;

            if (uprot_OPC_PROG_START == reply->opc)
            {
                if (rxipv4addr != myIPV4addr)
                {

                    // search for the ip inside boards2prog. if found then...
                    for(int i=0; i<progdata.selected.size(); i++)
                    {
                        if(rxipv4addr == (progdata.selected[i]->getIPV4()))
                        {   // found!
                            if(uprot_RES_OK == reply->res)
                            {
                                progdata.steps[i]++;
                                numberOfOKreplies++;
                            }
                            else
                            {
                                if(_debugprint)
                                {
                                    // print that the board tells that we cannot program that partition
                                    printf("board %s tells that we cannot program the %s partition\n", progdata.selected[i]->getIPV4string().c_str(), partname.c_str());
                                    fflush(stdout);
                                }
                            }

                            if (++success >= progdata.selected.size())
                            {
                                n = 1000;
                            }
                        }
                    }

                }
            }
        }
    }


    if(0 == numberOfOKreplies)
    {
        std::string earlyexit;

        for(int i=0; i<progdata.selected.size(); ++i)
        {
            eOipv4addr_t ipv4adr = progdata.selected[i]->getIPV4();
            char ipv4rxaddr_string[20];
            eo_common_ipv4addr_to_string(ipv4adr, ipv4rxaddr_string, sizeof(ipv4rxaddr_string));

            earlyexit += ipv4rxaddr_string;
            earlyexit += ": ";
            earlyexit += partname;
            earlyexit += (progdata.steps[i] == progdata.mNProgSteps)?" OK\r\n":" CANT\r\n";
        }

        stringresult = earlyexit;

        return false;
    }

    int addrH=0;
    int baseAddress=0;
    int bytesToWrite=0;
    int bytesWritten=0;

    char buffer[1024];

    bool beof=false;

    // sending data
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

                int addressHL=addrH<<16|addrL;

                if (!baseAddress) baseAddress=addressHL;

                if (bytesToWrite+size>uprot_PROGmaxsize || addressHL!=baseAddress+bytesToWrite)
                {
                    if (bytesToWrite)
                    {
                        cmdData->size[0]= bytesToWrite    &0xFF;
                        cmdData->size[1]=(bytesToWrite>>8)&0xFF;

                        progdata.data = cmdData;
                        progdata.size = HEAD_SIZE+bytesToWrite;
                        progdata.answers = progdata.selected.size();
                        progdata.retries = 1000;
                        sendPROG2(uprot_OPC_PROG_DATA, progdata);

                        updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                        bytesToWrite=0;
                    }
                }

                if (!bytesToWrite)
                {
                    baseAddress=addressHL;
                    cmdData->opc = uprot_OPC_PROG_DATA;
                    cmdData->address[0] = addrL&0xFF;
                    cmdData->address[1] = (addrL>>8)&0xFF;
                    cmdData->address[2] = addrH&0xFF;
                    cmdData->address[3] = (addrH>>8)&0xFF;
                }

                for (int i=0; i<size; ++i)
                {
                    cmdData->data[bytesToWrite+i]=(unsigned char)strtol(line.substr(i*2+9,2).c_str(),NULL,16);
                }

                bytesToWrite+=size;

                break;
            }
        case 1: //end of file
            if (bytesToWrite) // force write
            {
                beof=true;
                cmdData->size[0] =  bytesToWrite    &0xFF;
                cmdData->size[1] = (bytesToWrite>>8)&0xFF;

                progdata.data = cmdData;
                progdata.size = HEAD_SIZE+bytesToWrite;
                progdata.answers = progdata.selected.size(); // mN2Prog
                progdata.retries = 1000;
                sendPROG2(uprot_OPC_PROG_DATA, progdata);

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
                    cmdData->size[0] =  bytesToWrite    &0xFF;
                    cmdData->size[1] = (bytesToWrite>>8)&0xFF;

                    progdata.data = cmdData;
                    progdata.size = HEAD_SIZE+bytesToWrite;
                    progdata.answers = progdata.selected.size(); // mN2Prog
                    progdata.retries = 1000;
                    sendPROG2(uprot_OPC_PROG_DATA, progdata);

                    updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                    bytesToWrite=0;
                }

                addrH=strtol(line.substr(9,4).c_str(),NULL,16);

                break;
            }
        case 5: // jump
            if (bytesToWrite) // force write
            {
                cmdData->size[0]=  bytesToWrite    &0xFF;
                cmdData->size[1]= (bytesToWrite>>8)&0xFF;

                progdata.data = cmdData;
                progdata.size = HEAD_SIZE+bytesToWrite;
                progdata.answers = progdata.selected.size(); // mN2Prog
                progdata.retries = 1000;
                sendPROG2(uprot_OPC_PROG_DATA, progdata);

                updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                bytesToWrite=0;
            }

            break;
        }
    }


    // now we send the end
    memset(cmdEnd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_PROG_END_t));
    cmdEnd->opc = uprot_OPC_PROG_END;
    cmdEnd->numberofpkts[0] = progdata.mNChunks & 0xFF;
    cmdEnd->numberofpkts[1] = (progdata.mNChunks>>8) & 0xFF;

    progdata.data = cmdEnd;
    progdata.size = sizeEnd;
    progdata.answers = progdata.selected.size(); // mN2Prog
    progdata.retries = 1000;
    sendPROG2(uprot_OPC_PROG_END, progdata);

    updateProgressBar(1.0f);


    std::string sOutput;
    bool result = true;

    for(int i=0; i<progdata.selected.size(); ++i)
    {
        bool ok = (progdata.steps[i]==progdata.mNProgSteps) ? true : false;
        if(false == ok)
        {
            result = false;
        }
        eOipv4addr_t ipv4adr = progdata.selected[i]->getIPV4();
        char ipv4rxaddr_string[20];
        eo_common_ipv4addr_to_string(ipv4adr, ipv4rxaddr_string, sizeof(ipv4rxaddr_string));

        sOutput += ipv4rxaddr_string;
        sOutput += ": ";
        sOutput += partname;
        sOutput += (ok)?" OK\r\n":" NOK\r\n";
    }

    stringresult = sOutput;

    return result;
}


// helper functions


void EthMaintainer::sendCommand(eOipv4addr_t ipv4, void * cmd, uint16_t len)
{
    if(0 != ipv4)
    {
        mSocket.SendTo(ipv4, myIPV4port, cmd, len);
    }
    else
    {
        vector<EthBoard *> selected = mEthBoards.get(ipv4);
        for(int i=0; i<selected.size(); i++)
        {
            mSocket.SendTo(selected[i]->getIPV4(), myIPV4port, cmd, len);
        }
    }
}



bool EthMaintainer::isInMaintenance(eOipv4addr_t ipv4)
{
    vector<EthBoard *> selected = mEthBoards.get(ipv4);

    if(selected.empty())
    {
        return false;
    }

    for(int i=0; i<selected.size(); i++)
    {
       if(false == selected[i]->isInMaintenance())
       {
           return false;
       }
    }

    return true;
}


void EthMaintainer::processDiscoveryReplies(void)
{
    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 1000) > 0)
    {
        eOuprot_cmd_DISCOVER_REPLY_t * disc = (eOuprot_cmd_DISCOVER_REPLY_t*) mRxBuffer;
        eOuprot_cmd_LEGACY_SCAN_REPLY_t * scan = (eOuprot_cmd_LEGACY_SCAN_REPLY_t*) mRxBuffer;

        char ipv4rxaddr_string[20];
        eo_common_ipv4addr_to_string(rxipv4addr, ipv4rxaddr_string, sizeof(ipv4rxaddr_string));

        if(uprot_OPC_DISCOVER == disc->reply.opc)
        {
            // the board has replied with the new protocol.

            if(rxipv4addr != myIPV4addr)
            {
                boardInfo2_t binfo;

                // the protococol version and capabilities are properties of the running process.
                binfo.protversion = disc->reply.protversion;
                binfo.capabilities = disc->capabilities;

                memcpy(&binfo.macaddress, disc->mac48, 6);
                binfo.boardtype = (eObrd_ethtype_t)disc->boardtype;

                memcpy(&binfo.processes, &disc->processes, sizeof(eOuprot_proctable_t));
                memcpy(binfo.boardinfo32, disc->boardinfo32, sizeof(binfo.boardinfo32));

                binfo.maintenanceIsActive = false;
                if((eApplPROGupdater == binfo.processes.runningnow) || (eUpdater == binfo.processes.runningnow))
                {
                    binfo.maintenanceIsActive = true;
                }

                binfo.versionOfRunning.major = binfo.processes.info[binfo.processes.runningnow].version.major;
                binfo.versionOfRunning.minor = binfo.processes.info[binfo.processes.runningnow].version.minor;

                binfo.moreinfostring = prepareMoreInfoText(disc, ipv4rxaddr_string);

                mEthBoards.add(binfo, rxipv4addr);


                if(_debugprint)
                {
                    uint8_t index = eouprot_process2index((eOuprot_process_t)disc->processes.runningnow);
                    printf("Attempt to send board @ %s in maintainance: %s w/ %s v %d.%d running protocol v %d w/ capabilities = 0x%x. mainteinance = %s\n",
                            ipv4rxaddr_string,
                            eoboards_type2string((eObrd_type_t)disc->boardtype),
                            eouprot_process2string((eOuprot_process_t)disc->processes.runningnow),
                            disc->processes.info[index].version.major,
                            disc->processes.info[index].version.minor,
                            disc->reply.protversion,
                            disc->capabilities,
                            (binfo.maintenanceIsActive) ? ("ON") : ("OFF")
                           );
                    fflush(stdout);
                }
            }

        }
        else if (uprot_OPC_LEGACY_SCAN == scan->opc)
        {
            // we have an old board ... by definition it has protocol version 0.

            if(rxipv4addr != myIPV4addr)
            {

                // the protococol version and capabilities are properties of the running process.
                // from the legacy answer uprot_OPC_LEGACY_SCAN i know that we have protocol version 0.
                // however ... which process is it replying? the most probable is eUpdater, the second most probable is
                // eApplication. It is unlikely that it is a legacy eMaintainer because we should not use it.
                // i decide the following:
                // i give the permissions of legacy updater which contains those of legacy application.
                // because even if it was the application which replied then, surely now it is the updater running.

                // in this case we add the board ...


                uint8_t procmajor = scan->version.major;
                uint8_t procminor = scan->version.minor;


                //uint32_t mask=*(ACE_UINT32*)(scan->ipmask);
                uint64_t mac=0;

                for(int i=7; i>=0; --i)
                {
                    mac=(mac<<8)|scan->mac48[i];
                }



                boardInfo2_t binfo;

                binfo.reset();

                // the protococol version and capabilities are properties of the running process.
                binfo.protversion = 0;
                binfo.capabilities = eouprot_get_capabilities(eUpdater, binfo.protversion);

                memcpy(&binfo.macaddress, &mac, 6);
                binfo.boardtype = eobrd_ethtype_unknown;

                // we decide that we are in maintenance, even if we dont really know ...
                binfo.maintenanceIsActive = true;

                binfo.versionOfRunning.major = procmajor;
                binfo.versionOfRunning.minor = procminor;

                mEthBoards.add(binfo, rxipv4addr);


                if(_debugprint)
                {
                    printf("Attempt to send board @ %s in maintenance: the running process is v %d.%d, uses legacy protocol, i assume protocol capabilities = 0x%x.\n",
                                                    ipv4rxaddr_string, procmajor, procminor, binfo.capabilities);
                    fflush(stdout);
                }
            }
        }
    }

}


std::string EthMaintainer::processMoreInfoReplies(void)
{
    std::string info;

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 500) > 0)
    {
        eOuprot_cmd_MOREINFO_REPLY_t *moreinfo = (eOuprot_cmd_MOREINFO_REPLY_t*) mRxBuffer;
        eOuprot_cmd_LEGACY_PROCS_REPLY_t *procs = (eOuprot_cmd_LEGACY_PROCS_REPLY_t*) mRxBuffer;

        char ipv4rxaddr_string[20];
        eo_common_ipv4addr_to_string(rxipv4addr, ipv4rxaddr_string, sizeof(ipv4rxaddr_string));

        if(uprot_OPC_MOREINFO == moreinfo->discover.reply.opc)
        {
            // the board has replied with the new protocol.

            if(rxipv4addr != myIPV4addr)
            {
                eOuprot_cmd_DISCOVER_REPLY_t * disc = &moreinfo->discover;

                boardInfo2_t binfo;

                // the protococol version and capabilities are properties of the running process.
                binfo.protversion = disc->reply.protversion;
                binfo.capabilities = disc->capabilities;

                memcpy(&binfo.macaddress, disc->mac48, 6);
                binfo.boardtype = (eObrd_ethtype_t)disc->boardtype;

                memcpy(&binfo.processes, &disc->processes, sizeof(eOuprot_proctable_t));
                memcpy(binfo.boardinfo32, disc->boardinfo32, sizeof(binfo.boardinfo32));

                binfo.maintenanceIsActive = false;
                if((eApplPROGupdater == binfo.processes.runningnow) || (eUpdater == binfo.processes.runningnow))
                {
                    binfo.maintenanceIsActive = true;
                }

                binfo.versionOfRunning.major = binfo.processes.info[binfo.processes.runningnow].version.major;
                binfo.versionOfRunning.minor = binfo.processes.info[binfo.processes.runningnow].version.minor;

                binfo.moreinfostring = prepareMoreInfoText(disc, ipv4rxaddr_string);

                mEthBoards.add(binfo, rxipv4addr);

                // now we add into the string
                {
                    info += binfo.moreinfostring;
                }


                if(_debugprint)
                {
                    // print binfo.
                    // it would be much bettwer, however, store it somewhere and made it available
                    // through some method


                    printf("\nBOARD at address %s:", ipv4rxaddr_string);
                    printf("\n prot = %d, boardtype = %s, startup proc = %s, def2run proc = %s. it has %d processes:",
                                binfo.protversion,
                                eoboards_type2string((eObrd_type_t)binfo.boardtype),
                                eouprot_process2string((eOuprot_process_t)binfo.processes.startup),
                                eouprot_process2string((eOuprot_process_t)binfo.processes.def2run),
                                binfo.processes.numberofthem
                           );
                    for(int n=0; n<binfo.processes.numberofthem; n++)
                    {
                        char strdate[24] = {0};
                        char builton[24] = {0};
                        eo_common_date_to_string(binfo.processes.info[n].date, strdate, sizeof(strdate));
                        eo_common_date_to_string(binfo.processes.info[n].compilationdate, builton, sizeof(builton));
                        printf("\n proc-%d: type %s w/ appl version = (%d, %d), dated %s, built on %s, rom = [%d, %d) kb",
                               n,
                               eouprot_process2string((eOuprot_process_t)binfo.processes.info[n].type),
                               binfo.processes.info[n].version.major, binfo.processes.info[n].version.minor,
                               strdate,
                               builton,
                               binfo.processes.info[n].rom_addr_kb, binfo.processes.info[n].rom_addr_kb + binfo.processes.info[n].rom_size_kb
                               );

                    }

                    printf("\n now process %s is running", eouprot_process2string((eOuprot_process_t)binfo.processes.runningnow));

                    if(0xff == binfo.boardinfo32[0])
                    {
                        printf("\n stored info32 is .. ");
                        for(int m=0; m<32; m++)
                        {
                            printf("0x%x ", binfo.boardinfo32[m]);
                        }

                    }
                    else
                    {
                        printf("\n stored info32 is .. ");
                        printf("l = %d, string = %s \n", binfo.boardinfo32[0], &binfo.boardinfo32[1]);
                    }

                    printf("\n\n");

                }


            }
        }
        else if(uprot_OPC_LEGACY_PROCS == procs->opc)
        {
            // old boards reply with uprot_OPC_LEGACY_PROCS
            // unfortunately this field only has ... numofprocesses and textual description
            // I DONT add a  entry ....

            if (rxipv4addr != myIPV4addr)
            {

                // 1. i prepare the string

                string moreinfostring;

                moreinfostring += "------------------------------\r\n";
                moreinfostring += std::string("Board\t")+std::string(ipv4rxaddr_string);
                moreinfostring += "\r\n\r\n";
                moreinfostring += std::string((char*)procs->description);


                // 2. i retrieve the boards
                vector<EthBoard *> boards = mEthBoards.get(rxipv4addr);
                for(int i=0; i<boards.size(); i++)
                {
                    boards[i]->setMoreInfo(moreinfostring);
                }

                // finally we update return string

                info += moreinfostring;


                if(_debugprint)
                {
                    printf("\n received a uprot_OPC_LEGACY_PROCS from IP %s \n", ipv4rxaddr_string);
                    fflush(stdout);
                }

            }

        }
    }


    return info;

}


std::string EthMaintainer::prepareMoreInfoText(eOuprot_cmd_DISCOVER_REPLY_t * disc, char *ipv4rxaddr_string)
{
    std::string info;

    char tmp[512] = {0};


    boardInfo2_t binfo;

    binfo.reset();


    binfo.protversion = disc->reply.protversion;
    binfo.capabilities = disc->capabilities;
    memcpy(&binfo.macaddress, disc->mac48, 6);
    binfo.boardtype = (eObrd_ethtype_t)disc->boardtype;
    memcpy(&binfo.processes, &disc->processes, sizeof(eOuprot_proctable_t));
    memcpy(binfo.boardinfo32, disc->boardinfo32, sizeof(binfo.boardinfo32));

    binfo.maintenanceIsActive = false;
    if((eApplPROGupdater == binfo.processes.runningnow) || (eUpdater == binfo.processes.runningnow))
    {
        binfo.maintenanceIsActive = true;
    }

    binfo.versionOfRunning.major = binfo.processes.info[binfo.processes.runningnow].version.major;
    binfo.versionOfRunning.minor = binfo.processes.info[binfo.processes.runningnow].version.minor;



    char status[64] = "normal";
    if(true == binfo.maintenanceIsActive)
    {
        snprintf(status, sizeof(status), "maintenance");
    }
    else
    {
        snprintf(status, sizeof(status), "normal");
    }

    char mac_string[64] = "00-00-00-00-00-00";
    snprintf(mac_string, sizeof(mac_string), "%02X-%02X-%02X-%02X-%02X-%02X",
                        (uint8_t)(binfo.macaddress >> 40) & 0xff,
                        (uint8_t)(binfo.macaddress >> 32) & 0xff,
                        (uint8_t)(binfo.macaddress >> 24) & 0xff,
                        (uint8_t)(binfo.macaddress >> 16) & 0xff,
                        (uint8_t)(binfo.macaddress >> 8 ) & 0xff,
                        (uint8_t)(binfo.macaddress      ) & 0xff
            );


    snprintf(tmp, sizeof(tmp), "BOARD: \n- type: %s \n- mac: %s \n- ip: %s \n- status: %s",
                eoboards_type2string2((eObrd_type_t)binfo.boardtype, eobool_true),
                mac_string,
                ipv4rxaddr_string,
                status);
    info += tmp;

    snprintf(tmp, sizeof(tmp), "\n\nBOOTSTRAP PROCESSES:"
           );
    info += tmp;

    snprintf(tmp, sizeof(tmp), "\n- startup: %s, \n- default: %s, \n- running: %s.",
                eouprot_process2string((eOuprot_process_t)binfo.processes.startup),
                eouprot_process2string((eOuprot_process_t)binfo.processes.def2run),
                eouprot_process2string((eOuprot_process_t)binfo.processes.runningnow)
           );
    info += tmp;

    snprintf(tmp, sizeof(tmp), "\n\nPROPS OF THE %d PROCESSES:",
                binfo.processes.numberofthem
           );
    info += tmp;

    for(int n=0; n<binfo.processes.numberofthem; n++)
    {
        char strdate[24] = {0};
        char builton[24] = {0};
        eo_common_date_to_string(binfo.processes.info[n].date, strdate, sizeof(strdate));
        eo_common_date_to_string(binfo.processes.info[n].compilationdate, builton, sizeof(builton));
        snprintf(tmp, sizeof(tmp), "\n- proc-%d: \n  type: %s \n  version: %d.%d, \n  dated: %s, \n  built on: %s, \n  rom: [%d, %d) kb",
               n,
               eouprot_process2string((eOuprot_process_t)binfo.processes.info[n].type),
               binfo.processes.info[n].version.major, binfo.processes.info[n].version.minor,
               strdate,
               builton,
               binfo.processes.info[n].rom_addr_kb, binfo.processes.info[n].rom_addr_kb + binfo.processes.info[n].rom_size_kb
               );
        info += tmp;

    }



    info += "\n\n";

    return info;
}



std::string EthMaintainer::getMoreInfoText(eOuprot_cmd_MOREINFO_REPLY_t *moreinfo, char *ipaddr)
{
    std::string info;

    info += "------------------------------\r\n";
    info += std::string("Board\t")+std::string(ipaddr);
    info += "\r\n\r\n";

    if(1 == moreinfo->hasdescription)
    {
        info += std::string((char*)moreinfo->description);
    }
    else
    {
        info += std::string("the message does not have a textual description");
    }

    return info;
}


int EthMaintainer::sendPROG2(const uint8_t opc, progData_t &progdata)
{
    // data can be either a eOuprot_cmd_PROG_DATA_t* or a eOuprot_cmd_PROG_END_t*
    // both have the same layout of the opc in first position


    // use unicast to all selected boards
    for(int k=0;k<progdata.selected.size(); k++)
    {
        mSocket.SendTo(progdata.selected[k]->getIPV4(), myIPV4port, progdata.data, progdata.size);
    }


    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    ++progdata.mNChunks;

    if(progdata.answers)
    {
        ++progdata.mNProgSteps;

        for (int r=0; r<progdata.retries; ++r)
        {
            for (int a=0; a<progdata.answers; ++a)
            {
                if(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 10) > 0)
                {
                    eOuprot_cmdREPLY_t * reply = (eOuprot_cmdREPLY_t*) mRxBuffer;

                    if (opc == reply->opc)
                    {
                        if (rxipv4addr != myIPV4addr)
                        {

                            // search for the ip inside boards2prog. if found then...
                            for(int i=0; i<progdata.selected.size(); i++)
                            {
                                if(rxipv4addr == (progdata.selected[i]->getIPV4()))
                                {
                                    if (uprot_RES_OK == reply->res)
                                    {
                                        ++(progdata.steps[i]);
                                    }
                                    break;

                                }
                            }

                            if (!--progdata.answers) return 0;
                        }
                    }
                }
            }
        }
    }

    return progdata.answers;
}


// eof


