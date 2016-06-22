/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it, Marco Accame marco.accame@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "EthUpdater.h"

#include <ace/ACE.h>
#include <yarp/os/Time.h>

#include <yarp/os/Log.h>

using namespace yarp::os;

const int EthUpdater::partition_APPLICATION = uprot_partitionAPPLICATION;
const int EthUpdater::partition_LOADER = uprot_partitionLOADER;
const int EthUpdater::partition_UPDATER = uprot_partitionUPDATER;


#define PRINT_DEBUG_INFO_ON_TERMINAL


void EthUpdater::cmdDiscover()
{
    mBoardList.empty();

    eOuprot_cmd_DISCOVER_t * cmd = (eOuprot_cmd_DISCOVER_t*) mTxBuffer;

    memset(cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_DISCOVER_t));

    cmd->opc = uprot_OPC_LEGACY_SCAN;
    cmd->opc2 = uprot_OPC_DISCOVER;


    mSocket.SendBroad(cmd, sizeof(eOuprot_cmd_DISCOVER_t), mPort);

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress = mMyAddress;

    while (mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 1000)>0)
    {
        eOuprot_cmd_DISCOVER_REPLY_t * disc = (eOuprot_cmd_DISCOVER_REPLY_t*) mRxBuffer;
        eOuprot_cmd_LEGACY_SCAN_REPLY_t * scan = (eOuprot_cmd_LEGACY_SCAN_REPLY_t*) mRxBuffer;
        char ipaddr[20];
        sprintf(ipaddr,"%d.%d.%d.%d",(rxAddress>>24)&0xFF, (rxAddress>>16)&0xFF, (rxAddress>>8)&0xFF, rxAddress&0xFF);

        if(uprot_OPC_DISCOVER == disc->reply.opc)
        {
            // the board has replied with the new protocol.

            if (rxAddress != mMyAddress)
            {
                boardInfo_t binfo = {0};

                // the protococol version and capabilities are properties of the running process.
                binfo.protversion = disc->reply.protversion;
                binfo.capabilities = disc->capabilities;

                memcpy(&binfo.macaddress, disc->mac48, 6);
                binfo.boardtype = disc->boardtype;

                memcpy(&binfo.processes, &disc->processes, sizeof(eOuprot_proctable_t));
                memcpy(binfo.boardinfo32, disc->boardinfo32, sizeof(binfo.boardinfo32));

                BoardInfo *pBoard = new BoardInfo(rxAddress, binfo);

                mBoardList.addBoard(pBoard);

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
                uint8_t index = eouprot_process2index((eOuprot_process_t)disc->processes.runningnow);
                printf("Discovered a board @ %s: %s w/ %s v %d.%d running protocol v %d w/ capabilities = 0x%x\n",
                        ipaddr,
                        eoboards_type2string((eObrd_type_t)disc->boardtype),
                        eouprot_process2string((eOuprot_process_t)disc->processes.runningnow),
                        disc->processes.info[index].version.major,
                        disc->processes.info[index].version.minor,
                        disc->reply.protversion,
                        disc->capabilities);
                fflush(stdout);
#endif
            }

        }
        else if (uprot_OPC_LEGACY_SCAN == scan->opc)
        {
            // we have an old board ... by definition it has protocol version 0.

            if (rxAddress!=mMyAddress)
            {

                // the protococol version and capabilities are properties of the running process.
                // from the legacy answer uprot_OPC_LEGACY_SCAN i know that we have protocol version 0.
                // however ... which process is it replying? the most probable is eUpdater, the second most probable is
                // eApplication. It is unlikely that it is a legacy eMaintainer because we should not use it.
                // i decide the following:
                // i give the permissions of legacy updater which contains those of legacy application.
                // because even if it was the application which replied then, surely now it is the updater running.
                ACE_UINT8 protocol_version = 0;
                // i dont have such a mas, hence i use the default for protocol 0, thinking that ....
                ACE_UINT32 protocol_capabilities = eouprot_get_capabilities(eUpdater, protocol_version);

                ACE_UINT8 procmajor = scan->version.major;
                ACE_UINT8 procminor = scan->version.minor;


                ACE_UINT32 mask=*(ACE_UINT32*)(scan->ipmask);
                ACE_UINT64 mac=0;

                for(int i=7; i>=0; --i)
                {
                    mac=(mac<<8)|scan->mac48[i];
                }

                BoardInfo *pBoard = new BoardInfo(rxAddress, mask, mac, procmajor, procminor, protocol_version, protocol_capabilities);

                mBoardList.addBoard(pBoard);

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
                printf("Discovered a board @ %s: the running process is v %d.%d, uses legacy protocol, i assume protocol capabilities = 0x%x.\n",
                                                ipaddr, procmajor, procminor, protocol_capabilities);
                fflush(stdout);
#endif
            }
        }
    }
}


bool EthUpdater::isCmdSupported(eOuprot_proc_capabilities_t capability, ACE_UINT32 address)
{
    // search for the BoardInfo with a given address ... if we have a double address then we return a vector
    // we also may return a vector if address is 0 because we search for the selected.
    vector<BoardInfo *> boards = mBoardList.getBoards(address);

    if(0 == boards.size())
    {
        return false;
    }

    // i assume it is true. i return true only if all the boards support it.
    bool ret = true;
    for(int i=0; i<boards.size(); i++)
    {
        uint32_t mask = boards[i]->mProtocolCapabilities;
        bool r = ((mask & capability) == capability) ? true : false;
        ret = ret && r;
    }

    return ret;
}


std::string EthUpdater::cmdGetMoreInfo(bool refreshInfo, ACE_UINT32 address)
{
    eOuprot_cmd_MOREINFO_t command;

    command.opc = uprot_OPC_LEGACY_PROCS;
    command.opc2 = uprot_OPC_MOREINFO;
    command.plusdescription = 1;
    command.filler[0] = EOUPROT_VALUE_OF_UNUSED_BYTE;

    if(0 == address)
    {
        sendCommandSelected(&command, sizeof(command));
    }
    else
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    std::string info;

    while (mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 500)>0)
    {
        eOuprot_cmd_MOREINFO_REPLY_t *moreinfo = (eOuprot_cmd_MOREINFO_REPLY_t*) mRxBuffer;
        eOuprot_cmd_LEGACY_PROCS_REPLY_t *procs = (eOuprot_cmd_LEGACY_PROCS_REPLY_t*) mRxBuffer;

        char ipaddr[20];
        sprintf(ipaddr,"%d.%d.%d.%d",(rxAddress>>24)&0xFF, (rxAddress>>16)&0xFF, (rxAddress>>8)&0xFF, rxAddress&0xFF);

        if(uprot_OPC_LEGACY_PROCS == procs->opc)
        {
            // old boards reply with uprot_OPC_LEGACY_PROCS

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
            printf("\n received a uprot_OPC_LEGACY_PROCS from IP %s \n", ipaddr);
            fflush(stdout);
#endif
            if (rxAddress != mMyAddress)
            {
                info+="------------------------------\r\n";
                info+=std::string("Board\t")+std::string(ipaddr);
                info+="\r\n\r\n";
                info+=std::string((char*)procs->description);
            }
        }
        else if(uprot_OPC_MOREINFO == moreinfo->discover.reply.opc)
        {
            // a new board has replied

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
            printf("\n received a uprot_OPC_MOREINFO with prot version %d from IP %s\n", moreinfo->discover.reply.protversion, ipaddr);
            fflush(stdout);
#endif

            if (rxAddress != mMyAddress)
            {
                eOuprot_cmd_DISCOVER_REPLY_t * disc = &moreinfo->discover;

                boardInfo_t binfo = {0};

                binfo.protversion = disc->reply.protversion;
                binfo.capabilities = disc->capabilities;
                memcpy(&binfo.macaddress, disc->mac48, 6);
                binfo.boardtype = disc->boardtype;
                memcpy(&binfo.processes, &disc->processes, sizeof(eOuprot_proctable_t));
                memcpy(binfo.boardinfo32, disc->boardinfo32, sizeof(binfo.boardinfo32));

                if(refreshInfo)
                {
                    BoardInfo *pBoard = new BoardInfo(rxAddress, binfo);
                    mBoardList.replaceBoard(pBoard);
                }

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)

                // print binfo.
                // it would be much bettwer, however, store it somewhere and made it available
                // through some method


                printf("\nBOARD at address %s:", ipaddr);
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
                    printf("\n proc-%d: type %s w/ appl version = (%d, %d), dated %s, built on %s",
                           n,
                           eouprot_process2string((eOuprot_process_t)binfo.processes.info[n].type),
                           binfo.processes.info[n].version.major, binfo.processes.info[n].version.minor,
                           strdate,
                           builton
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

#endif
                // now we put into info all what is needed
                {
                    info+="------------------------------\r\n";
                    info+=std::string("Board\t")+std::string(ipaddr);
                    info+="\r\n\r\n";
                    if(1 == moreinfo->hasdescription)
                    {
                        info+=std::string((char*)moreinfo->description);
                    }
                    else
                    {
                        info+=std::string("the message does not have a textual description");
                    }
                }
            }
        }
    }

    return info;
}


void EthUpdater::cmdInfo32Clear(ACE_UINT32 address)
{
    eOuprot_cmd_PAGE_CLR_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_CLR;
    command.pagesize = 32;

    if(0 == address)
    {
        sendCommandSelected(&command, sizeof(command));
    }
    else
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }
}

void EthUpdater::cmdInfo32Set(const string &info32, ACE_UINT32 address)
{

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

    if(0 == address)
    {
        sendCommandSelected(cmd, sizeofcmd);
    }
    else
    {
        mSocket.SendTo(cmd, sizeofcmd, mPort, address);
    }
}


vector<string> EthUpdater::cmdInfo32Get(ACE_UINT32 address)
{
    vector<string> thestrings(0);

    eOuprot_cmd_PAGE_GET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_GET;
    command.pagesize = 32;

    if(0 == address)
    {
        sendCommandSelected(&command, sizeof(command));
    }
    else
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;


    while(mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 500)>0)
    {
        eOuprot_cmd_PAGE_GET_REPLY_t * pageget = (eOuprot_cmd_PAGE_GET_REPLY_t*) mRxBuffer;

        if(uprot_OPC_PAGE_GET == pageget->reply.opc)
        {
            string readstring;

            if((rxAddress != mMyAddress) && (uprot_RES_OK == pageget->reply.res) && (32 == pageget->pagesize) && (0xff != pageget->page[0]))
            {
                readstring = string((const char*)&pageget->page[1]);
                thestrings.push_back(readstring);
            }

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)

            char ip32[20];
            snprintf(ip32, sizeof(ip32), "%d.%d.%d.%d",(rxAddress>>24)&0xFF, (rxAddress>>16)&0xFF, (rxAddress>>8)&0xFF, rxAddress&0xFF);

            if(uprot_RES_OK != pageget->reply.res)
            {
                printf("\n received a eOuprot_cmd_PAGE_GET_REPLY_t from IP %s with a failure result %d for size %d", ip32, pageget->reply.res, pageget->pagesize);
            }
            else if(rxAddress != mMyAddress)
            {
                printf("\n received a eOuprot_cmd_PAGE_GET_REPLY_t from IP %s with size %d: ", ip32, pageget->pagesize);

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
#endif

        }
    }

    return thestrings;
}


void EthUpdater::cmdRestart(ACE_UINT32 address)
{
    eOuprot_cmd_RESTART_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_RESTART;

    if(0 != address)
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }
    else
    {
        sendCommandSelected(&command, sizeof(command));
    }
}


void EthUpdater::cmdSetDEF2RUN(eOuprot_process_t process, ACE_UINT32 address)
{
    eOuprot_cmd_DEF2RUN_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_DEF2RUN;
    command.proc = process;

    if(0 != address)
    {

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
        char ipaddr[20];
        sprintf(ipaddr,"%d.%d.%d.%d",(address>>24)&0xFF, (address>>16)&0xFF, (address>>8)&0xFF, address&0xFF);
        printf("send command uprot_OPC_DEF2RUN w/ process = %s to %s\n", eouprot_process2string(process), ipaddr);
#endif

        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }
    else
    {
        for(int i=0; i<mBoardList.size(); ++i)
        {
            if(mBoardList[i].mSelected)
            {
                address = mBoardList[i].mAddress;

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
                char ipaddr[20];
                sprintf(ipaddr,"%d.%d.%d.%d",(address>>24)&0xFF, (address>>16)&0xFF, (address>>8)&0xFF, address&0xFF);
                printf("send command uprot_OPC_DEF2RUN w/ process = %s to %s\n", eouprot_process2string(process), ipaddr);
#endif

                mSocket.SendTo(&command, sizeof(command), mPort, address);
            }
        }
    }
}


void EthUpdater::cmdJumpUpd(ACE_UINT32 address)
{
    eOuprot_cmd_JUMP2UPDATER_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_JUMP2UPDATER;

    if(0 != address)
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }
    else
    {
        sendCommandSelected(&command, sizeof(command));
    }
}


void EthUpdater::cmdBlink(ACE_UINT32 address)
{
    eOuprot_cmd_BLINK_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_BLINK;

    if(0 != address)
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }
    else
    {
        sendCommandSelected(&command, sizeof(command));
    }
}


void EthUpdater::cmdEraseEEPROM(ACE_UINT32 address)
{
    eOuprot_cmd_EEPROM_ERASE_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_EEPROM_ERASE;
    command.opc2 = uprot_OPC_EEPROM_ERASE;
    command.sysrestart = 0;
    command.address = 0;
    command.size = 0;

    if(0 != address)
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }
    else
    {
        sendCommandSelected(&command, sizeof(command));
    }
}

// TODO: at date 22 jun 16: yet to be tested
bool EthUpdater::cmdReadEEPROM(uint16_t from, uint16_t size, ACE_UINT32 address, uint8_t **value)
{
    bool ret = false;

    if(NULL == value)
    {
        return ret;
    }

    if(0 == address)
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

    mSocket.SendTo(&command, sizeof(command), mPort, address);

    // now we wait for teh reply ...

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress = mMyAddress;

    while (mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 1000)>0)
    {
        eOuprot_cmd_EEPROM_READ_REPLY_t * eepromread = (eOuprot_cmd_EEPROM_READ_REPLY_t*) mRxBuffer;

        char ipaddr[20];
        sprintf(ipaddr,"%d.%d.%d.%d",(rxAddress>>24)&0xFF, (rxAddress>>16)&0xFF, (rxAddress>>8)&0xFF, rxAddress&0xFF);

        if(uprot_OPC_EEPROM_READ == eepromread->reply.opc)
        {
            // the board has replied.
            if (rxAddress == address)
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

void EthUpdater::cmdChangeAddress(ACE_UINT32 newaddress, ACE_UINT32 address)
{
    eOuprot_cmd_IP_ADDR_SET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_IP_ADDR_SET;
    command.opc2 = uprot_OPC_IP_ADDR_SET;
    command.sysrestart = 0;

    command.address[0] = (newaddress>>24) & 0xFF;
    command.address[1] = (newaddress>>16) & 0xFF;
    command.address[2] = (newaddress>>8 ) & 0xFF;
    command.address[3] = (newaddress    ) & 0xFF;

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
    char ipaddr[20];
    sprintf(ipaddr,"%d.%d.%d.%d",(address>>24)&0xFF, (address>>16)&0xFF, (address>>8)&0xFF, address&0xFF);
    char newipaddr[20];
    sprintf(newipaddr,"%d.%d.%d.%d",(newaddress>>24)&0xFF, (newaddress>>16)&0xFF, (newaddress>>8)&0xFF, newaddress&0xFF);
#endif

    bool stopit = false;

    if(0 == address)
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
#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
        printf("cannot send command uprot_OPC_IP_ADDR_SET to %s with new address %s because either one or both are not valid\n", ipaddr, newipaddr);
#endif
        return;
    }


#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
    printf("sent command uprot_OPC_IP_ADDR_SET to %s, new address is %s. w/%s sysrestart\n", ipaddr, newipaddr, (0 == command.sysrestart) ? "out" : "");
#endif

    mSocket.SendTo(&command, sizeof(command), mPort, address);
}




std::string EthUpdater::cmdProgram(FILE *programFile, int partition, void (*updateProgressBar)(float), ACE_UINT32 address)
{
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

    mN2Prog = 0;

    if(0 == address)
    {
        // we use the selected

        for (int i=0; i<mBoardList.size(); ++i)
        {
            if (mBoardList[i].mSelected)
            {
                mBoard2Prog[mN2Prog++]=&mBoardList[i];
                mBoardList[i].mSuccess=0;
                mSocket.SendTo(cmdStart, sizeStart, mPort, mBoardList[i].mAddress);
                yarp::os::Time::delay(0.01);
            }
        }

    }
    else
    {
        // we use address

        vector<BoardInfo *> boards = mBoardList.getBoards(address);

        std::string errorstring;
        char addr[20];
        snprintf(addr, sizeof(addr), "%d.%d.%d.%d: ",(address>>24)&0xFF,(address>>16)&0xFF,(address>>8)&0xFF,address&0xFF);

        errorstring += addr;

        if(0 == boards.size())
        {
            errorstring += "CANT find it\r\n";
            return errorstring;
        }
        else if(boards.size() > 1)
        {
//            printf("error: size = %d\n", boards.size());
//            fflush(stdout);
            errorstring += "CANT prog more than one";
            return errorstring;
        }

        boards[0]->mSuccess = 0;
        mBoard2Prog[mN2Prog++] = boards[0];
        mSocket.SendTo(cmdStart, sizeStart, mPort, address);
        yarp::os::Time::delay(0.01);
    }

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)

    printf("EthUpdater::cmdProgram() is about to program the %s partition of %d boards:", partname.c_str(), mN2Prog);
    for(int j=0; j< mN2Prog; j++)
    {
        char ipv4str[20];
        ACE_UINT32 ipv4 = mBoard2Prog[j]->mAddress;
        bool ok = isCmdSupported(capability, ipv4);
        snprintf(ipv4str, sizeof(ipv4str), "%d.%d.%d.%d",(ipv4>>24)&0xFF,(ipv4>>16)&0xFF,(ipv4>>8)&0xFF,ipv4&0xFF);

        printf(" %s (%s)", ipv4str, ok ? ("candoit") : ("CANTdoit"));
    }
    printf("\n");
    fflush(stdout);

#endif


    // now we start

    mNProgSteps = 0;
    mNChunks = 1;

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    ++mNProgSteps;

    int success=0;

    int numberOfOKreplies = 0;

    // waiting for reply of start
    for (int n=0; n<1000; ++n)
    {
        while (mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 10)>0)
        {
            eOuprot_cmdREPLY_t * reply = (eOuprot_cmdREPLY_t*) mRxBuffer;

            if (uprot_OPC_PROG_START == reply->opc)
            {
                if (rxAddress!=mMyAddress)
                {
                    for (int i=0; i<mN2Prog; ++i)
                    {
                        if (rxAddress == mBoard2Prog[i]->mAddress)
                        {
                            if(uprot_RES_OK == reply->res)
                            {
                                ++(mBoard2Prog[i]->mSuccess);
                                numberOfOKreplies++;
                            }
                            else
                            {
#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
            // print that we the board tells that we cannot program that partition
                                char ipv4str[20];
                                ACE_UINT32 ipv4 = mBoard2Prog[i]->mAddress;
                                //bool ok = isCmdSupported(capability, ipv4);
                                snprintf(ipv4str, sizeof(ipv4str), "%d.%d.%d.%d",(ipv4>>24)&0xFF,(ipv4>>16)&0xFF,(ipv4>>8)&0xFF,ipv4&0xFF);

                                printf("board %s tells that we cannot program the %s partition\n", ipv4str, partname.c_str());
                                fflush(stdout);
#endif

                            }

                            if (++success>=mN2Prog) n=1000;
                        }
                    }
                }
            }
        }
    }


    if(0 == numberOfOKreplies)
    {
        std::string earlyexit;
        char addr[16];

        for (int i=0; i<mN2Prog; ++i)
        {
            ACE_UINT32 ip=mBoard2Prog[i]->mAddress;
            snprintf(addr, sizeof(addr), "%d.%d.%d.%d: ",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
            earlyexit+=addr;
            earlyexit+=partname;
            earlyexit+=(mBoard2Prog[i]->mSuccess==mNProgSteps)?" OK\r\n":" CANT\r\n";
        }

        return earlyexit;
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

                int address=addrH<<16|addrL;

                if (!baseAddress) baseAddress=address;

                if (bytesToWrite+size>uprot_PROGmaxsize || address!=baseAddress+bytesToWrite)
                {
                    if (bytesToWrite)
                    {
                        cmdData->size[0]= bytesToWrite    &0xFF;
                        cmdData->size[1]=(bytesToWrite>>8)&0xFF;
                        sendPROG(uprot_OPC_PROG_DATA, cmdData, HEAD_SIZE+bytesToWrite, mN2Prog,1000);
                        updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                        bytesToWrite=0;
                    }
                }

                if (!bytesToWrite)
                {
                    baseAddress=address;
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
                sendPROG(uprot_OPC_PROG_DATA, cmdData, HEAD_SIZE+bytesToWrite, mN2Prog, 1000);
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
                    sendPROG(uprot_OPC_PROG_DATA, cmdData, HEAD_SIZE+bytesToWrite, mN2Prog, 1000);
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
                sendPROG(uprot_OPC_PROG_DATA, cmdData, HEAD_SIZE+bytesToWrite, mN2Prog,1000);
                updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                bytesToWrite=0;
            }

            break;
        }
    }


    // now we send the end
    memset(cmdEnd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_PROG_END_t));
    cmdEnd->opc = uprot_OPC_PROG_END;
    cmdEnd->numberofpkts[0] = mNChunks & 0xFF;
    cmdEnd->numberofpkts[1] = (mNChunks>>8) & 0xFF;


    sendPROG(uprot_OPC_PROG_END, cmdEnd, sizeEnd, mN2Prog, 1000);

    updateProgressBar(1.0f);

    std::string sOutput;
    char addr[16];

    for (int i=0; i<mN2Prog; ++i)
    {
        ACE_UINT32 ip=mBoard2Prog[i]->mAddress;
        sprintf(addr,"%d.%d.%d.%d: ",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
        sOutput+=addr;
        sOutput+=partname;
        sOutput+=(mBoard2Prog[i]->mSuccess==mNProgSteps)?" OK\r\n":" NOK\r\n";
    }

    return sOutput;
}



void EthUpdater::sendCommandSelected(void * cmd, uint16_t len)
{
    for (int i=0; i<mBoardList.size(); ++i)
    {
        if (mBoardList[i].mSelected)
        {
            mSocket.SendTo(cmd, len, mPort, mBoardList[i].mAddress);
        }
    }
}



int EthUpdater::sendPROG(const uint8_t opc, void * data, int size, int answers, int retry)
{
    // data can be either a eOuprot_cmd_PROG_DATA_t* or a eOuprot_cmd_PROG_END_t*
    // both have the same layout of the opc in first position

    // use unicast to all selected boards
    for(int k=0;k<mN2Prog; k++)
    {
        mSocket.SendTo(data, size, mPort, mBoard2Prog[k]->mAddress);
    }

    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    ++mNChunks;

    if (answers)
    {
        ++mNProgSteps;

        for (int r=0; r<retry; ++r)
        {
            for (int a=0; a<answers; ++a)
            {
                if (mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 10)>0)
                {
                    eOuprot_cmdREPLY_t * reply = (eOuprot_cmdREPLY_t*) mRxBuffer;

                    if (opc == reply->opc)
                    {
                        if (rxAddress!=mMyAddress)
                        {
                            for (int i=0; i<mN2Prog; ++i)
                            {
                                if (rxAddress==mBoard2Prog[i]->mAddress)
                                {
                                    if (uprot_RES_OK == reply->res)
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


void EthUpdater::cmdChangeMask(ACE_UINT32 newMask, ACE_UINT32 address)
{

    eOuprot_cmd_LEGACY_IP_MASK_SET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_IP_MASK_SET;

    command.mask[0] = (newMask>>24)&0xFF;
    command.mask[1] = (newMask>>16)&0xFF;
    command.mask[2] = (newMask>>8) &0xFF;
    command.mask[3] =  newMask     &0xFF;

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
    char ipaddr[20];
    sprintf(ipaddr,"%d.%d.%d.%d",(address>>24)&0xFF, (address>>16)&0xFF, (address>>8)&0xFF, address&0xFF);
    char newm[20];
    sprintf(newm,"%d.%d.%d.%d",(newMask>>24)&0xFF, (newMask>>16)&0xFF, (newMask>>8)&0xFF, newMask&0xFF);
#endif

    bool stopit = false;

    if(0 == address)
    {
        stopit = true;
    }

    if(true == stopit)
    {
#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
        printf("cannot send command uprot_OPC_LEGACY_IP_MASK_SET to %s with new mask %s because either one or both are not valid\n", ipaddr, newm);
#endif
        return;
    }


#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
    printf("send command eOuprot_cmd_LEGACY_IP_MASK_SET_t to %s, new mask is %s\n", ipaddr, newm);
#endif

    mSocket.SendTo(&command, sizeof(command), mPort, address);
}

// TODO: at date 22 jun 16: yet to be tested
void EthUpdater::cmdChangeMAC(uint64_t newMAC48, ACE_UINT32 address)
{

    eOuprot_cmd_LEGACY_MAC_SET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_MAC_SET;

    command.mac48[0] = (newMAC48>>40) & 0xFF;
    command.mac48[1] = (newMAC48>>32) & 0xFF;
    command.mac48[2] = (newMAC48>>24) & 0xFF;
    command.mac48[3] = (newMAC48>>16) & 0xFF;
    command.mac48[4] = (newMAC48>>8 ) & 0xFF;
    command.mac48[5] = (newMAC48    ) & 0xFF;

#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
    char ipaddr[20];
    sprintf(ipaddr,"%d.%d.%d.%d",(address>>24)&0xFF, (address>>16)&0xFF, (address>>8)&0xFF, address&0xFF);
    char newmac[32];
    sprintf(newmac,"%x:%x:%x:%x:%x:%x", command.mac48[0], command.mac48[1], command.mac48[2], command.mac48[3], command.mac48[4], command.mac48[5]);
#endif

    bool stopit = false;

    if(0 == address)
    {
        stopit = true;
    }

    if(0 == newMAC48)
    {
        stopit = true;
    }

    if(true == stopit)
    {
#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
        printf("cannot send command uprot_OPC_LEGACY_MAC_SET to %s with new mac %s because either one or both are not valid\n", ipaddr, newmac);
#endif
        return;
    }


#if defined(PRINT_DEBUG_INFO_ON_TERMINAL)
    printf("send command uprot_OPC_LEGACY_MAC_SET to %s, new mac is %s\n", ipaddr, newmac);
#endif

    mSocket.SendTo(&command, sizeof(command), mPort, address);

}

// TODO: at date 22 jun 16: yet to be tested
bool EthUpdater::cmdPageClr(eOuprot_pagesize_t pagesize, ACE_UINT32 address)
{
    eOuprot_cmd_PAGE_CLR_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_CLR;
    command.pagesize = pagesize;

    if(0 == address)
    {
        sendCommandSelected(&command, sizeof(command));
    }
    else
    {
        mSocket.SendTo(&command, sizeof(command), mPort, address);
    }

    return true;
}

// TODO: at date 22 jun 16: yet to be tested
bool EthUpdater::cmdPageSet(eOuprot_pagesize_t pagesize, uint8_t *data, ACE_UINT32 address)
{
    if(NULL == data)
    {
        return false;
    }

    eOuprot_cmd_PAGE_SET_t *cmd = (eOuprot_cmd_PAGE_SET_t*) mTxBuffer;
    uint16_t sizeofcmd = sizeof(eOuprot_cmd_PAGE_SET_t) - uprot_pagemaxsize + pagesize;
    // think of the best way to specify the length of themessage in a proper way
    // we could decide to send always a length of sizeof(eOuprot_cmd_PAGE_SET_t) which is 132, or ...

    memset(cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeofcmd);

    cmd->opc = uprot_OPC_PAGE_SET;
    cmd->pagesize = pagesize;
    memcpy(&cmd->page[0], data, pagesize);

    if(0 == address)
    {
        sendCommandSelected(cmd, sizeofcmd);
    }
    else
    {
        mSocket.SendTo(cmd, sizeofcmd, mPort, address);
    }

    return true;
}


// TODO: at date 22 jun 16: yet to be tested
bool EthUpdater::cmdPageGet(eOuprot_pagesize_t pagesize, uint8_t **data, ACE_UINT32 address)
{
    if(NULL == data)
    {
        return false;
    }

    if(0 == address)
    {
        return false;
    }

    eOuprot_cmd_PAGE_GET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_GET;
    command.pagesize = pagesize;

    mSocket.SendTo(&command, sizeof(command), mPort, address);


    ACE_UINT16 rxPort;
    ACE_UINT32 rxAddress;

    bool ret = false;

    while(mSocket.ReceiveFrom(mRxBuffer, sizeof(mRxBuffer), rxAddress, rxPort, 500)>0)
    {
        eOuprot_cmd_PAGE_GET_REPLY_t * pageget = (eOuprot_cmd_PAGE_GET_REPLY_t*) mRxBuffer;

        if(uprot_OPC_PAGE_GET == pageget->reply.opc)
        {
            if(address == rxAddress)
            {
                if((uprot_RES_OK == pageget->reply.res) && (pagesize == pageget->pagesize))
                {
                    *data = pageget->page;
                    ret = true;
                }
            }
        }
    }

    return ret;
}


// eof


