
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

string ipv4tostring(eOipv4addr_t ipv4)
{
    char ipv4text[20];
    eo_common_ipv4addr_to_string(ipv4, ipv4text, sizeof(ipv4text));
    string ret(ipv4text);
    return ret;
}

bool string2ipv4(const string &ipv4string, eOipv4addr_t &ipv4)
{
    int ip1, ip2, ip3, ip4;
    int n = sscanf(ipv4string.c_str(), "%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);
    if(4 != n)
    {
        return false;
    }

    ipv4 = EO_COMMON_IPV4ADDR(ip1, ip2, ip3, ip4);

    return true;
}


eOipv4addr_t acetoipv4(ACE_UINT32 address)
{
    return htonl(address);
}


ACE_UINT32 ipv4toace(eOipv4addr_t ipv4)
{
    return ntohl(ipv4);
}

const eOipv4addr_t EthMaintainer::hostIPaddress = EO_COMMON_IPV4ADDR(10, 0, 1, 104);
const eOipv4port_t EthMaintainer::mainIPport = 3333;

const eOipv4addr_t EthMaintainer::ipv4OfAllSelected = EO_COMMON_IPV4ADDR(0, 0, 0, 0);

const eOipv4addr_t EthMaintainer::ipv4Broadcast = EO_COMMON_IPV4ADDR(255, 255, 255, 255);




EthMaintainer::EthMaintainer()
{
    _opened = false;
    _verbose = true;
    _debugprint = false;

    _useofinternalboardlist = true;
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
        _internalboardlist.clear();
        _opened = false;
    }
}


void EthMaintainer::verbose(bool on)
{
    _verbose = on;
}


void EthMaintainer::debugprint(bool on)
{
    _debugprint = on;
}

//EthBoardList& EthMaintainer::getBoards()
//{
//    return _internalboardlist;
//}


//void EthMaintainer::clearBoards()
//{
//    _internalboardlist.clear();
//}

//int EthMaintainer::addBoard(eOipv4addr_t ipv4)
//{
//    boardInfo2_t info2;

//    bool force = true;
//    _internalboardlist.add(info2, ipv4, force);

//    return _internalboardlist.size();
//}

//int EthMaintainer::remBoard(eOipv4addr_t ipv4)
//{
//    _internalboardlist.rem(ipv4);
//    return _internalboardlist.size();
//}






bool EthMaintainer::command_eeprom_erase(eOipv4addr_t ipv4)
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


bool EthMaintainer::command_eeprom_read(eOipv4addr_t ipv4, uint16_t from, uint16_t size, uint8_t **value)
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



bool EthMaintainer::command_program(eOipv4addr_t ipv4, FILE *programFile, eOuprot_partition2prog_t partition, void (*updateProgressBar)(float), EthBoardList *pboardlist, string &stringresult)
{

    EthBoardList *boardlist2use = pboardlist;
    if(NULL == boardlist2use)
    {
        boardlist2use = &_internalboardlist;
    }

    progData_t progdata;
    progdata.mN2Prog = 0;
    progdata.mNProgSteps = 0;
    progdata.mNChunks = 0;

    progdata.selected = boardlist2use->get(ipv4);

    progdata.steps.resize(progdata.selected.size(), 0);

    if(NULL != updateProgressBar)
    {
        updateProgressBar(0.0f);
    }

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

    // send the start command to all selected
    sendCommand(ipv4, cmdStart, sizeStart, boardlist2use);
    // wait a tick
    yarp::os::Time::delay(0.01);



    if(_verbose)
    {
        printf("EthMaintainer::cmdProgram() is about to program the %s partition of %d boards:", partname.c_str(), (int)progdata.selected.size());
        for(int j=0; j< progdata.selected.size(); j++)
        {
            eOipv4addr_t ipv4 = progdata.selected[j]->getIPV4();
//            bool ok = isCommandSupported(capability, ipv4);
            bool ok = command_supported(ipv4, capability);

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
                                if(_verbose)
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
            string ipv4adrstring = ipv4tostring(ipv4adr);

            earlyexit += ipv4adrstring;
            earlyexit += ": ";
            earlyexit += partname;
            earlyexit += (progdata.steps[i] == progdata.mNProgSteps)?" OK\r\n":" CANT (early exit)\r\n";
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
                        if(NULL != updateProgressBar)
                        {
                            updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                        }
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
                if(NULL != updateProgressBar)
                {
                    updateProgressBar(1.0f);
                }
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
                    if(NULL != updateProgressBar)
                    {
                        updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                    }
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
                if(NULL != updateProgressBar)
                {
                    updateProgressBar(float(bytesWritten+=bytesToWrite)/fileSize);
                }
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

    if(NULL != updateProgressBar)
    {
        updateProgressBar(1.0f);
    }


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
        string ipv4adrstring = ipv4tostring(ipv4adr);

        sOutput += ipv4adrstring;
        sOutput += ": ";
        sOutput += partname;
        sOutput += (ok)?" OK\r\n":" NOK\r\n";
    }

    stringresult = sOutput;

    return result;
}


// helper functions


bool EthMaintainer::sendCommand(eOipv4addr_t ipv4, void * cmd, uint16_t len, EthBoardList *boardlist)
{
    bool ret = false;
    if(ipv4OfAllSelected == ipv4)
    {
        if(NULL == boardlist)
        {
            boardlist = &_internalboardlist;
        }
        vector<EthBoard *> selected = boardlist->get(ipv4);
        for(int i=0; i<selected.size(); i++)
        {
            mSocket.SendTo(selected[i]->getIPV4(), myIPV4port, cmd, len);
            ret = true;
        }
    }
    else if(ipv4Broadcast == ipv4)
    {
        mSocket.SendBroad(myIPV4port, cmd, len);
        ret = true;
    }
    else
    {
        mSocket.SendTo(ipv4, myIPV4port, cmd, len);
        ret = true;
    }
    return ret;
}



bool EthMaintainer::isInMaintenance(eOipv4addr_t ipv4, EthBoardList &boardlist)
{
    vector<EthBoard *> selected = boardlist.get(ipv4);

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


bool EthMaintainer::isInApplication(eOipv4addr_t ipv4, EthBoardList &boardlist)
{
    vector<EthBoard *> selected = boardlist.get(ipv4);

    if(selected.empty())
    {
        return false;
    }

    for(int i=0; i<selected.size(); i++)
    {
       if(false == selected[i]->isInApplication())
       {
           return false;
       }
    }

    return true;
}


string EthMaintainer::processDiscoveryReplies2(EthBoardList &boardlist, double waittimeout)
{
    string info;

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), waittimeout*1000.0) > 0)
    {
        eOuprot_cmd_DISCOVER_REPLY_t * disc = (eOuprot_cmd_DISCOVER_REPLY_t*) mRxBuffer;
        eOuprot_cmd_LEGACY_SCAN_REPLY_t * scan = (eOuprot_cmd_LEGACY_SCAN_REPLY_t*) mRxBuffer;

        string ipv4rxstring = ipv4tostring(rxipv4addr);

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

                uint8_t index = eouprot_process2index((eOuprot_process_t)disc->processes.runningnow);

                binfo.versionOfRunning.major = binfo.processes.info[index].version.major;
                binfo.versionOfRunning.minor = binfo.processes.info[index].version.minor;

                binfo.moreinfostring = prepareMoreInfoText(disc, ipv4rxstring.c_str());

                boardlist.add(binfo, rxipv4addr);

                // now we add into the string
                {
                    info += binfo.moreinfostring;
                }


                if(_verbose)
                {
                    printf("EthMaintainer::processDiscoveryReplies2() has found board @ %s: %s w/ %s v %d.%d running protocol v %d w/ capabilities = 0x%x. mainteinance = %s\n",
                            ipv4rxstring.c_str(),
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
                // because: if the applition replied with legacy protocol then it surely goes into updater.
                //          if it was teh updater then ... ok.
                //          we dont use an old maintainer anymore .
                // moreover ... we decide that the running process is the updater.
                binfo.maintenanceIsActive = true;
                binfo.processes.runningnow = eUpdater;
                binfo.processes.info[eUpdater].type = eUpdater;
                binfo.processes.info[eUpdater].version.major = procmajor;
                binfo.processes.info[eUpdater].version.minor = procminor;

                binfo.versionOfRunning.major = procmajor;
                binfo.versionOfRunning.minor = procminor;

                boardlist.add(binfo, rxipv4addr);


                if(_verbose)
                {
                    printf("EthMaintainer::processDiscoveryReplies2() has found board @ %s in maintenance: the running process is v %d.%d, uses legacy protocol, i assume protocol capabilities = 0x%x.\n",
                                                    ipv4rxstring.c_str(), procmajor, procminor, binfo.capabilities);
                    fflush(stdout);
                }
            }
        }
    }

    return info;

}



std::string EthMaintainer::processMoreInfoReplies(EthBoardList &boardlist)
{
    std::string info;

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 500) > 0)
    {
        eOuprot_cmd_MOREINFO_REPLY_t *moreinfo = (eOuprot_cmd_MOREINFO_REPLY_t*) mRxBuffer;
        eOuprot_cmd_LEGACY_PROCS_REPLY_t *procs = (eOuprot_cmd_LEGACY_PROCS_REPLY_t*) mRxBuffer;

        string ipv4rxstring = ipv4tostring(rxipv4addr);

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

                binfo.moreinfostring = prepareMoreInfoText(disc, ipv4rxstring.c_str());

                boardlist.add(binfo, rxipv4addr);

                // now we add into the string
                {
                    info += binfo.moreinfostring;
                }


                if(_verbose)
                {
                    // print binfo.
                    // it would be much bettwer, however, store it somewhere and made it available
                    // through some method


                    printf("\nBOARD at address %s:", ipv4rxstring.c_str());
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
                moreinfostring += std::string("Board\t")+ipv4rxstring;
                moreinfostring += "\r\n\r\n";
                moreinfostring += std::string((char*)procs->description);


                // 2. i retrieve the boards
                vector<EthBoard *> boards = boardlist.get(rxipv4addr);
                for(int i=0; i<boards.size(); i++)
                {
                    boards[i]->setMoreInfo(moreinfostring);
                }

                // finally we update return string

                info += moreinfostring;


                if(_verbose)
                {
                    printf("\n received a uprot_OPC_LEGACY_PROCS from IP %s \n", ipv4rxstring.c_str());
                    fflush(stdout);
                }

            }

        }
    }


    return info;

}


std::string EthMaintainer::prepareMoreInfoText(eOuprot_cmd_DISCOVER_REPLY_t * disc, const char *ipv4rxaddr_string)
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

bool EthMaintainer::boards_useinternal(bool on)
{
    _useofinternalboardlist = on;
    return true;
}


EthBoardList EthMaintainer::discover(bool clearbeforediscovery, int numberofdiscoveries, double waittimeout)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    if(clearbeforediscovery)
    {
        list2use->clear();
    }

    const bool forceUpdatingMode = false;
    eOuprot_cmd_DISCOVER_t * cmd = (eOuprot_cmd_DISCOVER_t*) mTxBuffer;
    memset(cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_DISCOVER_t));
    cmd->opc = uprot_OPC_LEGACY_SCAN;
    cmd->opc2 = uprot_OPC_DISCOVER;
    cmd->jump2updater = (true == forceUpdatingMode) ? (1) : (0);

    for(int i=0; i<numberofdiscoveries; i++)
    {
        sendCommand(ipv4Broadcast, cmd, sizeof(eOuprot_cmd_DISCOVER_t), list2use);

        processDiscoveryReplies2(*list2use, waittimeout);
    }

    return *list2use;
}


bool EthMaintainer::boards_set(EthBoardList &boards)
{
    _internalboardlist.theboards = boards.theboards;
    return true;
}


EthBoardList& EthMaintainer::boards_get(void)
{
    return _internalboardlist;
}


bool EthMaintainer::boards_clr(void)
{
    _internalboardlist.clear();
    return true;
}

bool EthMaintainer::boards_select(eOipv4addr_t ipv4, bool on)
{
    _internalboardlist.select(on, ipv4);
    return true;
}

int EthMaintainer::boards_add(eOipv4addr_t ipv4, boardInfo2_t &info2, bool force)
{
    _internalboardlist.add(info2, ipv4, force);

    return _internalboardlist.size();
}

int EthMaintainer::boards_rem(eOipv4addr_t ipv4)
{
    _internalboardlist.rem(ipv4);
    return _internalboardlist.size();
}


bool EthMaintainer::command_supported(eOipv4addr_t ipv4, eOuprot_proc_capabilities_t capability, bool ask2board)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    if(ask2board)
    {
        //boardlist = information(ipv4, true, false, 1, 1.0);
    }

    vector<EthBoard *> boards = list2use->get(ipv4);

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


EthBoardList EthMaintainer::information(eOipv4addr_t ipv4, bool ask2board, bool forcemaintenance, int numberofrequests, double waittimeout)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    if(true == ask2board)
    {
        // send the command

        if(forcemaintenance)
        {
            go2maintenance(ipv4, true, 6, 1.0);
        }

        // now we send the info request. we use the uprot_OPC_DISCOVER w/out go2updater

        const bool forceUpdatingMode = false;
        eOuprot_cmd_DISCOVER_t * cmd = (eOuprot_cmd_DISCOVER_t*) mTxBuffer;
        memset(cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_DISCOVER_t));
        cmd->opc = uprot_OPC_LEGACY_SCAN;
        cmd->opc2 = uprot_OPC_DISCOVER;
        cmd->jump2updater = (true == forceUpdatingMode) ? (1) : (0);

        for(int i=0; i<numberofrequests; i++)
        {
            sendCommand(ipv4Broadcast, cmd, sizeof(eOuprot_cmd_DISCOVER_t), list2use);

            processDiscoveryReplies2(*list2use, waittimeout);
        }
    }

    // extract from *list2use all the relevant ipv4 ....
    EthBoardList ret;

    vector<EthBoard *> bb = list2use->get(ipv4);

    for(int i=0; i<bb.size(); i++)
    {   // we add the info and we keep the selection ...
        ret.add(bb.at(i)->getInfo(), bb.at(i)->getIPV4());
        ret.select(bb.at(i)->isSelected(), bb.at(i)->getIPV4());
    }

    return ret;
}


std::string EthMaintainer::moreinformation(eOipv4addr_t ipv4, bool forcemaintenance)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    const bool ask2board =  true;
    const int numberofrequests = 1;
    //double waittimeout = 1.0;
    string ret;

    if(true == ask2board)
    {
        // send the command

        if(forcemaintenance)
        {
            go2maintenance(ipv4, true, 6, 1.0);
        }

        // now we send the info request. we use the uprot_OPC_DISCOVER w/out go2updater

        eOuprot_cmd_MOREINFO_t command;

        command.opc = uprot_OPC_LEGACY_PROCS;
        command.opc2 = uprot_OPC_MOREINFO;
        command.plusdescription = 1;
        command.jump2updater = 0;

        for(int i=0; i<numberofrequests; i++)
        {
            sendCommand(ipv4, &command, sizeof(command), list2use);

            ret += processMoreInfoReplies(*list2use);
        }
    }

    return ret;
}

bool EthMaintainer::go2maintenance(eOipv4addr_t ipv4, bool verify, int retries, double timegap)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }


    const bool forceUpdatingMode = true;
    eOuprot_cmd_DISCOVER_t command;
    memset(&command, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(eOuprot_cmd_DISCOVER_t));
    command.opc = uprot_OPC_LEGACY_SCAN;
    command.opc2 = uprot_OPC_DISCOVER;
    command.jump2updater = (true == forceUpdatingMode) ? (1) : (0);

    // step 1. we send command and process replies a first time
    sendCommand(ipv4, &command, sizeof(command), list2use);
    processDiscoveryReplies2(*list2use);


    // step 2. must verify that all boards to which we sent the command are in maintenance
    if(true == verify)
    {
        for(int iter=0; iter<retries; iter++)
        {
            if(isInMaintenance(ipv4, *list2use))
            {
                if(_verbose)
                {
                    printf("EthMaintainer::go2maintenance() succesfully sent in maintenance board %s after %d attempts\n", ipv4tostring(ipv4).c_str(), iter+1);
                }
                return true;
            }

            yarp::os::Time::delay(timegap);

            if(_verbose)
            {
                printf("EthMaintainer::go2maintenance() resent command for the %d-th time to board %s after %f secs\n", iter+1, ipv4tostring(ipv4).c_str(), timegap);
            }

            // re-send the message and process replies
            sendCommand(ipv4, &command, sizeof(command), list2use);
            processDiscoveryReplies2(*list2use);
        }

    }

    bool ret = isInMaintenance(ipv4, *list2use);

    if(_verbose)
    {
        if(ret)
        {
            printf("EthMaintainer::go2maintenance() succesfully sent in maintenance board %s after %d retries\n", ipv4tostring(ipv4).c_str(), retries);
        }
        else
        {
            printf("EthMaintainer::go2maintenance() could not send in maintenance board %s even after %d retries\n", ipv4tostring(ipv4).c_str(), retries);
        }
    }

    return ret;
}


bool EthMaintainer::go2application(eOipv4addr_t ipv4, bool checkdef2runapplication, double bootstraptime, bool verify)
{    
    bool ret = false;

    EthBoardList boardlist = information(ipv4, true, false, 2, 1.0);
    vector<EthBoard *> pboards;

    // check if the application is running and ifde2run is application.
    // if ok we do nothing.
    // else ...

    bool forcemaintainance = false;
    bool forcedef2runapplication = false;

    if(true == isInApplication(ipv4, boardlist))
    {
        if(checkdef2runapplication)
        {
            // must verify for all the boards
            pboards = boardlist.get(ipv4);
            for(int i=0; i<pboards.size(); i++)
            {
                boardInfo2_t info = pboards[i]->getInfo();
                if(eApplication != info.processes.def2run)
                {
                    forcemaintainance = true;
                    forcedef2runapplication = true;
                    break;
                }
            }

            if((false == forcemaintainance) && (false == forcedef2runapplication))
            {
                return true;
            }
        }
    }

    // if in here ... i send them all in maintenance anyway
    if(false == go2maintenance(ipv4))
    {
        if(_verbose)
        {
            printf("EthMaintainer::go2application() has called go2maintenance() which has failed for board %s.\n", ipv4tostring(ipv4).c_str());
            fflush(stdout);
        }
        return false;
    }

    // and i set the def2run anyway ...

    if(false == command_def2run(ipv4, eApplication, false, true))
    {
        if(_verbose)
        {
            printf("EthMaintainer::go2application() has called command_def2run() which has failed for board %s.\n", ipv4tostring(ipv4).c_str());
            fflush(stdout);
        }
        return false;
    }


    // now we are ok to send a restart, wait for some seconds, discover, check ...
    command_restart(ipv4);

    yarp::os::Time::delay(bootstraptime);

    if(!verify)
    {
        return true;
    }

    boardlist = information(ipv4, true, false, 2, 1.0);

    ret = isInApplication(ipv4, boardlist);

    if(false == ret)
    {
        pboards = boardlist.get(ipv4);
        if(pboards.empty())
        {
            if(_verbose)
            {
                printf("EthMaintainer::go2application(): could not find any board %s after restart and wait of %f seconds \n", ipv4tostring(ipv4).c_str(), bootstraptime);
            }
            return false;
        }
        else
        {
            if(_verbose)
            {
                printf("EthMaintainer::go2application(): some boards %s are not in eApplication after restart and wait of %f seconds:\n", ipv4tostring(ipv4).c_str(), bootstraptime);
                for(int i=0; i<pboards.size(); i++)
                {
                    boardInfo2_t info = pboards[i]->getInfo();
                    printf("- board %s is in %s\n", pboards[i]->getIPV4string().c_str(), eouprot_process2string((eOuprot_process_t)info.processes.runningnow));
                }
            }
        }
    }


    return ret;
}



bool EthMaintainer::program(eOipv4addr_t ipv4, eObrd_ethtype_t type, eOuprot_process_t process, eOversion_t targetversion, FILE *fp, bool forcemaintenance, void progress(float), bool restart2application)
{
    bool ret = false;
    string ipv4string = ipv4tostring(ipv4);
    const char *targetboardtext = eoboards_type2string2(eoboards_ethtype2type(type), eobool_true);

    bool checkversion = (0 == (targetversion.major+targetversion.minor)) ? false : true;
    bool checktype = ((eobrd_ethtype_none == type) || (eobrd_ethtype_unknown == type)) ? false : true;

    if(NULL == fp)
    {
        if(_verbose)
        {
            printf("ERROR: EthMaintainer::program() could not open the file with the binary code.\n");
        }
        return false;
    }

    EthBoardList boardlist;
    vector<EthBoard *> pboards;


    if(_verbose)
    {
        printf("\nEthMaintainer::program() is about to program the board @ %s of type %s with an %s.\n", ipv4string.c_str(), targetboardtext, eouprot_process2string(process));
    }

    if(eUpdater == process)
    {
        if(_verbose)
        {
            printf("EthMaintainer::program(): is forcing restart2application = false\n");
        }
        restart2application =  false;
    }


    // check the address
    if(ipv4 == ipv4Broadcast)
    {
        if(_verbose)
        {
            printf("\nERROR: EthMaintainer::program() cannot program the board @ %s because address is invalid.\n", ipv4string.c_str());
        }
        return false;
    }


    if(forcemaintenance)
    {
        if(_verbose)
        {
            printf("\nEthMaintainer::program() is about to send the board @ %s in maintenance.\n", ipv4string.c_str());
        }
        // i send the board in maintenance.
        const bool verify = true;
        if(false == go2maintenance(ipv4, verify, 6, 1.0))
        {
            if(_verbose)
            {
                printf("ERROR: EthMaintainer::program() cannot send the board @ %s in maintenance.\n", ipv4string.c_str());
            }
            return false;
        }
    }

    // i need its info ...
    const bool forcemaintenance4information = false;
    boardlist = information(ipv4, true, forcemaintenance4information, 2, 1.0);

    pboards = boardlist.get(ipv4Broadcast); // i get all the boards ....

    uint8_t nboards = pboards.size();

    if(_verbose)
    {
        printf("WARNING: EthMaintainer::program() has found %d boards @ %s.\n", (int)nboards, ipv4string.c_str());
    }


    for(int i=0; i<nboards; i++)
    {
        boardInfo2_t boardinfo = pboards.at(i)->getInfo();

        const char *found = eoboards_type2string2(eoboards_ethtype2type(boardinfo.boardtype), eobool_true);

        if(_verbose)
        {
            printf("- board %d of %d: %s\n", i+1, nboards, pboards.at(i)->getIPV4string().c_str());
        }

        if(checktype)
        {
            // do the check
            if((eobrd_ethtype_unknown == boardinfo.boardtype) || (eobrd_ethtype_none == boardinfo.boardtype))
            {
                if(_verbose)
                    printf("WARNING: EthMaintainer::program() has found @ %s: type %s and not %s as wanted.\n", pboards.at(i)->getIPV4string().c_str(), found, targetboardtext);
            }
            else if(type != boardinfo.boardtype)
            {
                if(_verbose)
                {
                    printf("ERROR: EthMaintainer::program() has found @ %s: type %s and not %s as wanted.\n", pboards.at(i)->getIPV4string().c_str(), found, targetboardtext);
                }
                return false;
            }
        }

        switch(process)
        {
            case eUpdater:
            {
                if(eApplPROGupdater == boardinfo.processes.runningnow)
                {
                }
                else
                {
                    if(_verbose)
                    {
                        const char *runningprocess = eouprot_process2string((eOuprot_process_t)boardinfo.processes.runningnow);
                        printf("ERROR: EthMaintainer::program() has found @ %s: %s is running and not the eApplPROGupdater as needed to program a %s.\n",
                               pboards.at(i)->getIPV4string().c_str(), runningprocess, eouprot_process2string(process));
                    }
                    return false;
                }
            } break;

            default:
            {
                if(eUpdater == boardinfo.processes.runningnow)
                {
                }
                else
                {
                    if(_verbose)
                    {
                        const char *runningprocess = eouprot_process2string((eOuprot_process_t)boardinfo.processes.runningnow);
                        printf("ERROR: EthMaintainer::program() has found @ %s: %s is running and not the eUpdater as needed to program a %s.\n",
                               pboards.at(i)->getIPV4string().c_str(), runningprocess, eouprot_process2string(process));
                    }
                    return false;
                }
            } break;
        }

    }



    string result;
    eOuprot_partition2prog_t partition = uprot_partitionAPPLICATION;
    switch(process)
    {
        case eLoader:
        {
            partition = uprot_partitionLOADER;
        } break;

        case eApplPROGupdater:
        case eApplication:
        {
            partition = uprot_partitionAPPLICATION;
        } break;

        case eUpdater:
        {
            partition = uprot_partitionUPDATER;
        } break;

        default:
        {
            partition = uprot_partitionAPPLICATION;
        } break;
    };


    // we program the boards .......
    ret = command_program(ipv4, fp, partition, progress, &boardlist, result);


    if(false == ret)
    {
        if(_verbose)
        {
            printf("ERROR: EthMaintainer::program() could not program board %s @ %s: %s.\n",
                   targetboardtext, ipv4string.c_str(),
                   result.c_str());
        }
        return false;
    }


    if(_verbose)
    {
        printf("OK: EthMaintainer::program() has succesfully programmed board %s @ %s.\n", targetboardtext, ipv4string.c_str());
    }


    if(checkversion)
    {
        //  i verify the version

        if(_verbose)
        {
            printf("EthMaintainer::program() will now verify if target version matches w/ programmed version.\n");
        }

        boardlist = information(ipv4, true, false, 2, 1.0);
        pboards = boardlist.get(ipv4Broadcast); // i get them all ...

        if(nboards != pboards.size())
        {
            if(_verbose)
            {
                printf("WARNING: there were %d boards and there are %d now...\n", (int)nboards, (int)boardlist.size());
            }
        }

        for(int i=0; i<pboards.size(); i++)
        {

            boardInfo2_t boardinfo = pboards.at(i)->getInfo();

            uint8_t index = eouprot_process2index(process);
            eOuprot_procinfo_t procinfo = boardinfo.processes.info[index];
            char datestr[32];
            eo_common_date_to_string(procinfo.date, datestr, sizeof(datestr));

            if((procinfo.version.major != targetversion.major) || (procinfo.version.minor != targetversion.minor))
            {
                if(_verbose)
                {
                    printf("\nWARNING: EthMaintainer::program() found an unexpected version for board %s: instead of %d.%d, there is %d.%d dated %s:\n",
                           pboards.at(i)->getIPV4string().c_str(),
                           targetversion.major, targetversion.minor,
                           procinfo.version.major, procinfo.version.minor,
                           datestr
                           );
                }
            }


            if(_verbose)
            {
                printf("\nOK: EthMaintainer::program() has succesfully verified that board @ %s contains a %s w/ v = (%d.%d) dated %s\n",
                            pboards.at(i)->getIPV4string().c_str(),
                            eouprot_process2string(process),
                            procinfo.version.major, procinfo.version.minor,
                            datestr
                            );
                fflush(stdout);
            }
        }
    }

    ret = true;

    if(restart2application)
    {
        ret = go2application(ipv4, true, 10, true);
    }

    return ret;
}


bool EthMaintainer::command_def2run(eOipv4addr_t ipv4, eOuprot_process_t process, bool forcemaintenance, bool verify)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    if(forcemaintenance)
    {
        if(false == go2maintenance(ipv4, true, 6, 1.0))
        {
            if(_verbose)
            {
                printf("EthMaintainer::command_def2run(): cannot send the boards to maintenance.\n");
            }
            return false;
        }
    }

    // send the command

    eOuprot_cmd_DEF2RUN_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_DEF2RUN;
    command.proc = process;

    sendCommand(ipv4, &command, sizeof(command), list2use);

    if(!verify)
    {
        return true;
    }

    // discover again and check

    boardlist = information(ipv4, true, false, 2, 1.0);

    vector<EthBoard *> boards = boardlist.get(ipv4);
    if(0 == boards.size())
    {
        if(_verbose)
        {
            printf("EthMaintainer::command_def2run(): found no boards in verify step.\n");
        }
        return false;
    }

    bool ret = true;
    for(int i=0; i< boards.size(); i++)
    {
        boardInfo2_t info = boards[i]->getInfo();
        if(process != info.processes.def2run)
        {
            if(0 != info.protversion)
            {
                if(_verbose)
                {
                    printf("EthMaintainer::command_def2run(): could not set DEF2RUN = %s for board %s. detected value is %s",
                           eouprot_process2string(process),
                           boards[i]->getIPV4string().c_str(), eouprot_process2string((eOuprot_process_t)info.processes.def2run)
                           );
                }
                ret = false;
            }
            else
            {
                printf("EthMaintainer::command_def2run(): could not check DEF2RUN = %s for board %s beacuse protocolversion is 0",
                       eouprot_process2string(process),
                       boards[i]->getIPV4string().c_str()
                       );
            }

        }
    }

    return ret;
}


bool EthMaintainer::command_restart(eOipv4addr_t ipv4)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    eOuprot_cmd_RESTART_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_RESTART;

    bool ret = sendCommand(ipv4, &command, sizeof(command), list2use);

    if(_verbose)
    {
        printf("EthMaintainer::command_restart(%s): %d\n", ipv4tostring(ipv4).c_str(), ret);
    }

    return ret;
}



bool EthMaintainer::command_changeaddress(eOipv4addr_t ipv4, eOipv4addr_t ipv4new, bool checkifnewispresent, bool forcemaintenance, bool restart, bool verify)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    bool ret = false;

    eOuprot_cmd_IP_ADDR_SET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};

    command.opc = uprot_OPC_LEGACY_IP_ADDR_SET;
    command.opc2 = uprot_OPC_IP_ADDR_SET;
    command.sysrestart = 0; //(restart) ? (1) : (0);

    command.address[3] = (ipv4new>>24) & 0xFF;
    command.address[2] = (ipv4new>>16) & 0xFF;
    command.address[1] = (ipv4new>>8 ) & 0xFF;
    command.address[0] = (ipv4new    ) & 0xFF;


    string ipv4string = ipv4tostring(ipv4);
    string ipv4newstring = ipv4tostring(ipv4new);


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
        if(_verbose)
        {
            printf("EthMaintainer::command_changeaddress(): cannot send command uprot_OPC_IP_ADDR_SET to %s with new address %s because either one or both are not valid\n", ipv4string.c_str(), ipv4newstring.c_str());
        }

        return ret;
    }

    if(checkifnewispresent)
    {   // ipv4 cannot be 0 ...
        boardlist = information(ipv4new, true, false, 1, 1.0);
        if(0 != boardlist.numberof(ipv4new))
        {
            if(_verbose)
            {
                printf("EthMaintainer::command_changeaddress(): cannot send command uprot_OPC_IP_ADDR_SET to %s with new address %s because the new address is already present\n", ipv4string.c_str(), ipv4newstring.c_str());
            }

            return false;
        }
        else
        {
            if(_verbose)
            {
                printf("EthMaintainer::command_changeaddress(): new address %s is not present\n", ipv4newstring.c_str());
            }
        }
    }

    if(forcemaintenance)
    {
        bool r1 = go2maintenance(ipv4, true, 6, 1.0);

        if(false == r1)
        {
            if(_verbose)
            {
                printf("EthMaintainer::command_changeaddress(): failed to send %s in maintenance\n", ipv4newstring.c_str());
            }
            return false;
        }
        else
        {
            if(_verbose)
            {
                printf("EthMaintainer::command_changeaddress(): succesfully sent %s in maintenance\n", ipv4newstring.c_str());
            }
        }

    }


    if(_verbose)
    {
        printf("EthMaintainer::command_changeaddress(): sent command uprot_OPC_IP_ADDR_SET to %s, new address is %s. w/%s sysrestart\n", ipv4string.c_str(), ipv4newstring.c_str(), (0 == command.sysrestart) ? "out" : "");
    }

    bool r = sendCommand(ipv4, &command, sizeof(command), list2use);

    if(false == r)
    {
        if(_verbose)
        {
            printf("EthMaintainer::command_changeaddress(): could not send command\n");
        }
    }

    if(true == restart)
    {
        bool rr = command_restart(ipv4);
        if(_verbose)
        {
            printf("EthMaintainer::command_changeaddress(): called command_restart() w/ res %d\n", rr);
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


    ret = go2maintenance(ipv4new, true, 5, 0.5);

    if(_verbose)
    {
        printf("EthMaintainer::command_changeaddress(): board w/ new address %s is in maintenance: %s\n", ipv4newstring.c_str(), ret ? "YES" : "NO!");
    }

    if(false == ret)
    {
        return false;
    }


    boardlist = information(ipv4new, true, false, 1, 1.0);

    if(0 == boardlist.numberof(ipv4new))
    {
        if(_verbose)
        {
            printf("EthMaintainer::command_changeaddress(): error: we dont have the new address %s ...\n", ipv4newstring.c_str());
        }
        return false;
    }

    if(_verbose)
    {
        printf("EthMaintainer::command_changeaddress(): OK\n");
    }

    return true;
}


bool EthMaintainer::command_info32_clr(eOipv4addr_t ipv4)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    bool ret = true;

    eOuprot_cmd_PAGE_CLR_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_CLR;
    command.pagesize = 32;

    ret = sendCommand(ipv4, &command, sizeof(command), list2use);

    return ret;
}


bool EthMaintainer::command_info32_set(eOipv4addr_t ipv4, const string &info32)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

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

    ret = sendCommand(ipv4, cmd, sizeofcmd, list2use);

    return ret;
}


vector<string> EthMaintainer::command_info32_get(eOipv4addr_t ipv4)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    vector<string> thestrings(0);

    eOuprot_cmd_PAGE_GET_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_PAGE_GET;
    command.pagesize = 32;

    bool r = sendCommand(ipv4, &command, sizeof(command), list2use);

    eOipv4addr_t rxipv4addr;
    eOipv4port_t rxipv4port;

    while(mSocket.ReceiveFrom(rxipv4addr, rxipv4port, mRxBuffer, sizeof(mRxBuffer), 500) > 0)
    {
        eOuprot_cmd_PAGE_GET_REPLY_t * pageget = (eOuprot_cmd_PAGE_GET_REPLY_t*) mRxBuffer;

        string ipv4rxstring = ipv4tostring(rxipv4addr);

        if(uprot_OPC_PAGE_GET == pageget->reply.opc)
        {
            string readstring;

            if((rxipv4addr != myIPV4addr) && (uprot_RES_OK == pageget->reply.res) && (32 == pageget->pagesize) && (0xff != pageget->page[0]))
            {
                readstring = string((const char*)&pageget->page[1]);
                thestrings.push_back(readstring);
                // and update what in boardinfo32
                vector<EthBoard *> selected = _internalboardlist.get(rxipv4addr);
                for(int i=0; i<selected.size(); i++)
                {
                    boardInfo2_t binfo = selected[i]->getInfo();
                    memcpy(binfo.boardinfo32, pageget->page, sizeof(binfo.boardinfo32));
                }
            }

            if(_verbose)
            {
                if(uprot_RES_OK != pageget->reply.res)
                {
                    printf("\n received a eOuprot_cmd_PAGE_GET_REPLY_t from IP %s with a failure result %d for size %d", ipv4rxstring.c_str(), pageget->reply.res, pageget->pagesize);
                }
                else if(rxipv4addr != myIPV4addr)
                {
                    printf("\n received a eOuprot_cmd_PAGE_GET_REPLY_t from IP %s with size %d: ", ipv4rxstring.c_str(), pageget->pagesize);

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


bool EthMaintainer::command_jump2updater(eOipv4addr_t ipv4)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    bool ret = true;

    eOuprot_cmd_JUMP2UPDATER_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_JUMP2UPDATER;

    ret = sendCommand(ipv4, &command, sizeof(command), list2use);

    return ret;
}



bool EthMaintainer::command_jump2address(eOipv4addr_t ipv4, uint32_t romaddress)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    bool ret = true;

    eOuprot_cmd_JUMP2ADDRESS_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_JUMP2ADDRESS;
    command.address = romaddress;

    ret = sendCommand(ipv4, &command, sizeof(command), list2use);

    return ret;
}


bool EthMaintainer::command_blink(eOipv4addr_t ipv4)
{
    EthBoardList boardlist;
    EthBoardList *list2use = &boardlist;
    if(_useofinternalboardlist)
    {
        list2use = &_internalboardlist;
    }

    bool ret = true;

    eOuprot_cmd_BLINK_t command = {EOUPROT_VALUE_OF_UNUSED_BYTE};
    command.opc = uprot_OPC_BLINK;

    ret = sendCommand(ipv4, &command, sizeof(command), list2use);

    return ret;
}


// eof



