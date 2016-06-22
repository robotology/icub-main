// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_BOARDINFO_H__
#define __ETHUPDATER_BOARDINFO_H__


#include <string>
using namespace std;

#include "EoCommon.h"
#include "EoUpdaterProtocol.h"
#include "EoBoards.h"


typedef struct
{
    uint8_t             protversion;
    uint64_t            macaddress;
    uint8_t             boardtype;
    uint32_t            capabilities;       // very useful field. it allows to enable / disable some buttons of the GUI
    eOuprot_proctable_t processes;
    uint8_t             boardinfo32[32];
} boardInfo_t;


class BoardInfo
{
public:
    BoardInfo(ACE_UINT32 address,
              ACE_UINT32 mask,
              ACE_UINT64 mac,
              ACE_UINT8 version_major,
              ACE_UINT8 version_minor,
              ACE_UINT8 protocol_version,
              ACE_UINT32 protocol_capabilities)
    {
        mSelected=false;

        mAddress=address;
        mMask=mask;
        mMac=mac;
        
        mProtocolVersion = protocol_version;
        mProtocolCapabilities = protocol_capabilities;

        mVersionMajor=version_major;
        mVersionMinor=version_minor;

        mBoardType = string("??");

        mRunningProcess = string("??");

        mInfo32 = string("??");

        mReleasedOn = string("??");
        mBuiltOn = string("??");

        mSuccess=0; 
    }


    BoardInfo(ACE_UINT32 address, boardInfo_t &info)
    {
        mSelected = false;

        mAddress = address;

        mInfo = info;

        mProtocolVersion = info.protversion;
        mProtocolCapabilities = info.capabilities;

        mMask = 0xFFFFFF00;
        mMac = info.macaddress;

        eOuprot_process_t runningnow = eouprot_raw2process(info.processes.runningnow);
        mRunningProcess = from_process_to_string(runningnow);
        mBoardType = from_boardtype_to_string(info.boardtype);
        mInfo32 = (0xff == info.boardinfo32[0]) ? (string("N/A: PAGE32 IS UNFORMATTED")) : (string((const char*) &info.boardinfo32[1]));

        uint8_t index = eouprot_process2index(runningnow);
        if(255 == index)
        {   // error ...
            mVersionMajor = 0;
            mVersionMinor = 0;
            mReleasedOn = string("??");
            mBuiltOn = string("??");
        }
        else
        {
            mVersionMajor = info.processes.info[index].version.major;
            mVersionMinor = info.processes.info[index].version.minor;
            mReleasedOn = from_date_to_string(info.processes.info[index].date);
            mBuiltOn = from_date_to_string(info.processes.info[index].compilationdate);
        }

        mSuccess=0;
    }

    virtual ~BoardInfo(){}

    bool mSelected;

    ACE_UINT32 mAddress;
    ACE_UINT32 mMask;
    ACE_UINT64 mMac;

    ACE_UINT8 mProtocolVersion;
    ACE_UINT32 mProtocolCapabilities;
    ACE_UINT8 mVersionMajor;
    ACE_UINT8 mVersionMinor;

    ACE_UINT16 mSuccess;

    string mBoardType;
    string mRunningProcess;
    string mInfo32;
    string mBuiltOn;
    string mReleasedOn;


    boardInfo_t mInfo;

private:

    string from_boardtype_to_string(uint8_t type)
    {
        // marco.accame.TODO: use what in EoCommon.c .......
        string ret(eoboards_type2string((eObrd_type_t)type));
        return ret;
    }

    string from_process_to_string(eOuprot_process_t type)
    {
        string ret(eouprot_process2string(type));
        return ret;
    }

    string from_date_to_string(eOdate_t date)
    {
        char strtmp[64];
        eo_common_date_to_string(date, strtmp, sizeof(strtmp));
        string ret(strtmp);
        return ret;
    }

};

#endif
