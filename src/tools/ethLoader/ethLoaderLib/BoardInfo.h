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

// marco.accame.TODO: use what is inside EoCommon.h and .c ...... eO_date_t etc.
typedef struct
{
    uint32_t            year  : 12;    /**< the year a.d. upto 2047 */
    uint32_t            month : 4;     /**< the month, where Jan is 1, Dec is 12 */
    uint32_t            day   : 5;     /**< the day from 1 to 31 */
    uint32_t            hour  : 5;     /**< the hour from 0 to 23 */
    uint32_t            min   : 6;     /**< the minute from 0 to 59 */
} eO_E_date_t;

typedef struct
{
    uint8_t     type;
    uint8_t     major;
    uint8_t     minor;
    eO_E_date_t builddate;
} eO_E_procInfo_t;

typedef struct
{
    uint8_t         protversion;
    uint64_t        macaddress;
    uint8_t         boardtype;
    uint8_t         startup;
    uint8_t         def2run;
    uint8_t         nprocs;
    eO_E_procInfo_t procinfo[3];
    uint8_t         runningproc;
    uint8_t         info32[32];
} scan2rxdata_t;


class BoardInfo
{
public:
    BoardInfo(ACE_UINT32 address,
              ACE_UINT32 mask,
              ACE_UINT64 mac,
              ACE_UINT8 version_major,
              ACE_UINT8 version_minor)
    {
        mSelected=false;

        mAddress=address;
        mMask=mask;
        mMac=mac;
        
        mVersionMajor=version_major;
        mVersionMinor=version_minor;

        mBoardType = string("??");

        mRunningProcess = string("??");

        mInfo32 = string("??");

        mBuiltOn = string("??");

        mSuccess=0; 
    }

    BoardInfo(ACE_UINT32 address, scan2rxdata_t &data)
    {
        mSelected = false;

        mAddress = address;

        mData = data;


        mMask = 0xFFFFFF00;
        mMac = data.macaddress;

        mVersionMajor = mData.procinfo[mData.runningproc].major;
        mVersionMinor = mData.procinfo[mData.runningproc].minor;

        mBoardType = from_boardtype_to_string(mData.boardtype);

        mRunningProcess = from_process_to_string(mData.runningproc);

        mInfo32 = (0xff == mData.info32[0]) ? (string("NOT PRESENT ON BOARD")) : (string((const char*) &mData.info32[1]));

        mBuiltOn = from_date_to_string(mData.procinfo[mData.runningproc].builddate);

        mSuccess=0;
    }



    virtual ~BoardInfo(){}

    bool mSelected;

    ACE_UINT32 mAddress;
    ACE_UINT32 mMask;
    ACE_UINT64 mMac;

    ACE_UINT8 mVersionMajor;
    ACE_UINT8 mVersionMinor;

    ACE_UINT16 mSuccess;

    string mBoardType;
    string mRunningProcess;
    string mInfo32;
    string mBuiltOn;

    scan2rxdata_t mData;


private:

    string from_boardtype_to_string(uint8_t type)
    {
        // marco.accame.TODO: use what in EoCommon.c .......

        string ret("eobrd_unknown");

        switch(type)
        {
            case 32:
            {
                ret = string("eobrd_ems4");
            } break;
            case 33:
            {
                ret = string("eobrd_mc4plus");
            } break;
            case 34:
            {
                ret = string("eobrd_mc2plus");
            } break;
            default:
            {
            } break;
        }

        return ret;

    }

    string from_process_to_string(uint8_t type)
    {
        // marco.accame.TODO: use what in EoCommon.c .......

        string ret("unknown");

        switch(type)
        {
            case 0:
            {
                ret = string("eLoader");
            } break;
            case 1:
            {
                ret = string("eUpdater");
            } break;
            case 2:
            {
                ret = string("eApplication");
            } break;
            default:
            {
            } break;
        }

        return ret;

    }

    string from_date_to_string(eO_E_date_t date)
    {
        // marco.accame.TODO: use what in EoCommon.c .......

        string ret("unknown");

        const char * months[16] = {"ERR", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec", "ERR", "ERR", "ERR"};
        char str[32] = {0};
        snprintf(str, sizeof(str), "%d %s %.2d %d:%d", date.year, months[date.month], date.day, date.hour, date.min);

        ret = string(str);

        return ret;

    }
};

#endif
