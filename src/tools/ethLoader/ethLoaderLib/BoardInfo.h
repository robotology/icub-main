// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_BOARDINFO_H__
#define __ETHUPDATER_BOARDINFO_H__

class BoardInfo
{
public:
    BoardInfo(ACE_UINT32 address,
              ACE_UINT32 mask,
              ACE_UINT64 mac,
              ACE_UINT8 version,
              ACE_UINT8 release,
              ACE_UINT8 build)
    {
        mSelected=true;

        mAddress=address;
        mMask=mask;
        mMac=mac;
        
        mVersion=version;
        mRelease=release;
        mBuild=build;

        mSuccess=0; 
    }

    virtual ~BoardInfo(){}

    bool mSelected;

    ACE_UINT32 mAddress;
    ACE_UINT32 mMask;
    ACE_UINT64 mMac;

    ACE_UINT8 mVersion;
    ACE_UINT8 mRelease;
    ACE_UINT8 mBuild;

    ACE_UINT8 mSuccess;
};

#endif
