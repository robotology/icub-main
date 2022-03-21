/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include "EoBoards.h"

bool operator==(const eObrd_canmonitor_cfg_t &right, const eObrd_canmonitor_cfg_t &left)
{
    if (right.periodofcheck != left.periodofcheck)
        return false;
    if (right.reportmode != left.reportmode)
        return false;
    if (right.periodofreport != left.periodofreport)
        return false;
    return true;
}

bool operator==(const eObrd_firmwareversion_t &right, const eObrd_firmwareversion_t &left)
{
    if (right.major != left.major)
        return false;
    if (right.minor != left.minor)
        return false;
    if (right.build != left.build)
        return false;

    return true;
}
bool operator!=(const eObrd_firmwareversion_t &right, const eObrd_firmwareversion_t &left)
{
    return !(right==left);
}


bool operator==(const eObrd_protocolversion_t &right, const eObrd_protocolversion_t &left)
{
    if (right.major != left.major)
        return false;
    if (right.minor != left.minor)
        return false;

    return true;
}
bool operator!=(const eObrd_protocolversion_t &right, const eObrd_protocolversion_t &left)
{
    return !(right==left);
}



bool operator==(const eObrd_canlocation_t &right, const eObrd_canlocation_t &left)
{
    if (right.port != left.port)
        return false;
    if (right.addr != left.addr)
        return false;
    if (right.insideindex != left.insideindex)
        return false;

    return true;
}

bool operator!=(const eObrd_canlocation_t &right, const eObrd_canlocation_t &left)
{
    return !(right==left);
}


bool operator==(const eOas_ft_sensordescriptor_t &right, const eOas_ft_sensordescriptor_t &left)
{
    if (right.boardinfo.type != left.boardinfo.type)
        return false;
    if (right.boardinfo.protocol != left.boardinfo.protocol)
        return false;
    if (right.boardinfo.firmware != left.boardinfo.firmware)
        return false;
    if (right.canloc != left.canloc)
        return false;

    return true;
}
bool operator!=(const eOas_ft_sensordescriptor_t &right, const eOas_ft_sensordescriptor_t &left)
{
    return !(right==left);
}