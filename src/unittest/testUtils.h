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