/*
 * Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ICUB_POS_SERVICE_CONFIGURATION_H
#define ICUB_POS_SERVICE_CONFIGURATION_H

#include "EoManagement.h"

#include <mutex>

namespace eth {

// Owned by one Ethernet resource, so a dependency can only be resolved from
// standalone POS activated on the same board. It never changes the wire format.
class POSServiceConfiguration
{
public:
    bool prepare(eOmn_serv_category_t category, const eOmn_serv_parameter_t* requested,
                 eOmn_serv_parameter_t& prepared) const
    {
        prepared = requested ? *requested : eOmn_serv_parameter_t{};
        if(!requested || (category != eomn_serv_category_mc) ||
           (prepared.configuration.type != eomn_serv_MC_mc4plusfaps))
        {
            return true;
        }

        auto& dependency = prepared.configuration.data.mc.mc4plusfaps.pos;
        if(dependency.config.boardconfig[0].boardinfo.type != eobrd_cantype_none)
        {
            return true; // An explicit legacy POS dependency remains unchanged.
        }

        std::lock_guard<std::mutex> lock(mutex);
        if(!available)
        {
            return false;
        }
        dependency = configuration;
        return true;
    }

    // Call only after standalone POS verification/activation succeeds.
    void rememberActivated(eOmn_serv_category_t category, const eOmn_serv_parameter_t* parameter)
    {
        if(!parameter || (category != eomn_serv_category_pos) ||
           (parameter->configuration.type != eomn_serv_AS_pos))
        {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        // Firmware keeps its first active POS configuration when another owner
        // verifies a compatible service. Mirror that behavior in the host cache.
        if(!available)
        {
            configuration = parameter->configuration.data.as.pos;
            available = true;
        }
    }

    void clearOnStop(eOmn_serv_category_t category)
    {
        if((category == eomn_serv_category_pos) || (category == eomn_serv_category_all))
        {
            std::lock_guard<std::mutex> lock(mutex);
            configuration = {};
            available = false;
        }
    }

private:
    mutable std::mutex mutex;
    eOmn_serv_config_data_as_pos_t configuration {};
    bool available {false};
};

} // namespace eth

#endif
