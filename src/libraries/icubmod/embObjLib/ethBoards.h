// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
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

// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _ETHBOARDS_H_
#define _ETHBOARDS_H_


#include "IethResource.h"
#include "EoProtocol.h"
#include <abstractEthResource.h>

namespace eth {

    // -- class EthBoards
    // -- it collects all the ETH boards managed by ethManager.
    // -- each board surely has an EthResource object associated to it. and it may have one or more interfaces which use the
    // -- services of EthResource to transmit or receive.
    // -- it is responsibility of the object which owns EthBoards (it is ethManager) to protect the class EthBoards vs concurrent use.
    // -- examples of concurrent use are: transmit or receive using an ethresource and ... attempting to create or destroy a resource.

    typedef struct
    {
        IethResource* interface;
        iethresType_t type;
    } interfaceInfo_t;
           
    class EthBoards
    {

    public:

        enum { maxEthBoards = 32 };

    public:

        EthBoards();
        ~EthBoards();

        size_t number_of_resources(void);
        bool add(eth::AbstractEthResource* res);
        eth::AbstractEthResource* get_resource(eOipv4addr_t ipv4);
        bool rem(eth::AbstractEthResource* res);

        size_t number_of_interfaces(eth::AbstractEthResource* res);
        bool add(eth::AbstractEthResource* res, eth::IethResource* interface);
        eth::IethResource* get_interface(eOipv4addr_t ipv4, eOprotID32_t id32);
        eth::IethResource* get_interface(eOipv4addr_t ipv4, iethresType_t type);
        bool rem(eth::AbstractEthResource* res, iethresType_t type);
        

        // the name of the board
        const string & name(eOipv4addr_t ipv4);

        // executes an action on all EthResource which have been added in the class.
        bool execute(void (*action)(eth::AbstractEthResource* res, void* p), void* par);

        // executes an action on the ethResource having a specific ipv4.
        bool execute(eOipv4addr_t ipv4, void (*action)(eth::AbstractEthResource* res, void* p), void* par);


    private:

        // private types      

        typedef struct
        {
            eOipv4addr_t            ipv4;
            string                  name;
            uint8_t                 numberofinterfaces;
            uint8_t                 boardnumber;
            AbstractEthResource*    resource;
            IethResource*           interfaces[iethresType_numberof];
        } ethboardProperties_t;


    private:

        // private variables

        static const string defaultnames[EthBoards::maxEthBoards];
        static const string errorname[1];

        int sizeofLUT;
        ethboardProperties_t LUT[EthBoards::maxEthBoards];

    private:

        // private functions
        bool get_LUTindex(eOipv4addr_t ipv4, uint8_t &index);
    };

} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







