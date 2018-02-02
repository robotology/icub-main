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

#ifndef _STRAININTERFACE_H_
#define _STRAININTERFACE_H_


#include <stdio.h>
#include <vector>

using namespace std;

#include "downloader.h"


class strainInterface
{
public:

    enum class Network { ETH = 0, socketcan = 1, ecan = 2 };
    enum class CanBus { one = 1, two = 2, all = CanPacket::everyCANbus };
    enum class CanAddress { one = 1, two = 2, three = 3, four = 4, five = 5, six = 6, seven = 7, eigth = 8,
                            nine = 9, ten = 10, eleven = 11, twelve = 12, thirteen = 13, fourteen = 14 };
    struct Config
    {
        Network network;
        CanBus canbus;
        CanAddress canaddress;
        unsigned char txrate; // in ms
        Config() { load_default(); }
        const string& get_networkstring() const {
            static const string nets[] = {"ETH", "socketcan", "ecan"}; return nets[static_cast<unsigned char>(network)];
        }
        void load_default() { network = Network::ETH; canbus = CanBus::all; canaddress = CanAddress::thirteen; txrate = 2; }
        int get_canaddress() const { return static_cast<int>(canaddress); }
        int get_canbus() const { return static_cast<int>(canbus); }
        int get_txrate() const { return txrate; }
    };


    strainInterface() : opened(false) {}
    ~strainInterface(){}

    bool open(const Config &cfg = Config());
    bool close();

    bool get(const unsigned int number, vector<cDownloader::strain_value_t> &values);

    bool print(const vector<cDownloader::strain_value_t> &values, FILE *fp = NULL);

public:

    cDownloader downloader;

private:

    bool opened;
    Config config;

};

#endif  // include-guard

// marco.accame: end
