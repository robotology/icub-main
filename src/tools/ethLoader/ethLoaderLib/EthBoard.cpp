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



#include "EthBoard.h"


// - class EthBoard

EthBoard::EthBoard(boardInfo2_t &info2, eOipv4addr_t ipv4)
{
    _info2 = info2;
    _ipv4 = ipv4;
    char tmp[20];
    eo_common_ipv4addr_to_string(ipv4, tmp, sizeof(tmp));
    _ipv4string = string(tmp);
    _selected = false;
}


EthBoard::~EthBoard()
{

}

void EthBoard::setSelected(bool selected)
{
    _selected = selected;
}

bool EthBoard::isSelected()
{
    return _selected;
}

void EthBoard::setIPV4(eOipv4addr_t ipv4)
{
    _ipv4 = ipv4;
    char tmp[20];
    eo_common_ipv4addr_to_string(ipv4, tmp, sizeof(tmp));
    _ipv4string = string(tmp);
}

eOipv4addr_t EthBoard::getIPV4()
{
    return _ipv4;
}

string EthBoard::getIPV4string()
{
    return _ipv4string;
}

boardInfo2_t& EthBoard::getInfo()
{
    return _info2;
}

void EthBoard::setInfo(boardInfo2_t &info2)
{
    _info2 = info2;
}

bool EthBoard::isInMaintenance()
{
    return _info2.maintenanceIsActive;
}

bool EthBoard::isInApplication()
{
   return !_info2.maintenanceIsActive;
}

void EthBoard::setMoreInfo(string &moreinfo)
{
    _info2.moreinfostring = moreinfo;
}

const string EthBoard::getMoreInfo(void)
{
    return _info2.moreinfostring;
}

string EthBoard::getInfoOnEEPROM(void)
{
    string ret;
    ret = (0xff == _info2.boardinfo32[0]) ? (string("N/A: PAGE32 IS UNFORMATTED")) : (string((const char*) &_info2.boardinfo32[1]));
    return ret;
}

string EthBoard::getVersionfRunning(void)
{
    string ret;
    char tmp[32];
    if(uprot_proc_None == _info2.processes.runningnow)
    {
        // see if
        if((0 != _info2.versionOfRunning.major) || (0 != _info2.versionOfRunning.minor))
        {
            snprintf(tmp, sizeof(tmp), "%d.%d", _info2.versionOfRunning.major, _info2.versionOfRunning.minor);
            ret = tmp;
        }
        else
        {
            ret = "N/A";
        }
    }
    else
    {
        snprintf(tmp, sizeof(tmp), "%d.%d", _info2.processes.info[_info2.processes.runningnow].version.major, _info2.processes.info[_info2.processes.runningnow].version.minor);
        ret = tmp;
    }
    return ret;
}

string EthBoard::getDatefRunning(void)
{
    string ret;
    if(uprot_proc_None == _info2.processes.runningnow)
    {
        ret = "N/A";
    }
    else
    {
        char tmp[32];
        eo_common_date_to_string(_info2.processes.info[_info2.processes.runningnow].date, tmp, sizeof(tmp));
        ret = tmp;
    }
    return ret;
}

string EthBoard::getCompilationDateOfRunning(void)
{
    string ret;
    if(uprot_proc_None == _info2.processes.runningnow)
    {
        ret = "N/A";
    }
    else
    {
        char tmp[32];
        eo_common_date_to_string(_info2.processes.info[_info2.processes.runningnow].compilationdate, tmp, sizeof(tmp));
        ret = tmp;
    }
    return ret;
}


// -- class EthBoardList

const eOipv4addr_t EthBoardList::ipv4all = 0xffffffff;
const eOipv4addr_t EthBoardList::ipv4selected = 0;

EthBoardList::EthBoardList()
{
    theboards.clear();
}

EthBoardList::~EthBoardList()
{
    theboards.clear();
}

// it adds if the mac is not inside, else it refreshes the item
int EthBoardList::add(boardInfo2_t &info2, eOipv4addr_t ipv4, bool force)
{

    if(force)
    {
        EthBoard board(info2, ipv4);
        theboards.push_back(board);
        return theboards.size();
    }

    bool addit = true;

    const bool check_MAC_IPV4 = true;

    if(check_MAC_IPV4)
    {
        uint64_t mac = info2.macaddress;

        // look for the same mac
        for(int i=0; i<theboards.size(); i++)
        {
            if(mac == theboards[i].getInfo().macaddress)
            {
                addit = false;
                theboards[i].setInfo(info2);
                theboards[i].setIPV4(ipv4);
                return theboards.size();
            }
        }

        // look for the same ipv4
        for(int i=0; i<theboards.size(); i++)
        {
            if((0 == theboards[i].getInfo().macaddress) && (ipv4 == theboards[i].getIPV4()))
            {   // a board inside the list with a zero mac is a fake one
                addit = false;
                theboards[i].setInfo(info2);
                return theboards.size();
            }
        }

    }

    if(addit)
    {
        EthBoard board(info2, ipv4);
        theboards.push_back(board);
    }

    return theboards.size();
}


int EthBoardList::rem(eOipv4addr_t ipv4)
{
    if(ipv4all == ipv4)
    {
        return clear();
    }

    // look for the same ipv4
    for(int i=0; i<theboards.size(); i++)
    {
        if(ipv4selected == ipv4)
        {   // if selected
            if(true == theboards[i].isSelected())
            {
                theboards.erase(theboards.begin()+i);
            }
        }
        else if(ipv4 == theboards[i].getIPV4())
        {
            theboards.erase(theboards.begin()+i);
        }
    }

    return theboards.size();
}


int EthBoardList::clear()
{
    theboards.clear();
    return theboards.size();
}


int EthBoardList::size()
{
    return theboards.size();
}


int EthBoardList::numberof(eOipv4addr_t ipv4)
{
    if(ipv4all == ipv4)
    {
        return theboards.size();
    }

    int number = 0;
    for(int i=0; i<theboards.size(); i++)
    {
        if(ipv4selected == ipv4)
        {   // all the selected
            if(true == theboards[i].isSelected())
            {
                number++;
            }
        }
        else
        {
            if(ipv4 == theboards[i].getIPV4())
            {
                number++;
            }
        }

    }

    return number;
}

// vector of pointers so that we can modify them
vector<EthBoard *> EthBoardList::get(eOipv4addr_t ipv4)
{
    vector<EthBoard *>  tmp;
    bool getit = false;

    for(int i=0; i<theboards.size(); i++)
    {
        getit = false;

        if(EthBoardList::ipv4all == ipv4)
        {   // all boards
            getit = true;
        }
        else if(EthBoardList::ipv4selected == ipv4)
        {   // all the selected
            if(true == theboards[i].isSelected())
            {
                getit = true;
            }
        }
        else
        {   // equal ipv4
            if(ipv4 == theboards[i].getIPV4())
            {
                getit = true;
            }
        }

        if(getit)
        {
            tmp.push_back(&theboards[i]);
        }
    }

    return tmp;
}

void EthBoardList::select(bool on, eOipv4addr_t ipv4)
{
    // look for the same ipv4
    for(int i=0; i<theboards.size(); i++)
    {
        if(EthBoardList::ipv4all == ipv4)
        {
            theboards[i].setSelected(on);
        }
        else
        {
            if(ipv4 == theboards[i].getIPV4())
            {
                theboards[i].setSelected(on);
            }
        }
    }
}

EthBoard& EthBoardList::operator[](int i)
{
    return theboards[i];
}



// eof


