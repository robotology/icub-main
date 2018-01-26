// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


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


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "theNVmanager.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "ethManager.h"

#include <map>

#include <cstring>

#include "EoProtocol.h"
#include "EoProtocolMN.h"

#include<abstractEthResource.h>



// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------


    
struct eth::theNVmanager::Impl
{

    static yarp::os::Semaphore mtx;
           
    struct askTransaction
    {
        std::uint16_t           expectedrops;
        std::uint16_t           receivedrops;
        eOprotIP_t              _ipv4;
        eOprotID32_t            _id32;
        std::uint32_t           signature;
        yarp::os::Semaphore     semaphore;
        ACE_thread_t            owner;
        double                  timeofwait;
        double                  timeofpost;

        askTransaction() :
            expectedrops(0), receivedrops(0), _ipv4(0), _id32(0), signature(eo_rop_SIGNATUREdummy), owner(0), timeofwait(0), timeofpost(0)
        {
            semaphore.wait();
        }

        ~askTransaction()
        {
            owner = 0;
        }

        void load(eOprotIP_t _ip, eOprotID32_t _id, std::uint32_t _s)
        {
            expectedrops = 1;
            receivedrops = 0;

            _ipv4 = _ip;
            _id32 = _id;
            signature = _s;
            owner = ACE_Thread::self();
        }

        void load(eOprotIP_t _ip, const std::vector<eOprotID32_t> &_ids, std::uint32_t _s)
        {
            expectedrops =_ids.size();
            receivedrops = 0;

            _ipv4 = _ip;
            _id32 = 0xffffff;
            signature = _s;
            owner = ACE_Thread::self();
        }



        bool wait(std::uint16_t &numofrxrops, double timeout = 0.5)
        {
            timeofwait = SystemClock::nowSystem();
            bool r = semaphore.waitWithTimeout(timeout);
            numofrxrops = receivedrops;
            return r;
        }

        bool post()
        {
            receivedrops++;
            if(receivedrops == expectedrops)
            {
                timeofpost = SystemClock::nowSystem();
                semaphore.post();
            }
            return true;
        }
    };


    struct Data
    {
        std::uint32_t sequence;
        // the key is u64: either [0, signature] or [ipv4, id32]
        std::multimap<std::uint64_t, askTransaction*> themap;
        yarp::os::Semaphore locker;
        int pp1;
        Data() { reset(); }
        void reset()
        {
            themap.clear();
            sequence = 0;
            pp1 = 0;
        }

        std::uint32_t uniquesignature()
        {
            std::uint32_t r = sequence++;
            if(sequence >= 0xA0000000)
            {
                sequence = 0;
            }
            return r;
        }

        void insert(askTransaction *transaction, const eOprotIP_t ip, const eOprotID32_t id, std::uint32_t &assignedsignature)
        {
            assignedsignature = uniquesignature();
            transaction->load(ip, id, assignedsignature);
            std::uint64_t key = static_cast<std::uint64_t>(assignedsignature);
            themap.insert(std::make_pair(key, transaction));
        }

        void insert(askTransaction *transaction, const eOprotIP_t ip, const eOprotID32_t id)
        {
            transaction->load(ip, id, eo_rop_SIGNATUREdummy);
            std::uint64_t key = (static_cast<std::uint64_t>(ip) << 32) | static_cast<std::uint64_t>(id);
            themap.insert(std::make_pair(key, transaction));
        }

        void insert(askTransaction *transaction, const eOprotIP_t ip, const std::vector<eOprotID32_t> &ids, std::uint32_t &assignedsignature)
        {
            assignedsignature = uniquesignature();
            transaction->load(ip, ids, assignedsignature);
            std::uint64_t key = static_cast<std::uint64_t>(assignedsignature);
            themap.insert(std::make_pair(key, transaction));
        }

        void remove(const std::uint32_t signature)
        {
            std::uint64_t key = static_cast<std::uint64_t>(signature);
            themap.erase(themap.find(key));
        }

        bool alert(const std::uint32_t signature)
        {
            //yDebug() << "theNVmanager::Impl::Data::alert(): themap.size() =" << themap.size();
            if(true == themap.empty())
            {
                yDebug() << "theNVmanager::Impl::Data::alert(): themap is empty()";

                return false;
            }
            std::uint64_t key = static_cast<std::uint64_t>(signature);
            std::multimap<std::uint64_t, askTransaction*>::iterator it = themap.find(key);

            if(themap.end() == it)
            {
                yDebug() << "theNVmanager::Impl::Data::alert(): signature not found" << signature;
                return false;
            }

            (*it).second->post();

            return true;
        }

        void remove(const eOprotIP_t ip, const eOprotID32_t id)
        {
            std::uint64_t key = (static_cast<std::uint64_t>(ip) << 32) | static_cast<std::uint64_t>(id);
            themap.erase(themap.find(key));
        }

        bool alert(const eOprotIP_t ip, const eOprotID32_t id)
        {
            //yDebug() << "theNVmanager::Impl::Data::alert(): themap.size() =" << themap.size();
            if(true == themap.empty())
            {
                yDebug() << "theNVmanager::Impl::Data::alert(): themap is empty()";

                return false;
            }
            std::uint64_t key = (static_cast<std::uint64_t>(ip) << 32) | static_cast<std::uint64_t>(id);
            std::multimap<std::uint64_t, askTransaction*>::iterator it = themap.find(key);

            if(themap.end() == it)
            {
                yDebug() << "theNVmanager::Impl::Data::alert(): key not found" << key;
                return false;
            }

            (*it).second->post();

            return true;
        }

        void lock()
        {
            locker.wait();
        }

        void unlock()
        {
            locker.post();
        }
    };

    

    // see http://en.cppreference.com/w/cpp/container/map/find


//    Config config;
           
    Data data;

    // we can use: std::map, std::multimap, std::set, std::multiset because therya re ordered and thus quicker. they have teh find method.
    // strategy: find by signature. in such a way every transaction is unique. it must have ....
    // see https://www.fluentcpp.com/2017/01/26/searching-an-stl-container/

    Impl() 
    {   
        data.reset();
    }



    
//    bool initialise(const Config &_config);

    bool supported(const eOprotIP_t ipv4);
    bool supported(const eOprotIP_t ipv4, const eOprotID32_t id32);

    size_t sizeofnv(const eOprotID32_t id32);

    eth::AbstractEthResource * ethresource(const eOprotIP_t ipv4);
    eth::AbstractEthResource * ethresourceID32(const eOprotIP_t ipv4, const eOprotID32_t id32);
    eth::AbstractEthResource * ethresourceID32s(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s);


    eth::HostTransceiver * transceiver(eth::AbstractEthResource *res);
    eth::HostTransceiver * transceiver(const eOprotIP_t ipv4);

    bool validparameters(eth::HostTransceiver *t, const eOprotID32_t id32, void *value);

    bool validparameters(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value);


    bool validparameters(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values);

    bool set(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value);
    bool setcheck(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout);
    

    //size_t maxSizeOfNV(const eOprotIP_t ipv4);

    bool ping(eth::HostTransceiver *t, eoprot_version_t &mnprotversion, const double timeout = 0.5, const unsigned int retries = 20);

    bool ask(eth::HostTransceiver *t, const eOprotID32_t id32, void *value, const double timeout);

    bool ask(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout);

    bool check(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries);

    bool signatureisvalid(const std::uint32_t signature);
    bool onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature);

    bool wait(const ropCode ropcode, eth::HostTransceiver *t, const eOprotID32_t id32, const double timeout);

    bool read(eth::HostTransceiver *t, const eOprotID32_t id32, void *value);

    bool command(eth::HostTransceiver *t, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout = 0.5);

    const eth::AbstractEthResource::Properties & getboardproperties(eth::HostTransceiver *t);
    string getid32string(eOprotID32_t id32);
};


yarp::os::Semaphore eth::theNVmanager::Impl::mtx = 1;


//bool eth::theNVmanager::Impl::initialise(const Config &_config)
//{
//    config = _config;
//    data.pp1 = 1;
//    return true;
//}

const eth::AbstractEthResource::Properties& eth::theNVmanager::Impl::getboardproperties(eth::HostTransceiver *t)
{
    return t->getResource()->getProperties();
}

string eth::theNVmanager::Impl::getid32string(eOprotID32_t id32)
{
    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
    return nvinfo;
}

bool eth::theNVmanager::Impl::supported(const eOprotIP_t ipv4)
{
    eth::HostTransceiver *t = transceiver(ipv4);

    return (nullptr == t) ? false : true;
}

bool eth::theNVmanager::Impl::supported(const eOprotIP_t ipv4, const eOprotID32_t id32)
{
    eth::HostTransceiver *t = transceiver(ipv4);

    if(nullptr == t)
    {
        return false;
    }

    return t->isID32supported(id32);
}

size_t eth::theNVmanager::Impl::sizeofnv(const eOprotID32_t id32)
{
    return eoprot_variable_sizeof_get(eoprot_board_localboard, id32);
}




eth::HostTransceiver * eth::theNVmanager::Impl::transceiver(const eOprotIP_t ipv4)
{
    eth::AbstractEthResource * res = eth::TheEthManager::instance()->getEthResource(ipv4);
    if(nullptr == res)
    {
        char ipinfo[20];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yError("theNVmanager::Impl::transceiverZ() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
    }

    if(nullptr == res)
    {
        return nullptr;
    }

    return res->getTransceiver();
}


//eth::AbstractEthResource * eth::theNVmanager::Impl::ethresourceID32(const eOprotIP_t ipv4, const eOprotID32_t id32)
//{
//    eth::AbstractEthResource * res = eth::TheEthManager::instance()->getEthResource(ipv4);
//    if(nullptr == res)
//    {
//        char ipinfo[20];
//        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
//        yError("theNVmanager::Impl::ethresourceID32() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
//        return nullptr;
//    }

//    eth::HostTransceiver *t = transceiver(res);
//    if(nullptr == tra)
//    {
//        return nullptr;
//    }

//    if(false == t->isID32supported(id32))
//    {
//        char nvinfo[128];
//        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//        yError() << "theNVmanager::Impl::ethresourceID32() called with an invalid ID in BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << nvinfo;
//        return nullptr;
//    }

//    return res;
//}


//eth::AbstractEthResource * eth::theNVmanager::Impl::ethresourceID32s(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s)
//{
//    eth::AbstractEthResource * res = eth::TheEthManager::instance()->getEthResource(ipv4);
//    if(nullptr == res)
//    {
//        char ipinfo[20];
//        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
//        yError("theNVmanager::Impl::ethresourceID32() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
//        return nullptr;
//    }

//    eth::HostTransceiver *t = transceiver(res);
//    if(nullptr == tra)
//    {
//        return nullptr;
//    }

//    for(int i=0; i<id32s.size(); i++)
//    {
//        if(false == t->isID32supported(id32s[i]))
//        {
//            char nvinfo[128];
//            eoprot_ID2information(id32s[i], nvinfo, sizeof(nvinfo));
//            yError() << "theNVmanager::Impl::ethresourceID32s(ip, vector<eOprotID32_t>) called with an invalid ID in BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << nvinfo;
//            return nullptr;
//        }
//    }


//    return res;
//}

bool eth::theNVmanager::Impl::validparameters(eth::HostTransceiver *t, const eOprotID32_t id32, void *value)
{
    if(nullptr == t)
    {
        return false;
    }

    if(false == t->isID32supported(id32))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);;
        yError() << "theNVmanager::Impl::validparameters() called with an invalid ID in BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32);
        return false;
    }

    if(nullptr == value)
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::validparameters(res, ipv4, id32, value) found invalid params in BOARD" << props.boardnameString << "IP" << props.ipv4addrString;
        return false;
    }

    return true;
}

bool eth::theNVmanager::Impl::validparameters(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values)
{
    if(nullptr == t)
    {
        return false;
    }

    for(int i=0; i<id32s.size(); i++)
    {
        if(false == t->isID32supported(id32s[i]))
        {
            const AbstractEthResource::Properties & props = getboardproperties(t);
            yError() << "theNVmanager::Impl::validparameters(vector<>) called with an invalid ID in BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32s[i]);
            return false;
        }
    }

    if((0 == id32s.size()) || (id32s.size() != values.size()))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::validparameters(vector<>) found invalid params in BOARD" << props.boardnameString << "IP" << props.ipv4addrString;
        return false;
    }

    return true;
}

bool eth::theNVmanager::Impl::validparameters(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value)
{
    if(nullptr == t)
    {
        return false;
    }

    if(false == t->isID32supported(id32))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::validparameters() called with an invalid ID in BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32);
        return false;
    }

    if(nullptr == value)
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::validparameters(res, ipv4, id32, value) found invalid params in BOARD" << props.boardnameString << "IP" << props.ipv4addrString;
        return false;
    }

    return true;
}



bool eth::theNVmanager::Impl::setcheck(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout)
{
    int attempt = 0;
    bool done = false;


    if(false == validparameters(t, id32, value))
    {
        return false;
    }


    int maxattempts = retries + 1;

    for(attempt=0; (attempt<maxattempts) && (false == done); attempt++)
    {
        if(false == set(t, id32, value))
        {
            const AbstractEthResource::Properties & props = getboardproperties(t);
            yWarning() << "theNVmanager::Impl::setcheck() had an error while calling set() in BOARD" << props.boardnameString << "with IP" << props.ipv4addrString << "at attempt #" << attempt+1;
            continue;
        }

        // ok, now i wait some time before asking the value back for verification
        SystemClock::delaySystem(waitbeforecheck);

        if(false == check(t, id32, value, timeout, 0))
        {
            const AbstractEthResource::Properties & props = getboardproperties(t);
            yWarning() << "theNVmanager::Impl::setcheck() had an error while calling check() in BOARD" << props.boardnameString << "with IP" << props.ipv4addrString << "at attempt #" << attempt+1;
        }
        else
        {
            done = true;
        }

    }


    if(done)
    {
        if(attempt > 1)
        {
            const AbstractEthResource::Properties & props = getboardproperties(t);
            yWarning() << "theNVmanager::Impl::setcheck() has set and verified ID" << getid32string(id32) << "in BOARD" << props.boardnameString << "with IP" << props.ipv4addrString << "at attempt #" << attempt;
        }
        else
        {
//            if(verbosewhenok)
//            {
//                yDebug() << "EthResource::setRemoteValueUntilVerified has set and verified ID" << nvinfo << "in BOARD" << getProperties().boardnameString << "with IP" << getProperties().ipv4addrString << "at attempt #" << attempt;
//            }
        }
    }
    else
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "FATAL: theNVmanager::Impl::setcheck() could not set and verify ID" << getid32string(id32) << "in BOARD" << props.boardnameString << "with IP" << props.ipv4addrString << " even after " << attempt << "attempts";
    }


    return(done);
}


bool eth::theNVmanager::Impl::check(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries)
{    
    if(false == validparameters(t, id32, value))
    {
        return false;
    }

    bool equal = false;

    std::uint16_t size = sizeofnv(id32);
    std::uint8_t * vv = new std::uint8_t[size];

    for(int i=0; i<(retries+1); i++)
    {
        if(true == ask(t, id32, reinterpret_cast<void*>(vv), timeout))
        {
            if(0 == std::memcmp(value, vv, size))
            {
                equal = true;
            }
        }
    }


    delete[] vv;

    return equal;
}



bool eth::theNVmanager::Impl::wait(const ropCode ropcode, eth::HostTransceiver *t, const eOprotID32_t id32, const double timeout)
{
    const eOprotIP_t ipv4 = t->getIPv4();

    if(false == supported(ipv4, id32))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::wait() fails because the following ipv4-id32 is not supported: ipv4 =" << props.ipv4addrString << "id32 =" << getid32string(id32);
        return false;
    }
    // 2. must prepare wait data etc.

    askTransaction* transaction = new askTransaction;


    data.lock();

    data.insert(transaction, ipv4, id32);

    data.unlock();

    std::uint16_t numberOfReceivedROPs = 0;

    if(false == transaction->wait(numberOfReceivedROPs, timeout))
    {
        // a timeout occurred .... manage it.

        // remove the transaction
        data.lock();
        data.remove(ipv4, id32);
        data.unlock();
        // and delete it
        delete transaction;

        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::wait() had a timeout for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << getid32string(id32);
        return false;
    }

    // remove the transaction
    data.lock();
    data.remove(ipv4, id32);
    data.unlock();
    // and delete it
    delete transaction;

    return true;
}


bool eth::theNVmanager::Impl::read(eth::HostTransceiver *t, const eOprotID32_t id32, void *value)
{
    if(false == validparameters(t, id32, value))
    {
        return false;
    }


    // 2. must prepare wait data etc.
    if(false == t->read(id32, value))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::ask() fails t->read() for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << getid32string(id32);
        return false;
    }

    return true;
}


bool eth::theNVmanager::Impl::command(eth::HostTransceiver *t, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout)
{
    if(false == set(t, id32cmd, cmd))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::command() fails a set() to IP" << props.ipv4addrString << "for nv" << getid32string(id32cmd);
        return false;
    }

    if(false == wait(theNVmanager::ropCode::sig, t, id32rep, timeout))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::command() fails a wait() from IP" << props.ipv4addrString << "for nv" << getid32string(id32rep);
        return false;
    }

    if(false == read(t, id32rep, rep))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::command() fails a read() for IP" << props.ipv4addrString << "and nv" << getid32string(id32rep);
        return false;
    }

    return true;
}



//size_t eth::theNVmanager::Impl::maxSizeOfNV(const eOprotIP_t ipv4)
//{
//    eth::AbstractEthResource * res = ethresource(ipv4);

//    if(nullptr == res)
//    {
//        return 0;
//    }
//    return res->getMaxSizeofROP();
//}

bool eth::theNVmanager::Impl::ping(eth::HostTransceiver *t, eoprot_version_t &mnprotversion, const double timeout, const unsigned int retries)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_status_managementprotocolversion);
    bool replied = false;

    if(false == supported(t->getIPv4(), id32))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::ping() fails because the following ipv4-id32 is not supported: ipv4 =" << props.ipv4addrString << "id32 =" << getid32string(id32);
        return replied;
    }

    for(int i=0; i<(1+retries); i++)
    {
        if(true == ask(t, id32, &mnprotversion, timeout))
        {
            replied = true;
            break;
        }
    }

    return replied;
}



bool eth::theNVmanager::Impl::ask(eth::HostTransceiver *t, const eOprotID32_t id32, void *value, const double timeout)
{
    if(false == validparameters(t, id32, value))
    {
        yError() << "theNVmanager::Impl::ask() called with invalid parameters";
        return false;
    }

    // 1. must prepare wait data etc.

    askTransaction* transaction = new askTransaction;
    std::uint32_t assignedsignature = 0;

    data.lock();

    data.insert(transaction, t->getIPv4(), id32, assignedsignature);

    data.unlock();

    // 2. must send a request

    if(false == t->addROPask(id32, assignedsignature))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::ask() fails res->addROPask() to BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32);

        // remove the transaction
        data.lock();
        data.remove(assignedsignature);
        data.unlock();
        // and delete it
        delete transaction;

        return false;
    }

    // 3. must wait now and manage a possible timeout
    std::uint16_t numberOfReceivedROPs = 0;

    if(false == transaction->wait(numberOfReceivedROPs, timeout))
    {
        // a timeout occurred .... manage it.

        // remove the transaction
        data.lock();
        data.remove(assignedsignature);
        data.unlock();
        // and delete it
        delete transaction;

        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::ask() had a timeout for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << getid32string(id32);
        return false;
    }

    // remove the transaction
    data.lock();
    data.remove(assignedsignature);
    data.unlock();
    // and delete it
    delete transaction;

    // 4. can retrieve value now
    if(false == t->read(id32, value))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::ask() fails res->getLocalValue() for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << getid32string(id32);
        return false;
    }

    return true;
}


//bool eth::theNVmanager::Impl::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout)
//{
//    eth::AbstractEthResource * res = ethresource(ipv4);

//    if(false == validparameters(res, ipv4, id32, value))
//    {
//        return false;
//    }


//    // 2. must prepare wait data etc.


//    askTransaction* transaction = new askTransaction;
//    std::uint32_t assignedsignature = 0;

//    data.lock();

//    data.insert(transaction, ipv4, id32, assignedsignature);

//    data.unlock();

//    // 3. must send a request

//    if(false == res->addROPask(id32, assignedsignature))
//    {
//        char nvinfo[128];
//        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//        yError() << "theNVmanager::Impl::ask() fails res->addGetROP() to BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << nvinfo;

//        // remove the transaction
//        data.lock();
//        data.remove(assignedsignature);
//        data.unlock();
//        // and delete it
//        delete transaction;

//        return false;
//    }

//    // 4. must wait now and manage a possible timeout
//    std::uint16_t numberOfReceivedROPs = 0;

//    if(false == transaction->wait(numberOfReceivedROPs, timeout))
//    {
//        // a timeout occurred .... manage it.

//        // remove the transaction
//        data.lock();
//        data.remove(assignedsignature);
//        data.unlock();
//        // and delete it
//        delete transaction;

//        char nvinfo[128];
//        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//        yError() << "theNVmanager::Impl::ask() had a timeout for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << nvinfo;
//        return false;
//    }

//    // remove the transaction
//    data.lock();
//    data.remove(assignedsignature);
//    data.unlock();
//    // and delete it
//    delete transaction;

//    // 5. can retrieve value now
//    uint16_t size = 0;
//    if(false == res->readBufferedValue(id32, reinterpret_cast<uint8_t*>(value), &size))
//    {
//        char nvinfo[128];
//        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//        yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << nvinfo;
//        return false;
//    }

//    return true;
//}


bool eth::theNVmanager::Impl::ask(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout)
{
    const eOprotIP_t ipv4 = t->getIPv4();

    if(false == validparameters(t, id32s, values))
    {
        return false;
    }


    // 2. must prepare wait data etc.


    askTransaction* transaction = new askTransaction;
    std::uint32_t assignedsignature = 0;

    data.lock();

    data.insert(transaction, ipv4, id32s, assignedsignature);

    data.unlock();

    // 3. must send a request to all the id32s

    for(int i=0; i<id32s.size(); i++)
    {
        if(false == t->addROPask(id32s[i], assignedsignature))
        {
            const AbstractEthResource::Properties & props = getboardproperties(t);
            yError() << "theNVmanager::Impl::ask() fails t->addROPask() to BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32s[i]);

            // remove the transaction
            data.lock();
            data.remove(assignedsignature);
            data.unlock();
            // and delete it
            delete transaction;

            return false;
        }
    }

    // 4. must wait now and manage a possible timeout
    std::uint16_t numberOfReceivedROPs = 0;

    if(false == transaction->wait(numberOfReceivedROPs))
    {
        // a timeout occurred .... manage it.

        // remove the transaction
        data.lock();
        data.remove(assignedsignature);
        data.unlock();
        // and delete it
        delete transaction;

        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::ask() had a timeout for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "w/ multiple NVs. Received only" << numberOfReceivedROPs << "out of" << id32s.size();
        return false;
    }

    //yDebug() << "received" << numberOfReceivedROPs << "numberOfReceivedROPs";

    // remove the transaction
    data.lock();
    data.remove(assignedsignature);
    data.unlock();
    // and delete it
    delete transaction;

    // 5. can retrieve values now
    for(int i=0; i<id32s.size(); i++)
    {
        if(false == t->read(id32s[i], values[i]))
        {
            const AbstractEthResource::Properties & props = getboardproperties(t);
            yError() << "theNVmanager::Impl::ask() fails res->getLocalValue() for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "and nv" << getid32string(id32s[i]);
            return false;
        }
    }

    return true;
}


bool eth::theNVmanager::Impl::set(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value)
{

//    if(false == validparameters(t, id32, value))
//    {
//        return false;
//    }

    if(false == t->addROPset(id32, value))
    {
        const AbstractEthResource::Properties & props = getboardproperties(t);
        yError() << "theNVmanager::Impl::set() fails t->addSetROP() to BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32);
        return false;
    }

    return true;
}


bool eth::theNVmanager::Impl::signatureisvalid(const std::uint32_t signature)
{
    if((eo_rop_SIGNATUREdummy == signature) || (signature >= 0xaa000000))
    {
        return false;
    }
    return true;
}


bool eth::theNVmanager::Impl::onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature)
{
    static double tprev = SystemClock::nowSystem();
    double tcurr = SystemClock::nowSystem();

    double delta = tcurr - tprev;

    tprev = tcurr;

    eth::HostTransceiver *t = transceiver(ipv4);

    if(nullptr == t)
    {
        char ipinfo[32];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yDebug() << "theNVmanager::Impl::onarrival() called for unsupported IP" << ipinfo << "for nv" << getid32string(id32) << "w/ signature" << signature;
        //#warning meglio CONTROLLARE CHE NON CI SIA NULLA QUI ...
        return false;
    }

    if(ropCode::say == ropcode)
    {
        //const AbstractEthResource::Properties & props = getboardproperties(t);
        //yDebug() << "theNVmanager::Impl::onarrival(ropCode::say) called for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32) << "w/ signature" << signature << "after sec" << delta;

        // we manage a reply. we decide that we search only by signature.

        if(false == signatureisvalid(signature))
        {
            yDebug() << "theNVmanager::Impl::onarrival() has found a false signature";
            return false;
        }

        // 1. alert the thread which is waiting
        data.lock();

        data.alert(signature);

        data.unlock();

    }
    else if(ropCode::sig == ropcode)
    {
        //const AbstractEthResource::Properties & props = getboardproperties(t);
        //yDebug() << "theNVmanager::Impl::onarrival(ropCode::sig) called for BOARD" << props.boardnameString << "IP" << props.ipv4addrString << "for nv" << getid32string(id32) << "w/ signature" << signature << "after sec" << delta;

        // we manage an expected sig<>. we search by [ipv4-id32].


        // 1. alert the thread which is waiting
        data.lock();

        data.alert(ipv4, id32);

        data.unlock();

    }

    return true;
}


// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------

eth::theNVmanager& eth::theNVmanager::getInstance()
{
    static theNVmanager* p = nullptr;

    eth::theNVmanager::Impl::mtx.wait();
    if(nullptr == p)
    {
        p = new theNVmanager();
    }
    eth::theNVmanager::Impl::mtx.post();

    return *p;
}


eth::theNVmanager::theNVmanager()
: pImpl(new Impl)
{
    //Config cfg;
    //initialise(cfg);

}

         
//bool eth::theNVmanager::initialise(const Config &config)
//{

//    pImpl->initialise(config);
       
//    return true;
//}

//
//size_t eth::theNVmanager::sizeOfNV(const eOprotIP_t ipv4, const eOprotID32_t id32)
//{
//    eth::AbstractEthResource * res = pImpl->ethresourceID32(ipv4, id32);
//
//    if(nullptr == res)
//    {
//        return 0;
//    }
//
//    return eoprot_variable_sizeof_get(eoprot_board_localboard, id32);
//}
//
//size_t eth::theNVmanager::maxSizeOfNV(const eOprotIP_t ipv4)
//{
//    return pImpl->maxSizeOfNV(ipv4);
//}
//


bool eth::theNVmanager::supported(const eOprotIP_t ipv4)
{
    return pImpl->supported(ipv4);
}

bool eth::theNVmanager::supported(const eOprotIP_t ipv4, const eOprotID32_t id32)
{
    return pImpl->supported(ipv4, id32);
}

size_t eth::theNVmanager::sizeOfNV(const eOprotID32_t id32)
{
    return pImpl->sizeofnv(id32);
}

bool eth::theNVmanager::ping(const eOprotIP_t ipv4, eoprot_version_t &mnprotversion, const double timeout, const unsigned int retries)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->ping(t, mnprotversion, timeout, retries);
}

bool eth::theNVmanager::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->ask(t, id32, value, timeout);
}

bool eth::theNVmanager::ask(eth::HostTransceiver *t, const eOprotID32_t id32, void *value, const double timeout)
{
    return pImpl->ask(t, id32, value, timeout);
}



bool eth::theNVmanager::ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->ask(t, id32s, values, timeout);
}

bool eth::theNVmanager::ask(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout)
{
    return pImpl->ask(t, id32s, values, timeout);
}

bool eth::theNVmanager::set(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value)
{
    return pImpl->set(t, id32, value);
}

bool eth::theNVmanager::set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->set(t, id32, value);
}


bool eth::theNVmanager::check(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries)
{
    return pImpl->check(t, id32, value, timeout, retries);
}

bool eth::theNVmanager::check(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->check(t, id32, value, timeout, retries);
}


bool eth::theNVmanager::setcheck(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->setcheck(t, id32, value, retries, waitbeforecheck, timeout);
}

bool eth::theNVmanager::setcheck(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout)
{
    return pImpl->setcheck(t, id32, value, retries, waitbeforecheck, timeout);
}

bool eth::theNVmanager::onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature)
{
    return pImpl->onarrival(ropcode, ipv4, id32, signature);
}

//bool eth::theNVmanager::wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout)
//{
//    return pImpl->wait(ropcode, ipv4, id32, timeout);
//}

//bool eth::theNVmanager::read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value)
//{
//    return pImpl->read(ipv4, id32, value);
//}

bool eth::theNVmanager::command(const eOprotIP_t ipv4, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout)
{
    eth::HostTransceiver *t = pImpl->transceiver(ipv4);
    return pImpl->command(t, id32cmd, cmd, id32rep, rep, timeout);
}





// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





