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



// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------


    
struct tbd::theNVmanager::Impl
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
            timeofwait = yarp::os::Time::now();
            bool r = semaphore.waitWithTimeout(timeout);
            numofrxrops = receivedrops;
            return r;
        }

        bool post()
        {
            receivedrops++;
            if(receivedrops == expectedrops)
            {
                timeofpost = yarp::os::Time::now();
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

    yarp::dev::AbstractEthResource * ethresource(const eOprotIP_t ipv4);
    yarp::dev::AbstractEthResource * ethresourceID32(const eOprotIP_t ipv4, const eOprotID32_t id32);
    yarp::dev::AbstractEthResource * ethresourceID32s(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s);

    bool validparameters(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const eOprotID32_t id32, void *value);

    bool validparameters(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value);


    bool validparameters(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values);

    bool set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value);
    bool setcheck(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout);
    

    //size_t maxSizeOfNV(const eOprotIP_t ipv4);

    bool ping(const eOprotIP_t ipv4, eoprot_version_t &mnprotversion, const double timeout = 0.5, const unsigned int retries = 20);

    bool ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout);
    bool ask(yarp::dev::AbstractEthResource *res, const eOprotID32_t id32, void *value, const double timeout);

    bool askboard(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout);

    bool ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout);

    bool check(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries);

    bool signatureisvalid(const std::uint32_t signature);
    bool onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature);

    bool wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout);

    bool read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value);

    bool command(const eOprotIP_t ipv4, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout = 0.5);
                          
};


yarp::os::Semaphore tbd::theNVmanager::Impl::mtx = 1;


//bool tbd::theNVmanager::Impl::initialise(const Config &_config)
//{
//    config = _config;
//    data.pp1 = 1;
//    return true;
//}


bool tbd::theNVmanager::Impl::supported(const eOprotIP_t ipv4)
{
    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

    return (nullptr == res) ? false : true;
}

bool tbd::theNVmanager::Impl::supported(const eOprotIP_t ipv4, const eOprotID32_t id32)
{
    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

    if(nullptr == res)
    {
        return false;
    }

    return res->isID32supported(id32);
}

size_t tbd::theNVmanager::Impl::sizeofnv(const eOprotID32_t id32)
{
    return eoprot_variable_sizeof_get(eoprot_board_localboard, id32);
}


yarp::dev::AbstractEthResource * tbd::theNVmanager::Impl::ethresource(const eOprotIP_t ipv4)
{
    yarp::dev::AbstractEthResource * res = yarp::dev::TheEthManager::instance()->getEthResource(ipv4);
    if(nullptr == res)
    {
        char ipinfo[20];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yError("theNVmanager::Impl::ethresource() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
    }

    return res;
}


yarp::dev::AbstractEthResource * tbd::theNVmanager::Impl::ethresourceID32(const eOprotIP_t ipv4, const eOprotID32_t id32)
{
    yarp::dev::AbstractEthResource * res = yarp::dev::TheEthManager::instance()->getEthResource(ipv4);
    if(nullptr == res)
    {
        char ipinfo[20];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yError("theNVmanager::Impl::ethresourceID32() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
        return nullptr;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ethresourceID32() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return nullptr;
    }

    return res;
}


yarp::dev::AbstractEthResource * tbd::theNVmanager::Impl::ethresourceID32s(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s)
{
    yarp::dev::AbstractEthResource * res = yarp::dev::TheEthManager::instance()->getEthResource(ipv4);
    if(nullptr == res)
    {
        char ipinfo[20];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yError("theNVmanager::Impl::ethresourceID32() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
        return nullptr;
    }

    for(int i=0; i<id32s.size(); i++)
    {
        if(false == res->isID32supported(id32s[i]))
        {
            char nvinfo[128];
            eoprot_ID2information(id32s[i], nvinfo, sizeof(nvinfo));
            yError() << "theNVmanager::Impl::ethresourceID32s(ip, vector<eOprotID32_t>) called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
            return nullptr;
        }
    }


    return res;
}

bool tbd::theNVmanager::Impl::validparameters(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const eOprotID32_t id32, void* value)
{
    if(nullptr == res)
    {
        return false;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::validparameters() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }

    if(nullptr == value)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::validparameters(res, ipv4, id32, value) found invalid params in BOARD" << res->getName() << "IP" << res->getIPv4string();
        return false;
    }

    return true;
}

bool tbd::theNVmanager::Impl::validparameters(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values)
{
    if(nullptr == res)
    {
        return false;
    }

    for(int i=0; i<id32s.size(); i++)
    {
        if(false == res->isID32supported(id32s[i]))
        {
            char nvinfo[128];
            eoprot_ID2information(id32s[i], nvinfo, sizeof(nvinfo));
            yError() << "theNVmanager::Impl::theNVmanager::Impl::validparameters(res, ipv4, id32s, values, sizes) called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
            return false;
        }
    }

    if((0 == id32s.size()) || (id32s.size() != values.size()))
    {
        yError() << "theNVmanager::Impl::validparameters(res, ipv4, id32s, values) found invalid params in BOARD" << res->getName() << "IP" << res->getIPv4string();
        return false;
    }

    return true;
}

bool tbd::theNVmanager::Impl::validparameters(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value)
{
    if(nullptr == res)
    {
        return false;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::validparameters() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }

    if((nullptr == value))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::validparameters(res, ipv4, id32, value) found invalid params in BOARD" << res->getName() << "IP" << res->getIPv4string();
        return false;
    }

    return true;
}



bool tbd::theNVmanager::Impl::setcheck(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout)
{
    int attempt = 0;
    bool done = false;

    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

    if(false == validparameters(res, ipv4, id32, value))
    {
        return false;
    }


    int maxattempts = retries + 1;

    for(attempt=0; (attempt<maxattempts) && (false == done); attempt++)
    {
        if(false == set(ipv4, id32, value))
        {
            yWarning() << "theNVmanager::Impl::setcheck() had an error while calling set() in BOARD" << res->getName() << "with IP" << res->getIPv4string() << "at attempt #" << attempt+1;
            continue;
        }

        // ok, now i wait some time before asking the value back for verification
        Time::delay(waitbeforecheck);

        if(false == check(ipv4, id32, value, timeout, 0))
        {
            yWarning() << "theNVmanager::Impl::setcheck() had an error while calling check() in BOARD" << res->getName() << "with IP" << res->getIPv4string() << "at attempt #" << attempt+1;
        }
        else
        {
            done = true;
        }

    }

    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));

    if(done)
    {
        if(attempt > 1)
        {
            yWarning() << "theNVmanager::Impl::setcheck() has set and verified ID" << nvinfo << "in BOARD" << res->getName() << "with IP" << res->getIPv4string() << "at attempt #" << attempt;
        }
        else
        {
//            if(verbosewhenok)
//            {
//                yDebug() << "EthResource::setRemoteValueUntilVerified has set and verified ID" << nvinfo << "in BOARD" << getName() << "with IP" << getIPv4string() << "at attempt #" << attempt;
//            }
        }
    }
    else
    {
        yError() << "FATAL: theNVmanager::Impl::setcheck() could not set and verify ID" << nvinfo << "in BOARD" << res->getName() << "with IP" << res->getIPv4string() << " even after " << attempt << "attempts";
    }


    return(done);
}


bool tbd::theNVmanager::Impl::check(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries)
{
    if(false == supported(ipv4, id32))
    {
        return false;
    }

    if((nullptr == value))
    {
        return false;
    }

    bool equal = false;

    std::uint16_t size = sizeofnv(id32);
    std::uint8_t * vv = new std::uint8_t[size];

    for(int i=0; i<(retries+1); i++)
    {
        if(true == ask(ipv4, id32, reinterpret_cast<void*>(vv), timeout))
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



bool tbd::theNVmanager::Impl::wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout)
{
    yarp::dev::AbstractEthResource * res = ethresourceID32(ipv4, id32);

    if(nullptr == res)
    {
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

        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::wait() had a timeout for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
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

bool tbd::theNVmanager::Impl::read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value)
{
    if(nullptr == value)
    {
        return false;
    }

    yarp::dev::AbstractEthResource * res = ethresourceID32(ipv4, id32);

    if(nullptr == res)
    {
        return false;
    }


    // 2. must prepare wait data etc.
    uint16_t ss = 0;
    if(false == res->readBufferedValue(id32, reinterpret_cast<uint8_t*>(value), &ss))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
        return false;
    }

    return true;
}


bool tbd::theNVmanager::Impl::command(const eOprotIP_t ipv4, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout)
{
    if(false == set(ipv4, id32cmd, cmd))
    {
        char ipinfo[32];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        char nvinfo[128];
        eoprot_ID2information(id32cmd, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::command() fails a set() to IP" << ipinfo << "for nv" << nvinfo;
        return false;
    }

    if(false == wait(theNVmanager::ropCode::sig, ipv4, id32rep, timeout))
    {
        char ipinfo[32];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        char nvinfo[128];
        eoprot_ID2information(id32rep, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::command() fails a wait() from IP" << ipinfo << "for nv" << nvinfo;
        return false;
    }

    if(false == read(ipv4, id32rep, rep))
    {
        char ipinfo[32];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        char nvinfo[128];
        eoprot_ID2information(id32rep, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::command() fails a read() for IP" << ipinfo << "and nv" << nvinfo;
        return false;
    }

    return true;
}



//size_t tbd::theNVmanager::Impl::maxSizeOfNV(const eOprotIP_t ipv4)
//{
//    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

//    if(nullptr == res)
//    {
//        return 0;
//    }
//    return res->getMaxSizeofROP();
//}

bool tbd::theNVmanager::Impl::ping(const eOprotIP_t ipv4, eoprot_version_t &mnprotversion, const double timeout, const unsigned int retries)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_status_managementprotocolversion);
    bool replied = false;

    if(false == supported(ipv4, id32))
    {
        char ipinfo[32];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ping() fails because the following ipv4-id32 is not supported: ipv4 =" << ipinfo << "id32 =" << nvinfo;
        return replied;
    }

    for(int i=0; i<(1+retries); i++)
    {
        if(true == ask(ipv4, id32, &mnprotversion, timeout))
        {
            replied = true;
            break;
        }
    }

    return replied;
}



bool tbd::theNVmanager::Impl::ask(yarp::dev::AbstractEthResource *res, const eOprotID32_t id32, void *value, const double timeout)
{
    if(nullptr == res)
    {
        yError() << "theNVmanager::Impl::ask() called with a nullptr EthResource*";
        return false;
    }

    const eOprotIP_t ipv4 = res->getIPv4remoteAddress();

    if(false == validparameters(res, ipv4, id32, value))
    {
        return false;
    }

    return askboard(res, ipv4, id32, value, timeout);

}

bool tbd::theNVmanager::Impl::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout)
{
    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

    if(false == validparameters(res, ipv4, id32, value))
    {
        return false;
    }

    return askboard(res, ipv4, id32, value, timeout);
}



bool tbd::theNVmanager::Impl::askboard(yarp::dev::AbstractEthResource *res, const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout)
{
    // 1. must prepare wait data etc.

    askTransaction* transaction = new askTransaction;
    std::uint32_t assignedsignature = 0;

    data.lock();

    data.insert(transaction, ipv4, id32, assignedsignature);

    data.unlock();

    // 2. must send a request

    if(false == res->addGetMessage(id32, assignedsignature))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() fails res->addGetROP() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;

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

        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() had a timeout for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
        return false;
    }

    // remove the transaction
    data.lock();
    data.remove(assignedsignature);
    data.unlock();
    // and delete it
    delete transaction;

    // 4. can retrieve value now
    uint16_t size = 0;
    if(false == res->readBufferedValue(id32, reinterpret_cast<uint8_t*>(value), &size))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
        return false;
    }

    return true;
}


//bool tbd::theNVmanager::Impl::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout)
//{
//    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

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

//    if(false == res->addGetROPwithSignature(id32, assignedsignature))
//    {
//        char nvinfo[128];
//        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//        yError() << "theNVmanager::Impl::ask() fails res->addGetROP() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;

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
//        yError() << "theNVmanager::Impl::ask() had a timeout for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
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
//        yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
//        return false;
//    }

//    return true;
//}


bool tbd::theNVmanager::Impl::ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout)
{

    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

    if(false == validparameters(res, ipv4, id32s, values))
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
        if(false == res->addGetMessage(id32s[i], assignedsignature))
        {
            char nvinfo[128];
            eoprot_ID2information(id32s[i], nvinfo, sizeof(nvinfo));
            yError() << "theNVmanager::Impl::ask() fails res->addGetROP() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;

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

        yError() << "theNVmanager::Impl::ask() had a timeout for BOARD" << res->getName() << "IP" << res->getIPv4string() << "w/ multiple NVs. Received only" << numberOfReceivedROPs << "out of" << id32s.size();
        return false;
    }

    // remove the transaction
    data.lock();
    data.remove(assignedsignature);
    data.unlock();
    // and delete it
    delete transaction;

    // 5. can retrieve values now
    for(int i=0; i<id32s.size(); i++)
    {
        uint16_t size = 0;
        if(false == res->readBufferedValue(id32s[i], reinterpret_cast<uint8_t*>(values[i]), &size))
        {
            char nvinfo[128];
            eoprot_ID2information(id32s[i], nvinfo, sizeof(nvinfo));
            yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
            return false;
        }
    }

    return true;
}

bool tbd::theNVmanager::Impl::set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value)
{
    yarp::dev::AbstractEthResource * res = ethresource(ipv4);

    if(false == validparameters(res, ipv4, id32, value))
    {
        return false;
    }


    if(false == res->addSetMessage(id32, reinterpret_cast<uint8_t*>(const_cast<void*>(value))))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::set() fails res->addSetMessage() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }

    return true;
}


bool tbd::theNVmanager::Impl::signatureisvalid(const std::uint32_t signature)
{
    if((eo_rop_SIGNATUREdummy == signature) || (signature >= 0xaa000000))
    {
        return false;
    }
    return true;
}


bool tbd::theNVmanager::Impl::onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature)
{
    yarp::dev::AbstractEthResource * res = ethresource(ipv4);
    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));

    static double tprev = yarp::os::Time::now();
    double tcurr = yarp::os::Time::now();

    double delta = tcurr - tprev;

    tprev = tcurr;


    if(nullptr == res)
    {
        char ipinfo[32];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yDebug() << "theNVmanager::Impl::onarrival() called for unsupported IP" << ipinfo << "for nv" << nvinfo << "w/ signature" << signature;
        //#warning meglio CONTROLLARE CHE NON CI SIA NULLA QUI ...
        return false;
    }

    if(ropCode::say == ropcode)
    {
        //yDebug() << "theNVmanager::Impl::onarrival(ropCode::say) called for BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo << "w/ signature" << signature << "after sec" << delta;

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
        //yDebug() << "theNVmanager::Impl::onarrival(ropCode::sig) called for BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo << "w/ signature" << signature << "after sec" << delta;

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

tbd::theNVmanager& tbd::theNVmanager::getInstance()
{
    static theNVmanager* p = nullptr;

    tbd::theNVmanager::Impl::mtx.wait();
    if(nullptr == p)
    {
        p = new theNVmanager();
    }
    tbd::theNVmanager::Impl::mtx.post();

    return *p;
}


tbd::theNVmanager::theNVmanager()
: pImpl(new Impl)
{
    //Config cfg;
    //initialise(cfg);

}

         
//bool tbd::theNVmanager::initialise(const Config &config)
//{

//    pImpl->initialise(config);
       
//    return true;
//}

//
//size_t tbd::theNVmanager::sizeOfNV(const eOprotIP_t ipv4, const eOprotID32_t id32)
//{
//    yarp::dev::AbstractEthResource * res = pImpl->ethresourceID32(ipv4, id32);
//
//    if(nullptr == res)
//    {
//        return 0;
//    }
//
//    return eoprot_variable_sizeof_get(eoprot_board_localboard, id32);
//}
//
//size_t tbd::theNVmanager::maxSizeOfNV(const eOprotIP_t ipv4)
//{
//    return pImpl->maxSizeOfNV(ipv4);
//}
//


bool tbd::theNVmanager::supported(const eOprotIP_t ipv4)
{
    return pImpl->supported(ipv4);
}

bool tbd::theNVmanager::supported(const eOprotIP_t ipv4, const eOprotID32_t id32)
{
    return pImpl->supported(ipv4, id32);
}

size_t tbd::theNVmanager::sizeOfNV(const eOprotID32_t id32)
{
    return pImpl->sizeofnv(id32);
}

bool tbd::theNVmanager::ping(const eOprotIP_t ipv4, eoprot_version_t &mnprotversion, const double timeout, const unsigned int retries)
{
    return pImpl->ping(ipv4, mnprotversion, timeout, retries);
}

bool tbd::theNVmanager::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout)
{
    return pImpl->ask(ipv4, id32, value, timeout);
}

bool tbd::theNVmanager::ask(yarp::dev::AbstractEthResource *res, const eOprotID32_t id32, void *value, const double timeout)
{
    return pImpl->ask(res, id32, value, timeout);
}



bool tbd::theNVmanager::ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout)
{
    return pImpl->ask(ipv4, id32s, values, timeout);
}

bool tbd::theNVmanager::set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value)
{
    return pImpl->set(ipv4, id32, value);
}

bool tbd::theNVmanager::check(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const double timeout, const unsigned int retries)
{
    return pImpl->check(ipv4, id32, value, timeout, retries);
}


bool tbd::theNVmanager::setcheck(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const unsigned int retries, double waitbeforecheck, double timeout)
{
    return pImpl->setcheck(ipv4, id32, value, retries, waitbeforecheck, timeout);
}

bool tbd::theNVmanager::onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature)
{
    return pImpl->onarrival(ropcode, ipv4, id32, signature);
}

//bool tbd::theNVmanager::wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout)
//{
//    return pImpl->wait(ropcode, ipv4, id32, timeout);
//}

//bool tbd::theNVmanager::read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value)
//{
//    return pImpl->read(ipv4, id32, value);
//}

bool tbd::theNVmanager::command(const eOprotIP_t ipv4, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout)
{
    return pImpl->command(ipv4, id32cmd, cmd, id32rep, rep, timeout);
}





// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





