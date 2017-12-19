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
        bool initted;
        Data() { reset(); }
        void reset()
        {
            themap.clear();
            sequence = 0;
            pp1 = 0;
            initted = false;
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
            yDebug() << "theNVmanager::Impl::Data::alert(): themap.size() =" << themap.size();
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
            yDebug() << "theNVmanager::Impl::Data::alert(): themap.size() =" << themap.size();
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


    Config config;
           
    Data data;

    // we can use: std::map, std::multimap, std::set, std::multiset because therya re ordered and thus quicker. they have teh find method.
    // strategy: find by signature. in such a way every transaction is unique. it must have ....
    // see https://www.fluentcpp.com/2017/01/26/searching-an-stl-container/

    Impl() 
    {   
        data.reset();
    }



    
    bool initialise(const Config &_config);


    yarp::dev::EthResource * ethresource(const eOprotIP_t ipv4);

    bool set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size);
    bool setuntilverified(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, int retries, double waitbeforeverification, double verificationtimeout, int verificationretries);
    
    bool verify(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, const double timeout, const int retries);


    bool ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size, const double timeout);
    bool ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, std::vector<void*> &values, std::vector<std::uint16_t> &sizes , const double timeout);

    bool signatureisvalid(const std::uint32_t signature);
    bool onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature);

    bool wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout);

    bool read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size);
                          
};


yarp::os::Semaphore tbd::theNVmanager::Impl::mtx = 1;


bool tbd::theNVmanager::Impl::initialise(const Config &_config)
{ 
    if(true == data.initted)
    {
        return true;
    }

    config = _config;
    
    data.pp1 = 1;
    data.initted = true;
    return true;           
}


yarp::dev::EthResource * tbd::theNVmanager::Impl::ethresource(const eOprotIP_t ipv4)
{
    yarp::dev::EthResource * res = yarp::dev::TheEthManager::instance()->getEthResource(ipv4);
    if(nullptr == res)
    {
        char ipinfo[20];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        yError("theNVmanager::Impl::ethresource() cannot obtain from TheEthManager a EthResource * for IP = %s", ipinfo);
    }

    return res;
}


bool tbd::theNVmanager::Impl::set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size)
{
    if(false == data.initted)
    {
        yDebug() << "here 1";
        return false;
    }

    if((nullptr == value) || (0 == size))
    {
        yDebug() << "here 2";
        return false;
    }

    yarp::dev::EthResource * res = ethresource(ipv4);

    if(nullptr == res)
    {
        yDebug() << "here 3";
        return false;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::set() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }


    if(false == res->addSetMessage(id32, reinterpret_cast<uint8_t*>(const_cast<void*>(value))))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::set() fails res->addSetMessage() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }


    yDebug() << "OK all: returning true";
    return true;
}


//bool tbd::theNVmanager::Impl::setverify(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, int retries, double waitbeforeverification, double verificationtimeout, int verificationretries)
//{
//    if(false == set(ipv4, id32, value, size))
//    {
//        char nvinfo[128];
//        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//        yError() << "theNVmanager::Impl::setverify() fails in set() phase to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
//        return false;
//    }


//    // now i coudl use ask() but ...

//    std::uint8_t ddd[128] = {0};
//    std::uint16_t sss;


//]



//}

bool tbd::theNVmanager::Impl::setuntilverified(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, int retries, double waitbeforeverification, double verificationtimeout, int verificationretries)
{
    int attempt = 0;
    bool done = false;

    yarp::dev::EthResource * res = ethresource(ipv4);
    if(nullptr == res)
    {
        return false;
    }

    int maxattempts = retries + 1;

    for(attempt=0; (attempt<maxattempts) && (false == done); attempt++)
    {
        if(false == set(ipv4, id32, value, size))
        {
            yWarning() << "theNVmanager::Impl::setuntilverified() had an error while calling set() in BOARD" << res->getName() << "with IP" << res->getIPv4string() << "at attempt #" << attempt+1;
            continue;
        }

        // ok, now i wait some time before asking the value back for verification
        Time::delay(waitbeforeverification);

        if(false == verify(ipv4, id32, value, size, verificationtimeout, verificationretries))
        {
            yWarning() << "theNVmanager::Impl::setuntilverified() had an error while calling verify() in BOARD" << res->getName() << "with IP" << res->getIPv4string() << "at attempt #" << attempt+1;
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
            yWarning() << "theNVmanager::Impl::setuntilverified() has set and verified ID" << nvinfo << "in BOARD" << res->getName() << "with IP" << res->getIPv4string() << "at attempt #" << attempt;
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
        yError() << "FATAL: theNVmanager::Impl::setuntilverified() could not set and verify ID" << nvinfo << "in BOARD" << res->getName() << "with IP" << res->getIPv4string() << " even after " << attempt << "attempts";
    }


    return(done);
}


bool tbd::theNVmanager::Impl::verify(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, const double timeout, const int retries)
{
    bool replied = false;
    bool equal = false;

    std::uint8_t * vv = new std::uint8_t[size];
    std::uint16_t ss = 0;

    for(int i=0; i<(retries+1); i++)
    {
        if(true == ask(ipv4, id32, reinterpret_cast<void*>(vv), ss, timeout))
        {
            replied = true;
            if((size == ss) && (0 == std::memcmp(value, vv, size)))
            {
                equal = true;
            }
            break;
        }
    }

    delete[] vv;

    return equal;
}

bool tbd::theNVmanager::Impl::wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout)
{
    if(false == data.initted)
    {
        return false;
    }

    yarp::dev::EthResource * res = ethresource(ipv4);

    if(nullptr == res)
    {
        return false;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
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

bool tbd::theNVmanager::Impl::read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size)
{
    if(false == data.initted)
    {
        return false;
    }

    if(nullptr == value)
    {
        return false;
    }

    yarp::dev::EthResource * res = ethresource(ipv4);

    if(nullptr == res)
    {
        return false;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }

    // 2. must prepare wait data etc.

    if(false == res->readBufferedValue(id32, reinterpret_cast<uint8_t*>(value), &size))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
        return false;
    }

    return true;
}


bool tbd::theNVmanager::Impl::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size, const double timeout)
{   
    if(false == data.initted)
    {
        return false;
    }

    if(nullptr == value)
    {
        return false;
    }

    yarp::dev::EthResource * res = ethresource(ipv4);

    if(nullptr == res)
    {
        return false;
    }

    if(false == res->isID32supported(id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
        return false;
    }

    // 2. must prepare wait data etc.


    askTransaction* transaction = new askTransaction;
    std::uint32_t assignedsignature = 0;

    data.lock();

    data.insert(transaction, ipv4, id32, assignedsignature);

    data.unlock();

    // 3. must send a request

    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
    yDebug() << "theNVmanager::Impl::ask() will call res->addGetMessageWithSignature() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo << "w/ signature" << assignedsignature;


    if(false == res->addGetMessageWithSignature(id32, assignedsignature))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() fails res->addGetMessage() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;

        // remove the transaction
        data.lock();
        data.remove(assignedsignature);
        data.unlock();
        // and delete it
        delete transaction;

        return false;
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

    // 5. can retrieve value now

    if(false == res->readBufferedValue(id32, reinterpret_cast<uint8_t*>(value), &size))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "theNVmanager::Impl::ask() fails res->readBufferedValue() for BOARD" << res->getName() << "IP" << res->getIPv4string() << "and nv" << nvinfo;
        return false;
    }

    return true;
}


bool tbd::theNVmanager::Impl::ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, std::vector<void*> &values, std::vector<std::uint16_t> &sizes , const double timeout)
{
    if(false == data.initted)
    {
        return false;
    }

    if((0 == id32s.size()) || (id32s.size() != values.size()))
    {
        return false;
    }

    yarp::dev::EthResource * res = ethresource(ipv4);

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
            yError() << "theNVmanager::Impl::ask(ip, vector<eOprotID32_t>, ...) called with an invalid ID in BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;
            return false;
        }
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
        if(false == res->addGetMessageWithSignature(id32s[i], assignedsignature))
        {
            char nvinfo[128];
            eoprot_ID2information(id32s[i], nvinfo, sizeof(nvinfo));
            yError() << "theNVmanager::Impl::ask() fails res->addGetMessage() to BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo;

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
        sizes[i] = size;
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
    yarp::dev::EthResource * res = ethresource(ipv4);
    char nvinfo[128];
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));

    if(ropCode::say == ropcode)
    {
        yDebug() << "theNVmanager::Impl::onarrival(ropCode::say) called for BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo << "w/ signature" << signature;

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
        yDebug() << "theNVmanager::Impl::onarrival(ropCode::sig) called for BOARD" << res->getName() << "IP" << res->getIPv4string() << "for nv" << nvinfo << "w/ signature" << signature;

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
    Config cfg;
    initialise(cfg);

}

         
bool tbd::theNVmanager::initialise(const Config &config)
{

    pImpl->initialise(config);
       
    return true;
}


bool tbd::theNVmanager::set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size)
{
    return pImpl->set(ipv4, id32, value, size);
}

//bool tbd::theNVmanager::setverify(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, int retries = 3, double waitbeforeverification = 0.001, double verificationtimeout = 0.5, int verificationretries = 3)
//{
//    return pImpl->setverify(ipv4, id32, value, size, retries, waitbeforeverification, verificationtimeout, verificationretries);
//}

bool tbd::theNVmanager::verify(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, const double timeout, const int retries)
{
    return pImpl->verify(ipv4, id32, value, size, timeout, retries);
}

bool tbd::theNVmanager::setuntilverified(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, int retries, double waitbeforeverification, double verificationtimeout, int verificationretries)
{
    return pImpl->setuntilverified(ipv4, id32, value, size, retries, waitbeforeverification, verificationtimeout, verificationretries);
}


bool tbd::theNVmanager::ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size , const double timeout)
{    
    return pImpl->ask(ipv4, id32, value, size, timeout);
}


bool tbd::theNVmanager::ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, std::vector<void*> &values, std::vector<std::uint16_t> &sizes , const double timeout)
{
    return pImpl->ask(ipv4, id32s, values, sizes, timeout);
}

bool tbd::theNVmanager::onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature)
{
    return pImpl->onarrival(ropcode, ipv4, id32, signature);
}

bool tbd::theNVmanager::wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout)
{
    return pImpl->wait(ropcode, ipv4, id32, timeout);
}

bool tbd::theNVmanager::read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size)
{
    return pImpl->read(ipv4, id32, value, size);
}





// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





