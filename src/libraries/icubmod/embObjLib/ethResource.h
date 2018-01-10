// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup TheEthManager TheEthManager
 *
*/
/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

//
// $Id: TheEthManager.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __ethResource__
#define __ethResource__

// System includes
#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

// Yarp includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include "statExt.h"

// embobjlib includes
#include "hostTransceiver.hpp"
#include "FeatureInterface.h"
#include "IethResource.h"

// embobj includes
#include "EoProtocol.h"


// ACE stuff
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/SOCK_Dgram_Bcast.h>



class can_string_eth;

namespace yarp{
    namespace dev{
        class AbstractEthResource;
        class EthResource;
        class TheEthManager;
        class FakeEthResource;
    }
}

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev;
using namespace std;



class EthMonitorPresence
{
public:

    struct Config
    {
        bool    enabled;
        double  timeout;                // in seconds
        double  periodmissingreport;    // in seconds
        std::string name;
        Config() : enabled(true), timeout(0.020), periodmissingreport(30.0), name("generic") {}
        Config(bool en, double t, double p, const std::string& na) : enabled(en), timeout(t), periodmissingreport(p), name(na) {}
    };

    EthMonitorPresence();
    ~EthMonitorPresence();

    void config(const Config &cfg);
    void enable(bool en);
    bool isenabled();

    void tick();
    bool check();   // returns true if ok, false if missing

private:

    Config configuration;

    double lastTickTime;
    double lastMissingReportTime;
    double lastHeardTime;
    bool reportedMissing;
};




class yarp::dev::AbstractEthResource
{
public:

    //AbstractEthResource();
   // virtual ~AbstractEthResource() = 0;


    virtual bool            open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal) = 0;
    virtual bool            close() = 0;
    virtual bool            isEPsupported(eOprot_endpoint_t ep) = 0;

    virtual ACE_INET_Addr   getRemoteAddress(void) = 0;

    virtual eOipv4addr_t    getIPv4remoteAddress(void) = 0;

    virtual const char *    getName(void) = 0;
    virtual const char *    getIPv4string(void) = 0;

    virtual eObrd_ethtype_t getBoardType(void) = 0;
    virtual const char *    getBoardTypeString(void) = 0;

    virtual void getBoardInfo(eOdate_t &date, eOversion_t &version) = 0;

    // the function returns true if the packet can be transmitted. 
    // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
    virtual bool            getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops) = 0;

    virtual bool            canProcessRXpacket(uint64_t *data, uint16_t size) = 0;

    virtual void            processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics = true) = 0;


    virtual bool verifyRemoteValue(eOprotID32_t id32, void *value, uint16_t size, double timeout = 0.100, int retries = 10) = 0;

    virtual bool getRemoteValue(eOprotID32_t id32, void *value, uint16_t &size, double timeout = 0.100, int retries = 10) = 0;


    // very important note: it works only if there is an handler for the id32 and it manages the unlock of the mutex
    virtual bool setRemoteValueUntilVerified(eOprotID32_t id32, void *value, uint16_t size, int retries = 10, double waitbeforeverification = 0.001, double verificationtimeout = 0.050, int verificationretries = 2) = 0;


    virtual bool verifyEPprotocol(eOprot_endpoint_t ep) = 0;

    virtual bool aNetQueryReplyHasArrived(eOprotID32_t id32, uint32_t signature) = 0;


    virtual bool printRXstatistics(void) = 0;
    virtual bool CANPrintHandler(eOmn_info_basic_t* infobasic) = 0;


    virtual bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500) = 0;

    virtual bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500) = 0;

    virtual bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500) = 0;

    virtual bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500) = 0;

    virtual bool Tick() = 0;
    virtual bool Check() = 0;
     
    virtual bool readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size) = 0;
    virtual bool addSetMessage(eOprotID32_t id32, uint8_t* data) = 0;
    virtual bool addGetMessage(eOprotID32_t id32) = 0;
    virtual uint16_t getNVnumber(eOnvEP8_t ep) = 0;
    virtual EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv) = 0;
    virtual uint32_t translate_NVid2index(eOprotID32_t id32) = 0;
    virtual bool addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data) = 0;
    virtual bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size) = 0;
    virtual bool isFake() = 0;

};

// -- class EthResource
// -- it is used to manage udp communication towards a given board ... there is no need to derive it from DeviceDriver, is there?

class yarp::dev::EthResource:  public AbstractEthResource, public DeviceDriver,
                               public HostTransceiver
{
public:

    // the size of the boardname must belong to EthResources because it is this class which extracts it from the xml file.
    // all other places where this name is stored have copied it from here using EthResource::getName(void).
    // an example is TheEthManager::ethBoards which makes it available with TheEthManager::getName(eOipv4addr_t ipv4)

    enum { boardNameSize = 32 };

    // this is the maximum size of rx and tx packets managed by the ethresource. however, the HostTranceveiver can reduce these values.
    enum { maxRXpacketsize = 1496, maxTXpacketsize = 1496 };

public:

    EthResource();
    ~EthResource();


    bool            open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal);
    bool            close();
    bool            isEPsupported(eOprot_endpoint_t ep);

    ACE_INET_Addr   getRemoteAddress(void);

    eOipv4addr_t    getIPv4remoteAddress(void);

    const char *    getName(void);
    const char *    getIPv4string(void);

    eObrd_ethtype_t getBoardType(void);
    const char *    getBoardTypeString(void);

    void getBoardInfo(eOdate_t &date, eOversion_t &version);

    // the function returns true if the packet can be transmitted. 
    // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
    bool            getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops);

    bool            canProcessRXpacket(uint64_t *data, uint16_t size);

    void            processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics = true);


    bool getRemoteValue(const eOprotID32_t id32, void *value, const double timeout = 0.100, const unsigned int retries = 0);

    bool setRemoteValue(const eOprotID32_t id32, void *value);

    bool setcheckRemoteValue(const eOprotID32_t id32, void *value, const unsigned int retries = 10, const double waitbeforecheck = 0.001, const double timeout = 0.050);


    bool verifyEPprotocol(eOprot_endpoint_t ep);

    bool CANPrintHandler(eOmn_info_basic_t* infobasic);


    bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500);

    bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500);

    bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500);

    bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500);

    bool Tick();
    bool Check();
    bool readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size);
    bool addSetMessage(eOprotID32_t id32, uint8_t* data);
    bool addGetMessage(eOprotID32_t id32);
    uint16_t getNVnumber(eOnvEP8_t ep);
    uint32_t translate_NVid2index(eOprotID32_t id32);
    bool addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data);
    bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size);
    EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv);
    bool isFake();


private:

    eOipv4addr_t      ipv4addr;
    char              ipv4addrstring[20];
    char              boardName[32];
    char              boardTypeString[32];
    eObrd_ethtype_t   ethboardtype;
    ACE_INET_Addr     remote_dev;             //!< IP address of the EMS this class is talking to.
    double            lastRecvMsgTimestamp;   //! stores the system time of the last received message, gettable with getLastRecvMsgTimestamp()
    bool			  isInRunningMode;        //!< say if goToRun cmd has been sent to EMS

    yarp::os::Semaphore*  objLock;

    
    
    bool                verifiedEPprotocol[eoprot_endpoints_numberof];
    bool                verifiedBoardPresence;
    bool                askedBoardVersion;
    eOdate_t            boardDate;
    eOversion_t         boardVersion;
    eoprot_version_t    boardMNprotocolversion;
    eObrd_ethtype_t     detectedBoardType;

    bool                verifiedBoardTransceiver; // transceiver capabilities (size of rop, ropframe, etc.) + MN protocol version
    bool                txrateISset;
    bool                cleanedBoardBehaviour;    // the board is in config mode and does not have any regulars
    uint16_t            usedNumberOfRegularROPs;

    can_string_eth*     c_string_handler[16];

    TheEthManager       *ethManager;

    EthMonitorPresence monitorpresence;

    bool regularsAreSet;

private:


    bool verifyBoard();
    bool setTimingOfRunningCycle();
    bool verifyBoardPresence();
    bool verifyBoardTransceiver();
    bool cleanBoardBehaviour(void);
    bool askBoardVersion(void);
    // we keep isRunning() and we add a field in the reply of serviceStart()/Stop() which tells if the board is in run mode or not.
    bool isRunning(void);

    // lock of the object: on / off
    bool lock(bool on);

    bool serviceCommand(eOmn_serv_operation_t operation, eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout, int times);

    bool verbosewhenok;
};




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class yarp::dev::FakeEthResource :  public AbstractEthResource
{
public:
    
    // the size of the boardname must belong to EthResources because it is this class which extracts it from the xml file.
    // all other places where this name is stored have copied it from here using EthResource::getName(void).
    // an example is TheEthManager::ethBoards which makes it available with TheEthManager::getName(eOipv4addr_t ipv4)
    
    enum { boardNameSize = 32 };
    
    // this is the maximum size of rx and tx packets managed by the ethresource. however, the HostTranceveiver can reduce these values.
    enum { maxRXpacketsize = 1496, maxTXpacketsize = 1496 };
    
public:
    
    FakeEthResource();
    ~FakeEthResource();
    
    
    bool            open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal) override;
    bool            close();
    bool            isEPsupported(eOprot_endpoint_t ep);
    
    ACE_INET_Addr   getRemoteAddress(void);
    
    eOipv4addr_t    getIPv4remoteAddress(void);
    
    const char *    getName(void);
    const char *    getIPv4string(void);
    
    eObrd_ethtype_t getBoardType(void);
    const char *    getBoardTypeString(void);
    
    void getBoardInfo(eOdate_t &date, eOversion_t &version);
    
    // the function returns true if the packet can be transmitted. 
    // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
    bool            getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops);
    
    bool            canProcessRXpacket(uint64_t *data, uint16_t size);
    
    void            processRXpacket(uint64_t *data, uint16_t size, bool collectStatistics = true);
    
    
    bool verifyRemoteValue(eOprotID32_t id32, void *value, uint16_t size, double timeout = 0.100, int retries = 10);
    
    bool getRemoteValue(eOprotID32_t id32, void *value, uint16_t &size, double timeout = 0.100, int retries = 10);
    
    
    // very important note: it works only if there is an handler for the id32 and it manages the unlock of the mutex
    bool setRemoteValueUntilVerified(eOprotID32_t id32, void *value, uint16_t size, int retries = 10, double waitbeforeverification = 0.001, double verificationtimeout = 0.050, int verificationretries = 2);
    
    
    bool verifyEPprotocol(eOprot_endpoint_t ep);
    
    bool aNetQueryReplyHasArrived(eOprotID32_t id32, uint32_t signature);
    
    
    bool printRXstatistics(void);
    bool CANPrintHandler(eOmn_info_basic_t* infobasic);
    
    
    bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500);
    
    bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500);
    
    bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500);
    
    bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500);
    
    bool Tick();
    bool Check();
    bool readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size);
    bool addSetMessage(eOprotID32_t id32, uint8_t* data);
    bool addGetMessage(eOprotID32_t id32);
    uint16_t getNVnumber(eOnvEP8_t ep);
    uint32_t translate_NVid2index(eOprotID32_t id32);
    bool addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data);
    bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size);
    EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv);
    
    
    // -- not used at the moment
    void checkIsAlive(double curr_time);
    bool isFake();
    // double          getLastRecvMsgTimestamp(void);
    // int             getNumberOfAttachedInterfaces(void);
    // returns the capacity of the receiving buffer.
    // int getRXpacketCapacity();
    
private: //FAKE
    eOipv4addr_t      ipv4addr;
    char              ipv4addrstring[20];
    char              boardName[32];
    char              boardTypeString[32];
    eObrd_ethtype_t   ethboardtype;
    double            lastRecvMsgTimestamp;   //! stores the system time of the last received message, gettable with getLastRecvMsgTimestamp()
    bool              isInRunningMode;        //!< say if goToRun cmd has been sent to EMS
    ACE_INET_Addr     remote_dev;
    
    yarp::os::Semaphore*  objLock;
    
    bool                verifiedEPprotocol[eoprot_endpoints_numberof];
    bool                verifiedBoardPresence;
    
    bool                verifiedBoardTransceiver; // transceiver capabilities (size of rop, ropframe, etc.) + MN protocol version
    eOmn_comm_status_t  boardCommStatus;
    uint16_t            usedNumberOfRegularROPs;
    
    TheEthManager       *ethManager;
    HostTransceiver     *myHostTrans;

    
private:
    
    
    bool verifyBoard();



    bool cleanBoardBehaviour(void);
    // we keep isRunning() and we add a field in the reply of serviceStart()/Stop() which tells if the board is in run mode or not.
    bool isRunning(void);
    
    // lock of the object: on / off
    bool lock(bool on);
    
    
    bool verbosewhenok;
};

#endif

// eof




