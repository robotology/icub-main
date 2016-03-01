// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <string>
#include <ethManager.h>
#include <ethResource.h>
#include <errno.h>

#include "EOYtheSystem.h"

#include "EOropframe.h"

#include <stdexcept>      // std::out_of_range
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>
#include <ace/Time_Value.h>

#ifdef ICUB_USE_REALTIME_LINUX
#include <pthread.h>
#include <unistd.h>
#endif //ICUB_USE_REALTIME_LINUX

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::managerMutex = 1;

yarp::os::Semaphore TheEthManager::rxMutex = 1;
yarp::os::Semaphore TheEthManager::txMutex = 1;



// - class EthBoards

const char * EthBoards::names[EthBoards::maxEthBoards] =
{
    "error",
    "leftUpperArm", "leftForearm", "rightUpperArm", "rightForearm",
    "Torso",
    "leftLeg", "leftFoot", "rightLeg", "rightFoot",
    "leftLegSkin", "rightLegSkin",
    "unknown12", "unknown13", "unknown14", "unknown15",
    "unknown16", "unknown17", "unknown18", "unknown19", "unknown20", "unknown21", "unknown22", "unknown23",
    "unknown24", "unknown25", "unknown26", "unknown27", "unknown28", "unknown29", "unknown30", "unknown31"
};

EthBoards::EthBoards()
{
    memset(LUT, 0, sizeof(LUT));
    sizeofLUT = 0;
}


EthBoards::~EthBoards()
{
    memset(LUT, 0, sizeof(LUT));
    sizeofLUT = 0;
}


size_t EthBoards::number_of_resources(void)
{
    return(sizeofLUT);
}

size_t EthBoards::number_of_interfaces(ethResources * res)
{
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;


    if(index>=maxEthBoards)
    {
        return 0;
    }

    if(NULL != LUT[index].resource)
    {
        return 0;
    }

    return(LUT[index].numberofinterfaces);
}


bool EthBoards::add(ethResources* res)
{
    if(NULL == res)
    {
        return false;
    }

    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;

    if(index>=maxEthBoards)
    {
        return false;
    }

    if(NULL != LUT[index].resource)
    {
        return false;
    }

    LUT[index].resource = res;
    LUT[index].ipv4 = ipv4;
    LUT[index].boardnumber = index;
    memcpy(LUT[index].name, res->getName(), sizeof(LUT[index].name));
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<ethFeatType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT++;

    return true;
}


bool EthBoards::add(ethResources* res, IethResource* interface, ethFeatType_t type)
{
    if((NULL == res) || (NULL == interface) || (ethFeatType_NULL == type))
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    if(NULL != LUT[index].interfaces[type])
    {
        return false;
    }

    // ok, i add it
    LUT[index].interfaces[type] = interface;
    LUT[index].numberofinterfaces ++;


    return true;
}


bool EthBoards::rem(ethResources* res)
{
    if(NULL == res)
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    LUT[index].resource = NULL;
    LUT[index].ipv4 = 0;
    LUT[index].boardnumber = 0;
    memset(LUT[index].name, 0, sizeof(LUT[index].name));
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<ethFeatType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT--;

    return true;
}


bool EthBoards::rem(ethResources* res, ethFeatType_t type)
{
    if((NULL == res) || (ethFeatType_NULL == type))
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getIPv4remoteAddress();
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    if(NULL != LUT[index].interfaces[type])
    {
        LUT[index].interfaces[type] = NULL;
        LUT[index].numberofinterfaces --;
    }

    return true;
}



//ethResources* EthBoards::get(ACE_INET_Addr adr)
//{
//    ethResources * ret = NULL;

//    ACE_UINT32 a32 = adr.get_ip_address();
//    uint8_t index = a32 & 0xff;
//    index --;
//    if(index<maxEthBoards)
//    {
//        ret = ethresLUT[index];
//    }

//    return(ret);
//}


//ethResources* EthBoards::get(ethFeatIPaddress_t ipaddr)
//{
//    ethResources * ret = NULL;

//    ACE_UINT32 a32 = ipaddr.ip4;
//    uint8_t index = a32 & 0xff;
//    index --;
//    if(index<maxEthBoards)
//    {
//        ret = ethresLUT[index];
//    }

//    return(ret);
//}


ethResources* EthBoards::get_resource(eOipv4addr_t ipv4)
{
    ethResources * ret = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        ret = LUT[index].resource;
    }

    return(ret);
}


IethResource* EthBoards::get_interface(eOipv4addr_t ipv4, eOprotID32_t id32)
{
    IethResource *dev = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return NULL;
    }

    if(NULL == LUT[index].resource)
    {
        return dev;
    }

    ethFeatType_t type = ethFeatType_NULL;
    eOprotEndpoint_t ep = eoprot_ID2endpoint(id32);
    switch(ep)
    {
        case eoprot_endpoint_management:
        {
            type = ethFeatType_Management;
        } break;

        case eoprot_endpoint_motioncontrol:
        {
            type = ethFeatType_MotionControl;
        } break;

        case eoprot_endpoint_skin:
        {
            type = ethFeatType_Skin;
        } break;

        case eoprot_endpoint_analogsensors:
        {
            eOprotEntity_t en = eoprot_ID2entity(id32);
            if(eoprot_entity_as_strain == en)
                type = ethFeatType_AnalogStrain;
            else if(eoprot_entity_as_mais == en)
                type = ethFeatType_AnalogMais;
            else if(eoprot_entity_as_inertial == en)
                type = ethFeatType_AnalogInertial;
            else
                type = ethFeatType_NULL;
        } break;

        default:
        {
            type = ethFeatType_NULL;
        } break;
    }

    if(ethFeatType_NULL != type)
    {
        dev = LUT[index].interfaces[type];
    }



    return dev;
}


//const char * EthBoards::name(ACE_INET_Addr adr)
//{
//    const char * ret = NULL;

//    ACE_UINT32 a32 = adr.get_ip_address();
//    uint8_t index = a32 & 0xff;
//    index --;
//    if(index<maxEthBoards)
//    {
//        ret = names[index];
//    }

//    return(ret);
//}


const char * EthBoards::name(eOipv4addr_t ipv4)
{
    const char * ret = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        ret = names[index];
    }

    return(ret);
}



bool EthBoards::execute(void (*action)(ethResources* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    for(int i=0; i<maxEthBoards; i++)
    {
        ethResources* res = LUT[i].resource;
        if(NULL != res)
        {
            action(res, par);
        }

    }

    return(true);
}


// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


//const char * TheEthManager::boardNames[TheEthManager::maxBoards] =
//{
//    "error",
//    "leftUpperArm", "leftForearm", "rightUpperArm", "rightForearm",
//    "Torso",
//    "leftLeg", "leftFoot", "rightLeg", "rightFoot",
//    "leftLegSkin", "rightLegSkin",
//    "unknown12", "unknown13", "unknown14", "unknown15",
//    "unknown16", "unknown17", "unknown18", "unknown19", "unknown20", "unknown21", "unknown22", "unknown23",
//    "unknown24", "unknown25", "unknown26", "unknown27", "unknown28", "unknown29", "unknown30", "unknown31"
//};

bool TheEthManager::lock()
{
    managerMutex.wait();
    return true;
}

bool TheEthManager::unlock()
{
    managerMutex.post();
    return true;
}


bool TheEthManager::lockTX(bool on)
{
    if(on)
    {
        txMutex.wait();
    }
    else
    {
        txMutex.post();
    }

    return true;
}

void ethEvalTXropframe(ethResources *r, void* p)
{
    if((NULL == r) || (NULL == p))
    {
        return;
    }

    TheEthManager *ethman = (TheEthManager*)p;

    uint16_t numofbytes = 0;
    uint16_t numofrops = 0;
    uint8_t* data2send = NULL;
    bool transmitthepacket = r->getPointer2TxPack(&data2send, &numofbytes, &numofrops);

    if(true == transmitthepacket)
    {
        ACE_INET_Addr ipaddress = r->getRemoteAddress();
        ethman->send(data2send, (size_t)numofbytes, ipaddress);
    }
}


bool TheEthManager::Transmission(void)
{
    lockTX(true);

    ethBoards->execute(ethEvalTXropframe, this);

    lockTX(false);

    return true;
}


bool TheEthManager::lockRX(bool on)
{
    if(on)
    {
        rxMutex.wait();
    }
    else
    {
        rxMutex.post();
    }

    return true;
}

bool TheEthManager::lockTXRX(bool on)
{
    if(on)
    {
        txMutex.wait();
        rxMutex.wait();
    }
    else
    {
        rxMutex.post();
        txMutex.post();
    }

    return true;
}

ethResources *TheEthManager::requestResource(yarp::os::Searchable &cfgtotal, yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, ethFeature_t &request)
{
    yTrace() << " Requested BRD" << request.boardNumber;
    // Check if local socket is initted, if not do it.
    ACE_TCHAR address[64] = {0};
    snprintf(address, sizeof(address), "%s:%d", request.pc104IPaddr.string, request.pc104IPaddr.port);

    if(request.boardNumber > TheEthManager::maxBoards)
    {
        yError() << "FATAL ERROR: TheEthManager::requestResource() detected a board number beyond the maximum allowed (max, rqst) =" << maxBoards << request.boardNumber << ")";
        return NULL;
    }

    if(request.boardIPaddr.ip4 != request.boardNumber)
    {
        yError() << "FATAL ERROR: TheEthManager::requestResource() detected a board number different from its ip4 address (boardNumber, ip4) =" << request.boardNumber << request.boardIPaddr.ip4 << ")";
        return NULL;
    }

    if(0 == strlen(request.boardName))
    {
        // set default name: unset_xml_boardName__10.0.1.x
        snprintf(request.boardName, sizeof(request.boardName), "unsetXMLboardName__10.0.1.%d", request.boardNumber);
    }

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(request.pc104IPaddr.ip1, request.pc104IPaddr.ip2, request.pc104IPaddr.ip3, request.pc104IPaddr.ip4);
    ACE_UINT32 hostip = (request.pc104IPaddr.ip1 << 24) | (request.pc104IPaddr.ip2 << 16) | (request.pc104IPaddr.ip3 << 8) | (request.pc104IPaddr.ip4);
    ACE_INET_Addr myIP((u_short)request.pc104IPaddr.port, hostip);
    myIP.dump();

    int txrate = -1; // uses default
    int rxrate = -1; // uses default

    // if i find it in section ... of cfgtotal then i change it


    if(cfgtotal.findGroup("PC104").check("PC104TXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104TXrate").asInt();
        if(value > 0)
            txrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104TXrate. thus using default value";
    }

    if(cfgtotal.findGroup("PC104").check("PC104RXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104RXrate").asInt();
        if(value > 0)
            rxrate = value;
    }
    else
    {
        yWarning () << "TheEthManager::requestResource() cannot find ETH/PC104RXrate. thus using default value";
    }

    if(!createSocket(myIP, txrate, rxrate) )
    {  return NULL;  }


#if 1

    lockTXRX(true);

    ethResources *rr = ethBoards->get_resource(ipv4addr);

    if(NULL == rr)
    {
        // device doesn't exist yet: create it
        yTrace() << "Creating ethResource for IP " << request.boardIPaddr.string;
        rr = new ethResources;
        if(false == rr->open(cfgtotal, cfgtransceiver, cfgprotocol, request))
        {
            yError() << "Error creating new ethReasource for IP " << request.boardIPaddr.string;
            if(NULL != rr)
            {
                delete rr;
            }

            rr = NULL;
            return NULL;
        }

        ethBoards->add(rr);
    }

    ethBoards->add(rr, request.interface, request.type);
    //it is the same as addLUTelement(request);


//    rr->registerFeature(request);

    lockTXRX(false);

    return(rr);

#else
    //    int justCreated = false;
    //    ethResources *newRes = NULL;

//    lock();          // lock so that we can use EMS_list in exclusive way
//    ethResIt it = EMS_list.begin();
//    ethResIt itend = EMS_list.end();

//    while(it != itend)
//    {
//        (*it)->getRemoteAddress().addr_to_string(tmp_addr, 64);
//        if( strcmp(tmp_addr, request.boardIPaddr.string) == 0 )
//        {
//            // device already exists, return the pointer.
//            newRes = (*it);
//            break;
//        }
//        it++;
//    }
//    unlock(); // must unlock now. because ethResources::open() need to use the receiver which uses the lock() to get the EMS_list

//    if(NULL == newRes)
//    {
//        // device doesn't exist yet: create it
//        yTrace() << "Creating BRD device with IP " << request.boardIPaddr.string;
//        newRes = new ethResources;
//        if(!newRes->open(cfgtotal, cfgtransceiver, cfgprotocol, request))
//        {
//            yError() << "Error creating new BRD!!";
//            if(NULL != newRes)
//                delete newRes;

//            newRes = NULL;
//            return NULL;
//        }
//        justCreated = true;
//    }


//    lock(); // lock because i use the EMS_list and i call addLUTelement

//    // the push_back has to be done only once for EMS
//    if(justCreated)
//    {
//        EMS_list.push_back(newRes);
//        int index = request.boardNumber-1;
//        if(index < ethresLUTsize)
//        {
//            ethresLUT[index] = newRes;
//            numberOfUsedBoards++;
//        }
//    }

//    // The registerFeature has to be done always
//    newRes->registerFeature(request);
//    addLUTelement(request);

//    unlock();

//    return newRes;
#endif
}



int TheEthManager::releaseResource(ethFeature_t &resource)
{
#if 1

    int ret = 1; // -1 means that the singleton is not needed anymore

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(resource.boardIPaddr.ip1, resource.boardIPaddr.ip2, resource.boardIPaddr.ip3, resource.boardIPaddr.ip4);

    ethResources* rr = ethBoards->get_resource(ipv4addr);
    if(NULL != rr)
    {
        bool success = rr->serviceStop(eomn_serv_category_all); // but you must change later on with eomn_serv_category_mc or ...
        success = success;


        // now we operate on the boards .. disable tx and rx
        lockTXRX(true);

        // remove the interface
        ethBoards->rem(rr, resource.type);

        int remaining = ethBoards->number_of_interfaces(rr);
        if(0 == remaining)
        {   // remove also the resource
            rr->close();
            ethBoards->rem(rr);
            delete rr;
        }

        if(0 == ethBoards->number_of_resources())
        {   // we dont have any more resources
            ret = -1;
        }

        lockTXRX(false);

    }

    return(ret);


#else
    yTrace() << resource.boardNumber;
    int           ret           = 0;
    int           stillUsed     = 0;
    ethResources  *res2release  = NULL;
    char tmp_addr[64];


    ethResources *tmpEthRes;
    ACE_INET_Addr  tmp_ace_addr;

    lock();
    ethResIt it = EMS_list.begin();
    ethResIt itend = EMS_list.end();
    unlock();

    if(false == emsAlreadyClosed)
    {
        while(it != itend)
        {
            tmpEthRes = (*it);
            tmpEthRes->serviceStop(eomn_serv_category_all);
            it++;
        }
        // before stopping threads, flush all pkts not yet sent.
        flush();
        emsAlreadyClosed = true;
    }
    stopThreads();

    removeLUTelement(resource);

    lock();
    it = EMS_list.begin();
    itend = EMS_list.end();
    while(it != itend)
    {
        tmpEthRes = (*it);
        tmp_ace_addr = tmpEthRes->getRemoteAddress();
        tmp_ace_addr.addr_to_string(tmp_addr, 64);
        if( strcmp(tmp_addr, resource.boardIPaddr.string) == 0)
        {
            // device exists
            res2release = (*it);
            stillUsed = res2release->deregisterFeature(resource);
            if( !! stillUsed)
            {
                ret = 1;
                break;
            }
            else
            {
                res2release->close();
                EMS_list.remove(res2release);
                delete res2release;
                ret = 1;
                break;
            }
        }
        it++;
    }

    if(     (EMS_list.size() == 0 ) && (boards_map_ext.size() != 0 )
        ||  (EMS_list.size() != 0 ) && (boards_map_ext.size() == 0 ) )
    {
        yError() << "Something strange happened... EMS_list.size is" << EMS_list.size() << "while boards_map_ext.size is "<< boards_map_ext.size();
    }

    if(!ret)
        //yError() << "EthManager: Trying to release a non existing resource" << resource.name << " for boardId " << resource.boardNumber << "maybe already deleted?";
        yError() << "EthManager: Trying to release a non existing resource for boardId " << resource.boardNumber << "maybe already deleted?";

    // ret = -1 means that the singleton is not needed anymore
    if( (EMS_list.size() == 0 ) || (boards_map_ext.size() == 0 ) )
        ret = -1;

    unlock();

    return ret;
#endif
}


//void TheEthManager::addLUTelement(ethFeature_t &id)
//{
//    yTrace() << id.boardNumber;
//    //in maps use board num starts from 0 so
//    FEAT_boardnumber_t brdnum = id.boardNumber;
//    uint8_t entity = (eoprot_endpoint_motioncontrol == id.endpoint) ? (eoprot_entity_mc_joint) : id.entity;
//    //uint8_t entity = id.entity;
//    uint32_t item2 = (id.endpoint<<24)|(entity<<16);

//    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
//    /* Defined the var addLUT_result to have the true/false result of the insert operation...It helped catching a bug.
//     * It fails if the element is already present. This can happen if someone tries to read an element with
//     * the '[]' operator before the insert, because the std::map will create it automatically (hopefully initted
//     * with zeros.
//     */


//    bool addLUT_result =  boards_map_ext.insert(std::pair< std::pair<FEAT_boardnumber_t, uint32_t>, ethFeature_t>(std::make_pair(brdnum, item2), id)).second;

//    // Check result of insertion
//    addLUT_result ? (yTrace() << "ok add lut element for board " << id.boardNumber << " and ep " << id.endpoint << " and entity " << id.entity) :
//                    (yError() << "NON ok add lut element for board " << id.boardNumber << " and ep " << id.endpoint << " and entity " << id.entity);


//    std::pair<FEAT_boardnumber_t, uint32_t > key (brdnum, item2);
//    try
//    {
//        // USE .at AND NOT the '[ ]' alternative!!! It will create a bug!!!
//        /* The bug is caused by the fact that the [] version will create an unitialized element inside the map,
//            * causing the return of a wrong pointer.
//            * Furthermore the insert method used to correctly initialze the element will fail because a (wrong)
//            * element is already present preventing the map to be corrected.
//            */
//        IethResource * ret = boards_map_ext.at(key).interface;
//    }
//    catch (const std::out_of_range& errMsg)
//    {
//        yError() << "Error after  LUT insertion!!! (" << errMsg.what() << ")";
//    }
//}

//bool TheEthManager::removeLUTelement(ethFeature_t &element)
//{
//    yTrace() << element.boardNumber;
//    /* NO MUTEX HERE because it's a PRIVATE method, so called only inside other already mutexed methods */
//    bool ret = false;
//    uint8_t entity = (eoprot_endpoint_motioncontrol == element.endpoint) ? (eoprot_entity_mc_joint) : element.entity;
//    //uint8_t entity = element.entity;
//    uint32_t item2 = (element.endpoint<<24)|(entity<<16);
//    int n = (int) boards_map_ext.erase(std::make_pair(element.boardNumber, item2));

//    switch(n)
//    {
//        case 0:
//        {
//            yError() << "Trying to remove a non-existing element" << element.name << "maybe already removed?";
//            ret = false;
//            break;
//        }

//        case 1:
//        {
//            yTrace() << "ethFeature_t element removed succesfully from the map" << element.name;
//            ret = true;
//            break;
//        }
//        default:
//        {
//            yError() << "More than one element were removed with key board num " << element.boardNumber << "ep " << element.endpoint;
//            ret = true;
//            break;
//        }
//    }

//    return ret;
//}

bool TheEthManager::getHandle(eOipv4addr_t ipv4, eOprotID32_t id32, IethResource **interfacePointer)
{
    *interfacePointer = ethBoards->get_interface(ipv4, id32);

    if(NULL == *interfacePointer)
    {
        return false;
    }

    return true;

//    eOprotEndpoint_t ep = eoprot_ID2endpoint(id32);
//    eOprotEntity_t entity = eoprot_ID2entity(id32);

//    if(eoprot_endpoint_motioncontrol == ep)
//    {   // motion control is registered in board_maps-ext as (brd,ep-joint), thus if the id32 is about a motor entity we must be able to cope ...
//        entity = eoprot_entity_mc_joint;
//    }


////     lock(); // marco.accame: found already commented. see why
//    bool ret = false;
//    static int _error = 0;
//    uint32_t item2 = (ep<<24)|(entity<<16);
//    std::pair<FEAT_boardnumber_t, uint32_t > key (boardnum, item2);

//    try
//    {
//        // USE .at AND NOT the '[ ]' alternative!!! It will create a bug!!!
//        /* The bug is caused by the fact that the [] version will create an unitialized element inside the map,
//         * causing the return of a wrong pointer.
//         * Furthermore the insert method used to correctly initialze the element will fail because a (wrong)
//         * element is already present preventing the map to be corrected.
//         */
//        *interfacePointer = boards_map_ext.at(key).interface;
//        *type = boards_map_ext.at(key).type;
//        ret = true;
//    }
//    catch (const std::out_of_range& errMsg)
//    {
//        if(0 == (_error%1000))
//        {
//            char nvinfo[128];
//            eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
//            interfacePointer = NULL;
//            *type = ethFeatType_NULL;

//            yError() << "TheEthManager::getHandle() cannot find a handle for boardNum "<< boardnum << " and nv" << nvinfo << " ... maybe the board was in running mode before robotInterface was started (" << errMsg.what() <<")";
//        }
//        ret = false;
//        _error++;
//    }
////     unlock(); // marco.accame: found already commented. see why
//    return ret;
}




// this function is called by the embobj error manager
void embOBJerror(eOerrmanErrorType_t errtype, const char *info, eOerrmanCaller_t *caller, const eOerrmanDescriptor_t *des)
{
    const char defobjstr[] = "EO?";
    const char *eobjstr = (NULL == caller) ? (defobjstr) : (caller->eobjstr);

    yError() << "embOBJerror(): errtype = " << eo_errman_ErrorStringGet(eo_errman_GetHandle(), errtype) << "from EOobject = " << eobjstr << " w/ message = " << info;

    if(errtype == eo_errortype_fatal)
    {
        yError() << "embOBJerror(): FATAL ERROR: the calling thread shall now be stopped in a forever loop here inside";
        for(;;);
    }

}


TheEthManager::TheEthManager()
{
    yTrace();
    memset(info, 0, sizeof(info));
    snprintf(info, sizeof(info), "TheEthManager");

//    memset(ethresLUT, 0, sizeof(ethresLUT));
//    numberOfUsedBoards = 0;

//    EMS_list.clear();
    UDP_initted = false;
    UDP_socket  = NULL;
//    emsAlreadyClosed = false;

    ethBoards = new(EthBoards);

    TheEthManager::initEOYsystem();

    starttime = yarp::os::Time::now();
}


EthSender* TheEthManager::getEthSender(void)
{
    return sender;
}


EthReceiver* TheEthManager::getEthReceiver(void)
{
    return receiver;
}


double TheEthManager::getStartTime(void)
{
    return starttime;
}

//bool TheEthManager::getEMSlistRiterators(ethResRIt& rbegin, ethResRIt& rend)
//{   // marco.accame: added this function so that we can keep lock()/unlock() private
//    lock();
//    rbegin = EMS_list.rbegin();
//    rend = EMS_list.rend();
//    unlock();
//    return true;
//}

void TheEthManager::initEOYsystem(void)
{
    // marco.accame: in here we init the embOBJ system for YARP.
    eOerrman_cfg_t errmanconfig = {0};
    errmanconfig.extfn.usr_on_error     = embOBJerror;
    const eOysystem_cfg_t *syscfg       = NULL;
    const eOmempool_cfg_t *mpoolcfg     = NULL;     // uses standard mode
    //const eOerrman_cfg_t *errmancf      = NULL;     // uses default mode
    eoy_sys_Initialise(syscfg, mpoolcfg, &errmanconfig);
}


bool TheEthManager::createSocket(ACE_INET_Addr local_addr, int txrate, int rxrate)
{
    yTrace();
    lock();


    if(!UDP_initted)
    {
        UDP_socket = new ACE_SOCK_Dgram();
        char tmp[64];
        if(-1 == UDP_socket->open(local_addr))
        {
            local_addr.addr_to_string(tmp, 64);
            yError() <<   "\n/---------------------------------------------------\\"
                     <<   "\n| Unable to bind to local IP address " << tmp
                     <<   "\n\\---------------------------------------------------/";
            delete UDP_socket;
            UDP_socket = NULL;
            UDP_initted = false;
        }
        else
        {
            UDP_initted = true;

            if((txrate <= 0) || (txrate > EthSender::EthSenderMaxRate))
            {
                txrate = EthSender::EthSenderDefaultRate;
            }
            if((rxrate <= 0) | (rxrate > EthReceiver::EthReceiverMaxRate))
            {
                rxrate = EthReceiver::EthReceiverDefaultRate;
            }
            sender = new EthSender(txrate);
            receiver = new EthReceiver(rxrate);

            sender->config(UDP_socket, this);
            receiver->config(UDP_socket, this);
            /* Start the threads sending to and receiving messages from the boards.
             * It will execute the threadInit and pass its return value to the following calls
             * afterStart to check if they started correctly.
             */
            bool ret1, ret2;
            ret1 = sender->start();
            ret2 = receiver->start();

            if(!ret1 || !ret2)
            {
                yError() << "EthManager: issue while starting threads for UDP communication with EMSs";

                // stop threads
                stopThreads();

                delete UDP_socket;
                UDP_initted = false;
                return false;
            }
            else
            {
                yTrace() << "Both sending and Receiving thread started correctly!";

            }
        }
    }

    unlock();
    return UDP_initted;
}

bool TheEthManager::isInitted(void)
{
    yTrace();
    bool ret;
    lock();
    ret = UDP_initted;
    unlock();
    return ret;
}

TheEthManager *TheEthManager::instance()
{
    yTrace();
    managerMutex.wait();    // marco.accame: in here we dont use this->lock() because if object does not already exists, the function does not exists. i can use the static semaphore instead
    if (NULL == handle)
    {
        yTrace() << "Calling EthManager Constructor";
        handle = new TheEthManager();
        if (NULL == handle)
            yError() << "While calling EthManager constructor";
        else
            feat_Initialise(static_cast<void*>(handle));
    }
    managerMutex.post();

    return handle;
}

bool TheEthManager::killYourself()
{
    yTrace();
    //TODO harakiri
    delete handle;

    return true;
}

bool TheEthManager::stopThreads()
{
    bool ret = true;
    // Stop method also make a join waiting the thread to exit
    if(sender->isRunning() )
        sender->stop();
    if(receiver->isRunning() )
    {
#ifdef ETHRECEIVER_ISPERIODICTHREAD
        receiver->stop();
#else
        ret = receiver->stop();
#endif
    }
    return ret;
}

void delete_resources(ethResources *p, void* par)
{
    delete p;
}

TheEthManager::~TheEthManager()
{
    yTrace();

    // for sure we shoudl stop threads tx / rx
    // before stopping threads, flush all pkts not yet sent.
    flush();
    stopThreads();

    // Deinitialize fetaure interface
    feat_DeInitialise();

    // Close UDP socket
    if(isInitted())
    {
        lock();
        UDP_socket->close();
        delete UDP_socket;
        UDP_initted = false;
        unlock();
    }


    lock();
//    // Destroy all EMS boards
//    if(EMS_list.size() != 0)
//    {
//        memset(ethresLUT, 0, sizeof(ethresLUT));
//        numberOfUsedBoards = 0;

//        ethResIt iterator = EMS_list.begin();

//        while(iterator != EMS_list.end())
//        {
//            delete(*iterator);
//            iterator++;
//        }
//    }

    // remove all ethresource ... we dont need to call lockTXRX() because we are not transmitting now
    ethBoards->execute(delete_resources, NULL);
    delete ethBoards;

//    if(boards_map_ext.size() != 0)
//    {
//        std::map<std::pair<FEAT_boardnumber_t, uint32_t>, ethFeature_t>::iterator mIt;
//        for(mIt = boards_map_ext.begin(); mIt!=boards_map_ext.end(); mIt++)
//        {
//            yError() << "Feature " << mIt->second.name << "was not correctly removed from map.., removing it now in the EthManager destructor.";
//            removeLUTelement(mIt->second);
//        }
//    }
    unlock();
    handle = NULL;
}

int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
    ssize_t ret = UDP_socket->send(data, len, remote_addr);
    return ret;
}

bool TheEthManager::Reception(ACE_INET_Addr adr, uint64_t* data, ssize_t size, bool collectStatistics)
{
    ACE_UINT32 a32 = adr.get_ip_address();
    uint8_t ip4 = a32 & 0xff;
    uint8_t ip3 = (a32 >> 8) & 0xff;
    uint8_t ip2 = (a32 >> 16) & 0xff;
    uint8_t ip1 = (a32 >> 24) & 0xff;

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);

    lockRX(true);

    ethResources* r = ethBoards->get_resource(ipv4addr);

    if(NULL != r)
    {
        if(false == r->canProcessRXpacket(data, size))
        {   // cannot give packet to ethresource
            yError() << "TheEthManager::Reception() cannot give a received packet of size" << size << "to ethResources because ethResources::canProcessRXpacket() returns false.";
        }
        else
        {
            r->processRXpacket(data, size, collectStatistics);
        }

    }
    else
    {
    //    adr.addr_to_string(address, sizeof(address));
    //    yError() << "TheEthManager::Reception cannot get a ethres associated to address" << address;
    }

    lockRX(false);


    return(true);
}


ethResources* TheEthManager::IPtoResource(ACE_INET_Addr adr)
{
    ACE_UINT32 a32 = adr.get_ip_address();
    uint8_t ip4 = a32 & 0xff;
    uint8_t ip3 = (a32 >> 8) & 0xff;
    uint8_t ip2 = (a32 >> 16) & 0xff;
    uint8_t ip1 = (a32 >> 24) & 0xff;

    eOipv4addr_t ipv4addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    return(ethBoards->get_resource(ipv4addr));
}

int TheEthManager::GetNumberOfUsedBoards(void)
{
    return(ethBoards->number_of_resources());
}

const char * TheEthManager::getName(eOipv4addr_t ipv4)
{
    const char * ret = ethBoards->name(ipv4);
    if(NULL == ret)
    {
        ret = EthBoards::names[0];
    }

    return ret;
}

ethResources* TheEthManager::GetEthResource(eOipv4addr_t ipv4)
{
//    #warning --> see if protect with a mutex or not ... sure not in runtime. surely yes in opening or closing
//    ethResources *res = NULL;
//    lock();

//    ethResIt iterator = EMS_list.begin();

//    eOprotBRD_t targetbrd = featIdBoardNum2nvBoardNum(boardnum);
//    while(iterator != EMS_list.end())
//    {
//        eOprotBRD_t brdn = (*iterator)->get_protBRDnumber();

//        if(brdn == targetbrd)
//        {
//            res = *iterator;
//            break;
//        }
//        iterator++;
//    }


//    unlock();

    return(ethBoards->get_resource(ipv4));
}

// Probably useless
bool TheEthManager::open()
{
    yTrace();
    return true;
}

bool TheEthManager::close()
{
    yTrace();

    return true;
}


void TheEthManager::flush()
{
    //#warning --> marco.accame: TODO think about how removing delay of 1 second
    // here sleep is essential in order to let sender thread send gotoconfig command.
    yarp::os::Time::delay(1);
}


EthSender::EthSender(int txrate) : RateThread(txrate)
{

    rateofthread = txrate;
    yDebug() << "EthSender is a RateThread with rate =" << rateofthread << "ms";
    yTrace();
}

bool EthSender::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    send_socket = pSocket;
    ethManager  = _ethManager;

    return true;
}

bool EthSender::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#ifdef ICUB_USE_REALTIME_LINUX
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (50) and less than the receiver thread
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2 - 1; // = 48
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif //ICUB_USE_REALTIME_LINUX

    return true;
}

void EthSender::evalPrintTXstatistics(void)
{
#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // For statistic purpose

    unsigned int it=getIterations();
    if(it == ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_NUMBEROF_CYCLES)
    {
        printTXstatistics();
    }
#endif
}

void EthSender::printTXstatistics(void)
{
#ifdef ETHMANAGER_DEBUG_COMPUTE_STATS_FOR_CYCLE_TIME_
    // For statistic purpose

    unsigned int it=getIterations();

    double avPeriod, stdPeriod;
    double avThTime, stdTime;

    getEstUsed(avThTime, stdTime);
    getEstPeriod(avPeriod, stdPeriod);

    char string[128] = {0};
    snprintf(string, sizeof(string), "  (STATS-TX)-> EthSender::run() thread run %d times, est period: %.3lf, +-%.4lf[ms], est used: %.3lf, +-%.4lf[ms]\n",
                            it,
                            avPeriod, stdPeriod,
                            avThTime, stdTime);
    yDebug() << string;

    resetStat();

#endif
}


#define ETHMANAGER_TEST_NEW_SENDER_RUN
#if defined(ETHMANAGER_TEST_NEW_SENDER_RUN)


void EthSender::run()
{    
    ethManager->Transmission();
}


#else

void EthSender::run()
{
    ethResources  *ethRes;
    uint16_t      bytes_to_send = 0;
    uint16_t      numofrops = 0;
    ethResRIt     riterator, _rBegin, _rEnd;


    /*
        Usare un reverse iterator per scorrere la lista dalla fine verso l'inizio. Questo aiuta a poter scorrere
        la lista con un iteratore anche durante la fase iniziale in cui vengono ancora aggiunti degli elementi,
        in teoria senza crashare. Al più salvarsi il puntatore alla rbegin sotto mutex prima di iniziare il ciclo,
        giusto per evitare che venga aggiunto un elemento in concomitanza con la lettura dell rbegin stesso.
        Siccome gli elementi vengono aggiunti solamente in coda alla lista, questa iterazione a ritroso non
        dovrebbe avere altri problemi e quindi safe anche senza il mutex che prende TUTTO il ciclo.
    */

    ethManager->getEMSlistRiterators(_rBegin, _rEnd);

    for(riterator = _rBegin; riterator != _rEnd && (isRunning()); riterator++)
    {
        p_sendData = NULL;
        bytes_to_send = 0;
        numofrops = 0;
        if(NULL == *riterator)
        {
            yError() << "EthManager::run, iterator==NULL";
            continue;
        }

        ethRes = (*riterator);

        // This uses directly the pointer of the transceiver
        bool transmitthepacket = ethRes->getPointer2TxPack(&p_sendData, &bytes_to_send, &numofrops);

        if(true == transmitthepacket)
        {
            ACE_INET_Addr addr = ethRes->getRemoteAddress();
            int ret = ethManager->send(p_sendData, (size_t)bytes_to_send, addr);
        }

    }

}

#endif // defined(ETHMANAGER_TEST_NEW_SENDER_RUN)


#ifdef ETHRECEIVER_ISPERIODICTHREAD
EthReceiver::EthReceiver(int raterx): RateThread(raterx)
#else
EthReceiver::EthReceiver()
#endif
{
    yTrace();
#ifdef ETHRECEIVER_STATISTICS_ON
    stat = new StatExt();
    stat_onRecFunc  = new StatExt();
    stat_onMutex = new StatExt();
#endif

#ifdef ETHRECEIVER_ISPERIODICTHREAD
    count=0;
    isFirst=true;
    rateofthread = raterx;
    yDebug() << "EthReceiver is a RateThread with rate =" << rateofthread << "ms";
    // ok, and now i get it from xml file ... if i find it.
#endif

    ConstString tmp = NetworkBase::getEnvironment("ETHSTAT_PRINT_INTERVAL");
    if (tmp != "")
    {
        statPrintInterval = (double)NetType::toInt(tmp);
    }
    else
    {
        statPrintInterval = 0.0;
    }
}

void EthReceiver::onStop()
{
    uint8_t tmp;
    ethManager->send( &tmp, 1, ethManager->local_addr);
}

EthReceiver::~EthReceiver()
{
    yTrace();
#ifdef ETHRECEIVER_STATISTICS_ON
    delete stat;
    delete stat_onRecFunc;
    delete stat_onMutex;
#endif
}

bool EthReceiver::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    recv_socket = pSocket;
    ethManager  = _ethManager;

    ACE_HANDLE sockfd = pSocket->get_handle();
    int retval;
    int32_t mysize = 1024*1024; //1Mb note:actually kernel uses memory with size doblem of mysize
                            //with this size i'm sure ems pkts are not lost
    int len = sizeof(mysize);

    // the user can change buffer size by environment variable ETHRECEIVER_BUFFER_SIZE
    ConstString _dgram_buffer_size = NetworkBase::getEnvironment("ETHRECEIVER_BUFFER_SIZE");
    if (_dgram_buffer_size!="")
        mysize = NetType::toInt(_dgram_buffer_size);

    retval = ACE_OS::setsockopt  (sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&mysize, sizeof(mysize));
    if (retval != 0)
    {
        int myerr = errno;
        yError()<< "ERROR in SetSockOpt SO_RCVBUF";
    }

    int32_t sock_input_buf_size = 0;
    retval = ACE_OS::getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&sock_input_buf_size, &len);
    if (retval != 0)
    {
        int myerr = errno;
        yError() << "ERROR inGetSockOpt SO_RCVBUF";
    }

    yWarning() << "in EthReceiver::config() the config socket has queue size = "<< sock_input_buf_size<< "; you request ETHRECEIVER_BUFFER_SIZE=" << _dgram_buffer_size;


    for(int i=0; i<TheEthManager::maxBoards; i++)
    {
        recFirstPkt[i] = false;
        seqnumList[i] = 0;
    }
    return true;
}


bool EthReceiver::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#ifdef ICUB_USE_REALTIME_LINUX
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (< 50)
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2; // = 49
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif //ICUB_USE_REALTIME_LINUX

    return true;
}


uint64_t getRopFrameAge(char *pck)
{
    return(eo_ropframedata_age_Get((EOropframeData*)pck));
}



int EthReceiver::getBoardNum(ACE_INET_Addr addr)
{
    int board;
    ACE_UINT32 ip = addr.get_ip_address();

    // board is last number in ip address
    board = ip & 0xff;
    return(board);
}



void EthReceiver::checkPktSeqNum(char* pktpayload, ACE_INET_Addr addr)
{
    int board = getBoardNum(addr);
    uint64_t seqnum = eo_ropframedata_seqnum_Get((EOropframeData*)pktpayload);

    if(board > TheEthManager::maxBoards)
    {
        yError() << "EthReceiver::checkPktSeqNum() detected a board number beyond maximum allowed: (detected, maximum) =" << board << "," << TheEthManager::maxBoards << ")";
        return;
    }

    if(0 == board)
    {
        yError() << "EthReceiver::checkPktSeqNum() detected a zero board number";
        return;
    }

    int boardindex = board - 1;

    if(recFirstPkt[boardindex]==false)
    {
        seqnumList[boardindex] = seqnum;
        recFirstPkt[boardindex] = true;
        yError() << "EthREceiver: FIRST SEQ NUM for board=" << board << " is " << seqnum;
    }
    else
    {
        if(seqnum != (seqnumList[boardindex]+1))
        {
            yError() << "EthREceiver: ---LOST PKTS---board =" << board << " seq num rec=" << seqnum << " expected=" << seqnumList[boardindex]+1;
        }
        seqnumList[boardindex] = seqnum;
    }
}

#if 1

void EthReceiver::run()
{
    ssize_t       incoming_msg_size = 0;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[ethResources::maxRXpacketsize/8]; // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = ethResources::maxRXpacketsize;

    int flags = 0;
#ifndef WIN32
    flags |= MSG_DONTWAIT;
#endif

    static int earlyexit = 0;

    int maxUDPpackets = EthReceiver::rateofthread*ethManager->GetNumberOfUsedBoards() + 2 + earlyexit*5; // marco.accame: set it as the number of boards plus a ... margin of 30% plus a +/- offset which depends on activity

    earlyexit = 0;

    for(int i=0; i<maxUDPpackets; i++)
    {
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size <= 0)
        {
            // marco.accame: i prefer using <= 0.
            earlyexit = 1;
            return;
        }

        // we have a packet ... give it to the ethmanager
        bool collectStatistics = (statPrintInterval > 0) ? true : false;
        ethManager->Reception(sender_addr, incoming_msg_data, incoming_msg_size, collectStatistics);
    }

}

#else

#ifdef ETHRECEIVER_ISPERIODICTHREAD

void EthReceiver::run()
{

    //yTrace();

    ACE_TCHAR     address[64];
    ethResources  *ethRes;
    ssize_t       incoming_msg_size = 0;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[ethResources::maxRXpacketsize/8]; // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const int     incoming_msg_capacity = ethResources::maxRXpacketsize;


    //ACE_Time_Value recvTimeOut;
    //recvTimeOut.set(0.010f); // timeout of socket reception is 10 milliseconds

    //double myTestTimeout = recvTimeOut.sec() + (double)recvTimeOut.msec()/1000.0f;

    int flags = 0;
#ifndef WIN32
    flags |= MSG_DONTWAIT;
#endif

    static int earlyexit = 0;

    int maxUDPpackets = EthReceiver::rateofthread*ethManager->GetNumberOfUsedBoards() + 2 + earlyexit*5; // marco.accame: set it as the number of boards plus a ... margin of 30% plus a +/- offset which depends on activity

    earlyexit = 0;

    for(int i=0; i<maxUDPpackets; i++)
    {
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size <= 0)
        {
            // marco.accame: i prefer using <= 0.
            earlyexit = 1;
            return;
        }

        ethRes = ethManager->IPtoResource(sender_addr);
        if(NULL != ethRes)
        {
            if(false == ethRes->canProcessRXpacket(incoming_msg_data, incoming_msg_size))
            {   // cannot give packet to ethresource
                yError() << "EthReceiver::run() cannot give a received packet of size" << incoming_msg_size << "to ethResources because ethResources::canProcessRXpacket() returns false.";
            }
            else
            {
                // we collect statistics only if we ever print them, thus statPrintInterval > 0
                bool collectStatistics = (statPrintInterval > 0) ? true : false;
                ethRes->processRXpacket(incoming_msg_data, incoming_msg_size, collectStatistics);
            }

        }
        else
        {
        //    sender_addr.addr_to_string(address, sizeof(address));
        //    yError() << "EthReceiver::run() cannot get a ethres associated to address" << address;
        }


    }

}

#else
    #error -> ETHRECEIVER_ISPERIODICTHREAD must be defined
#endif

#endif // 1


// eof




