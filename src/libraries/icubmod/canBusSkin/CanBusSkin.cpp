// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale, Francesco Giovannini
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusSkin.h>

#include <canProtocolLib/iCubCanProtocol.h>
#include <yarp/os/Time.h>
#include <iostream>
#include <Debug.h>

const int CAN_DRIVER_BUFFER_SIZE = 2047;

#define SPECIAL_TRIANGLE_CFG_MAX_NUM    20
#define TRIANGLE_MAX_NUM_IN_BOARD       16

#define SKIN_DEBUG 0

using namespace std;
using yarp::os::Bottle;
using yarp::os::Property;
using yarp::os::Value;
using yarp::dev::CanMessage;
using yarp::os::Time;



bool CanBusSkin::open(yarp::os::Searchable& config)
{
    bool ret=true;
#if SKIN_DEBUG
    fprintf(stderr, "%s\n", config.toString().c_str());
#endif

    bool correct=true;
    // Mandatory parameters for all skin patches
    correct &= config.check("canbusDevice");
    correct &= config.check("canDeviceNum");
    correct &= config.check("skinCanIds");
    correct &= config.check("period");

    if (!correct)
    {
         yError()<< "Insufficient parameters to CanBusSkin.";
         return false;
     }


    _canBusNum = config.find("canDeviceNum").asInt();
    char name[80];
    sprintf(name, "canSkin on bus %d", _canBusNum);
    _cfgReader = new SkinConfigReader(name);

    int period=config.find("period").asInt();
    setRate(period);


    Bottle ids=config.findGroup("skinCanIds").tail();
    if (ids.size()>1)
    {
        yWarning()<< "CanBusSkin id list contains more than one entry -> devices will be merged. ";
    }

    for (int i=0; i<ids.size(); i++)
    {

        int id = ids.get(i).asInt();
        cardId.push_back (id);
        #if SKIN_DEBUG
            yDebug<< "Id reading from: " << id;
        #endif
    }

    Property prop;
    prop.put("device", config.find("canbusDevice").asString().c_str());
    prop.put("physDevice", config.find("physDevice").asString().c_str());
    prop.put("canTxTimeout", 500);
    prop.put("canRxTimeout", 500);
    prop.put("canDeviceNum", config.find("canDeviceNum").asInt());
    prop.put("canMyAddress", 0);
    prop.put("canTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    prop.put("canRxQueueSize", CAN_DRIVER_BUFFER_SIZE);

    pCanBus=0;
    pCanBufferFactory=0;

    driver.open(prop);
    if (!driver.isValid())
    {
        yError()<< "Error opening PolyDriver check parameters";
        return false;
    }

    driver.view(pCanBus);
    
    if (!pCanBus)
    {
        yError()<< "Error opening /ecan device not available";
        return false;
    }

    driver.view(pCanBufferFactory);
    pCanBus->canSetBaudRate(0); //default 1MB/s


    outBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);

    if( _cfgReader->isDefaultBoardCfgPresent(config) && _cfgReader->isDefaultTriangleCfgPresent(config))
    {
        _newCfg = true;
        yWarning() << "Skin on can bus " << _canBusNum << " uses new configuration version!!!";
        ret = readNewConfiguration(config);
    }
    else
    {
        _newCfg = false;
        yWarning() << "Skin on can bus " << _canBusNum << " uses old configuration version!!!";
        ret = readOldConfiguration(config);
    }

    if(!ret)
    {
        yError() << "Error reading config";
        if (pCanBufferFactory)
        {
            pCanBufferFactory->destroyBuffer(inBuffer);
            pCanBufferFactory->destroyBuffer(outBuffer);
        }
        driver.close();
        return false;
    }

    //set filter
    uint8_t can_msg_class = 0;
    if(_newCfg)
    {
        can_msg_class = ICUBCANPROTO_CLASS_PERIODIC_SKIN;
    }
    else
    {
        can_msg_class = ICUBCANPROTO_CLASS_PERIODIC_ANALOGSENSOR;
    }
    for (unsigned int i=0; i<cardId.size(); i++)
            for (unsigned int id=0; id<16; ++id)
            {
                //yDebug() << "---------- aggiungo id = " << (can_msg_class << 8)+(cardId[i]<<4)+id;
                pCanBus->canIdAdd((can_msg_class << 8)+(cardId[i]<<4)+id);
            }

    //elements are:
    sensorsNum=16*12*cardId.size();
    data.resize(sensorsNum);
    data.zero();

    //if I 'm here, config is ok ==> send message to enable transmission
    //(only in case of new configuration, skin boards need of explicit message in order to enable tx.)
    Time::delay(0.01);
    if(_newCfg)
    {
        uint8_t txmode = 1;
        for(size_t i=0; i< cardId.size(); i++)
        {
            if(! sendCANMessage(cardId[i], ICUBCANPROTO_POL_AS_CMD__SET_TXMODE, &txmode))
                return false;
        }
    }


    RateThread::start();
    return true;
}

bool CanBusSkin::sendCANMessage(uint8_t destAddr, uint8_t command, void *data)
{
    int len = 0;
    unsigned int msgSent = 0;
    CanMessage &msg = outBuffer[0];
    unsigned int id = 0;
    id= ((ICUBCANPROTO_CLASS_POLLING_ANALOGSENSOR << 8 ) | (destAddr & 0x0f));

    msg.setId(id);
    //set command in message
    msg.getData()[0] = command;

    switch(command)
    {
        case ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG: //77
        {
            SkinBoardCfgParam *brdCfg = (SkinBoardCfgParam *)data;
            msg.getData()[1] = 0;
            msg.getData()[1] = brdCfg->skinType & 0x0f;
            msg.getData()[2] = brdCfg->period;
            msg.getData()[3] = brdCfg->noLoad;
            len = 4;
        }break;

        case ICUBCANPROTO_POL_SK_CMD__SET_TRIANG_CFG: //80
        {
            SpecialSkinTriangleCfgParam *trCfg = (SpecialSkinTriangleCfgParam *)data;
            msg.getData()[1] = trCfg->triangleStart;
            msg.getData()[2] = trCfg->triangleEnd;
            msg.getData()[3] = trCfg->cfg.shift;
            msg.getData()[4] = 0;

            if(trCfg->cfg.enabled)
                msg.getData()[4] = 1;

            msg.getData()[5] = trCfg->cfg.cdcOffset & 0xff;
            msg.getData()[6] = (trCfg->cfg.cdcOffset & 0xff00) >> 8;
            len = 7;
        }break;
        case ICUBCANPROTO_POL_AS_CMD__SET_TXMODE: //7
        {
            uint8_t *txmode = (uint8_t *)data;
            if(*txmode)
            {
                msg.getData()[1] = 0;
            }
            else
            {
                msg.getData()[1] = 1;
            }
            len = 2;
        }break;
        default:
        {
            len = 0;
        }
    };



    if(0==len)
    {
        yWarning() << "skin on can bus " << _canBusNum << ":try to send a unknown message(command id=" <<command << ")";
    }
    msg.setLen(len);

   if (!pCanBus->canWrite(outBuffer, 1, &msgSent))
   {
       yError() << "skin on can bus " << _canBusNum << ": Could not write to the CAN interface.";
       return false;
   }
   if(msgSent != 1)
       return false;

    return true;
}



bool CanBusSkin::readNewSpecialConfiguration(yarp::os::Searchable& config)
{
    Bottle          bNumOfset;
    int             numOfSets;
    int             j;
    int             numofcfg;


    /* Read special board configuration */
    numofcfg = cardId.size();   //set size of my vector boardCfgList;
                                //in output the function returns number of special board cfg are in file xml
    SpecialSkinBoardCfgParam* boardCfgList = new SpecialSkinBoardCfgParam[numofcfg];

    if(!_cfgReader->readSpecialBoardCfg(config, boardCfgList, &numofcfg))
        return false;

    for(j=0; j<numofcfg; j++) //for each special board config
    {
        //check if patch exist
        if(_canBusNum != boardCfgList[j].patch)
        {
            yError() << "Skin on can bus " << _canBusNum << "configured SpecialBoardConfig on patch with a different id from my can bus " <<  _canBusNum << "patch=" <<boardCfgList[j].patch ;
            return false;
        }

        //check if card address are in patch
        for(int a=boardCfgList[j].boardAddrStart; a<=boardCfgList[j].boardAddrEnd; a++)
        {
            size_t i;
            for(i=0; i<cardId.size(); i++)
            {
                if(cardId[i]==a)//card address is in my patch
                    break;
            }
            if(i>=cardId.size())
            {
                yError() << "Skin on can bus %d" << _canBusNum << ": card with address " << a << "is not present ";
                return(false);
            }
        }
//        //uncomment  for debug only
//        yDebug() << "\n special cfg board: num " << j;
//        boardCfgList[j].debugPrint();

        //send special board cfg
        for(int k=boardCfgList[j].boardAddrStart; k<= boardCfgList[j].boardAddrEnd; k++)
        {
            if(!sendCANMessage(k, ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG, (void*)&boardCfgList[j].cfg))
                return false;
        }
    }

        Time::delay(0.01);
        /* Read special traingle configuration */
        numofcfg = SPECIAL_TRIANGLE_CFG_MAX_NUM; //set size of my vector boardCfgList;
                                    //in output the function return number of special board cfg are in file xml
        SpecialSkinTriangleCfgParam* triangleCfg = new SpecialSkinTriangleCfgParam[numofcfg];

        if(! _cfgReader->readSpecialTriangleCfg(config, &triangleCfg[0], &numofcfg))
            return false;

        for(j=0; j<numofcfg; j++)
        {
            yError() << "special traingle cfg: " << numofcfg;

            //check if patch exist
            if(_canBusNum != triangleCfg[j].patch)
            {
                yError() << "Skin on can bus " << _canBusNum << "configured SpecialTriangleConfig on patch with a different id from my can bus" <<  _canBusNum << "patch=" <<triangleCfg[j].patch ;
                return false;
            }

            for(size_t i=0; i<cardId.size(); i++)
            {
                if(cardId[i]==triangleCfg[j].boardAddr)//card address is in my patch
                    break;

                if(i>=cardId.size())
                {
                    yError() << "Skin on can bus %d" << _canBusNum << ": card with address " << triangleCfg[j].boardAddr << "is not present ";
                    return(false);
                }

            }
//            //uncomment  for debug only
//            yDebug() << "\n Special triangle cfg num " << j;
//            triangleCfg[j].debugPrint();

            //send triangle cfg
            if(!sendCANMessage(triangleCfg[j].boardAddr, ICUBCANPROTO_POL_SK_CMD__SET_TRIANG_CFG, (void*)&triangleCfg[j]))
                    return false;
        }

    return true;
}

bool CanBusSkin::readNewConfiguration(yarp::os::Searchable& config)
{
    /*read skin board default configuration*/
    _brdCfg.setDefaultValues();
    if(!_cfgReader->readDefaultBoardCfg(config, &_brdCfg))
        return false;

    /* send default board configuration (new configuration style)*/
    for(size_t i=0; i<cardId.size(); i++)
    {
        if(!sendCANMessage(cardId[i], ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG, (void*)&_brdCfg))
            return false;
    }

    /*read skin triangle default configuration*/
    _triangCfg.setDefaultValues();
    if(! _cfgReader->readDefaultTriangleCfg(config, &_triangCfg))
        return false;
    SpecialSkinTriangleCfgParam spTrCfg;
    spTrCfg.cfg = _triangCfg;
    spTrCfg.triangleStart = 0;
    spTrCfg.triangleEnd = TRIANGLE_MAX_NUM_IN_BOARD-1; //in each board there are 16 triagle max, their id goes from 0 to TRIANGLE_MAX_NUM_IN_BOARD-1;

    //send default board and triangle configuration (new configuration style)
    for(size_t i=0; i<cardId.size(); i++)
    {
        spTrCfg.boardAddr = cardId[i];
        if(!sendCANMessage(cardId[i], ICUBCANPROTO_POL_SK_CMD__SET_TRIANG_CFG, (void*)&spTrCfg))
            return false;
    }

    Time::delay(0.5);

    /*read and send special cfg for board and triangle */
    if(!readNewSpecialConfiguration(config))
        return false;

    return true;
}


bool CanBusSkin::readOldConfiguration(yarp::os::Searchable& config)
{
    /* ******* Parameters for hand skin patches. If these are not specified, default values are used. ******* */
    // Initialise parameter vectors

    // 4C Message
    msg4C_Timer = config.findGroup("4C_Timer").tail();                // 4C_Timer
    msg4C_CDCOffsetL = config.findGroup("4C_CDCOffsetL").tail();      // 4C_CDCOffsetL
    msg4C_CDCOffsetH = config.findGroup("4C_CDCOffsetH").tail();      // 4C_CDCOffsetR
    msg4C_TimeL = config.findGroup("4C_TimeL").tail();                // 4C_TimeL
    msg4C_TimeH = config.findGroup("4C_TimeH").tail();                // 4C_TimeH
    // 4E Message
    msg4E_Shift = config.findGroup("4E_Shift").tail();                // 4E_Shift
    msg4E_Shift3_1 = config.findGroup("4E_Shift3_1").tail();          // 4E_Shift3_1
    msg4E_NoLoad = config.findGroup("4E_NoLoad").tail();              // 4E_NoLoad
    msg4E_Param = config.findGroup("4E_Param").tail();                // 4E_Param
    msg4E_EnaL = config.findGroup("4E_EnaL").tail();                  // 4E_EnaL
    msg4E_EnaH = config.findGroup("4E_EnaH").tail();                  // 4E_EnaH

    int numofcards = cardId.size();
    // Check parameter list length
    // 4C Message
    checkParameterListLength("4C_Timer", msg4C_Timer, numofcards, 0x01);
    checkParameterListLength("4C_CDCOffsetL", msg4C_CDCOffsetL, numofcards, 0x00);
    checkParameterListLength("4C_CDCOffsetH", msg4C_CDCOffsetH, numofcards, 0x20);
    checkParameterListLength("4C_TimeL", msg4C_TimeL, numofcards, 0x00);
    checkParameterListLength("4C_TimeH", msg4C_TimeH, numofcards, 0x00);
    // 4C Message
    checkParameterListLength("4E_Shift", msg4E_Shift, numofcards, 0x02);
    checkParameterListLength("4E_Shift3_1", msg4E_Shift3_1, numofcards, 0x22);
    checkParameterListLength("4E_NoLoad", msg4E_NoLoad, numofcards, 0xF0);
    checkParameterListLength("4E_Param", msg4E_Param, numofcards, 0x00);
    checkParameterListLength("4E_EnaL", msg4E_EnaL, numofcards, 0xFF);
    checkParameterListLength("4E_EnaH", msg4E_EnaH, numofcards, 0xFF);
    /* ********************************************** */


    if(sendCANMessage4C())
    {
        return sendCANMessage4E();
    }
    else
    {
        return false;
    }

}
bool CanBusSkin::close()
{

    if(_newCfg)
    {
        uint8_t txmode = 0;
        for(size_t i=0; i< cardId.size(); i++)
        {
            sendCANMessage(cardId[i], ICUBCANPROTO_POL_AS_CMD__SET_TXMODE, &txmode);
        }
    }

    RateThread::stop();
    if (pCanBufferFactory) 
    {
        pCanBufferFactory->destroyBuffer(inBuffer);
        pCanBufferFactory->destroyBuffer(outBuffer);
    }
    driver.close();
    return true;
}

int CanBusSkin::read(yarp::sig::Vector &out) 
{
    mutex.wait();
    out=data;
    mutex.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

int CanBusSkin::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;;
}

int CanBusSkin::getChannels()
{
    return sensorsNum;
}

int CanBusSkin::calibrateSensor() {
    sendCANMessage4C();

    return AS_OK;
}

int CanBusSkin::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return calibrateSensor();
}

int CanBusSkin::calibrateSensor(const yarp::sig::Vector& v)
{
    return 0;
}

int CanBusSkin::calibrateChannel(int ch)
{
    return 0;
}

bool CanBusSkin::threadInit() {
//    if(sendCANMessage4C()) {
//        return sendCANMessage4E();
//    } else {
//        return false;
//    }
    return true;
}

void CanBusSkin::run() {    
    mutex.wait();

    unsigned int canMessages=0;

    bool res=pCanBus->canRead(inBuffer,CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
    {
        std::cerr<<"canRead failed\n";
    }

    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid=msg.getId();
        unsigned int id;
        unsigned int sensorId;
        id=(msgid & 0x00f0)>>4;
        sensorId=msgid&0x000f;

        unsigned int type=msg.getData()[0]&0x80;
        int len=msg.getLen();

        for (size_t i=0; i<cardId.size(); i++)
        {
            if (id==cardId[i])
            {
                int index=16*12*i + sensorId*12;
                
                if (type)
                    {
                        for(int k=0;k<5;k++)
                            data[index+k+7]=msg.getData()[k+1];
                    }
                else 
                    {
                        for(int k=0;k<7;k++)
                            data[index+k]=msg.getData()[k+1];
                    }
          //    else
          //        {
          //            yError()<< "Error: skin received malformed message\n";
          //        }
            }
        }
    }

    mutex.post();
}

void CanBusSkin::threadRelease()
{
#if SKIN_DEBUG
    yDebug<<"CanBusSkin Thread releasing...";    
    yDebug<<"... done.";
#endif
}


/* *********************************************************************************************************************** */
/* ******* Converts input parameter bottle into a std vector.               ********************************************** */
void CanBusSkin::checkParameterListLength(const string &i_paramName, Bottle &i_paramList, const int &i_length, const Value &i_defaultValue) {
    if ((i_paramList.isNull()) || (i_paramList.size() != i_length)) {
        yWarning() << "CanBusSkin: The number of " << i_paramName << " parameters provided is different than the number of CAN IDs for this BUS. Check your parameters.";
        yWarning() << "CanBusSkin: Using default values for " << i_paramName << " parameters.";

        size_t i;
        for (i= i_paramList.size() ; i <= i_length; ++i) {
            i_paramList.add(i_defaultValue);
        }
    }
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Sends CAN setup message type 4C.                                 ********************************************** */
bool CanBusSkin::sendCANMessage4C(void) {
    for (size_t i = 0; i < cardId.size(); ++i) {
#if SKIN_DEBUG
        printf("CanBusSkin: Thread initialising board ID: %d. \n",cardId[i]);
        printf("CanBusSkin: Sending 0x4C message to skin boards. \n");
#endif 

        unsigned int canMessages=0;
        unsigned id = 0x200 + cardId[i];
       
        CanMessage &msg4c=outBuffer[0];
        msg4c.setId(id);
        msg4c.getData()[0] = 0x4C; // Message type
        msg4c.getData()[1] = 0x01; 
        msg4c.getData()[2] = 0x01; 
        msg4c.getData()[3] = msg4C_Timer.get(i).asInt();
        msg4c.getData()[4] = msg4C_CDCOffsetL.get(i).asInt();
        msg4c.getData()[5] = msg4C_CDCOffsetH.get(i).asInt();
        msg4c.getData()[6] = msg4C_TimeL.get(i).asInt();
        msg4c.getData()[7] = msg4C_TimeH.get(i).asInt();
        msg4c.setLen(8);

#if SKIN_DEBUG
        yDebug << "CanBusSkin: Input parameters (msg 4C) are: " << std::hex << std::showbase
            << (int) msg4c.getData()[0] << " " << (int) msg4c.getData()[1] << " " << (int) msg4c.getData()[2] << " "
            << msg4C_Timer.get(i).asInt() << " " << msg4C_CDCOffsetL.get(i).asInt() << " " << msg4C_CDCOffsetH.get(i).asInt() << " " 
            << msg4C_TimeL.get(i).asInt() << " " << msg4C_TimeH.get(i).asInt()
            << ". \n";
        yDebug << "CanBusSkin: Output parameters (msg 4C) are: " << std::hex << std::showbase
            << (int) msg4c.getData()[0] << " " << (int) msg4c.getData()[1] << " " << (int) msg4c.getData()[2] << " " 
            << (int) msg4c.getData()[3] << " " << (int) msg4c.getData()[4] << " " << (int) msg4c.getData()[5] << " " 
            << (int) msg4c.getData()[6] << " " << (int) msg4c.getData()[7]
            << ". \n";
#endif

        if (!pCanBus->canWrite(outBuffer, 1, &canMessages)) {
            yError() << "CanBusSkin: Could not write to the CAN interface. \n";
            return false;
        }
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Sends CAN setup message type 4E.                                 ********************************************** */
bool CanBusSkin::sendCANMessage4E(void) {
    // Send 0x4E message to modify offset
    for (size_t i = 0; i < cardId.size(); ++i) {
        // FG: Sending 4E message to all MTBs
#if SKIN_DEBUG
        printf("CanBusSkin: Thread initialising board ID: %d. \n", cardId[i]);
        printf("CanBusSkin: Sending 0x4E message to skin boards. \n");
#endif 
    
        unsigned int canMessages = 0;
        unsigned id = 0x200 + cardId[i];
                
        CanMessage &msg4e = outBuffer[0];
        msg4e.setId(id);
        msg4e.getData()[0] = 0x4E; // Message type
        msg4e.getData()[1] = msg4E_Shift.get(i).asInt(); 
        msg4e.getData()[2] = msg4E_Shift3_1.get(i).asInt(); 
        msg4e.getData()[3] = msg4E_NoLoad.get(i).asInt();
        msg4e.getData()[4] = msg4E_Param.get(i).asInt();
        msg4e.getData()[5] = msg4E_EnaL.get(i).asInt();
        msg4e.getData()[6] = msg4E_EnaH.get(i).asInt();
        msg4e.getData()[7] = 0x0A;
        msg4e.setLen(8);
            
#if SKIN_DEBUG
            yDebug << "CanBusSkin: Input parameters (msg 4E) are: " << std::hex << std::showbase
            << (int) msg4e.getData()[0] << " " << msg4E_Shift.get(i).asInt() << " " << msg4E_Shift3_1.get(i).asInt() << " " 
            << msg4E_NoLoad.get(i).asInt() << " " << msg4E_Param.get(i).asInt() << " " << msg4E_EnaL.get(i).asInt() << " " 
            << msg4E_EnaH.get(i).asInt() << " " << (int) msg4e.getData()[7]
            << ". \n";
            yDebug << "CanBusSkin: Output parameters (msg 4E) are: " << std::hex << std::showbase
            << (int) msg4e.getData()[0] << " " << (int) msg4e.getData()[1] << " " << (int) msg4e.getData()[2] << " " 
            << (int) msg4e.getData()[3] << " " << (int) msg4e.getData()[4] << " " << (int) msg4e.getData()[5] << " " 
            << (int) msg4e.getData()[6] << " " << (int) msg4e.getData()[7]
            << ". \n";
#endif

        if (!pCanBus->canWrite(outBuffer, 1, &canMessages)) {
            yError() <<  "CanBusSkin: Could not write to the CAN interface.";
            return false;
        }
    }

    return true;
}
/* *********************************************************************************************************************** */
