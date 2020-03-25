// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale, Francesco Giovannini
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusSkin.h>

#include <iCubCanProtocol.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <yarp/os/LogStream.h>
#include <deque>


const int CAN_DRIVER_BUFFER_SIZE = 2047;

#define SPECIAL_TRIANGLE_CFG_MAX_NUM    20
#define TRIANGLE_MAX_NUM_IN_BOARD       16

#define SKIN_DEBUG 0

using namespace std;
using namespace iCub::skin::diagnostics;
using yarp::os::Bottle;
using yarp::os::Property;
using yarp::os::Value;
using yarp::dev::CanMessage;


CanBusSkin::CanBusSkin() :  PeriodicThread(0.02),
                            _verbose(false),
                            _isDiagnosticPresent(false)
{ }


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
    _cfgReader.setName(name);

    int period=config.find("period").asInt();
    setPeriod((double)period/1000.0);

    netID = config.find("canDeviceNum").asInt(); 

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

    //elements are:  // this is needed duringg initialization (readDefaultBoard)
    sensorsNum=16*12*cardId.size();
    data.resize(sensorsNum);
    data.zero();

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

    /* when the message will be available in the firmware
    if(!checkFirmwareVersion())
    {
        diagnosticAvailable = false;
    }
    else
        diagnosticAvailable = true;
    */


    if( _cfgReader.isDefaultBoardCfgPresent(config) && _cfgReader.isDefaultTriangleCfgPresent(config))
    {
        _newCfg = true;
        yInfo() << "Skin on can bus " << _canBusNum << " uses NEW configuration version!!!";
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



    /* ****** Skin diagnostics ****** */
    portSkinDiagnosticsOut.open("/diagnostics/skin/errors:o");

    //if I 'm here, config is ok ==> send message to enable transmission
    //(only in case of new configuration, skin boards need of explicit message in order to enable tx.)
    yarp::os::Time::delay(0.01);
    if(_newCfg)
    {
        uint8_t txmode = 1;
        for(size_t i=0; i< cardId.size(); i++)
        {
            if(! sendCANMessage(cardId[i], ICUBCANPROTO_POL_AS_CMD__SET_TXMODE, &txmode))
                return false;
        }
    }

    PeriodicThread::start();
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
    unsigned int             j;
    unsigned int             numofcfg;

    /* Read special board configuration */
    numofcfg = cardId.size();   //set size of my vector boardCfgList;
                                //in output the function returns number of special board cfg are in file xml
    SpecialSkinBoardCfgParam* boardCfgList = new SpecialSkinBoardCfgParam[numofcfg];

    if(!_cfgReader.readSpecialBoardCfg(config, boardCfgList, &numofcfg))
        return false;

    for(j=0; j<numofcfg; j++) //for each special board config
    {
        //check if patch exist
        if(_canBusNum != boardCfgList[j].patch)
        {
            yError() << "Skin on can bus " << _canBusNum << "configured SpecialBoardConfig on patch with a different id from my can bus " <<  _canBusNum << "patch=" <<boardCfgList[j].patch ;
            return false;
        }

        size_t boardIdx;
        //check if card address are in patch
        for(int a=boardCfgList[j].boardAddrStart; a<=boardCfgList[j].boardAddrEnd; a++)
        {
            for(boardIdx=0; boardIdx<cardId.size(); boardIdx++)
            {
                if(cardId[boardIdx]==a)//card address is in my patch
                    break;
            }
            if(boardIdx>=cardId.size())
            {
                yError() << "Skin on can bus %d" << _canBusNum << ": card with address " << a << "is not present ";
                return(false);
            }
        }
//        //uncomment  for debug only
//        yDebug() << "\n special cfg board: num " << j;
//        boardCfgList[j].debugPrint();

        //send special board cfg
        for(int listIdx=boardCfgList[j].boardAddrStart; listIdx<= boardCfgList[j].boardAddrEnd; listIdx++)
        {
            if(!sendCANMessage(listIdx, ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG, (void*)&boardCfgList[j].cfg))
                return false;

            // Init the data vector with special config values from "noLoad" param in config file.
            // This is to have a correct initilization for the data sent through yarp port
            for (unsigned int triangleId = 0; triangleId < 16; triangleId++)
            {
                unsigned int index = 16*12*boardIdx + triangleId*12;

                // Message head
                for(unsigned int k = 0; k < 12; k++)
                {
//                    yDebug() << "readNewSpecialConfiguration size is: " << data.size() << " index is " << (index+k) << " value is: " << boardCfgList[j].cfg.noLoad;
                    if((index+k) >= data.size())
                        yError() << "readNewSpecialConfiguration: index too big";
                    data[index + k] = boardCfgList[j].cfg.noLoad;
                }
            }
        }
    }

        yarp::os::Time::delay(0.01);
        /* Read special triangle configuration */
        numofcfg = SPECIAL_TRIANGLE_CFG_MAX_NUM; //set size of my vector boardCfgList;
                                    //in output the function return number of special board cfg are in file xml
        SpecialSkinTriangleCfgParam* triangleCfg = new SpecialSkinTriangleCfgParam[numofcfg];

        if(! _cfgReader.readSpecialTriangleCfg(config, &triangleCfg[0], &numofcfg))
            return false;

        for(j=0; j<numofcfg; j++)
        {
//            yDebug() << "Special triangle cfg: " << numofcfg << " on can bus: " << _canBusNum;

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
    if(!_cfgReader.readDefaultBoardCfg(config, &_brdCfg))
        return false;

    // Fill the data vector with default values from "noLoad" param in config file.
    for (size_t board_idx = 0; board_idx < cardId.size(); board_idx++)
    {
        for (unsigned int triangleId = 0; triangleId < 16; triangleId++)
        {
            int index = 16*12*board_idx + triangleId*12;

            // Message head
            for(unsigned int k = 0; k < 12; k++)
            {
//                yDebug() << "readNewConfiguration (default) size is: " << data.size() << " index is " << (index+k) << " value is: " << _brdCfg.noLoad;
                if((index+k) >= data.size())
                    yError() << "readNewConfiguration: index too big";
                data[index + k] = _brdCfg.noLoad;
            }
        }
    }


    /* send default board configuration (new configuration style)*/
    for(size_t i=0; i<cardId.size(); i++)
    {
        if(!sendCANMessage(cardId[i], ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG, (void*)&_brdCfg))
            return false;
    }

    /*read skin triangle default configuration*/
    _triangCfg.setDefaultValues();
    if(! _cfgReader.readDefaultTriangleCfg(config, &_triangCfg))
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

    yarp::os::Time::delay(0.5);

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

//    yDebug() << "msg4E_NoLoad size is " << msg4E_NoLoad.size();
//    yDebug() << "msg4E_NoLoad content is " << msg4E_NoLoad.toString();

    int numofcards = cardId.size();
    // Check parameter list length
    // 4C Message
    checkParameterListLength("4C_Timer", msg4C_Timer, numofcards, Value(0x01));
    checkParameterListLength("4C_CDCOffsetL", msg4C_CDCOffsetL, numofcards, Value(0x00));
    checkParameterListLength("4C_CDCOffsetH", msg4C_CDCOffsetH, numofcards, Value(0x20));
    checkParameterListLength("4C_TimeL", msg4C_TimeL, numofcards, Value(0x00));
    checkParameterListLength("4C_TimeH", msg4C_TimeH, numofcards, Value(0x00));
    // 4C Message
    checkParameterListLength("4E_Shift", msg4E_Shift, numofcards, Value(0x02));
    checkParameterListLength("4E_Shift3_1", msg4E_Shift3_1, numofcards, Value(0x22));
    checkParameterListLength("4E_NoLoad", msg4E_NoLoad, numofcards, Value(0xF0));
    checkParameterListLength("4E_Param", msg4E_Param, numofcards, Value(0x00));
    checkParameterListLength("4E_EnaL", msg4E_EnaL, numofcards, Value(0xFF));
    checkParameterListLength("4E_EnaH", msg4E_EnaH, numofcards, Value(0xFF));
    /* ********************************************** */

//    yDebug() << "msg4E_NoLoad size is " << msg4E_NoLoad.size();
//    yDebug() << "msg4E_NoLoad content is " << msg4E_NoLoad.toString();


    // Fill the data vector with default values from "noLoad" param in config file.
    for (size_t idx=0; idx < cardId.size(); idx++)
    {
        int board_idx = idx;
        int baseLine = msg4E_NoLoad.get(idx).asInt();

        for (unsigned int triangleId = 0; triangleId < 16; triangleId++)
        {
            unsigned int index = 16*12*board_idx + triangleId*12;

            // Message head
            for(unsigned int k = 0; k < 12; k++)
            {
//                yDebug() << "readOldConfiguration size is: " << data.size() << " index is " << (index+k) << " value is: " << baseLine;
                if((index+k) >= data.size())
                    yError() << "readOldConfiguration: index too big";
                data[index + k] = baseLine;
            }
        }
    }

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

    PeriodicThread::stop();
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
    lock_guard<mutex> lck(mtx);
    out=data;
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
    if(!_newCfg)
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

    lock_guard<mutex> lck(mtx);

    unsigned int canMessages = 0;
    bool res = pCanBus->canRead(inBuffer, CAN_DRIVER_BUFFER_SIZE, &canMessages);

    if (!res) 
    {
        yError("CanBusSkin: CanRead failed");
    } 
    else 
    {
        // Allocate error vector
        errors.resize(canMessages);

        for (unsigned int i = 0; i < canMessages; i++) {

            CanMessage &msg = inBuffer[i];

            unsigned int msgid = msg.getId();
            unsigned int id = (msgid & 0x00F0) >> 4;
            unsigned int sensorId = (msgid & 0x000F);
            unsigned int msgType = (int) msg.getData()[0];
            int len = msg.getLen();

#if 0
            cout << "DEBUG: CanBusSkin: Board ID (" << id <<"): Sensor ID (" << sensorId << "): " 
                << "Message type (" << std::uppercase << std::showbase << std::hex << (int) msg.getData()[0] << ") " 
                << std::nouppercase << std::noshowbase << std::dec << " Length (" << len << "): ";
            cout << "Content: " << std::uppercase << std::showbase << std::hex;
            for (int k = 0; k < len; ++k) {
                cout << (int) msg.getData()[k] << " ";
            }
            cout << "\n" << std::nouppercase << std::noshowbase << std::dec;
#endif

            for (size_t j = 0; j < cardId.size(); j++) {
                if (id == cardId[j]) {
                    int index = 16*12*j + sensorId*12;
                    
                    if (msgType == 0x40) {
                        // Message head
                        for(int k = 0; k < 7; k++) {
                            data[index + k] = msg.getData()[k + 1];
                        }
                    } else if (msgType == 0xC0) {
                        // Message tail
                        for(int k = 0; k < 5; k++) {
                            data[index + k + 7] = msg.getData()[k + 1];
                        }

                        // Skin diagnostics
                        if (_brdCfg.useDiagnostic)  // if user requests to check the diagnostic
                        {
                            if (len == 8)   // firmware is sending diagnostic info
                            {
                                _isDiagnosticPresent = true;

                                // Get error code head and tail
                                short head = msg.getData()[6];
                                short tail = msg.getData()[7];
                                int fullMsg = (head << 8) | (tail & 0xFF);

                                // Store error message
                                errors[i].net = netID;
                                errors[i].board = id;
                                errors[i].sensor = sensorId;
                                errors[i].error = fullMsg;

                                if(fullMsg != SkinErrorCode::StatusOK)
                                {
                                    yError() << "canBusSkin error code: " <<
                                                "canDeviceNum: " << errors[i].net <<
                                                "board: " <<  errors[i].board <<
                                                "sensor: " << errors[i].sensor <<
                                                "error: " << iCub::skin::diagnostics::printErrorCode(errors[i].error).c_str();

                                    yarp::sig::Vector &out = portSkinDiagnosticsOut.prepare();
                                    out.clear();

                                    out.push_back(errors[i].net);
                                    out.push_back(errors[i].board);
                                    out.push_back(errors[i].sensor);
                                    out.push_back(errors[i].error);

                                    portSkinDiagnosticsOut.write(true);

                                }
                            }
                            else
                            {
                                _isDiagnosticPresent = false;
                            }
                        }
                    }
                }
          //    else
          //        {
          //            yError()<< "Error: skin received malformed message\n";
          //        }
            }
        }
    }
}

void CanBusSkin::threadRelease()
{
#if SKIN_DEBUG
    yDebug<<"CanBusSkin Thread releasing...";    
    yDebug<<"... done.";
#endif
}


/* *********************************************************************************************************************** */
/* ******* Diagnose skin errors.                                            ********************************************** */
bool CanBusSkin::diagnoseSkin(void) {
    using iCub::skin::diagnostics::DetectedError;   // FG: Skin diagnostics errors
    using yarp::sig::Vector;

    // Write errors to port
    for (size_t i = 0; i < errors.size(); ++i)
    {
        Vector &out = portSkinDiagnosticsOut.prepare();
        out.clear();

        out.push_back(errors[i].net);
        out.push_back(errors[i].board);
        out.push_back(errors[i].sensor);
        out.push_back(errors[i].error);

        portSkinDiagnosticsOut.write(true);
    }

    return true;
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Converts input parameter bottle into a std vector.               ********************************************** */
void CanBusSkin::checkParameterListLength(const string &i_paramName, Bottle &i_paramList, const int &i_length, const Value &i_defaultValue) {
    if ((i_paramList.isNull()) || (i_paramList.size() != i_length)) {
        yWarning() << "CanBusSkin: The number of " << i_paramName << " parameters provided is different than the number of CAN IDs for this BUS. Check your parameters.";
        yWarning() << "CanBusSkin: Using default values for " << i_paramName << " parameters.";

        size_t i;
        for (i= i_paramList.size() ; i <= (unsigned int) i_length; ++i) {
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
            << msg4C_TimeL.get(i).asInt() << " " << msg4C_TimeH.get(i).asInt() << std::dec << std::noshowbase
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
            << msg4E_EnaH.get(i).asInt() << " " << (int) msg4e.getData()[7] << std::dec << std::noshowbase
            << ". \n";
            yDebug << "CanBusSkin: Output parameters (msg 4E) are: " << std::hex << std::showbase
            << (int) msg4e.getData()[0] << " " << (int) msg4e.getData()[1] << " " << (int) msg4e.getData()[2] << " " 
            << (int) msg4e.getData()[3] << " " << (int) msg4e.getData()[4] << " " << (int) msg4e.getData()[5] << " " 
            << (int) msg4e.getData()[6] << " " << (int) msg4e.getData()[7] << std::dec << std::noshowbase
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


/************************************************************************************************************************ */

/// When the firmware will implement this.
bool CanBusSkin::checkFirmwareVersion(void)
{
    bool firmware_version_ok = true;

    for (size_t i = 0; i < cardId.size(); ++i)
    {
#if 1 //SKIN_DEBUG
        yDebug("CanBusSkin: check firmware version for board ID: %d. \n",cardId[i]);
#endif

        unsigned int canMessages=0;
        unsigned int id = (ICUBCANPROTO_CLASS_POLLING_ANALOGSENSOR << 8) + (0x00 /*master address*/ << 4) + (cardId[i]);

        // enable reading of messages from the bootloader class (already done?)
        unsigned int recvId = ( (ICUBCANPROTO_CLASS_POLLING_ANALOGSENSOR << 8) + (cardId[i] << 4) );
        pCanBus->canIdAdd(recvId);

        //CanMessage &txBuffer=outBuffer[0];
        // Send command
        outBuffer[0].setId(id);
        outBuffer[0].setLen(1);
        outBuffer[0].getData()[0]= ICUBCANPROTO_POL_MC_CMD__GET_FIRMWARE_VERSION; // message for firmware version

        if (!pCanBus->canWrite(outBuffer, 1, &canMessages))
        {
            yError() << "CanBusSkin: Could not write to the CAN interface. \n";
            return false;
        }


        // pause
        yarp::os::Time::delay(0.3);

        //riceve la risposta
        bool done=false;
        unsigned int read_messages=0;
        while(!done)
        {
            bool res = pCanBus->canRead(inBuffer, CAN_DRIVER_BUFFER_SIZE, &read_messages);
            if (!res)
            {
                yError() << "CanBusSkin: CanRead failed \n";
            }

            //Timeout: no answers
            if (read_messages==0)
            {
                yError ("No answers\n");
                return false;
            }

            //One (or more) answers received
            //Counts the number of the boards
            for (i=0; i<read_messages; i++)
            {
                ///////
#if 0
                fprintf(stderr, "%.4x ", inBuffer[i].getId());
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[0]);
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[1]);
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[2]);
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[3]);
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[4]);
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[5]);
                fprintf(stderr, "%.2x ", inBuffer[i].getData()[6]);
                fprintf(stderr, "%.2x\n", inBuffer[i].getData()[7]);
#endif
                ////////////////

                int  type;
                int  version;
                int  release;
                int  build;

                if (0 /* check the message received is right*/ )
                    {
                        /* parse it somehow */
                        type    = 0; // inBuffer[i].getData()[1];
                        version = 0; // inBuffer[i].getData()[2];
                        release = 0; // inBuffer[i].getData()[3];
                        build   = 0; // inBuffer[i].getData()[4];

                        fprintf(stderr, "type %d\n", type);
                        fprintf(stderr, "version %d\n", version);
                        fprintf(stderr, "release %d\n", release);
                        fprintf(stderr, "build %d\n", build);

                        if(version > 2) { firmware_version_ok = true; }
                        else
                            if(version < 2)  { firmware_version_ok = false; }
                        else
                        {
                            if(release > 16) { firmware_version_ok = true; }
                            else
                                if(release < 16)  { firmware_version_ok = false; }
                            else
                                {
                                    if(build >= 25) { firmware_version_ok = true; }
                                    else
                                        { firmware_version_ok = false; }
                                }
                        }
                        done = true;
                    }

                if(!firmware_version_ok)
                {
                    yWarning() << "Diagnostic check was enabled in the config file, but the firwmare does not support it yet! \
                                   Please verify that the firmware version is at least 2.10.15 as shown in the canLoader\
                                   disabling parsing of diagnostic messages";
                }
                 else
                    yInfo() << "Firwmare check for skin diagnostic messages ok";

            }
        }
    }
    return firmware_version_ok;
}
