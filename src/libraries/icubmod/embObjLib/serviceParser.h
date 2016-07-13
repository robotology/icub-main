// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules
 * \defgroup ServiceParser ServiceParser
 *
 * To Do: add description
 *
 */

#ifndef __serviceParser_h__
#define __serviceParser_h__


#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include "FeatureInterface.h"


#include <yarp/os/LogStream.h>

#include "EoBoards.h"
#include "EoManagement.h"
#include "EoAnalogSensors.h"

using namespace yarp::os;
using namespace std;


//#define SERVICE_PARSER_USE_MC

typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
    string                  nameOfMais;
} servConfigMais_t;


typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
    bool                    useCalibration;
    string                  nameOfStrain;
} servConfigStrain_t;


typedef struct
{
    eOmn_serv_parameter_t               ethservice;
    int                                 acquisitionrate;
    vector<eOas_inertial_descriptor_t>  inertials;
    vector<string>                      id;
} servConfigInertials_t;


#if defined(SERVICE_PARSER_USE_MC)
typedef struct
{
    eOmn_serv_parameter_t               ethservice;
    uint32_t                            id; // only for test: this param contains num of board
} servConfigMC_t;
#endif

struct servCanBoard_t
{
    eObrd_cantype_t             type;
    eObrd_protocolversion_t     protocol;
    eObrd_firmwareversion_t     firmware;
    servCanBoard_t() { type = eobrd_cantype_none; protocol.major = 0; protocol.minor = 0; firmware.major = 0; firmware.minor = 0; firmware.build = 0; }
};

struct servBoard_t
{
    eObrd_type_t                type;
    eObrd_protocolversion_t     protocol;
    eObrd_firmwareversion_t     firmware;
    servBoard_t() { type = eobrd_none; protocol.major = 0; protocol.minor = 0; firmware.major = 0; firmware.minor = 0; firmware.build = 0; }
};


typedef struct
{
    string                      id;
    eOas_sensor_t               type;
    eObrd_location_t            location;
} servAnalogSensor_t;


typedef struct
{
    vector<servCanBoard_t>      canboards;
    vector<servAnalogSensor_t>  sensors;
} servASproperties_t;


typedef struct
{
    uint16_t                    acquisitionrate;
    vector<servAnalogSensor_t>  enabledsensors;
} servASsettings_t;


typedef struct
{
    eOmn_serv_type_t            type;
    servASproperties_t          properties;
    servASsettings_t            settings;
} servAScollector_t;


typedef struct
{
    bool                        useCalibration;
} servASstrainSettings_t;


// todo: add definition of static const array of strings containing the names of boards, sensors, etc.


// -- class ServiceParser

class ServiceParser
{
public:

    ServiceParser();
    ~ServiceParser(){}

public:

    bool parseService(yarp::os::Searchable &config, servConfigMais_t& maisconfig);
    bool parseService(Searchable &config, servConfigStrain_t &strainconfig);
    bool parseService(Searchable &config, servConfigInertials_t &inertialsconfig);

#if defined(SERVICE_PARSER_USE_MC)
    bool parseService(Searchable &config, servConfigMC_t &mcconfig);
#endif

    bool convert(ConstString const &fromstring, eOmn_serv_type_t &toservicetype, bool &formaterror);
    bool convert(ConstString const &fromstring, eObrd_type_t& tobrdtype, bool& formaterror);
    bool convert(ConstString const &fromstring, eObrd_cantype_t &tobrdcantype, bool &formaterror);
    bool convert(ConstString const &fromstring, bool &tobool, bool &formaterror);
    bool convert(const int number, uint8_t &tou8, bool &formaterror);
    bool convert(const int number, uint16_t &tou16, bool &formaterror);
    bool convert(ConstString const &fromstring, eOas_sensor_t &tosensortype, bool &formaterror);
    bool convert(ConstString const &fromstring, string &str, bool &formaterror);
    bool convert(ConstString const &fromstring, const uint8_t strsize, char *str, bool &formaterror);
    bool convert(ConstString const &fromstring, eObrd_location_t &location, bool &formaterror);

    bool convert(eObrd_location_t const &loc, char *str, int len);
    bool convert(eObrd_canlocation_t const &canloc, char *str, int len);

    bool convert(eObrd_protocolversion_t const &prot, char *str, int len);
    bool convert(eObrd_firmwareversion_t const &firm, char *str, int len);

public:

    servAScollector_t           as_service;
    servASstrainSettings_t      as_strain_settings;

private:

    bool check_analog(yarp::os::Searchable &config, eOmn_serv_type_t type);

};



#endif

