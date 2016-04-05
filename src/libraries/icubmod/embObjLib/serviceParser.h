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


#include <iCub/DebugInterfaces.h>

#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include "FeatureInterface.h"  


#include <yarp/os/LogStream.h>

#include "EoBoards.h"


using namespace yarp::os;
using namespace std;


typedef enum
{
    eoas_strain                 = 0,
    eoas_mais                   = 1,
    eoas_accel_mtb_int          = 2,
    eoas_accel_mtb_ext          = 3,
    eoas_gyros_mtb_ext          = 4,
    eoas_accel_st_lis3x         = 5,
    eoas_gyros_st_l3g4200d      = 6,
    // add in here eoas_xxxnameetc
    eoas_unknown                = 254,
    eoas_none                   = 255
} eOas_sensor_t;


typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
} servConfigMais_t;


typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
    bool                    useCalibration;
} servConfigStrain_t;


typedef struct
{
    uint16_t        internal_accel[2];      /**< fill with bitwise OR of can addresses of can locations */
    uint16_t        external_accel[2];      /**< fill with bitwise OR of can addresses of can locations  */
    uint16_t        external_gyros[2];      /**< fill with bitwise OR of can addresses of can locations (can1 is lsb, can2 is msb) */
    uint8_t         datarate;               /**< it specifies the acquisition rate in ms with accepted range [10, 200]. bug: if 250 the mtb emits every 35  */
    uint8_t         filler[3];
} eOas_inertial_mtbsensorsconfig_t;


typedef struct
{
    eOmn_serv_parameter_t               ethservice;
    int                                 acquisitionrate;
    eOas_inertial_mtbsensorsconfig_t    mtbconfig;
} servConfigInertials_t;

typedef struct
{
    eObrd_cantype_t             type;
    bool                        useglobalparams;
    eOmn_canprotocolversion_t   protocol;
    eOmn_canfirmwareversion_t   firmware;
} servCanBoard_t;


typedef enum
{
    eobrd_place_loc   = 0,
    eobrd_place_can   = 1
} eObrd_place_t;


typedef union
{
    uint32_t                local;
    eOmn_serv_canlocation_t oncan;
} eObrd_address_t;

typedef struct
{
    uint8_t             place; // use enum eObrd_place_t ... so that size is 2 bytes
    eObrd_address_t     address;
} eObrd_location_t;

typedef struct
{
    char                        id[64];
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


    bool convert(ConstString const &fromstring, eOmn_serv_type_t &toservicetype, bool &formaterror);
    bool convert(ConstString const &fromstring, eObrd_cantype_t &tobrdcantype, bool &formaterror);
    bool convert(ConstString const &fromstring, bool &tobool, bool &formaterror);
    bool convert(const int number, uint8_t &tou8, bool &formaterror);
    bool convert(const int number, uint16_t &tou16, bool &formaterror);
    bool convert(ConstString const &fromstring, eOas_sensor_t &tosensortype, bool &formaterror);
    bool convert(ConstString const &fromstring, const uint8_t strsize, char *str, bool &formaterror);
    bool convert(ConstString const &fromstring, eObrd_location_t &location, bool &formaterror);

public:

    servAScollector_t           as_service;
    servASstrainSettings_t      as_strain_settings;

private:

    bool check_analog(yarp::os::Searchable &config, eOmn_serv_type_t type);

};



#endif

