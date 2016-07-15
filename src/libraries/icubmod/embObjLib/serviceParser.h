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


#define SERVICE_PARSER_USE_MC

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



#if defined(SERVICE_PARSER_USE_MC)

typedef struct
{
    eOmn_servMC_controller_type_t   type;
    vector<double>                  matrixJ2M;    // 16 numbers
    vector<double>                  matrixM2J;    // 16 numbers
    vector<double>                  matrixE2J;    // 16 numbers
} servMCcontroller_t;

typedef enum
{
    servMC_actutator_place_local = 0,
    servMC_actutator_place_can = 1
}servMC_actutator_place_t;

typedef enum
{
    servMC_board_connector_P0 = 0,
    servMC_board_connector_P1 = 1,
    servMC_board_connector_P2 = 2,
    servMC_board_connector_P3 = 3,
    servMC_board_connector_P4 = 4,
    servMC_board_connector_P5 = 5,
    servMC_board_connector_P6 = 6,
    servMC_board_connector_P7 = 7,
    servMC_board_connector_P8 = 8,
    servMC_board_connector_P9 = 9,
    servMC_board_connector_P10 = 10,
    servMC_board_connector_P11 = 11,
    servMC_board_connector_P12 = 12,
    servMC_board_connector_PMAX = 13
}servMC_board_connector_t;


typedef struct
{
    uint8_t place :2;        //use servMC_actutator_place_t
    uint8_t port  : 1;       /**< use eOcanport_t */
    uint8_t addr  : 4;       /**< use 0->14 */   
    uint8_t index : 1;       // use eobrd_caninsideindex_first or eobrd_caninsideindex_second 
}servMC_actutator_location_on_can_t;

typedef struct
{
    uint8_t place :2;        //use servMC_actutator_place_t
    uint8_t boardConnector  : 6;       /**< use servMC_board_connector_t */
}servMC_actutator_location_local_t;

typedef union
{
   servMC_actutator_location_on_can_t oncan;
   servMC_actutator_location_local_t  local;
}servMC_actutator_location_t;

typedef enum
{
    servMC_actuator_type_foc  = 0,
    servMC_actuator_type_mc4  = 1,
    servMC_actuator_type_pwm  = 2,
    servMC_actuator_type_unknown = 255
}  servMC_actuator_type_t;

typedef struct
{
    servMC_actutator_location_t     location;
    servMC_actuator_type_t          type;
}servMC_actuator_t;

typedef enum
{
    servMC_encoder_place_local  = 0,
    servMC_actutator_place_mais = 1
   // servMC_actutator_place_can  = 2, //maybe in future we can specify encoder on motor read by 2foc
}servMC_encoder_place_t;


typedef enum
{
    servMC_encoder_onmais_index_proximal  = 0,
    servMC_encoder_onmais_index_distal    = 1,
    servMC_encoder_onmais_middle_distal   = 2,
   // servMC_actutator_place_can  = 2, //maybe in future we can specify encoder on motor read by 2foc
}servMC_encoder_onmais_t;

typedef struct
{
    uint8_t place :2;        //use servMC_encoder_place_t
    uint8_t index : 6;       /**< use servMC_encoder_onmais_t */
}servMC_encoder_location_on_mais_t;

typedef struct
{
    uint8_t place :2;        //use servMC_actutator_place_t
    uint8_t boardConnector  : 6;       /**< use servMC_board_connector_t */
}servMC_encoder_location_local_t;


typedef union
{
   servMC_encoder_location_on_mais_t oncan;
   servMC_encoder_location_local_t  local;
}servMC_encoder_location_t;


typedef struct
{
    servMC_encoder_location_t       location;
    eOmn_serv_mc_sensor_type_t      type;
    eOmn_serv_mc_sensor_position_t  position;
}servMC_encoder_t;

typedef struct
{
    vector<servCanBoard_t>              canboards;
    servMCcontroller_t                  controller;
    vector<servMC_actuator_t>           actuators;
    vector<servMC_encoder_t>            encoder1s;
    vector<servMC_encoder_t>            encoder2s;
    eObrd_type_t                        controllerboardtype; //is used?
    vector<int>                         joint2set;
} servMCproperties_t;


typedef struct
{
    uint16_t        tbd1;
    vector<int>     tbd2;
} servMCsettings_t;



typedef struct
{
    eOmn_serv_type_t            type;
    servMCproperties_t          properties;
    servMCsettings_t            settings;
} servMCcollector_t;

#endif

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
    bool parseService2(Searchable &config, servConfigMC_t &mcconfig); // the fixed one.
    bool convert(ConstString const &fromstring, eOmn_servMC_controller_type_t &mc_controller_type, bool &formaterror);
    bool convert(Bottle &bottle, vector<double> &matrix, bool &formaterror);
    bool convert(ConstString const &fromstring, servMC_actuator_type_t &toactuatortype, bool &formaterror);
    bool convert(ConstString const &fromstring, servMC_actutator_location_t &location, bool &formaterror);
    bool convert(ConstString const &fromstring, eOmn_serv_mc_sensor_position_t &tosensorposition, bool &formaterror);
    bool convert(ConstString const &fromstring, eOmn_serv_mc_sensor_type_t &tosensortype, bool &formaterror);
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

#if defined(SERVICE_PARSER_USE_MC)
    servMCcollector_t           mc_service;
#endif

private:

    bool check_analog(yarp::os::Searchable &config, eOmn_serv_type_t type);

    bool check_motion(yarp::os::Searchable &config);
#if defined(SERVICE_PARSER_USE_MC)
    bool parseMCEncoderItem(Bottle &b_ENCODER,  vector<servMC_encoder_t> &encoders, char *encString);
#endif
    
};



#endif

