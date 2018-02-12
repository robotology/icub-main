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
#include "EoMotionControl.h"

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

// typedef struct
// {
//     vector<double>                  matrixJ2M;
//     vector<double>                  matrixM2J;
//     vector<double>                  matrixE2J;
// } servMC_controller_t;



typedef struct
{
    eOmc_actuator_t                 type;
    eOmc_actuator_descriptor_t      desc;
} servMC_actuator_t;



typedef struct
{
    eOmc_encoder_descriptor_t       desc;
    int32_t                         resolution;
    double                          tolerance;
} servMC_encoder_t;


typedef struct
{

    int                                 numofjoints;
    eObrd_ethtype_t                     ethboardtype;
    vector<servCanBoard_t>              canboards;

    eObrd_canlocation_t                 maislocation;

    eOmc_mc4shifts_t                    mc4shifts;
    vector<eOmc_mc4broadcast_t>         mc4broadcasts;
    vector<eObrd_canlocation_t>         mc4joints;

    //servMC_controller_t                 controller;

    vector<servMC_actuator_t>           actuators;
    vector<servMC_encoder_t>            encoder1s;
    vector<servMC_encoder_t>            encoder2s;

    //vector<int>                         joint2set;
    //int                                 numofjointsets;
    //vector<eOmc_jointset_configuration_t> jointset_cfgs;
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
    bool convert(ConstString const &fromstring, eOmc_ctrlboard_t &controllerboard, bool &formaterror);
    //bool convert(Bottle &bottle, vector<double> &matrix, bool &formaterror, int targetsize);
    bool convert(ConstString const &fromstring, eOmc_actuator_t &toactuatortype, bool &formaterror);
    bool convert(ConstString const &fromstring, eOmc_position_t &toposition, bool &formaterror);
    bool convert(ConstString const &fromstring, eOmc_encoder_t &toencodertype, bool &formaterror);

    bool parse_connector(const ConstString &fromstring, eObrd_connector_t &toconnector, bool &formaterror);
    bool parse_mais(ConstString const &fromstring, eObrd_portmais_t &pmais, bool &formaterror);

    bool parse_port_conn(ConstString const &fromstring, eObrd_ethtype_t const ethboard, uint8_t &toport, bool &formaterror);
    bool parse_port_mais(ConstString const &fromstring, uint8_t &toport, bool &formaterror);

    bool parse_actuator_port(ConstString const &fromstring, eObrd_ethtype_t const ethboard, eOmc_actuator_t const type, eOmc_actuator_descriptor_t &todes, bool &formaterror);
    bool parse_encoder_port(ConstString const &fromstring, eObrd_ethtype_t const ethboard, eOmc_encoder_t type, uint8_t &toport, bool &formaterror);

#endif

    bool convert(ConstString const &fromstring, eOmn_serv_type_t &toservicetype, bool &formaterror);
    bool convert(ConstString const &fromstring, eObrd_type_t& tobrdtype, bool& formaterror);
    bool convert(ConstString const &fromstring, eObrd_cantype_t &tobrdcantype, bool &formaterror);
    bool convert(ConstString const &fromstring, eObrd_ethtype_t& tobrdethtype, bool& formaterror);
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

    bool convert(ConstString const &fromstring, eOmc_pidoutputtype_t& pidoutputtype, bool& formaterror);
    bool convert(ConstString const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror);
    servMC_encoder_t * getEncoderAtMotor(int index);
    servMC_encoder_t * getEncoderAtJoint(int index);
    bool parse_debugEmbBoardsNotConnected(Searchable &config, bool &embBoardsConnected);
   public:

    servAScollector_t           as_service;
    servASstrainSettings_t      as_strain_settings;

#if defined(SERVICE_PARSER_USE_MC)
    servMCcollector_t           mc_service;
#endif

private:

    bool check_analog(yarp::os::Searchable &config, eOmn_serv_type_t type);

    bool check_motion(yarp::os::Searchable &config);

    int getnumofjointsets(void);

    bool copyjomocouplingInfo(eOmc_4jomo_coupling_t *jc_dest);

    
    // suggestion: split check_motion() in sub-methods which parse the groups ...
};



#endif

