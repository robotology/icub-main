// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __serviceParser_h__
#define __serviceParser_h__

#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include "EoBoards.h"
#include "EoManagement.h"
#include "EoAnalogSensors.h"
#include "EoMotionControl.h"




#define SERVICE_PARSER_USE_MC

typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
    std::string                  nameOfMais;
} servConfigMais_t;


typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
    bool                    useCalibration;
    std::string                  nameOfStrain;
    eObrd_cantype_t         boardType;
} servConfigStrain_t;

typedef struct
{
    eOmn_serv_parameter_t   ethservice;
    int                     acquisitionrate;
    bool                    useCalibration;
    std::string             nameOfStrain;
    eObrd_cantype_t         boardType;
    int                     temperatureAcquisitionrate;
    std::string             frameName;
} servConfigFTsensor_t;

typedef struct
{
    eOmn_serv_parameter_t               ethservice;
    int                                 acquisitionrate;
    std::vector<eOas_inertial_descriptor_t>  inertials;
    std::vector<std::string>                      id;
} servConfigInertials_t;

typedef struct
{
    double accFactor;
    double gyrFactor;
    double magFactor;
    double eulFactor;
} imuConvFactors_t;

typedef struct
{
    eOmn_serv_parameter_t               ethservice;
    int                                 acquisitionrate;
    std::vector<eOas_inertial3_descriptor_t> inertials; //TODO to remove because information is already stored!
    std::vector<std::string>                      id;
    imuConvFactors_t                    convFactors;
} servConfigImu_t;


typedef struct
{
    eOmn_serv_parameter_t     ethservice;
    int                       acquisitionrate;
    std::vector<std::string>  idList;
} servConfigPSC_t;

typedef struct
{
    eOmn_serv_parameter_t     ethservice;
    int                       acquisitionrate;
    std::vector<std::string>  idList;
} servConfigPOS_t;


typedef struct
{
    eOmn_serv_parameter_t     ethservice;
    int                       acquisitionrate;
    std::vector<std::string>  idList;
} servConfigPOS2_t;


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
    servCanBoard_t() { clear(); }
    void clear() { type = eobrd_cantype_none; protocol.major = 0; protocol.minor = 0; firmware.major = 0; firmware.minor = 0; firmware.build = 0; }
};

struct servBoard_t
{
    eObrd_type_t                type;
    eObrd_protocolversion_t     protocol;
    eObrd_firmwareversion_t     firmware;
    servBoard_t() { type = eobrd_none; protocol.major = 0; protocol.minor = 0; firmware.major = 0; firmware.minor = 0; firmware.build = 0; }
};

struct servConfigSkin_t
{
   servCanBoard_t    canboard;
   servConfigSkin_t() { clear(); }
   void clear() { canboard.clear(); canboard.type = eobrd_cantype_mtb; }
};


struct servAnalogPOScalibration_t
{
    eoas_pos_TYPE_t type {eoas_pos_TYPE_none};
    eoas_pos_ROT_t rotation {eoas_pos_ROT_zero};
    float offset {0.0};
    bool invertdirection {false};
    void clear()
    {
        type = eoas_pos_TYPE_none; rotation = eoas_pos_ROT_zero; offset = 0.0; invertdirection = false;
    }
};

struct servAnalogPOSspecific_t
{
    eObrd_portpos_t port {eobrd_portpos_none};
    eObrd_connector_t connector {eobrd_conn_none};
    servAnalogPOScalibration_t calibration {};
    void clear()
    {
        port = eobrd_portpos_none; connector = eobrd_conn_none; calibration.clear();
    }
};

struct servAnalogSensor_t
{
    std::string                 id {""};
    eOas_sensor_t               type {eoas_none};
    eObrd_location_t            location {{eobrd_place_none, 0}};
    eObrd_type_t                boardtype {eobrd_none};
    std::string                 frameName {""};
    servAnalogPOSspecific_t     pos {};
    void clear()
    {
        id.clear(); type = eoas_none; location.any.place = eobrd_place_none;
        boardtype = eobrd_none; frameName.clear(); pos.clear();
    }
};


typedef struct
{
    std::vector<servCanBoard_t>      canboards;
    std::vector<servAnalogSensor_t>  sensors;
} servASproperties_t;


typedef struct
{
    uint16_t                    acquisitionrate;
    std::vector<servAnalogSensor_t>  enabledsensors;
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



typedef struct
{
    servCanBoard_t              canboard;
} servSKproperties_t;


typedef struct
{
    eOmn_serv_type_t            type;
    servSKproperties_t          properties;
    //servSKsettings_t            settings;
} servSKcollector_t;


#if defined(SERVICE_PARSER_USE_MC)

// typedef struct
// {
//     std::vector<double>                  matrixJ2M;
//     std::vector<double>                  matrixM2J;
//     std::vector<double>                  matrixE2J;
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
    std::vector<servCanBoard_t>         canboards;

    eObrd_canlocation_t                 maislocation;
    std::vector<eObrd_canlocation_t>    psclocations;
    std::vector<eObrd_canlocation_t>    poslocations;

    eOmc_mc4shifts_t                    mc4shifts;
    std::vector<eOmc_mc4broadcast_t>    mc4broadcasts;
    std::vector<eObrd_canlocation_t>    mc4joints;

    //servMC_controller_t                 controller;

    std::vector<servMC_actuator_t>      actuators;
    std::vector<servMC_encoder_t>       encoder1s;
    std::vector<servMC_encoder_t>       encoder2s;

    //std::vector<int>                    joint2set;
    //int                                 numofjointsets;
    //std::vector<eOmc_jointset_configuration_t> jointset_cfgs;
} servMCproperties_t;


typedef struct
{
    uint16_t        tbd1;
    std::vector<int>     tbd2;
} servMCsettings_t;



typedef struct
{
    eOmn_serv_type_t            type;
    eOmn_serv_diagn_cfg_t       diagconfig;
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
    bool parseService(yarp::os::Searchable &config, servConfigStrain_t &strainconfig);
    bool parseService(yarp::os::Searchable &config, servConfigFTsensor_t &ftconfig);
    bool parseService(yarp::os::Searchable &config, servConfigInertials_t &inertialsconfig);
    bool parseService(yarp::os::Searchable &config, servConfigImu_t &imuconfig);
    bool parseService(yarp::os::Searchable &config, servConfigSkin_t &skinconfig);
    bool parseService(yarp::os::Searchable &config, servConfigPSC_t &pscconfig);
    bool parseService(yarp::os::Searchable &config, servConfigPOS_t &pscconfig);
    bool parseService(yarp::os::Searchable &config, servConfigPOS2_t &pscconfig);
#if defined(SERVICE_PARSER_USE_MC)
    bool parseService(yarp::os::Searchable &config, servConfigMC_t &mcconfig);
    bool parseService2(yarp::os::Searchable &config, servConfigMC_t &mcconfig); // the fixed one.
    //bool convert(std::string const &fromstring, eOmc_ctrlboard_t &controllerboard, bool &formaterror);
    //bool convert(Bottle &bottle, std::vector<double> &matrix, bool &formaterror, int targetsize);
    bool convert(std::string const &fromstring, eOmc_actuator_t &toactuatortype, bool &formaterror);
    bool convert(std::string const &fromstring, eOmc_position_t &toposition, bool &formaterror);
    bool convert(std::string const &fromstring, eOmc_encoder_t &toencodertype, bool &formaterror);

    bool parse_connector(const std::string &fromstring, eObrd_connector_t &toconnector, bool &formaterror);
    bool parse_mais(std::string const &fromstring, eObrd_portmais_t &pmais, bool &formaterror);

    bool parse_port_conn(std::string const &fromstring, eObrd_type_t const board, uint8_t &toport, bool &formaterror);
    bool parse_port_mais(std::string const &fromstring, uint8_t &toport, bool &formaterror);

    bool parse_actuator_port(std::string const &fromstring, eObrd_ethtype_t const ethboard, eOmc_actuator_t const type, eOmc_actuator_descriptor_t &todes, bool &formaterror);
    bool parse_encoder_port(std::string const &fromstring, eObrd_ethtype_t const ethboard, eOmc_encoder_t type, uint8_t &toport, bool &formaterror);

    bool parse_psc(std::string const &fromstring, eObrd_portpsc_t &ppsc, bool &formaterror);
    bool parse_port_psc(std::string const &fromstring, uint8_t &toport, bool &formaterror);

    bool parse_POS_port(std::string const &fromstring, eObrd_portpos_t &ppos, bool &formaterror);
    bool parse_port_pos(std::string const &fromstring, uint8_t &toport, bool &formaterror);

    bool parse_POS_connector(std::string const &fromstring, const eObrd_type_t brd, eObrd_connector_t &conn, bool &formaterror);

#endif

    bool convert(std::string const &fromstring, eOmn_serv_type_t &toservicetype, bool &formaterror);
    bool convert(std::string const &fromstring, eOmn_serv_diagn_mode_t &todiagnmode, bool &formaterror);
    bool convert(std::string const &fromstring, eObrd_type_t& tobrdtype, bool& formaterror);
    bool convert(std::string const &fromstring, eObrd_cantype_t &tobrdcantype, bool &formaterror);
    bool convert(std::string const &fromstring, eObrd_ethtype_t& tobrdethtype, bool& formaterror);
    bool convert(std::string const &fromstring, bool &tobool, bool &formaterror);
    bool convert(const int number, uint8_t &tou8, bool &formaterror);
    bool convert(const int number, uint16_t &tou16, bool &formaterror);
    bool convert(std::string const &fromstring, eOas_sensor_t &tosensortype, bool &formaterror);
    bool convert(std::string const &fromstring, std::string &str, bool &formaterror);
    bool convert(std::string const &fromstring, const uint8_t strsize, char *str, bool &formaterror);
    bool convert(std::string const &fromstring, eObrd_location_t &location, bool &formaterror);


    bool convert(eObrd_location_t const &loc, char *str, int len);
    bool convert(eObrd_canlocation_t const &canloc, char *str, int len);

    bool convert(eObrd_protocolversion_t const &prot, char *str, int len);
    bool convert(eObrd_firmwareversion_t const &firm, char *str, int len);

    bool convert(std::string const &fromstring, eOmc_pidoutputtype_t& pidoutputtype, bool& formaterror);
    bool convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror);
    servMC_encoder_t * getEncoderAtMotor(int index);
    servMC_encoder_t * getEncoderAtJoint(int index);
    bool parse_debugEmbBoardsNotConnected(yarp::os::Searchable &config, bool &embBoardsConnected);
   public:

    servAScollector_t           as_service;
    servASstrainSettings_t      as_strain_settings;

    servSKcollector_t             sk_service;
    eOmn_serv_config_data_sk_skin_t tmp;

#if defined(SERVICE_PARSER_USE_MC)
    servMCcollector_t           mc_service;
#endif

private:

    bool check_analog(yarp::os::Searchable &config, eOmn_serv_type_t type);

    bool check_skin(yarp::os::Searchable &config);

    bool check_motion(yarp::os::Searchable &config);

    int getnumofjointsets(void);

    bool copyjomocouplingInfo(eOmc_4jomo_coupling_t *jc_dest);

    
    // suggestion: split check_motion() in sub-methods which parse the groups ...
};

#endif

