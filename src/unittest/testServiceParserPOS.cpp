// Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "serviceParser.h"
#include "posServiceConfiguration.h"
#include "fakeEthResource.h"

#include <yarp/os/Property.h>

#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

// Keep the host packet compatible with the shared firmware ABI, including fillers.
static_assert(sizeof(eoas_pos_SensorConfig_t) == 4);
static_assert(sizeof(eoas_pos_BoardConfig_t) == 24);
static_assert(sizeof(eOmn_serv_config_data_as_pos_t) == 24);
static_assert(sizeof(eOmn_serv_config_data_mc_mc4plusfaps_t) == 340);
static_assert(sizeof(eOmn_serv_configuration_t) == 344);
static_assert(sizeof(eOmn_serv_parameter_t) == 344);
static_assert(sizeof(eOmn_service_cmmnds_command_t) == 348);
static_assert(offsetof(eoas_pos_BoardConfig_t, sensors) == 8);
static_assert(offsetof(eOmn_serv_config_data_mc_mc4plusfaps_t, pos) == 0);
static_assert(offsetof(eOmn_serv_config_data_mc_mc4plusfaps_t, arrayofjomodescriptors) == 24);
static_assert(offsetof(eOmn_serv_config_data_mc_mc4plusfaps_t, jomocoupling) == 48);
static_assert(offsetof(eOmn_serv_configuration_t, data) == 4);
static_assert(offsetof(eOmn_service_cmmnds_command_t, parameter) == 4);

namespace {

const std::string canboards = R"(
    (CANBOARDS (type mtb4c)
        (PROTOCOL (major 2) (minor 0))
        (FIRMWARE (major 0) (minor 0) (build 0)))
)";

yarp::os::Property motionConfig(const std::string& pos,
                                const std::string& boards = canboards,
                                const std::string& type = "eomn_serv_MC_mc4plusfaps")
{
    yarp::os::Property config;
    config.fromString("(SERVICE (type " + type + ") (PROPERTIES "
        "(ETHBOARD (type mc4plus)) " + boards + " " + pos + R"(
        (JOINTMAPPING
            (ACTUATOR (type pwm pwm pwm pwm) (port CONN:P5 CONN:P4 CONN:P3 CONN:P2))
            (ENCODER1 (type pos pos pos pos)
                (port POS:hand_thumb_oc POS:hand_index_oc POS:hand_middle_oc POS:hand_ring_pinky_oc)
                (position atjoint atjoint atjoint atjoint)
                (resolution 65535 65535 65535 65535) (tolerance 5 5 5 5))
            (ENCODER2 (type qenc qenc qenc qenc) (port CONN:P5 CONN:P4 CONN:P3 CONN:P2)
                (position atmotor atmotor atmotor atmotor)
                (resolution 1600 1600 1600 1600) (tolerance 0 0 0 0)))))
    )");
    return config;
}

void check(bool condition, const char* description)
{
    if (!condition) { throw std::runtime_error(description); }
    std::cout << "PASS: " << description << '\n';
}

bool parse(ServiceParser& parser, const std::string& pos, servConfigMC_t& result)
{
    auto config = motionConfig(pos);
    return parser.parseService(config, result);
}

} // namespace

int main(int argc, char* argv[])
{
    try
    {
        ServiceParser parser;
        servConfigMC_t omitted {};

        // Poison POS data to detect fields left uninitialized by the parser.
        auto& dependency = omitted.ethservice.configuration.data.mc.mc4plusfaps.pos;
        std::memset(&dependency, 0xa5, sizeof(dependency));
        check(parse(parser, "", omitted), "Entire MC POS group can be omitted");
        const auto& omittedData = omitted.ethservice.configuration.data.mc.mc4plusfaps;
        eOmn_serv_config_data_as_pos_t expected {};
        expected.config.boardconfig[0].boardinfo.type = eobrd_cantype_none;
        check(std::memcmp(&omittedData.pos, &expected, sizeof(expected)) == 0,
              "Absent POS dependency is explicitly marked and otherwise zero-initialized");
        check(parser.mc_service.properties.numofjoints == 4 &&
              parser.getEncoderAtJoint(0)->resolution == 65535 &&
              parser.getEncoderAtJoint(0)->tolerance == 5 &&
              parser.getEncoderAtMotor(0)->resolution == 1600,
              "Encoder placement, resolution and tolerance remain available");
        // POS belongs to the standalone service, including an empty group.
        servConfigMC_t ignored {};
        for (const std::string pos : {"(POS)", "(POS (location CAN1:1))",
             "(POS (location invalid))", "(POS (location CAN1:1 CAN1:2))",
             "(POS (SENSORS (connector ignored)) (SETTINGS (acquisitionRate invalid)))"})
        {
            check(!parse(parser, pos, ignored), "MC POS group is rejected");
        }
        auto basicMC = motionConfig("", "", "eomn_serv_MC_mc4plus");
        check(parser.parseService(basicMC, ignored), "Plain mc4plus service remains supported");
        auto basicMCWithPOS = motionConfig("(POS)", "", "eomn_serv_MC_mc4plus");
        check(!parser.parseService(basicMCWithPOS, ignored), "Plain mc4plus also rejects a POS group");
        auto genericMCWithPOS = motionConfig("(POS)", "", "eomn_serv_MC_generic");
        check(!parser.parseService(genericMCWithPOS, ignored), "Generic MC cannot bypass POS group rejection");
        auto noBoards = motionConfig("", "");
        check(!parser.parseService(noBoards, ignored), "MC still requires CANBOARDS when POS is omitted");
        auto malformedBoards = motionConfig("", "(CANBOARDS (type mtb4c))");
        check(!parser.parseService(malformedBoards, ignored), "MC still validates CANBOARDS protocol and firmware fields");
        yarp::os::Property noMapping;
        noMapping.fromString("(SERVICE (type eomn_serv_MC_mc4plusfaps) (PROPERTIES "
            "(ETHBOARD (type mc4plus)) " + canboards + "))");
        check(!parser.parseService(noMapping, ignored), "Omitting POS does not bypass mandatory JOINTMAPPING validation");
        auto badEncoderText = motionConfig("").toString();
        const auto port = badEncoderText.find("POS:hand_thumb_oc");
        check(port != std::string::npos, "Encoder test fixture contains the expected POS port");
        badEncoderText.replace(port, std::string("POS:hand_thumb_oc").size(), "POS:invalid");
        yarp::os::Property badEncoder;
        badEncoder.fromString(badEncoderText);
        check(!parser.parseService(badEncoder, ignored), "Omitting POS does not bypass encoder port validation");

        yarp::os::Property posConfig;
        posConfig.fromString("(SERVICE (type eomn_serv_AS_pos) (PROPERTIES " + canboards + R"(
            (SENSORS (id thumb index middle ring) (sensorName hand hand hand hand)
                (type eoas_pos_angle eoas_pos_angle eoas_pos_angle eoas_pos_angle)
                (location CAN1:1 CAN1:1 CAN1:1 CAN1:1)
                (port POS:hand_thumb_oc POS:hand_index_oc POS:hand_middle_oc POS:hand_ring_pinky_oc)
                (boardType mtb4c mtb4c mtb4c mtb4c)
                (connector CONN:J3_SDA3 CONN:J3_SDA0 CONN:J3_SDA1 CONN:J3_SDA2)))
            (SETTINGS (acquisitionRate 50) (enabledSensors thumb index middle ring)))
        )");
        servConfigPOS_t standalone {};
        check(parser.parseService(posConfig, standalone), "Standalone POS still parses");
        const auto& sensors = standalone.ethservice.configuration.data.as.pos.config.boardconfig[0].sensors;
        check(standalone.acquisitionrate == 50 && standalone.idList.size() == 4 &&
              sensors[0].connector == 3 && sensors[1].connector == 0 &&
              sensors[2].connector == 1 && sensors[3].connector == 2,
              "Standalone POS settings and physical connector mapping are preserved");

        eth::POSServiceConfiguration cache;
        eOmn_serv_parameter_t prepared {};
        check(!cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "MC dependency cannot be resolved before standalone POS activation");
        cache.rememberActivated(eomn_serv_category_mc, &omitted.ethservice);
        check(!cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "An MC request cannot masquerade as standalone POS activation");
        cache.rememberActivated(eomn_serv_category_pos, &standalone.ethservice);
        check(cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "Activated standalone POS resolves the MC dependency");
        const auto& resolved = prepared.configuration.data.mc.mc4plusfaps;
        check(std::memcmp(&resolved.pos, &standalone.ethservice.configuration.data.as.pos,
                          sizeof(resolved.pos)) == 0,
              "Full POS board, location and sensor configuration is inherited");
        auto expectedRequest = omitted.ethservice;
        expectedRequest.configuration.data.mc.mc4plusfaps.pos = standalone.ethservice.configuration.data.as.pos;
        check(std::memcmp(&prepared, &expectedRequest, sizeof(prepared)) == 0,
              "Resolving POS preserves every other MC service field");
        check(std::memcmp(&resolved.arrayofjomodescriptors, &omittedData.arrayofjomodescriptors,
                          sizeof(resolved.arrayofjomodescriptors)) == 0,
              "Resolving POS preserves MC actuator and encoder descriptors");
        check(omittedData.pos.config.boardconfig[0].boardinfo.type == eobrd_cantype_none,
              "Resolving POS does not mutate the original parser output");
        eth::POSServiceConfiguration otherBoard;
        check(!otherBoard.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "POS configuration cannot leak into a different Ethernet resource");
        auto embedded = omitted.ethservice;
        embedded.configuration.data.mc.mc4plusfaps.pos.config.boardconfig[0].boardinfo.type = eobrd_cantype_mtb4c;
        check(!otherBoard.prepare(eomn_serv_category_mc, &embedded, prepared),
              "Prepopulated embedded POS cannot bypass standalone POS activation");
        check(cache.prepare(eomn_serv_category_mc, &embedded, prepared) &&
              std::memcmp(&prepared.configuration.data.mc.mc4plusfaps.pos,
                          &standalone.ethservice.configuration.data.as.pos, sizeof(resolved.pos)) == 0,
              "Activated standalone POS always replaces embedded MC POS data");
        auto alternative = standalone.ethservice;
        alternative.configuration.data.as.pos.config.boardconfig[0].sensors[0].connector = 0;
        cache.rememberActivated(eomn_serv_category_pos, &alternative);
        check(cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared) &&
              prepared.configuration.data.mc.mc4plusfaps.pos.config.boardconfig[0].sensors[0].connector == 3,
              "A second POS owner does not overwrite the first active mapping");
        cache.clearOnStop(eomn_serv_category_mc);
        check(cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "Stopping MC preserves standalone POS configuration");
        cache.clearOnStop(eomn_serv_category_pos);
        check(!cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "Stopping POS invalidates its cached dependency");
        cache.rememberActivated(eomn_serv_category_pos, &alternative);
        check(cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared) &&
              prepared.configuration.data.mc.mc4plusfaps.pos.config.boardconfig[0].sensors[0].connector == 0,
              "POS can be activated again with a new configuration");
        cache.clearOnStop(eomn_serv_category_all);
        check(!cache.prepare(eomn_serv_category_mc, &omitted.ethservice, prepared),
              "Stopping all services invalidates the POS configuration");
        check(cache.prepare(eomn_serv_category_pos, &standalone.ethservice, prepared) &&
              std::memcmp(&prepared, &standalone.ethservice, sizeof(prepared)) == 0,
              "Standalone POS requests pass through unchanged");
        check(cache.prepare(eomn_serv_category_mc, nullptr, prepared), "Null requests retain existing handling");

        eth::FakeEthResource fake;
        check(!fake.serviceVerifyActivate(eomn_serv_category_mc, &omitted.ethservice),
              "Fake resource enforces POS-first activation as the real resource does");
        check(fake.serviceVerifyActivate(eomn_serv_category_pos, &standalone.ethservice) &&
              fake.serviceVerifyActivate(eomn_serv_category_mc, &omitted.ethservice),
              "Fake resource accepts MC after standalone POS activation");
        check(fake.serviceStop(eomn_serv_category_pos) &&
              !fake.serviceVerifyActivate(eomn_serv_category_mc, &omitted.ethservice),
              "Fake resource does not reuse a stopped POS service");
        check(!fake.serviceVerifyActivate(eomn_serv_category_mc, &embedded),
              "Fake resource rejects prepopulated MC POS without standalone activation");
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
