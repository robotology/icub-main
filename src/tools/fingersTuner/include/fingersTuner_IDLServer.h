// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_fingersTuner_IDLServer
#define YARP_THRIFT_GENERATOR_fingersTuner_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Value.h>

class fingersTuner_IDLServer;


/**
 * fingersTuner_IDLServer
 * IDL Interface to \ref fingersTuner services.
 */
class fingersTuner_IDLServer : public yarp::os::Wire {
public:
  fingersTuner_IDLServer() { yarp().setOwner(*this); }
/**
 * Synchronize PID values with values stored
 * in the configuration file.
 * @param part specifies the part name as per
 * configuration file.
 * @param val accounts for a single joint if
 * the corresponding integer is given or a set
 * of joints if the corresponding alias is provided
 * as defined within the configuration file.
 * @return true/false on success/failure
 */
  virtual bool sync(const std::string& part, const yarp::os::Value& val);
/**
 * Tune PID of a joint or a set of joints.
 * @param part specifies the part name as per
 * configuration file.
 * @param val accounts for a single joint if
 * the corresponding integer is given or a set
 * of joints if the corresponding alias is provided
 * as defined within the configuration file.
 * @return true/false on success/failure
 */
  virtual bool tune(const std::string& part, const yarp::os::Value& val);
/**
 * Save the PID parameters on configuration file.
 * @return true/false on success/failure
 */
  virtual bool save();
/**
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

