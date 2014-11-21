// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_IRpcServer
#define YARP_THRIFT_GENERATOR_IRpcServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class IRpcServer;


/**
 * IRpcServer
 * Interface for an example module.
 */
class IRpcServer : public yarp::os::Wire {
public:
  IRpcServer();
  /**
   * Get answer from server
   * @return the answer
   */
  virtual int32_t get_answer();
  /**
   * Set value for future answers.
   * @param rightAnswer new answer
   * @return true if connection was successful
   */
  virtual bool set_answer(const int32_t rightAnswer);
  /**
   * Add one integer to future answers.
   * @param x value to add
   * @return new value
   */
  virtual int32_t add_int(const int32_t x);
  /**
   * Start service
   * @return true if service started correctly
   */
  virtual bool start();
  /**
   * Stop service
   * @return true if service stopped correctly
   */
  virtual bool stop();
  /**
   * Check is service is running
   * @return true/false if service is/is not running
   */
  virtual bool is_running();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

