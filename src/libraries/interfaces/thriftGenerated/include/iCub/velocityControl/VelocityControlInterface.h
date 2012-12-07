// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_VelocityControlInterface
#define YARP_THRIFT_GENERATOR_VelocityControlInterface

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  class VelocityControlInterface;
}


/**
 * VelocityControlInterface
 */
class iCub::VelocityControlInterface : public yarp::os::Wire {
public:
  VelocityControlInterface() { yarp().setOwner(*this); }
/**
 * Suspend the controller (command zero velocity)
 *  @return true if command was accepted
 */
  virtual bool susp();
/**
 * Start (and resume after being suspended) the controller
 *  @return true if command was accepted
 */
  virtual bool run();
/**
 * Quit the module (exit)
 *  @return true if command was accepted
 */
  virtual bool quit();
/**
 * move joint j to p (degrees)
 *  @return true if command was accepted
 */
  virtual bool set_(const int32_t j, const double p);
/**
 * Set maximum speed for joint j to v (deg/sec)
 *  @return true if command was accepted
 */
  virtual bool svel(const int32_t j, const double v);
/**
 * Set gain for joint j to k
 *  @return true if command was accepted
 */
  virtual bool gain(const int32_t j, const double k);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

