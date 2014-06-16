// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_PointReq
#define YARP_THRIFT_GENERATOR_STRUCT_PointReq

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class PointReq;


/**
 * PointReq
 * IDL structure to send/receive points.
 */
class PointReq : public yarp::os::idl::WirePortable {
public:
  // Fields
/**
 * contain [ok]/[fail] on success/failure.
 */
  std::string result;
/**
 * the x-coordinate.
 */
  double x;
/**
 * the y-coordinate.
 */
  double y;
/**
 * the z-coordinate.
 */
  double z;
  
  // Default constructor
  PointReq() : result(""), x(0), y(0), z(0) {
  }
  
  // Constructor with field values
  PointReq(const std::string& result,const double x,const double y,const double z) : result(result), x(x), y(y), z(z) {
  }
  
  // Copy constructor
  PointReq(const PointReq& __alt) {
    this->result = __alt.result;
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
  }
  
  // Assignment operator
  const PointReq& operator = (const PointReq& __alt) {
    this->result = __alt.result;
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
    return *this;
  }
  
  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
  
  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<PointReq > unwrapped;
};

#endif

