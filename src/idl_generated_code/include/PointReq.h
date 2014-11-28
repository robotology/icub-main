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
  
private:
  bool write_result(yarp::os::idl::WireWriter& writer);
  bool nested_write_result(yarp::os::idl::WireWriter& writer);
  bool write_x(yarp::os::idl::WireWriter& writer);
  bool nested_write_x(yarp::os::idl::WireWriter& writer);
  bool write_y(yarp::os::idl::WireWriter& writer);
  bool nested_write_y(yarp::os::idl::WireWriter& writer);
  bool write_z(yarp::os::idl::WireWriter& writer);
  bool nested_write_z(yarp::os::idl::WireWriter& writer);
  bool read_result(yarp::os::idl::WireReader& reader);
  bool nested_read_result(yarp::os::idl::WireReader& reader);
  bool read_x(yarp::os::idl::WireReader& reader);
  bool nested_read_x(yarp::os::idl::WireReader& reader);
  bool read_y(yarp::os::idl::WireReader& reader);
  bool nested_read_y(yarp::os::idl::WireReader& reader);
  bool read_z(yarp::os::idl::WireReader& reader);
  bool nested_read_z(yarp::os::idl::WireReader& reader);

public:
  
  yarp::os::ConstString toString();
  
  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<PointReq > unwrapped;
  
  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:
    
    Editor() {
      group = 0;
      obj_owned = true;
      obj = new PointReq;
      dirty_flags(false);
      yarp().setOwner(*this);
    }
    
    Editor(PointReq& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }
    
    bool edit(PointReq& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }
    
    virtual ~Editor() {
    if (obj_owned) delete obj;
    }
    
    bool isValid() const {
      return obj!=0/*NULL*/;
    }
    
    PointReq& state() { return *obj; }
    
    void begin() { group++; }
    
    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_result(const std::string& result) {
      will_set_result();
      obj->result = result;
      mark_dirty_result();
      communicate();
      did_set_result();
    }
    void set_x(const double x) {
      will_set_x();
      obj->x = x;
      mark_dirty_x();
      communicate();
      did_set_x();
    }
    void set_y(const double y) {
      will_set_y();
      obj->y = y;
      mark_dirty_y();
      communicate();
      did_set_y();
    }
    void set_z(const double z) {
      will_set_z();
      obj->z = z;
      mark_dirty_z();
      communicate();
      did_set_z();
    }
    const std::string& get_result() {
      return obj->result;
    }
    double get_x() {
      return obj->x;
    }
    double get_y() {
      return obj->y;
    }
    double get_z() {
      return obj->z;
    }
    virtual bool will_set_result() { return true; }
    virtual bool will_set_x() { return true; }
    virtual bool will_set_y() { return true; }
    virtual bool will_set_z() { return true; }
    virtual bool did_set_result() { return true; }
    virtual bool did_set_x() { return true; }
    virtual bool did_set_y() { return true; }
    virtual bool did_set_z() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:
    
    PointReq *obj;
    
    bool obj_owned;
    int group;
    
    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_result() {
      if (is_dirty_result) return;
      dirty_count++;
      is_dirty_result = true;
      mark_dirty();
    }
    void mark_dirty_x() {
      if (is_dirty_x) return;
      dirty_count++;
      is_dirty_x = true;
      mark_dirty();
    }
    void mark_dirty_y() {
      if (is_dirty_y) return;
      dirty_count++;
      is_dirty_y = true;
      mark_dirty();
    }
    void mark_dirty_z() {
      if (is_dirty_z) return;
      dirty_count++;
      is_dirty_z = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_result = flag;
      is_dirty_x = flag;
      is_dirty_y = flag;
      is_dirty_z = flag;
      dirty_count = flag ? 4 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_result;
    bool is_dirty_x;
    bool is_dirty_y;
    bool is_dirty_z;
  };
};

#endif

