// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/velocityControl/VelocityControlInterface.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {


class VelocityControlInterface_susp : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("susp",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class VelocityControlInterface_run : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("run",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class VelocityControlInterface_quit : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("quit",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class VelocityControlInterface_set_ : public yarp::os::Portable {
public:
  int32_t j;
  double p;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(4)) return false;
    if (!writer.writeTag("set_",1,2)) return false;
    if (!writer.writeI32(j)) return false;
    if (!writer.writeDouble(p)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class VelocityControlInterface_svel : public yarp::os::Portable {
public:
  int32_t j;
  double v;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("svel",1,1)) return false;
    if (!writer.writeI32(j)) return false;
    if (!writer.writeDouble(v)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class VelocityControlInterface_gain : public yarp::os::Portable {
public:
  int32_t j;
  double k;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("gain",1,1)) return false;
    if (!writer.writeI32(j)) return false;
    if (!writer.writeDouble(k)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

bool VelocityControlInterface::susp() {
  bool _return = false;
  VelocityControlInterface_susp helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool VelocityControlInterface::run() {
  bool _return = false;
  VelocityControlInterface_run helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool VelocityControlInterface::quit() {
  bool _return = false;
  VelocityControlInterface_quit helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool VelocityControlInterface::set_(const int32_t j, const double p) {
  bool _return = false;
  VelocityControlInterface_set_ helper;
  helper.j = j;
  helper.p = p;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool VelocityControlInterface::svel(const int32_t j, const double v) {
  bool _return = false;
  VelocityControlInterface_svel helper;
  helper.j = j;
  helper.v = v;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool VelocityControlInterface::gain(const int32_t j, const double k) {
  bool _return = false;
  VelocityControlInterface_gain helper;
  helper.j = j;
  helper.k = k;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool VelocityControlInterface::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {    // TODO: use quick lookup, this is just a test
    if (tag == "susp") {
      bool _return;
      _return = susp();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "run") {
      bool _return;
      _return = run();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_") {
      int32_t j;
      double p;
      if (!reader.readI32(j)) return false;
      if (!reader.readDouble(p)) return false;
      bool _return;
      _return = set_(j,p);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "svel") {
      int32_t j;
      double v;
      if (!reader.readI32(j)) return false;
      if (!reader.readDouble(v)) return false;
      bool _return;
      _return = svel(j,v);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "gain") {
      int32_t j;
      double k;
      if (!reader.readI32(j)) return false;
      if (!reader.readDouble(k)) return false;
      bool _return;
      _return = gain(j,k);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}
} // namespace


