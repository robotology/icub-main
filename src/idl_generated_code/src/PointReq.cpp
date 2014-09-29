// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <PointReq.h>

bool PointReq::read_result(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(result)) {
    reader.fail();
    return false;
  }
  return true;
}
bool PointReq::read_x(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(x)) {
    reader.fail();
    return false;
  }
  return true;
}
bool PointReq::read_y(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(y)) {
    reader.fail();
    return false;
  }
  return true;
}
bool PointReq::read_z(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(z)) {
    reader.fail();
    return false;
  }
  return true;
}
bool PointReq::read(yarp::os::idl::WireReader& reader) {
  if (!read_result(reader)) return false;
  if (!read_x(reader)) return false;
  if (!read_y(reader)) return false;
  if (!read_z(reader)) return false;
  return !reader.isError();
}

bool PointReq::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(4)) return false;
  return read(reader);
}

bool PointReq::write_result(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(result)) return false;
  return true;
}
bool PointReq::write_x(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(x)) return false;
  return true;
}
bool PointReq::write_y(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(y)) return false;
  return true;
}
bool PointReq::write_z(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(z)) return false;
  return true;
}
bool PointReq::write(yarp::os::idl::WireWriter& writer) {
  if (!write_result(writer)) return false;
  if (!write_x(writer)) return false;
  if (!write_y(writer)) return false;
  if (!write_z(writer)) return false;
  return !writer.isError();
}

bool PointReq::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  return write(writer);
}
bool PointReq::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_result) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("result")) return false;
    if (!obj->write_result(writer)) return false;
  }
  if (is_dirty_x) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("x")) return false;
    if (!obj->write_x(writer)) return false;
  }
  if (is_dirty_y) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("y")) return false;
    if (!obj->write_y(writer)) return false;
  }
  if (is_dirty_z) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("z")) return false;
    if (!obj->write_z(writer)) return false;
  }
  return !writer.isError();
}
bool PointReq::Editor::read(yarp::os::ConnectionReader& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  int len = reader.getLength();
  if (len==0) {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(1)) return false;
    writer.writeString("send: 'help' or 'patch (param1 val1) (param2 val2)'");
    return true;
  }
  yarp::os::ConstString tag;
  if (!reader.readString(tag)) return false;
  if (tag!="patch") {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("many",1, 0)) return false;
    if (tag=="help" && reader.getLength()>0) {
      yarp::os::ConstString field;
      if (!reader.readString(field)) return false;
      if (field=="result") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("std::string result")) return false;
        if (!writer.writeString("contain [ok]/[fail] on success/failure.")) return false;
      }
      if (field=="x") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double x")) return false;
        if (!writer.writeString("the x-coordinate.")) return false;
      }
      if (field=="y") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double y")) return false;
        if (!writer.writeString("the y-coordinate.")) return false;
      }
      if (field=="z") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double z")) return false;
        if (!writer.writeString("the z-coordinate.")) return false;
      }
    }
    if (!writer.writeListHeader(5)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("result");
    writer.writeString("x");
    writer.writeString("y");
    writer.writeString("z");
    return true;
  }
  for (int i=1; i<len; i++) {
    if (!reader.readListHeader(3)) return false;
    yarp::os::ConstString act;
    yarp::os::ConstString key;
    if (!reader.readString(act)) return false;
    if (!reader.readString(key)) return false;
    // inefficient code follows, bug paulfitz to improve it
    if (key == "result") {
      will_set_result();
      if (!obj->read_result(reader)) return false;
      did_set_result();
    } else if (key == "x") {
      will_set_x();
      if (!obj->read_x(reader)) return false;
      did_set_x();
    } else if (key == "y") {
      will_set_y();
      if (!obj->read_y(reader)) return false;
      did_set_y();
    } else if (key == "z") {
      will_set_z();
      if (!obj->read_z(reader)) return false;
      did_set_z();
    } else {
      // would be useful to have a fallback here
    }
  }
  reader.accept();
  return true;
}

yarp::os::ConstString PointReq::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
