// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <fingersTuner_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class fingersTuner_IDLServer_sync : public yarp::os::Portable {
public:
  std::string part;
  yarp::os::Value val;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("sync",1,1)) return false;
    if (!writer.writeString(part)) return false;
    if (!writer.write(val)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class fingersTuner_IDLServer_tune : public yarp::os::Portable {
public:
  std::string part;
  yarp::os::Value val;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("tune",1,1)) return false;
    if (!writer.writeString(part)) return false;
    if (!writer.write(val)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class fingersTuner_IDLServer_save : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("save",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class fingersTuner_IDLServer_quit : public yarp::os::Portable {
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
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

bool fingersTuner_IDLServer::sync(const std::string& part, const yarp::os::Value& val) {
  bool _return = false;
  fingersTuner_IDLServer_sync helper;
  helper.part = part;
  helper.val = val;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDLServer::sync(const std::string& part, const yarp::os::Value& val)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool fingersTuner_IDLServer::tune(const std::string& part, const yarp::os::Value& val) {
  bool _return = false;
  fingersTuner_IDLServer_tune helper;
  helper.part = part;
  helper.val = val;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDLServer::tune(const std::string& part, const yarp::os::Value& val)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool fingersTuner_IDLServer::save() {
  bool _return = false;
  fingersTuner_IDLServer_save helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDLServer::save()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool fingersTuner_IDLServer::quit() {
  bool _return = false;
  fingersTuner_IDLServer_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool fingersTuner_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "sync") {
      std::string part;
      yarp::os::Value val;
      if (!reader.readString(part)) {
        reader.fail();
        return false;
      }
      if (!reader.read(val)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = sync(part,val);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "tune") {
      std::string part;
      yarp::os::Value val;
      if (!reader.readString(part)) {
        reader.fail();
        return false;
      }
      if (!reader.read(val)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = tune(part,val);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "save") {
      bool _return;
      _return = save();
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
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}


