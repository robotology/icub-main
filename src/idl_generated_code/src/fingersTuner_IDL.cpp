// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <fingersTuner_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class fingersTuner_IDL_sync : public yarp::os::Portable {
public:
  std::string part;
  yarp::os::Value val;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
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

class fingersTuner_IDL_tune : public yarp::os::Portable {
public:
  std::string part;
  yarp::os::Value val;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
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

class fingersTuner_IDL_save : public yarp::os::Portable {
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

class fingersTuner_IDL_quit : public yarp::os::Portable {
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

bool fingersTuner_IDL::sync(const std::string& part, const yarp::os::Value& val) {
  bool _return = false;
  fingersTuner_IDL_sync helper;
  helper.part = part;
  helper.val = val;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDL::sync(const std::string& part, const yarp::os::Value& val)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool fingersTuner_IDL::tune(const std::string& part, const yarp::os::Value& val) {
  bool _return = false;
  fingersTuner_IDL_tune helper;
  helper.part = part;
  helper.val = val;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDL::tune(const std::string& part, const yarp::os::Value& val)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool fingersTuner_IDL::save() {
  bool _return = false;
  fingersTuner_IDL_save helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDL::save()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool fingersTuner_IDL::quit() {
  bool _return = false;
  fingersTuner_IDL_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool fingersTuner_IDL::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool fingersTuner_IDL::read(yarp::os::ConnectionReader& connection) {
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
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
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

std::vector<std::string> fingersTuner_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("sync");
    helpString.push_back("tune");
    helpString.push_back("save");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="sync") {
      helpString.push_back("bool sync(const std::string& part, const yarp::os::Value& val) ");
      helpString.push_back("Synchronize PID values with values stored ");
      helpString.push_back("in the configuration file. ");
      helpString.push_back("@param part specifies the part name as per ");
      helpString.push_back("configuration file. ");
      helpString.push_back("@param val accounts for a single joint if ");
      helpString.push_back("the corresponding integer is given or a set ");
      helpString.push_back("of joints if the corresponding alias is provided ");
      helpString.push_back("as defined within the configuration file. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="tune") {
      helpString.push_back("bool tune(const std::string& part, const yarp::os::Value& val) ");
      helpString.push_back("Tune PID of a joint or a set of joints. ");
      helpString.push_back("@param part specifies the part name as per ");
      helpString.push_back("configuration file. ");
      helpString.push_back("@param val accounts for a single joint if ");
      helpString.push_back("the corresponding integer is given or a set ");
      helpString.push_back("of joints if the corresponding alias is provided ");
      helpString.push_back("as defined within the configuration file. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="save") {
      helpString.push_back("bool save() ");
      helpString.push_back("Save the PID parameters on configuration file. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


