// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <depth2kin_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class depth2kin_IDL_getNumExperts : public yarp::os::Portable {
public:
  int32_t _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_clearExperts : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_load : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_save : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_log : public yarp::os::Portable {
public:
  std::string type;
  bool _return;
  void init(const std::string& type);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_explore : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_stop : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setMaxDist : public yarp::os::Portable {
public:
  double max_dist;
  bool _return;
  void init(const double max_dist);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getMaxDist : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setRoi : public yarp::os::Portable {
public:
  int32_t side;
  bool _return;
  void init(const int32_t side);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getRoi : public yarp::os::Portable {
public:
  int32_t _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setBlockEyes : public yarp::os::Portable {
public:
  double block_eyes;
  bool _return;
  void init(const double block_eyes);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getBlockEyes : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_blockEyes : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_clearEyes : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setArm : public yarp::os::Portable {
public:
  std::string arm;
  bool _return;
  void init(const std::string& arm);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getArm : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setCalibrationType : public yarp::os::Portable {
public:
  std::string type;
  std::string extrapolation;
  bool _return;
  void init(const std::string& type, const std::string& extrapolation);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getCalibrationType : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_calibrate : public yarp::os::Portable {
public:
  bool rm_outliers;
  yarp::os::Property _return;
  void init(const bool rm_outliers);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_pushCalibrator : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setTouchWithExperts : public yarp::os::Portable {
public:
  std::string sw;
  bool _return;
  void init(const std::string& sw);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getTouchWithExperts : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_touch : public yarp::os::Portable {
public:
  int32_t u;
  int32_t v;
  bool _return;
  void init(const int32_t u, const int32_t v);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getPoint : public yarp::os::Portable {
public:
  std::string arm;
  double x;
  double y;
  double z;
  PointReq _return;
  void init(const std::string& arm, const double x, const double y, const double z);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getPoints : public yarp::os::Portable {
public:
  std::string arm;
  std::vector<double>  coordinates;
  std::vector<PointReq>  _return;
  void init(const std::string& arm, const std::vector<double> & coordinates);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setExperiment : public yarp::os::Portable {
public:
  std::string exp;
  std::string v;
  bool _return;
  void init(const std::string& exp, const std::string& v);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getExperiment : public yarp::os::Portable {
public:
  std::string exp;
  std::string _return;
  void init(const std::string& exp);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getExtrinsics : public yarp::os::Portable {
public:
  std::string eye;
  yarp::sig::Vector _return;
  void init(const std::string& eye);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_resetExtrinsics : public yarp::os::Portable {
public:
  std::string eye;
  bool _return;
  void init(const std::string& eye);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setExplorationWait : public yarp::os::Portable {
public:
  double wait;
  bool _return;
  void init(const double wait);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getExplorationWait : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setExplorationInTargetTol : public yarp::os::Portable {
public:
  double tol;
  bool _return;
  void init(const double tol);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getExplorationInTargetTol : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setTouchInTargetTol : public yarp::os::Portable {
public:
  double tol;
  bool _return;
  void init(const double tol);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getTouchInTargetTol : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setExplorationSpace : public yarp::os::Portable {
public:
  double cx;
  double cy;
  double cz;
  double a;
  double b;
  bool _return;
  void init(const double cx, const double cy, const double cz, const double a, const double b);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_setExplorationSpaceDelta : public yarp::os::Portable {
public:
  double dcx;
  double dcy;
  double dcz;
  double da;
  double db;
  bool _return;
  void init(const double dcx, const double dcy, const double dcz, const double da, const double db);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_getExplorationData : public yarp::os::Portable {
public:
  yarp::os::Property _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_clearExplorationData : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_posture : public yarp::os::Portable {
public:
  std::string type;
  bool _return;
  void init(const std::string& type);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_calibrateDepth : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class depth2kin_IDL_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool depth2kin_IDL_getNumExperts::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getNumExperts",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getNumExperts::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readI32(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getNumExperts::init() {
  _return = 0;
}

bool depth2kin_IDL_clearExperts::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("clearExperts",1,1)) return false;
  return true;
}

bool depth2kin_IDL_clearExperts::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_clearExperts::init() {
  _return = false;
}

bool depth2kin_IDL_load::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("load",1,1)) return false;
  return true;
}

bool depth2kin_IDL_load::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_load::init() {
  _return = false;
}

bool depth2kin_IDL_save::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("save",1,1)) return false;
  return true;
}

bool depth2kin_IDL_save::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_save::init() {
  _return = false;
}

bool depth2kin_IDL_log::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("log",1,1)) return false;
  if (!writer.writeString(type)) return false;
  return true;
}

bool depth2kin_IDL_log::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_log::init(const std::string& type) {
  _return = false;
  this->type = type;
}

bool depth2kin_IDL_explore::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("explore",1,1)) return false;
  return true;
}

bool depth2kin_IDL_explore::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_explore::init() {
  _return = false;
}

bool depth2kin_IDL_stop::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("stop",1,1)) return false;
  return true;
}

bool depth2kin_IDL_stop::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_stop::init() {
  _return = false;
}

bool depth2kin_IDL_setMaxDist::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setMaxDist",1,1)) return false;
  if (!writer.writeDouble(max_dist)) return false;
  return true;
}

bool depth2kin_IDL_setMaxDist::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setMaxDist::init(const double max_dist) {
  _return = false;
  this->max_dist = max_dist;
}

bool depth2kin_IDL_getMaxDist::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getMaxDist",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getMaxDist::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getMaxDist::init() {
  _return = (double)0;
}

bool depth2kin_IDL_setRoi::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setRoi",1,1)) return false;
  if (!writer.writeI32(side)) return false;
  return true;
}

bool depth2kin_IDL_setRoi::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setRoi::init(const int32_t side) {
  _return = false;
  this->side = side;
}

bool depth2kin_IDL_getRoi::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getRoi",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getRoi::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readI32(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getRoi::init() {
  _return = 0;
}

bool depth2kin_IDL_setBlockEyes::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setBlockEyes",1,1)) return false;
  if (!writer.writeDouble(block_eyes)) return false;
  return true;
}

bool depth2kin_IDL_setBlockEyes::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setBlockEyes::init(const double block_eyes) {
  _return = false;
  this->block_eyes = block_eyes;
}

bool depth2kin_IDL_getBlockEyes::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getBlockEyes",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getBlockEyes::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getBlockEyes::init() {
  _return = (double)0;
}

bool depth2kin_IDL_blockEyes::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("blockEyes",1,1)) return false;
  return true;
}

bool depth2kin_IDL_blockEyes::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_blockEyes::init() {
  _return = false;
}

bool depth2kin_IDL_clearEyes::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("clearEyes",1,1)) return false;
  return true;
}

bool depth2kin_IDL_clearEyes::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_clearEyes::init() {
  _return = false;
}

bool depth2kin_IDL_setArm::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setArm",1,1)) return false;
  if (!writer.writeString(arm)) return false;
  return true;
}

bool depth2kin_IDL_setArm::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setArm::init(const std::string& arm) {
  _return = false;
  this->arm = arm;
}

bool depth2kin_IDL_getArm::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getArm",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getArm::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getArm::init() {
  _return = "";
}

bool depth2kin_IDL_setCalibrationType::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("setCalibrationType",1,1)) return false;
  if (!writer.writeString(type)) return false;
  if (!writer.writeString(extrapolation)) return false;
  return true;
}

bool depth2kin_IDL_setCalibrationType::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setCalibrationType::init(const std::string& type, const std::string& extrapolation) {
  _return = false;
  this->type = type;
  this->extrapolation = extrapolation;
}

bool depth2kin_IDL_getCalibrationType::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getCalibrationType",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getCalibrationType::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getCalibrationType::init() {
  _return = "";
}

bool depth2kin_IDL_calibrate::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("calibrate",1,1)) return false;
  if (!writer.writeBool(rm_outliers)) return false;
  return true;
}

bool depth2kin_IDL_calibrate::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_calibrate::init(const bool rm_outliers) {
  this->rm_outliers = rm_outliers;
}

bool depth2kin_IDL_pushCalibrator::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("pushCalibrator",1,1)) return false;
  return true;
}

bool depth2kin_IDL_pushCalibrator::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_pushCalibrator::init() {
  _return = false;
}

bool depth2kin_IDL_setTouchWithExperts::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setTouchWithExperts",1,1)) return false;
  if (!writer.writeString(sw)) return false;
  return true;
}

bool depth2kin_IDL_setTouchWithExperts::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setTouchWithExperts::init(const std::string& sw) {
  _return = false;
  this->sw = sw;
}

bool depth2kin_IDL_getTouchWithExperts::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getTouchWithExperts",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getTouchWithExperts::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getTouchWithExperts::init() {
  _return = "";
}

bool depth2kin_IDL_touch::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("touch",1,1)) return false;
  if (!writer.writeI32(u)) return false;
  if (!writer.writeI32(v)) return false;
  return true;
}

bool depth2kin_IDL_touch::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_touch::init(const int32_t u, const int32_t v) {
  _return = false;
  this->u = u;
  this->v = v;
}

bool depth2kin_IDL_getPoint::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("getPoint",1,1)) return false;
  if (!writer.writeString(arm)) return false;
  if (!writer.writeDouble(x)) return false;
  if (!writer.writeDouble(y)) return false;
  if (!writer.writeDouble(z)) return false;
  return true;
}

bool depth2kin_IDL_getPoint::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getPoint::init(const std::string& arm, const double x, const double y, const double z) {
  this->arm = arm;
  this->x = x;
  this->y = y;
  this->z = z;
}

bool depth2kin_IDL_getPoints::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("getPoints",1,1)) return false;
  if (!writer.writeString(arm)) return false;
  {
    if (!writer.writeListBegin(BOTTLE_TAG_DOUBLE, static_cast<uint32_t>(coordinates.size()))) return false;
    std::vector<double> ::iterator _iter0;
    for (_iter0 = coordinates.begin(); _iter0 != coordinates.end(); ++_iter0)
    {
      if (!writer.writeDouble((*_iter0))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}

bool depth2kin_IDL_getPoints::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  {
    _return.clear();
    uint32_t _size1;
    yarp::os::idl::WireState _etype4;
    reader.readListBegin(_etype4, _size1);
    _return.resize(_size1);
    uint32_t _i5;
    for (_i5 = 0; _i5 < _size1; ++_i5)
    {
      if (!reader.readNested(_return[_i5])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}

void depth2kin_IDL_getPoints::init(const std::string& arm, const std::vector<double> & coordinates) {
  this->arm = arm;
  this->coordinates = coordinates;
}

bool depth2kin_IDL_setExperiment::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("setExperiment",1,1)) return false;
  if (!writer.writeString(exp)) return false;
  if (!writer.writeString(v)) return false;
  return true;
}

bool depth2kin_IDL_setExperiment::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setExperiment::init(const std::string& exp, const std::string& v) {
  _return = false;
  this->exp = exp;
  this->v = v;
}

bool depth2kin_IDL_getExperiment::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("getExperiment",1,1)) return false;
  if (!writer.writeString(exp)) return false;
  return true;
}

bool depth2kin_IDL_getExperiment::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getExperiment::init(const std::string& exp) {
  _return = "";
  this->exp = exp;
}

bool depth2kin_IDL_getExtrinsics::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("getExtrinsics",1,1)) return false;
  if (!writer.writeString(eye)) return false;
  return true;
}

bool depth2kin_IDL_getExtrinsics::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getExtrinsics::init(const std::string& eye) {
  this->eye = eye;
}

bool depth2kin_IDL_resetExtrinsics::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("resetExtrinsics",1,1)) return false;
  if (!writer.writeString(eye)) return false;
  return true;
}

bool depth2kin_IDL_resetExtrinsics::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_resetExtrinsics::init(const std::string& eye) {
  _return = false;
  this->eye = eye;
}

bool depth2kin_IDL_setExplorationWait::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setExplorationWait",1,1)) return false;
  if (!writer.writeDouble(wait)) return false;
  return true;
}

bool depth2kin_IDL_setExplorationWait::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setExplorationWait::init(const double wait) {
  _return = false;
  this->wait = wait;
}

bool depth2kin_IDL_getExplorationWait::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getExplorationWait",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getExplorationWait::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getExplorationWait::init() {
  _return = (double)0;
}

bool depth2kin_IDL_setExplorationInTargetTol::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setExplorationInTargetTol",1,1)) return false;
  if (!writer.writeDouble(tol)) return false;
  return true;
}

bool depth2kin_IDL_setExplorationInTargetTol::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setExplorationInTargetTol::init(const double tol) {
  _return = false;
  this->tol = tol;
}

bool depth2kin_IDL_getExplorationInTargetTol::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getExplorationInTargetTol",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getExplorationInTargetTol::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getExplorationInTargetTol::init() {
  _return = (double)0;
}

bool depth2kin_IDL_setTouchInTargetTol::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setTouchInTargetTol",1,1)) return false;
  if (!writer.writeDouble(tol)) return false;
  return true;
}

bool depth2kin_IDL_setTouchInTargetTol::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setTouchInTargetTol::init(const double tol) {
  _return = false;
  this->tol = tol;
}

bool depth2kin_IDL_getTouchInTargetTol::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getTouchInTargetTol",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getTouchInTargetTol::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getTouchInTargetTol::init() {
  _return = (double)0;
}

bool depth2kin_IDL_setExplorationSpace::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  if (!writer.writeTag("setExplorationSpace",1,1)) return false;
  if (!writer.writeDouble(cx)) return false;
  if (!writer.writeDouble(cy)) return false;
  if (!writer.writeDouble(cz)) return false;
  if (!writer.writeDouble(a)) return false;
  if (!writer.writeDouble(b)) return false;
  return true;
}

bool depth2kin_IDL_setExplorationSpace::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setExplorationSpace::init(const double cx, const double cy, const double cz, const double a, const double b) {
  _return = false;
  this->cx = cx;
  this->cy = cy;
  this->cz = cz;
  this->a = a;
  this->b = b;
}

bool depth2kin_IDL_setExplorationSpaceDelta::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  if (!writer.writeTag("setExplorationSpaceDelta",1,1)) return false;
  if (!writer.writeDouble(dcx)) return false;
  if (!writer.writeDouble(dcy)) return false;
  if (!writer.writeDouble(dcz)) return false;
  if (!writer.writeDouble(da)) return false;
  if (!writer.writeDouble(db)) return false;
  return true;
}

bool depth2kin_IDL_setExplorationSpaceDelta::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_setExplorationSpaceDelta::init(const double dcx, const double dcy, const double dcz, const double da, const double db) {
  _return = false;
  this->dcx = dcx;
  this->dcy = dcy;
  this->dcz = dcz;
  this->da = da;
  this->db = db;
}

bool depth2kin_IDL_getExplorationData::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getExplorationData",1,1)) return false;
  return true;
}

bool depth2kin_IDL_getExplorationData::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_getExplorationData::init() {
}

bool depth2kin_IDL_clearExplorationData::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("clearExplorationData",1,1)) return false;
  return true;
}

bool depth2kin_IDL_clearExplorationData::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_clearExplorationData::init() {
  _return = false;
}

bool depth2kin_IDL_posture::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("posture",1,1)) return false;
  if (!writer.writeString(type)) return false;
  return true;
}

bool depth2kin_IDL_posture::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_posture::init(const std::string& type) {
  _return = false;
  this->type = type;
}

bool depth2kin_IDL_calibrateDepth::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("calibrateDepth",1,1)) return false;
  return true;
}

bool depth2kin_IDL_calibrateDepth::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_calibrateDepth::init() {
  _return = false;
}

bool depth2kin_IDL_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool depth2kin_IDL_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void depth2kin_IDL_quit::init() {
  _return = false;
}

depth2kin_IDL::depth2kin_IDL() {
  yarp().setOwner(*this);
}
int32_t depth2kin_IDL::getNumExperts() {
  int32_t _return = 0;
  depth2kin_IDL_getNumExperts helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","int32_t depth2kin_IDL::getNumExperts()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::clearExperts() {
  bool _return = false;
  depth2kin_IDL_clearExperts helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::clearExperts()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::load() {
  bool _return = false;
  depth2kin_IDL_load helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::load()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::save() {
  bool _return = false;
  depth2kin_IDL_save helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::save()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::log(const std::string& type) {
  bool _return = false;
  depth2kin_IDL_log helper;
  helper.init(type);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::log(const std::string& type)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::explore() {
  bool _return = false;
  depth2kin_IDL_explore helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::explore()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::stop() {
  bool _return = false;
  depth2kin_IDL_stop helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::stop()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setMaxDist(const double max_dist) {
  bool _return = false;
  depth2kin_IDL_setMaxDist helper;
  helper.init(max_dist);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setMaxDist(const double max_dist)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double depth2kin_IDL::getMaxDist() {
  double _return = (double)0;
  depth2kin_IDL_getMaxDist helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double depth2kin_IDL::getMaxDist()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setRoi(const int32_t side) {
  bool _return = false;
  depth2kin_IDL_setRoi helper;
  helper.init(side);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setRoi(const int32_t side)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
int32_t depth2kin_IDL::getRoi() {
  int32_t _return = 0;
  depth2kin_IDL_getRoi helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","int32_t depth2kin_IDL::getRoi()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setBlockEyes(const double block_eyes) {
  bool _return = false;
  depth2kin_IDL_setBlockEyes helper;
  helper.init(block_eyes);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setBlockEyes(const double block_eyes)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double depth2kin_IDL::getBlockEyes() {
  double _return = (double)0;
  depth2kin_IDL_getBlockEyes helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double depth2kin_IDL::getBlockEyes()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::blockEyes() {
  bool _return = false;
  depth2kin_IDL_blockEyes helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::blockEyes()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::clearEyes() {
  bool _return = false;
  depth2kin_IDL_clearEyes helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::clearEyes()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setArm(const std::string& arm) {
  bool _return = false;
  depth2kin_IDL_setArm helper;
  helper.init(arm);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setArm(const std::string& arm)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string depth2kin_IDL::getArm() {
  std::string _return = "";
  depth2kin_IDL_getArm helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string depth2kin_IDL::getArm()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setCalibrationType(const std::string& type, const std::string& extrapolation) {
  bool _return = false;
  depth2kin_IDL_setCalibrationType helper;
  helper.init(type,extrapolation);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setCalibrationType(const std::string& type, const std::string& extrapolation)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string depth2kin_IDL::getCalibrationType() {
  std::string _return = "";
  depth2kin_IDL_getCalibrationType helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string depth2kin_IDL::getCalibrationType()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Property depth2kin_IDL::calibrate(const bool rm_outliers) {
  yarp::os::Property _return;
  depth2kin_IDL_calibrate helper;
  helper.init(rm_outliers);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Property depth2kin_IDL::calibrate(const bool rm_outliers)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::pushCalibrator() {
  bool _return = false;
  depth2kin_IDL_pushCalibrator helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::pushCalibrator()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setTouchWithExperts(const std::string& sw) {
  bool _return = false;
  depth2kin_IDL_setTouchWithExperts helper;
  helper.init(sw);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setTouchWithExperts(const std::string& sw)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string depth2kin_IDL::getTouchWithExperts() {
  std::string _return = "";
  depth2kin_IDL_getTouchWithExperts helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string depth2kin_IDL::getTouchWithExperts()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::touch(const int32_t u, const int32_t v) {
  bool _return = false;
  depth2kin_IDL_touch helper;
  helper.init(u,v);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::touch(const int32_t u, const int32_t v)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
PointReq depth2kin_IDL::getPoint(const std::string& arm, const double x, const double y, const double z) {
  PointReq _return;
  depth2kin_IDL_getPoint helper;
  helper.init(arm,x,y,z);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","PointReq depth2kin_IDL::getPoint(const std::string& arm, const double x, const double y, const double z)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::vector<PointReq>  depth2kin_IDL::getPoints(const std::string& arm, const std::vector<double> & coordinates) {
  std::vector<PointReq>  _return;
  depth2kin_IDL_getPoints helper;
  helper.init(arm,coordinates);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::vector<PointReq>  depth2kin_IDL::getPoints(const std::string& arm, const std::vector<double> & coordinates)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setExperiment(const std::string& exp, const std::string& v) {
  bool _return = false;
  depth2kin_IDL_setExperiment helper;
  helper.init(exp,v);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setExperiment(const std::string& exp, const std::string& v)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string depth2kin_IDL::getExperiment(const std::string& exp) {
  std::string _return = "";
  depth2kin_IDL_getExperiment helper;
  helper.init(exp);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string depth2kin_IDL::getExperiment(const std::string& exp)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::sig::Vector depth2kin_IDL::getExtrinsics(const std::string& eye) {
  yarp::sig::Vector _return;
  depth2kin_IDL_getExtrinsics helper;
  helper.init(eye);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::sig::Vector depth2kin_IDL::getExtrinsics(const std::string& eye)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::resetExtrinsics(const std::string& eye) {
  bool _return = false;
  depth2kin_IDL_resetExtrinsics helper;
  helper.init(eye);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::resetExtrinsics(const std::string& eye)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setExplorationWait(const double wait) {
  bool _return = false;
  depth2kin_IDL_setExplorationWait helper;
  helper.init(wait);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setExplorationWait(const double wait)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double depth2kin_IDL::getExplorationWait() {
  double _return = (double)0;
  depth2kin_IDL_getExplorationWait helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double depth2kin_IDL::getExplorationWait()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setExplorationInTargetTol(const double tol) {
  bool _return = false;
  depth2kin_IDL_setExplorationInTargetTol helper;
  helper.init(tol);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setExplorationInTargetTol(const double tol)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double depth2kin_IDL::getExplorationInTargetTol() {
  double _return = (double)0;
  depth2kin_IDL_getExplorationInTargetTol helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double depth2kin_IDL::getExplorationInTargetTol()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setTouchInTargetTol(const double tol) {
  bool _return = false;
  depth2kin_IDL_setTouchInTargetTol helper;
  helper.init(tol);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setTouchInTargetTol(const double tol)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double depth2kin_IDL::getTouchInTargetTol() {
  double _return = (double)0;
  depth2kin_IDL_getTouchInTargetTol helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double depth2kin_IDL::getTouchInTargetTol()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setExplorationSpace(const double cx, const double cy, const double cz, const double a, const double b) {
  bool _return = false;
  depth2kin_IDL_setExplorationSpace helper;
  helper.init(cx,cy,cz,a,b);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setExplorationSpace(const double cx, const double cy, const double cz, const double a, const double b)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::setExplorationSpaceDelta(const double dcx, const double dcy, const double dcz, const double da, const double db) {
  bool _return = false;
  depth2kin_IDL_setExplorationSpaceDelta helper;
  helper.init(dcx,dcy,dcz,da,db);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::setExplorationSpaceDelta(const double dcx, const double dcy, const double dcz, const double da, const double db)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Property depth2kin_IDL::getExplorationData() {
  yarp::os::Property _return;
  depth2kin_IDL_getExplorationData helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Property depth2kin_IDL::getExplorationData()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::clearExplorationData() {
  bool _return = false;
  depth2kin_IDL_clearExplorationData helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::clearExplorationData()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::posture(const std::string& type) {
  bool _return = false;
  depth2kin_IDL_posture helper;
  helper.init(type);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::posture(const std::string& type)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::calibrateDepth() {
  bool _return = false;
  depth2kin_IDL_calibrateDepth helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::calibrateDepth()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool depth2kin_IDL::quit() {
  bool _return = false;
  depth2kin_IDL_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool depth2kin_IDL::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool depth2kin_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "getNumExperts") {
      int32_t _return;
      _return = getNumExperts();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeI32(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "clearExperts") {
      bool _return;
      _return = clearExperts();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "load") {
      bool _return;
      _return = load();
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
    if (tag == "log") {
      std::string type;
      if (!reader.readString(type)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = log(type);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "explore") {
      bool _return;
      _return = explore();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stop") {
      bool _return;
      _return = stop();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setMaxDist") {
      double max_dist;
      if (!reader.readDouble(max_dist)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setMaxDist(max_dist);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getMaxDist") {
      double _return;
      _return = getMaxDist();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setRoi") {
      int32_t side;
      if (!reader.readI32(side)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setRoi(side);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getRoi") {
      int32_t _return;
      _return = getRoi();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeI32(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setBlockEyes") {
      double block_eyes;
      if (!reader.readDouble(block_eyes)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setBlockEyes(block_eyes);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getBlockEyes") {
      double _return;
      _return = getBlockEyes();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "blockEyes") {
      bool _return;
      _return = blockEyes();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "clearEyes") {
      bool _return;
      _return = clearEyes();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setArm") {
      std::string arm;
      if (!reader.readString(arm)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setArm(arm);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getArm") {
      std::string _return;
      _return = getArm();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setCalibrationType") {
      std::string type;
      std::string extrapolation;
      if (!reader.readString(type)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(extrapolation)) {
        extrapolation = "auto";
      }
      bool _return;
      _return = setCalibrationType(type,extrapolation);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getCalibrationType") {
      std::string _return;
      _return = getCalibrationType();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibrate") {
      bool rm_outliers;
      if (!reader.readBool(rm_outliers)) {
        rm_outliers = 1;
      }
      yarp::os::Property _return;
      _return = calibrate(rm_outliers);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "pushCalibrator") {
      bool _return;
      _return = pushCalibrator();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setTouchWithExperts") {
      std::string sw;
      if (!reader.readString(sw)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setTouchWithExperts(sw);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getTouchWithExperts") {
      std::string _return;
      _return = getTouchWithExperts();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "touch") {
      int32_t u;
      int32_t v;
      if (!reader.readI32(u)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(v)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = touch(u,v);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getPoint") {
      std::string arm;
      double x;
      double y;
      double z;
      if (!reader.readString(arm)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(x)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(y)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(z)) {
        reader.fail();
        return false;
      }
      PointReq _return;
      _return = getPoint(arm,x,y,z);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(4)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getPoints") {
      std::string arm;
      std::vector<double>  coordinates;
      if (!reader.readString(arm)) {
        reader.fail();
        return false;
      }
      {
        coordinates.clear();
        uint32_t _size6;
        yarp::os::idl::WireState _etype9;
        reader.readListBegin(_etype9, _size6);
        coordinates.resize(_size6);
        uint32_t _i10;
        for (_i10 = 0; _i10 < _size6; ++_i10)
        {
          if (!reader.readDouble(coordinates[_i10])) {
            reader.fail();
            return false;
          }
        }
        reader.readListEnd();
      }
      std::vector<PointReq>  _return;
      _return = getPoints(arm,coordinates);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<PointReq> ::iterator _iter11;
          for (_iter11 = _return.begin(); _iter11 != _return.end(); ++_iter11)
          {
            if (!writer.writeNested((*_iter11))) return false;
          }
          if (!writer.writeListEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "setExperiment") {
      std::string exp;
      std::string v;
      if (!reader.readString(exp)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(v)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setExperiment(exp,v);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getExperiment") {
      std::string exp;
      if (!reader.readString(exp)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = getExperiment(exp);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getExtrinsics") {
      std::string eye;
      if (!reader.readString(eye)) {
        reader.fail();
        return false;
      }
      yarp::sig::Vector _return;
      _return = getExtrinsics(eye);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "resetExtrinsics") {
      std::string eye;
      if (!reader.readString(eye)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = resetExtrinsics(eye);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setExplorationWait") {
      double wait;
      if (!reader.readDouble(wait)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setExplorationWait(wait);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getExplorationWait") {
      double _return;
      _return = getExplorationWait();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setExplorationInTargetTol") {
      double tol;
      if (!reader.readDouble(tol)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setExplorationInTargetTol(tol);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getExplorationInTargetTol") {
      double _return;
      _return = getExplorationInTargetTol();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setTouchInTargetTol") {
      double tol;
      if (!reader.readDouble(tol)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setTouchInTargetTol(tol);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getTouchInTargetTol") {
      double _return;
      _return = getTouchInTargetTol();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setExplorationSpace") {
      double cx;
      double cy;
      double cz;
      double a;
      double b;
      if (!reader.readDouble(cx)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(cy)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(cz)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(a)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(b)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setExplorationSpace(cx,cy,cz,a,b);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setExplorationSpaceDelta") {
      double dcx;
      double dcy;
      double dcz;
      double da;
      double db;
      if (!reader.readDouble(dcx)) {
        dcx = 0;
      }
      if (!reader.readDouble(dcy)) {
        dcy = 0;
      }
      if (!reader.readDouble(dcz)) {
        dcz = 0;
      }
      if (!reader.readDouble(da)) {
        da = 0;
      }
      if (!reader.readDouble(db)) {
        db = 0;
      }
      bool _return;
      _return = setExplorationSpaceDelta(dcx,dcy,dcz,da,db);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getExplorationData") {
      yarp::os::Property _return;
      _return = getExplorationData();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "clearExplorationData") {
      bool _return;
      _return = clearExplorationData();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "posture") {
      std::string type;
      if (!reader.readString(type)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = posture(type);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibrateDepth") {
      bool _return;
      _return = calibrateDepth();
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

std::vector<std::string> depth2kin_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("getNumExperts");
    helpString.push_back("clearExperts");
    helpString.push_back("load");
    helpString.push_back("save");
    helpString.push_back("log");
    helpString.push_back("explore");
    helpString.push_back("stop");
    helpString.push_back("setMaxDist");
    helpString.push_back("getMaxDist");
    helpString.push_back("setRoi");
    helpString.push_back("getRoi");
    helpString.push_back("setBlockEyes");
    helpString.push_back("getBlockEyes");
    helpString.push_back("blockEyes");
    helpString.push_back("clearEyes");
    helpString.push_back("setArm");
    helpString.push_back("getArm");
    helpString.push_back("setCalibrationType");
    helpString.push_back("getCalibrationType");
    helpString.push_back("calibrate");
    helpString.push_back("pushCalibrator");
    helpString.push_back("setTouchWithExperts");
    helpString.push_back("getTouchWithExperts");
    helpString.push_back("touch");
    helpString.push_back("getPoint");
    helpString.push_back("getPoints");
    helpString.push_back("setExperiment");
    helpString.push_back("getExperiment");
    helpString.push_back("getExtrinsics");
    helpString.push_back("resetExtrinsics");
    helpString.push_back("setExplorationWait");
    helpString.push_back("getExplorationWait");
    helpString.push_back("setExplorationInTargetTol");
    helpString.push_back("getExplorationInTargetTol");
    helpString.push_back("setTouchInTargetTol");
    helpString.push_back("getTouchInTargetTol");
    helpString.push_back("setExplorationSpace");
    helpString.push_back("setExplorationSpaceDelta");
    helpString.push_back("getExplorationData");
    helpString.push_back("clearExplorationData");
    helpString.push_back("posture");
    helpString.push_back("calibrateDepth");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="getNumExperts") {
      helpString.push_back("int32_t getNumExperts() ");
      helpString.push_back("Return the number of available experts. ");
      helpString.push_back("@return the number of available experts. ");
    }
    if (functionName=="clearExperts") {
      helpString.push_back("bool clearExperts() ");
      helpString.push_back("Clear the list of currently available experts. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="load") {
      helpString.push_back("bool load() ");
      helpString.push_back("Reload the list of experts stored within the ");
      helpString.push_back("configuration file. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="save") {
      helpString.push_back("bool save() ");
      helpString.push_back("Save the current list of experts into the ");
      helpString.push_back("configuration file. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="log") {
      helpString.push_back("bool log(const std::string& type) ");
      helpString.push_back("Store on file the log of system response computed ");
      helpString.push_back("out of the explored set of input-output pairs. ");
      helpString.push_back("@param type can be \"experts\" or \"calibrator\", accounting either ");
      helpString.push_back("for the response of mixture of available experts or the output ");
      helpString.push_back("of the current calibrator, respectively. ");
      helpString.push_back("@return true/false on success/failure. It returns false also if ");
      helpString.push_back("\"calibrator\" is selected and calibration has not been performed yet. ");
      helpString.push_back("@note Each row of the file will contain the following data: \n ");
      helpString.push_back("\f$ d_x d_y d_z k_x k_y k_z r_x r_y r_z e, \f$ where \f$ d \f$ ");
      helpString.push_back("is the depth point, \f$ k \f$ is the kinematic point, \f$ r \f$ ");
      helpString.push_back("is the system response and \f$ e=|k-r| \f$ is the error. ");
    }
    if (functionName=="explore") {
      helpString.push_back("bool explore() ");
      helpString.push_back("Start the exploration phase. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="stop") {
      helpString.push_back("bool stop() ");
      helpString.push_back("Yield an asynchronous stop of the exploration phase. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setMaxDist") {
      helpString.push_back("bool setMaxDist(const double max_dist) ");
      helpString.push_back("Set the maximum allowed distance between the depth point and ");
      helpString.push_back("kinematic prediction to enable data collection. ");
      helpString.push_back("@param max_dist the value in meters. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getMaxDist") {
      helpString.push_back("double getMaxDist() ");
      helpString.push_back("Return the maximum allowed distance between depth point and ");
      helpString.push_back("kinematic prediction to enable data collection. ");
      helpString.push_back("@return the distance. ");
    }
    if (functionName=="setRoi") {
      helpString.push_back("bool setRoi(const int32_t side) ");
      helpString.push_back("Set the side of the squared window used to filter data ");
      helpString.push_back("collection in the image plane. ");
      helpString.push_back("@param side the length of the window side. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getRoi") {
      helpString.push_back("int32_t getRoi() ");
      helpString.push_back("Return the side of the squared window used to filter data ");
      helpString.push_back("collection in the image plane. ");
      helpString.push_back("@return the window side. ");
    }
    if (functionName=="setBlockEyes") {
      helpString.push_back("bool setBlockEyes(const double block_eyes) ");
      helpString.push_back("Set the vergence angle used to keep the gaze fixed. ");
      helpString.push_back("@param block_eyes the value in degrees of the vergence. It must ");
      helpString.push_back("be equal or greater than the minimum vergence angle allowed ");
      helpString.push_back("by the gaze controller. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getBlockEyes") {
      helpString.push_back("double getBlockEyes() ");
      helpString.push_back("Return the current angle to keep the vergence at. ");
      helpString.push_back("@return the vergence angle in degrees. ");
    }
    if (functionName=="blockEyes") {
      helpString.push_back("bool blockEyes() ");
      helpString.push_back("Tell the gaze to immediately steer the eyes to the stored ");
      helpString.push_back("vergence angle and stay still. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="clearEyes") {
      helpString.push_back("bool clearEyes() ");
      helpString.push_back("Remove the block on the eyes. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setArm") {
      helpString.push_back("bool setArm(const std::string& arm) ");
      helpString.push_back("Select the arm to deal with. ");
      helpString.push_back("@param arm is \"left\" or \"right\". ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getArm") {
      helpString.push_back("std::string getArm() ");
      helpString.push_back("Return the current arm. ");
      helpString.push_back("@return \"left\" or \"right\". ");
    }
    if (functionName=="setCalibrationType") {
      helpString.push_back("bool setCalibrationType(const std::string& type, const std::string& extrapolation = \"auto\") ");
      helpString.push_back("Set up the calibrator type. ");
      helpString.push_back("@param type can be one of the following: \n ");
      helpString.push_back("\"se3\", \"se3+scale\", \"affine\", \"lssvm\". ");
      helpString.push_back("@param extrapolation specifies whether the calibrator will be ");
      helpString.push_back("used for extrapolating data (\"true\") or not (\"false\"); if \"auto\" ");
      helpString.push_back("is provided, then automatic choice is taken depending on the type. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getCalibrationType") {
      helpString.push_back("std::string getCalibrationType() ");
      helpString.push_back("Return the current calibration type. ");
      helpString.push_back("@return the calibration type. ");
    }
    if (functionName=="calibrate") {
      helpString.push_back("yarp::os::Property calibrate(const bool rm_outliers = 1) ");
      helpString.push_back("Ask the current calibrator to carry out the calibration. ");
      helpString.push_back("@param rm_outliers if true outliers removal is performed. ");
      helpString.push_back("@return a property containing the output in terms of ");
      helpString.push_back("calibration errors for each subsystem: \"calibrator\", \"aligner\". ");
    }
    if (functionName=="pushCalibrator") {
      helpString.push_back("bool pushCalibrator() ");
      helpString.push_back("Push the current calibrator in the list of experts. ");
      helpString.push_back("@return true/false on success/failure. ");
      helpString.push_back("@note the calibrator needs to have been calibrated at least once. ");
    }
    if (functionName=="setTouchWithExperts") {
      helpString.push_back("bool setTouchWithExperts(const std::string& sw) ");
      helpString.push_back("Enable/disable the use of experts for touch test. ");
      helpString.push_back("@param switch is \"on\"/\"off\" to use/not-use the experts. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getTouchWithExperts") {
      helpString.push_back("std::string getTouchWithExperts() ");
      helpString.push_back("Return the current status of the switch for experts usage ");
      helpString.push_back("during touch test. ");
      helpString.push_back("@return \"on\"/\"off\" if experts are used/not-used. ");
    }
    if (functionName=="touch") {
      helpString.push_back("bool touch(const int32_t u, const int32_t v) ");
      helpString.push_back("Yield a <i>touch</i> action with the finger on a depth point. ");
      helpString.push_back("@param u the u-coordinate of the depth point in the image plane. ");
      helpString.push_back("@param v the v-coordinate of the depth point in the image plane. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getPoint") {
      helpString.push_back("PointReq getPoint(const std::string& arm, const double x, const double y, const double z) ");
      helpString.push_back("Retrieve the compensated kinematic point corresponding to the input ");
      helpString.push_back("depth point. ");
      helpString.push_back("@param arm accounts for \"left\" or \"right\" list of experts. ");
      helpString.push_back("@param x the x-coordinate of the depth point. ");
      helpString.push_back("@param y the y-coordinate of the depth point. ");
      helpString.push_back("@param z the z-coordinate of the depth point. ");
      helpString.push_back("@return the requested point in \ref PointReq format. ");
    }
    if (functionName=="getPoints") {
      helpString.push_back("std::vector<PointReq>  getPoints(const std::string& arm, const std::vector<double> & coordinates) ");
      helpString.push_back("Retrieve the compensated kinematic points corresponding to the input ");
      helpString.push_back("depth points. ");
      helpString.push_back("@param arm accounts for \"left\" or \"right\" list of experts. ");
      helpString.push_back("@param coordinates the 3D coordinates of the depth points. ");
      helpString.push_back("@return the requested points in \ref PointReq format. ");
    }
    if (functionName=="setExperiment") {
      helpString.push_back("bool setExperiment(const std::string& exp, const std::string& v) ");
      helpString.push_back("Set on/off an experiment. ");
      helpString.push_back("@param exp the experiment (\"depth2kin\" or \"aligneyes\") to switch on/off. ");
      helpString.push_back("@param v is \"on\" or \"off\". ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getExperiment") {
      helpString.push_back("std::string getExperiment(const std::string& exp) ");
      helpString.push_back("Return the current status of the experiment. ");
      helpString.push_back("@param exp the experiment (\"depth2kin\" or \"aligneyes\") ");
      helpString.push_back("@return \"on\"/\"off\". ");
    }
    if (functionName=="getExtrinsics") {
      helpString.push_back("yarp::sig::Vector getExtrinsics(const std::string& eye) ");
      helpString.push_back("Retrieve the current extrinsics camera parameters. ");
      helpString.push_back("@param eye is \"left\" or \"right\" camera eye. ");
      helpString.push_back("@return a 6x1 Vector containing the translational and the ");
      helpString.push_back("rotational (in roll-pith-yaw convention) parts of the ");
      helpString.push_back("extrinsics matrix. ");
    }
    if (functionName=="resetExtrinsics") {
      helpString.push_back("bool resetExtrinsics(const std::string& eye) ");
      helpString.push_back("Reset the extrinsics matrix to default eye matrix. ");
      helpString.push_back("@param eye is \"left\" or \"right\" camera eye. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setExplorationWait") {
      helpString.push_back("bool setExplorationWait(const double wait) ");
      helpString.push_back("Set up the wait timeout used during exploration between ");
      helpString.push_back("two consecutive data points. ");
      helpString.push_back("@param wait the timeout in seconds. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getExplorationWait") {
      helpString.push_back("double getExplorationWait() ");
      helpString.push_back("Return the current wait timeout used during exploration ");
      helpString.push_back("between two consecutive data points. ");
      helpString.push_back("@return the wait timeout in seconds. ");
    }
    if (functionName=="setExplorationInTargetTol") {
      helpString.push_back("bool setExplorationInTargetTol(const double tol) ");
      helpString.push_back("Set up the cartesian tolerance used during exploration. ");
      helpString.push_back("@param tol the overall tolerance employed for the ");
      helpString.push_back("cartesian movements. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getExplorationInTargetTol") {
      helpString.push_back("double getExplorationInTargetTol() ");
      helpString.push_back("Return the current cartesian tolerance used ");
      helpString.push_back("during exploration. ");
      helpString.push_back("@return the tolerance. ");
    }
    if (functionName=="setTouchInTargetTol") {
      helpString.push_back("bool setTouchInTargetTol(const double tol) ");
      helpString.push_back("Set up the cartesian tolerance used during a touch actions. ");
      helpString.push_back("@param tol the overall tolerance employed for the ");
      helpString.push_back("cartesian movements. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getTouchInTargetTol") {
      helpString.push_back("double getTouchInTargetTol() ");
      helpString.push_back("Return the current cartesian tolerance used ");
      helpString.push_back("during touch actions. ");
      helpString.push_back("@return the tolerance. ");
    }
    if (functionName=="setExplorationSpace") {
      helpString.push_back("bool setExplorationSpace(const double cx, const double cy, const double cz, const double a, const double b) ");
      helpString.push_back("Set up the internally coded exploration space composed by ");
      helpString.push_back("two co-centered ellipses, one orthogonal to other, and defined ");
      helpString.push_back("by means of the center and the two semi-axes. ");
      helpString.push_back("@param cx the center x-coordinate. ");
      helpString.push_back("@param cy the center y-coordinate. ");
      helpString.push_back("@param cz the center z-coordiante. ");
      helpString.push_back("@param a the major semi-axis length. ");
      helpString.push_back("@param b the minor semi-axis length. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setExplorationSpaceDelta") {
      helpString.push_back("bool setExplorationSpaceDelta(const double dcx = 0, const double dcy = 0, const double dcz = 0, const double da = 0, const double db = 0) ");
      helpString.push_back("Set up the exploration space in terms of differences with respect ");
      helpString.push_back("to the internally coded couple of ellipses. ");
      helpString.push_back("@param dcx the center delta x-coordinate. ");
      helpString.push_back("@param dcy the center delta y-coordinate. ");
      helpString.push_back("@param dcz the center delta z-coordiante. ");
      helpString.push_back("@param da the major semi-axis delta length. ");
      helpString.push_back("@param db the minor semi-axis delta length. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="getExplorationData") {
      helpString.push_back("yarp::os::Property getExplorationData() ");
      helpString.push_back("Return some progress about the ongoing exploration. ");
      helpString.push_back("@return a property that looks like ");
      helpString.push_back("(\"status\" [\"idle\"|\"ongoing\"]) (\"total_points\" <int>) (\"remaining_points\" <int>) ");
      helpString.push_back("(\"calibrator_points\" <int>) (\"aligner_points\" <int>) ");
    }
    if (functionName=="clearExplorationData") {
      helpString.push_back("bool clearExplorationData() ");
      helpString.push_back("Clean up the internal list of explored points pairs. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="posture") {
      helpString.push_back("bool posture(const std::string& type) ");
      helpString.push_back("Make the robot reach a predefined posture. ");
      helpString.push_back("@param type can be one of the following: \n ");
      helpString.push_back("\"home\", \"look_hands\". ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="calibrateDepth") {
      helpString.push_back("bool calibrateDepth() ");
      helpString.push_back("Put the robot in a suitable predefined posture ");
      helpString.push_back("and then execute depth calibration. ");
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


