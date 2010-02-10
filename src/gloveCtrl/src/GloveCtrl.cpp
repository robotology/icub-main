/*******************************************************************************
 * Copyright (C) 2009-2010 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#include "iCub/vislab/GloveCtrl.h"

#include <iCub/vislab/Hand.h>
#include <iCub/vislab/HandMetrics.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include <iostream>
#include <stdexcept>

//#include "iCub/vislab/glovectrl/config.h"
// TODO: To use the flock of birds, un-comment the line above and install the library
// CMake will set the USE_FLOCKOFBIRDS for you!

#ifdef USE_FLOCKOFBIRDS

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <wincon.h>
#define _WIN32_WINNT 0x0400

#ifdef __cplusplus
  extern "C" {
  #endif
  #include "Bird.h"
  #ifdef __cplusplus
  }
#endif

#define GROUP_ID            1                       // Designation for the group
#define READ_TIMEOUT        2000                    // 2000 ms
#define WRITE_TIMEOUT       2000                    // 2000 ms
#define BAUD_RATE           115200                  // 115.2K baud

#endif

using namespace vislab::yarp::util;

using namespace yarp::os;
using namespace yarp::sig;

using namespace std;
using namespace glove::calib;
using namespace glove::devices;
using namespace glove::devices::remote;

namespace vislab {
namespace control {

GloveCtrl::GloveCtrl() :
  ThreadedRFModule("gloveCtrl"), devMgr(NULL) {
}

GloveCtrl::~GloveCtrl() {
  // This would only be the case if YARP fails doing its usual job
  if (devMgr != NULL) {
    delete devMgr;
  }
}

bool GloveCtrl::StartCommand::execute(const Bottle& params, Bottle& reply) const {
  GloveCtrl* g = (GloveCtrl*) parent;
  g->workerThread->enableStreaming(true);
  reply.addVocab(Vocab::encode("ack"));
  return true;
}

bool GloveCtrl::StopCommand::execute(const Bottle& params, Bottle& reply) const {
  GloveCtrl* g = (GloveCtrl*) parent;
  g->workerThread->enableStreaming(false);
  reply.addVocab(Vocab::encode("ack"));
  return true;
}

bool GloveCtrl::configure(ResourceFinder &rf) {
  if (!RFModule2::configure(rf)) {
    return false;
  }

  addRemoteCommand(new StartCommand(this));
  addRemoteCommand(new StopCommand(this));

  ConstString str = "Dis-/Enables the broadcasting of the glove values";
  addModuleOption(new Option("streaming", str, Option::ON_OFF, Value("off")));

  str = "TCP/IP address of the device manager";
  str = rf.check("devMgr.address", Value("127.0.0.1")).asString();
  string address = str.c_str();

  str = "TCP/IP port of the device manager";
  str = rf.check("devMgr.port", Value("12345")).toString();
  // TODO: toString() vs asString() for numbers
  string port = str.c_str();

  str = "The linear calibration of the CyberGlove(R) device to be used";
  str = rf.check("gloveCalib", Value("glove.calib")).asString();
  string gloveCalib = str.c_str();

  str = "Output port providing the joint angles as Vector(16)";
  str = prefix + getName(rf.check("out", Value("/out"), str).asString());
  dataPorts.add(id.Output_Q, str, new BufferedPort<Vector> );

  // Set up the connection to the CyberGlove's device manager
  try {
    devMgr = new RemoteDeviceManager(address, port);
    vector<string> list = devMgr->getDeviceList();

    device_info devInfo;
    devInfo.id = -1;

    for (size_t i = 0; i < list.size(); i++) {
      devInfo = DeviceManager<RemoteDevice>::parseDeviceInfo(list[i]);
      if (devInfo.type == CyberGlove::DEVICE_ID) {
        // Use the first "cyberglove" found in the device list
        break;
      }
    }

    if (devInfo.id < 0) {
      cerr << "No CyberGlove(R) devices connected" << endl;
      return false;
    }

    cout << "Using '" << devInfo.name << "' (" << devInfo.id << ") ..." << endl;
    glove = &devMgr->getDevice<CyberGlove> (devInfo.id, CyberGlove::DEVICE_ID);

    glove->setCalibration(new CyberGloveLinearCalibration(gloveCalib.c_str()));
    if (!glove->connect()) {
      throw runtime_error("Unable to connect to device");
    }
  } catch (exception& e) {
    cerr << "CyberGlove: " << e.what() << endl;
    return false;
  }

#ifdef USE_FLOCKOFBIRDS
  WORD COM_port = 1;

  cout << "Initializing Flock of Birds..." << endl;
  if ((!birdRS232WakeUp(GROUP_ID, true, 1, &COM_port, BAUD_RATE,
              READ_TIMEOUT, WRITE_TIMEOUT, GMS_GROUP_MODE_NEVER))) {
    cout << birdGetErrorMessage() << endl;
    return false;
  }

  BIRDDEVICECONFIG devconfig[1];
  if (!birdGetFastDeviceConfig(GROUP_ID, 1, &devconfig[0])) {
    cout << birdGetErrorMessage() << endl;
    return false;
  }

  devconfig[0].byDataFormat = 2; // ORIENTATION ONLY

  if (!birdSetFastDeviceConfig(GROUP_ID, 0, &devconfig[0])) {
    cout << birdGetErrorMessage() << endl;
    return false;
  }

  birdStartFrameStream( GROUP_ID);
#endif

  vector<unsigned int> errorLog;
  if (!dataPorts.open(&errorLog)) {
    for (unsigned int i = 0; i < errorLog.size(); i++) {
      cout << getName() << ": unable to open port " << dataPorts.getName(i) << endl;
    }
    return false; // unable to open; let RFModule know so that it won't run
  }

  return startThread();
}

Thread* GloveCtrl::createWorkerThread() {
  workerThread = new WorkerThread(moduleOptions, dataPorts, id, glove);
  return workerThread;
}

GloveCtrl::WorkerThread::WorkerThread(const OptionManager& moduleOptions,
    const Contactables& ports, const struct PortIds id, CyberGlove* const glove) :
  RFWorkerThread(moduleOptions, ports) {
  if (glove == NULL) {
    throw invalid_argument("Illegal CyberGlove II object");
  }
  this->glove = glove;
  this->id = id;
}

void GloveCtrl::WorkerThread::enableStreaming(const bool b) {
  moduleOptions["streaming"].setValue(Value(b ? "on" : "off"));
}

//#define DEBUG_OFFLINE

void GloveCtrl::WorkerThread::run() {
  BufferedPort<Vector> *out = (BufferedPort<Vector>*) dataPorts[id.Output_Q];

  double MIN[HandMetrics::numAxes] = { 0, 0, 0, 0, -80, -70, -10, -300, 0, 0, 10, 10, 10, 10, 10,
      20 };
  double MAX[HandMetrics::numAxes] = { 0, 0, 0, 0, 80, 0, 15, 0, 140, 35, 80, 80, 80, 80, 80, 160 };
  Vector RANGE(HandMetrics::numAxes);

  for (int i = 0; i < HandMetrics::numAxes; i++) {
    RANGE[i] = MAX[i] - MIN[i];
  }

  while (!isStopping()) {
    if (moduleOptions["streaming"].getValue().asString() == "on") {

      Vector &q = out->prepare();
      q.resize(HandMetrics::numAxes);

      try {
        vector<double> angles = glove->getAngles(true);

        Vector v(HandMetrics::numAxes);
        v[Hand::HAND_FINGER] = -(angles[CyberGlove::MIDDLE_ABDUCTION]
            + angles[CyberGlove::RING_ABDUCTION] + angles[CyberGlove::PINKY_ABDUCTION]);

        v[Hand::THUMB_OPPOSE] = angles[CyberGlove::THUMB_METACARPAL];
        v[Hand::THUMB_PROXIMAL] = angles[CyberGlove::THUMB_PROXIMAL];
        v[Hand::THUMB_DISTAL] = angles[CyberGlove::THUMB_DISTAL];

        v[Hand::INDEX_PROXIMAL] = angles[CyberGlove::INDEX_METACARPAL];
        v[Hand::INDEX_DISTAL] = angles[CyberGlove::INDEX_PROXIMAL];
        v[Hand::MIDDLE_PROXIMAL] = angles[CyberGlove::MIDDLE_METACARPAL];
        v[Hand::MIDDLE_DISTAL] = angles[CyberGlove::MIDDLE_PROXIMAL];

        v[Hand::PINKY] = angles[CyberGlove::RING_PROXIMAL] + angles[CyberGlove::PINKY_PROXIMAL];

        v[Hand::WRIST_YAW] = -angles[CyberGlove::WRIST_YAW];
        v[Hand::WRIST_PITCH] = -angles[CyberGlove::WRIST_PITCH];

#ifdef USE_FLOCKOFBIRDS
        if (birdFrameReady(GROUP_ID)) {
          BIRDFRAME frame;
          birdGetMostRecentFrame(GROUP_ID, &frame);

          BIRDREADING& bird_data = frame.reading[0];
          v[Hand::WRIST_PROSUP] = -bird_data.angles.nRoll * 180.0 / 32767.0;
        }

#endif

        for (int i = Hand::WRIST_PROSUP; i < HandMetrics::numAxes; i++) {
          v[i] = min(MAX[i], max(MIN[i], v[i]));
          q[i] = Hand::RANGES[i] * (v[i] - MIN[i]) / RANGE[i] + Hand::LIMITS[i].first;
#ifdef DEBUG
          cout << " v[" << i << "] = " << v[i] << "[" << v[i] << ": (" << MIN[i] << "," << MAX[i]
              << ") ]" << endl;
#endif
        }
#ifdef DEBUG
        cout << "q(16) = ";
        printVector(q, cout);
#endif
        out->write();
        Time::delay(0.1);

      } catch (runtime_error&) {
        cout << "CyberGlove: Lost connection to the device manager. Stop streaming..." << endl;
        moduleOptions["streaming"].setValue(Value("off"));
      }
    }
  }

  glove->disconnect();
}

}
}
