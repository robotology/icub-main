/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
\defgroup iKinCartesianSolver iKinCartesianSolver 
 
@ingroup icub_module  
 
Just a container which runs the \ref iKinSlv "Cartesian Solver" 
taking parameters from a configuration file. 
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description 
See \ref iKinSlv "Cartesian Solver" for documentation. 
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT: see the 
  <a
  href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>).

\section parameters_sec Parameters
--part \e type [mandatory]
- select the part to run. \e type is the group name within the 
  configuration file (e.g. left_arm, right_arm, ...).

--context \e directory [optional]
- allow specifying a different path where to search the
  configuration file (beware of the  
  <a href="http://www.yarp.it/yarp_data_dirs.html">context  
  search policy</a>).  
 
--from \e file [optional]
- allow specifying a different configuration file from the 
  default one which is \e cartesianSolver.ini.
 
\section portsa_sec Ports Accessed
 
All ports which allow the access to motor interface shall be 
previously open. 

\section portsc_sec Ports Created 
 
- /<solverName>/in : for requests in streaming mode.
- /<solverName>/rpc : for requests and replies.
- /<solverName>/out : for output streaming. 
 
\note for a detailed description, see \ref iKinSlv.
 
\section conf_file_sec Configuration Files
 
Here's how the configuration file will look like for the 
specific icub part left_arm: 
 
\code 
[left_arm]
robot          icub
name           cartesianSolver/left_arm
type           left
period         20
dof            (0 0 0 1 1 1 1 1 1 1)
rest_pos       (0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
rest_weights   (1.0 1.0 1.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0) 
pose           full
mode           shot
verbosity      off
maxIter        200
tol            0.001
interPoints    off 
ping_robot_tmo 20.0 
\endcode 
 
Here's how the configuration file will look like for a custom 
robot part: 
 
\code 
[left_arm]
robot           my-robot
name            cartesianSolver/left_arm 
type            left
period          20
dof             (1 1 1 1 1 1 1)
rest_pos        (0.0 0.0 0.0 0.0 0.0 0.0 0.0)
rest_weights    (0.0 0.0 0.0 0.0 0.0 0.0 0.0) 
pose            full
mode            shot
verbosity       off
maxIter         200
tol             0.001
interPoints     off 
ping_robot_tmo  20.0 
 
CustomKinFile   cartesian/kinematics.ini 
NumberOfDrivers 1 
driver_0        (Key left_arm) (JointsOrder direct)
  
LIC_num         7
LIC_0           (C (1.71 -1.71    0.0  0.0 0.0 0.0 0.0)) (lB -6.051)
LIC_1           (C (1.71 -1.71  -1.71  0.0 0.0 0.0 0.0)) (lB -6.397) (uB 1.962)
LIC_2           (C ( 0.0   1.0    1.0  0.0 0.0 0.0 0.0)) (lB -1.162) (uB 3.722)
LIC_3           (C ( 0.0   1.0 0.0427  0.0 0.0 0.0 0.0)) (lB  0.461)
LIC_4           (C ( 0.0   1.0    0.0  0.0 0.0 0.0 0.0))             (uB 1.745)
LIC_5           (C ( 0.0   0.0    0.0  2.5 1.0 0.0 0.0))             (uB 5.280)
LIC_6           (C ( 0.0   0.0    0.0 -2.5 1.0 0.0 0.0)) (lB -5.280)
\endcode 
 
\note for a detailed description of options, see \ref iKinSlv
      "Cartesian Solver".
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/

#include <cstdlib>
#include <memory>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinSlv.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

static string pathToCustomKinFile;


/************************************************************************/
class GenericLinIneqConstr : public iKinLinIneqConstr {
 protected:
  bool configured{false};
  iKinChain* chain{nullptr};
  Matrix C_orig;
  Vector lB_orig;
  Vector uB_orig;

  /************************************************************************/
  void clone(const iKinLinIneqConstr* obj)override {
    iKinLinIneqConstr::clone(obj);

    const auto* ptr = static_cast<const GenericLinIneqConstr*>(obj);
    configured = ptr->configured;
    chain = ptr->chain;
    C_orig = ptr->C_orig;
    lB_orig = ptr->lB_orig;
    uB_orig = ptr->uB_orig;
  }

 public:
  /************************************************************************/
  GenericLinIneqConstr(iKinChain* chain_, Searchable& options) :
      chain(chain_), C_orig(0, chain_->getN()), iKinLinIneqConstr() {
    if (options.check("LIC_num")) {
      auto LIC_num = options.find("LIC_num").asInt();
      if (LIC_num > 0) {
        for (auto i = 0; i < LIC_num; i++) {
          ostringstream tag;
          tag << "LIC_" << i;
          Bottle& group = options.findGroup(tag.str());
          if (group.isNull()) {
            yError() << "Unable to find group \"" << tag.str() << "\"";
            return;
          }
          Bottle* row = group.find("C").asList();
          if (row == nullptr) {
            yError() << "Unable to find option \"C\" for group \"" << tag.str() << "\"";
            return;
          }
          if (row->size() != chain->getN()) {
            yError() << "Option \"C\" of group \"" << tag.str() << "\" has wrong size";
            return;
          }
          C_orig = pile(C_orig, zeros(chain->getN()));
          for (auto i = 0; i < row->size(); i++) {
            C_orig(C_orig.rows() - 1, i) = row->get(i).asDouble();
          }
          lB_orig = cat(lB_orig, group.check("lB", Value(lowerBoundInf)).asDouble());
          uB_orig = cat(uB_orig, group.check("uB", Value(upperBoundInf)).asDouble());
        }

        yInfo() << "Detected generic linear inequalities constraints";
        configured = true;
        update(nullptr);
      }
    }
  }

  /************************************************************************/
  void update(void*)override {
    setActive(false);
    if (configured) {
      getC().resize(C_orig.rows(), 0);
      getlB() = lB_orig;
      getuB() = uB_orig;
      for (auto i = 0U; i < chain->getN(); i++) {
        if (chain->isLinkBlocked(i)) {
          auto v = chain->getAng(i) * C_orig.getCol(i);
          getlB() -= v;
          getuB() -= v;
        } else {
          getC() = cat(getC(), C_orig.getCol(i));
        }
      }
      setActive(true);
    }
  }
};


/************************************************************************/
class CustomCartesianSolver : public CartesianSolver {
  shared_ptr<iKinLimb> limb{nullptr};
  shared_ptr<GenericLinIneqConstr> cns{nullptr};
  shared_ptr<PartDescriptor> desc{nullptr};

 protected:
  /************************************************************************/
  PartDescriptor* getPartDesc(Searchable& options) {
    if (!options.check("robot")) {
      yError() << "\"robot\" option is missing!";
      return nullptr;
    }

    if (!options.check("NumberOfDrivers")) {
      yError() << "\"NumberOfDrivers\" option is missing!";
      return nullptr;
    }

    string robot = options.find("robot").asString();
    yInfo() << "Configuring solver for " << robot << " ...";

    Property linksOptions;
    linksOptions.fromConfigFile(pathToCustomKinFile);
    limb = shared_ptr<iKinLimb>(new iKinLimb(linksOptions));
    if (!limb->isValid()) {
      yError() << "invalid links parameters!";
      return nullptr;
    }

    cns = shared_ptr<GenericLinIneqConstr>(new GenericLinIneqConstr(limb->asChain(), options));
    desc = shared_ptr<PartDescriptor>(new PartDescriptor);
    desc->lmb = limb.get();
    desc->chn = limb->asChain();
    desc->cns = cns.get();

    bool failure = false;
    desc->num = options.find("NumberOfDrivers").asInt();
    for (int cnt = 0; cnt < desc->num; cnt++) {
      ostringstream str;
      str << "driver_" << cnt;
      Bottle& driver = options.findGroup(str.str());
      if (driver.isNull()) {
        yError() << "\"" << str.str() << "\" option is missing!";
        failure = true;
        break;
      }

      if (!driver.check("Key")) {
        yError() << "\"Key\" option is missing!";
        failure = true;
        break;
      }

      if (!driver.check("JointsOrder")) {
        yError() << "\"JointsOrder\" option is missing!";
        failure = true;
        break;
      }

      string part = driver.find("Key").asString();
      bool directOrder = (driver.find("JointsOrder").asString() == "direct");

      Property optPart;
      optPart.put("device", "remote_controlboard");
      optPart.put("remote", "/" + robot + "/" + part);
      optPart.put("local", "/" + slvName + "/" + part);
      optPart.put("robot", robot);
      optPart.put("part", part);
      desc->prp.push_back(optPart);
      desc->rvs.push_back(!directOrder);
    }

    return (failure ? nullptr : desc.get());
  }

 public:
  /************************************************************************/
  CustomCartesianSolver(const string& name) : CartesianSolver(name) { }
};



/************************************************************************/
class SolverModule : public RFModule {
 protected:
  shared_ptr<CartesianSolver> slv{nullptr};

 public:
  /************************************************************************/
  bool configure(ResourceFinder& rf) {
    string part, slvName;
    if (rf.check("part")) {
      part = rf.find("part").asString();
    } else {
      yError() << "part option is not specified";
      return false;
    }

    Bottle& group = rf.findGroup(part);
    if (group.isNull()) {
      yError() << "unable to locate " << part << " definition";
      return false;
    }

    if (group.check("name")) {
      slvName = group.find("name").asString();
    } else {
      yError() << "name option is missing";
      return false;
    }

    if (group.check("CustomKinFile")) {
      yInfo() << "Custom Cartesian Solver detected!";

      ResourceFinder rf_kin;
      rf_kin.setDefaultContext(rf.getContext());
      rf_kin.configure(0, nullptr);
      pathToCustomKinFile = rf_kin.findFileByName(group.find("CustomKinFile").asString());

      slv = shared_ptr<CartesianSolver>(new CustomCartesianSolver(slvName));
    } else if ((part == "left_arm") || (part == "right_arm")) {
      slv = shared_ptr<CartesianSolver>(new iCubArmCartesianSolver(slvName));
    } else if ((part == "left_leg") || (part == "right_leg")) {
      slv = shared_ptr<CartesianSolver>(new iCubLegCartesianSolver(slvName));
    } else {
      yError() << part << " is invalid";
      return false;
    }

    return slv->open(group);
  }

  /************************************************************************/
  bool interruptModule() {
    if (slv) {
      slv->interrupt();
    }

    return true;
  }

  /************************************************************************/
  double getPeriod() {
    return 1.0;
  }

  /************************************************************************/
  bool updateModule() {
    if (slv->isClosed()) {
      return false;
    }

    if (slv->getTimeoutFlag()) {
      slv->getTimeoutFlag() = false;
      slv->suspend();
    }

    return true;
  }
};


/************************************************************************/
int main(int argc, char* argv[]) {
  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "YARP server not available!";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.setDefaultContext("cartesianSolver");
  rf.setDefaultConfigFile("cartesianSolver.ini");
  rf.configure(argc, argv);

  SolverModule mod;
  return mod.runModule(rf);
}



