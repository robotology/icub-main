#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <string>
#include <fstream>

#include "shared.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

static int aligner_active = 1;

BufferedPort<Bottle> aligner_port;

bool aligner_interrupt() {
  aligner_active = 0;
  aligner_port.interrupt();
  return true;
}

int aligner_main(yarp::os::Property& conf) {
  printf("aligner\n");

  aligner_port.setStrict(true);
  aligner_port.open("/aligner");
  Network::connect("/babbler","/aligner");

  Bottle arm, head;
  double at = -1000;

  ofstream fout("head_to_arm.txt");
  while (aligner_active) {
    Bottle *in = aligner_port.read();
    if (!aligner_active) continue;
    if (in==NULL) continue;
    printf("Got %s\n", in->toString().c_str());
    string key = in->get(0).asString().c_str();
    double t = in->get(1).asDouble();
    if (t-at>0.001) {
      at = t;
      arm.clear();
      head.clear();
    }
    if (key.find("arm")!=string::npos) {
      arm = *(in->get(3).asList());
    }
    if (key.find("head")!=string::npos) {
      head = *(in->get(3).asList());
    }
    printf("STATUS %s // %s\n", head.toString().c_str(), arm.toString().c_str());
    if (head.size()>0 && arm.size()>0) {
      printf("Got data! %s // %s\n", head.toString().c_str(), arm.toString().c_str());
      fout << head.toString().c_str() << "      " << arm.toString().c_str() << endl;
    }
  }

  printf("ALIGNER STOPPED\n");

  return 0;
}

