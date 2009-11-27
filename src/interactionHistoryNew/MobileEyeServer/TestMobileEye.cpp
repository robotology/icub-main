#include <stdio.h>
#include "MobileEyeServer.h"

using namespace yarp::dev;
using namespace yarp::os;

int main(int argc, char *argv[]) {


  MobileEyeServer me = MobileEyeServer();
  Bottle msg;

  Property p;
  p.fromCommand(argc,argv);

  me.open(p);
  while(1) {
  }
  // msg.clear();
  //me.receive(msg);
  //if (msg.size() > 0)
  //  fprintf(stderr,"%s \n",msg.toString().c_str());
  //}
  me.close();
}
