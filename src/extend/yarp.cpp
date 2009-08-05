#include <iostream>
#include <stdio.h>

#include "Sequence.h"
#include "Unit.h"

#include <yarp/os/all.h>

using namespace yarp::os;
using namespace std;

int netmain(int argc, char *argv[]) {
  Network::init();

  BufferedPort<Bottle> port;
  port.setStrict(true);
  port.open("/predictor");
  Network::connect("/seq","/present");
  Network::connect("/seq","/predictor");
  Network::connect("/predictor","/future");

  Bottle cache;
  while (true) {
    Bottle *bot = port.read(true);
    bool more = true;
    while (bot!=NULL) {
      for (int i=0; i<bot->size(); i++) {
	cache.add(bot->get(i));
      }
      bot = port.read(false);
    }

    Bottle& out = port.prepare();

    Sequence seq;
    int win = 100;
    for (int i=max(0,cache.size()-win); i<cache.size(); i++) {
      seq.add(cache.get(i).asDouble());
    }
    Unit unit(seq.specialize(seq));
    Sequence fut;
    fut.takeFuture(unit,50);
    cout << fut.toString() << endl;
    out.clear();
    for (int i=0; i<fut.size(); i++) {
      out.addDouble(fut[i]);
    }

    port.write();
  }

  Network::fini();
  return 0;
}



