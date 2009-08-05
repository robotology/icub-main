// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <yarp/os/all.h>
#include <iostream>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv) {
    Network yarp;
    BufferedPort<Bottle> port;
    port.open("/midiDrum/server/out");
    Time::delay(10);
    while (true) 
      {

	Time::delay(5);
        Bottle& output = port.prepare();
        output.clear();
        output.addString("NOTE_ON");
        output.addInt(0);
        output.addInt(47);
        output.addInt(100);
        cout << "writing " << output.toString().c_str() << endl;
        port.write();
 
        /*	Time::delay(5);
        output = port.prepare();
        output.clear();
        output.addString("NOTE_OFF");
        output.addInt(0);
        output.addInt(31);
        output.addInt(0);
        cout << "writing " << output.toString().c_str() << endl;
        port.write();*/

    }
    return 0;
}
