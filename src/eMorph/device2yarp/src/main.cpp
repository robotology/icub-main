#include "device2yarp.h"

int main(int argc, char* argv[])
{
	Network yarp;
	std::string in;
	bool end = false;
	bool _save = false;
	string fileName;
	if(argc >= 2)
        _save = (bool)atoi(argv[1]);
    if(argc == 3)
        fileName = argv[2];
    else
        fileName = "raw_events.bin";
	C_device2yarp D2Y(_save, fileName);
	D2Y.start();
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
	D2Y.stop();
    return 0;
}
