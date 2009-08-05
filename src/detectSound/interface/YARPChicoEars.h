
#include <yarp/os/all.h>

#include <yarp/sig/Sound.h>
#include <yarp/os/BufferedPort.h>

#include <iostream>
//#include <conio.h>



using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

#ifdef main
#undef main
#endif

class YARPChicoEars
{
	
public:

	FILE *outFile;
	
	BufferedPort<Sound> port;	
	//Port port;	
	
	YARPChicoEars(char *portname, char *netname, char *grabname);
	~YARPChicoEars();
	
	int init();
	int grab();
	int close();
	void separateEars(double *left_ear, double *right_ear);

}; 
