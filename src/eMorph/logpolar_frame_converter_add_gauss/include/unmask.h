#ifndef C_UNMASK
#define C_UNMASK

//#ifdef _DEBUG
#include <iostream>
//#endif
#include <sstream>
#include <ctime>
#include <list>

#include <yarp/os/all.h>

//Other dependency
#include "config.h"

using namespace std;
class C_unmask
{
public:
	//C_unmaskingthread(){};
	C_unmask();
	~C_unmask();

	list<AER_struct> unmaskData(char*, int);
private:
	/**
	* @brief This method unmasked the raw which come from the TCP socket
	* This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
	* @param *evPU A pointer on the raw casted from char* to int*
	* @param x Set with the x coordinate of the pixel
	* @param y Set with the y coordinate of the pixel
	* @param pol Set with the ON/OFF polarity of the pixel.
	* @param ts ...
	*/
	void unmaskEvent(unsigned int, short&, short&, short&);

	int id;

#ifdef _DEBUG
	int nb_trame;
#endif

	/*Variables needed by the unmask method*/
	int sz;
	char* buffer;
	unsigned int timestamp;
	short cartX, cartY, polarity;

    int wrapAdd;
	unsigned int xmask;
	unsigned int ymask;
	int yshift;
	int xshift;
	int polshift;
	int polmask;
	int retinalSize;
	//**************************************

	FILE* uEvents;
};

#endif //C_UNMASKINGTHREAD
