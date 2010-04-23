// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __evenConversionh__
#define __evenConversionh__

/** 
 * a simple class to handle event unmasking.
 */
class Unmask
{
public:
	Unmask();
	~Unmask();

//	list<AER_struct> unmaskData(char*, int);
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

	/* Variables needed by the unmask method */
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
};

#endif /* __evenConversionh__ */
