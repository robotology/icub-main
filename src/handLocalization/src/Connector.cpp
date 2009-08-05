// include header
#include <ImageProcessing.h>

// namespaces
using namespace thesis::imageprocessing;

// ***************************************************************************

/**
 * Implementation of Connector class
 *
 * This class is a pool of methods /functionality that can serve every module. 
 *
 */

Connector::Connector(Motion *_motionModule_ptr, Flow *_opticalFlowModule_ptr) {
	//printf("Start:\t[Connector]\n");
	this->motion_ptr		= _motionModule_ptr;
	this->flow_ptr			= _opticalFlowModule_ptr;
}

Connector::~Connector() {
	//printf("Quit:\t[Connector]\n");
}



