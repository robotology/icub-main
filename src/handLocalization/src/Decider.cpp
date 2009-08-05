// include header
#include <Tools.h>

//#include <math.h>

// namespaces
using namespace thesis::tools;

// ***************************************************************************

/**
 * Implementation of DeciderAndConnector class
 *
 * This class is a pool of methods /functionality that can serve every module. 
 *
 */

Decider::Decider() {
	//printf("Start:\t[Decider]\n");
	this->motionTriggered				= false;
	this->patchClassificationTriggered	= false;
}

Decider::~Decider() {
	//printf("Quit:\t[Decider]\n");
}

bool Decider::reset() {
	this->motionTriggered = false;
	this->patchClassificationTriggered = false;
	return true;
}

bool Decider::decide(string _onWhat) {
	if (_onWhat == "flow") {
		return this->motionTriggered;
	}
	else if (_onWhat == "tracking") {
		return this->patchClassificationTriggered;
	}
	else if (_onWhat == "both") {
		return (this->motionTriggered && this->patchClassificationTriggered);
	}
	else {
		printf("Info:\t[Tools]\t{Decider: not able to decide on <%s>.}\n", _onWhat.c_str());
		exit(-99);
	}
}

void Decider::setTrigger(string _onWhat) {
	if (!this->decide(_onWhat)) {
		if (_onWhat == "flow") {
			this->motionTriggered = true;;
		}
		else if (_onWhat == "tracking") {
			if (this->decide("flow")) {
				this->patchClassificationTriggered = true;
			}
		}
		else {
			printf("Info:\t[Tools]\t{setTrigger: not able to set trigger on <%s>.}\n", _onWhat.c_str());
		}
	}
}
