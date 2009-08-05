// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2009 Jonas Ruesch
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __ATTENTIONSELECTIONINTERFACES__
#define __ATTENTIONSELECTIONINTERFACES__

#include <string>
#include <yarp/os/Bottle.h>

namespace iCub {
	namespace contrib {
		class IAttentionSelectionControls;
	}
}

/**
*
* Remote interface for the AttentionSelection module
*/
class iCub::contrib::IAttentionSelectionControls {

public:
	virtual ~IAttentionSelectionControls(){}
	virtual void setInhibitOutput(bool on) = 0;
	virtual bool getInhibitOutput() = 0;
	// TODO:
	// virtual void setThresholdDistance(float dist) = 0;
	// virtual float getThresholdDistance() = 0;
	// virtual void setThresholdDifference(float diff) = 0;
	// virtual float getThresholdDifference() = 0;
};

#define ATTENTIONSELECTION_VOCAB_SET VOCAB3('s','e','t')
#define ATTENTIONSELECTION_VOCAB_GET VOCAB3('g','e','t')
#define ATTENTIONSELECTION_VOCAB_IS VOCAB2('i','s')
#define ATTENTIONSELECTION_VOCAB_FAILED VOCAB4('f','a','i','l')
#define ATTENTIONSELECTION_VOCAB_OK VOCAB2('o','k')

#define ATTENTIONSELECTION_VOCAB_THRESHOLD_DIFFERENCE VOCAB4('t','d','i','f')
#define ATTENTIONSELECTION_VOCAB_THRESHOLD_DISTANCE VOCAB4('t','d','i','s')
#define ATTENTIONSELECTION_VOCAB_THRESHOLD_OUTPUT VOCAB3('o','u','t')

inline bool ATTENTIONSELECTION_CHECK_FAIL(bool ok, yarp::os::Bottle& response) {
	if (ok) {
		if (response.get(0).isVocab() && response.get(0).asVocab() == ATTENTIONSELECTION_VOCAB_FAILED) {
			return false;
		}
	}
	else
		return false;

	return true;
}

#endif


