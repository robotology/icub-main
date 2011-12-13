/*
 * Copyright (C) 2007-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#ifndef LM_DUMMYLEARNER__
#define LM_DUMMYLEARNER__

#include <vector>
#include <sstream>

#include "iCub/learningMachine/IFixedSizeLearner.h"


namespace iCub {
namespace learningmachine {

std::string printVector(const yarp::sig::Vector& v);
/**
 * \ingroup icub_libLM_learning_machines
 *
 * This dummy machine learner demonstrates how the IMachineLearner interface can
 * be used in practice. The functionality, however, is rather useless.
 *
 * \see iCub::contrib::IMachineLearner
 *
 * \author Arjan Gijsberts
 *
 */
class DummyLearner : public IFixedSizeLearner {
private:
    /**
     * Number of samples during last training routine.
     */
    int sampleCount;

    /**
     * Number of training routines performed in total.
     */
    int trainCount;

    /**
     * The stored inputs.
     */
    std::vector<yarp::sig::Vector> inputs;

    /**
     * The stored outputs.
     */
    std::vector<yarp::sig::Vector> outputs;


public:
    /**
     * Constructor.
     */
    DummyLearner(unsigned int dom = 1, unsigned int cod = 1) : sampleCount(0), trainCount(0) {
        this->setName("Dummy");
        inputs.resize(0);
        outputs.resize(0);
        this->setDomainSize(dom);
        this->setCoDomainSize(cod);
    }

    /**
     * Destructor (empty).
     */
    virtual ~DummyLearner() { }

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

    /*
     * Inherited from IMachineLearner.
     */
    Prediction predict(const yarp::sig::Vector& input);

    /*
     * Inherited from IMachineLearner.
     */
    void reset();

    /*
     * Inherited from IMachineLearner.
     */
    DummyLearner* clone() {
        return new DummyLearner(*this);
    }

    /*
     * Inherited from IMachineLearner.
     */
    std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(yarp::os::Bottle& bot);

};

} // learningmachine
} // iCub
#endif
