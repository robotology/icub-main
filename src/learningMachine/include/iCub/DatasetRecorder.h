/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef LM_DATASETRECORDER__
#define LM_DATASETRECORDER__

#include <string>
#include <fstream>

#include <yarp/os/Bottle.h>

#include "iCub/IMachineLearner.h"


using namespace yarp::sig;

namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_learning_machines
 *
 * This 'machine learner' demonstrates how the IMachineLearner interface can
 * be used to easily record samples to a file.
 *
 * \see iCub::contrib::IMachineLearner
 *
 * \author Arjan Gijsberts
 *
 */
class DatasetRecorder : public IMachineLearner {
private:
    /**
     * The filename of the file we are writing to.
     */
    std::string filename;

    /**
     * The filestream.
     */
    std::ofstream stream;

    /**
     * Precision for the serialization of the doubles.
     */
    int precision;

    /**
     * Number of recorded samples.
     */
    int sampleCount;

public:
    /**
     * Constructor.
     */
    DatasetRecorder() : filename("dataset.dat"), precision(8), sampleCount(0) {
        this->setName("Recorder");
    }

    /**
     * Copy constructor.
     */
    DatasetRecorder(const DatasetRecorder& other)
      : IMachineLearner(other), precision(other.precision),
        sampleCount(other.sampleCount), filename(other.filename) {
    }

    /**
     * Destructor.
     */
    virtual ~DatasetRecorder() {
        if(!this->stream.is_open()) {
            this->stream.close();
        }
    }

    /**
     * Assignment operator.
     */
    DatasetRecorder& operator=(const DatasetRecorder& other);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const Vector& input, const Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train() { }

    /*
     * Inherited from IMachineLearner.
     */
    Vector predict(const Vector& input) {
        return Vector();
    }

    /*
     * Inherited from IMachineLearner.
     */
    void reset() {
        this->stream.close();
        this->sampleCount = 0;
    }

    /*
     * Inherited from IMachineLearner.
     */
    DatasetRecorder* clone() {
        return new DatasetRecorder(*this);
    }

    /*
     * Inherited from IMachineLearner.
     */
    std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(Bottle& bot);

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);
};

} // learningmachine
} // iCub
#endif
