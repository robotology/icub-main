/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Raw dataset recorder using the MachineLearner interface.
 *
 */

#ifndef __ICUB_DATASETRECORDER__
#define __ICUB_DATASETRECORDER__

#include <yarp/os/Bottle.h>
#include "iCub/IMachineLearner.h"
//#include <iostream>
#include <string>
#include <fstream>
//#include <sstream>


using namespace yarp;

namespace iCub {
namespace learningmachine {

/**
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
    DatasetRecorder() {
        this->setName("Recorder");
        this->filename = "dataset.dat";
        this->precision = 8;
        this->sampleCount = 0;
    }

    /**
     * Destructor.
     */
    ~DatasetRecorder() {
    }

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
    IMachineLearner* create() {
        return new DatasetRecorder();
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
