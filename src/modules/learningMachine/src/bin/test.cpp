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

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <cassert>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/PortablePair.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>

#define TWOPI  6.283185307179586

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {
namespace test {

// the Prediction class is compatible with a PortablePair of Vector objects.
// in this class, we demonstrate that we in fact only need standard Yarp
// classes to make use of the learningMachine library: no learningMachine headers
// or library linking required!
typedef PortablePair<Vector,Vector> Prediction;

// it's 2011 and I still have to implement a double to string conversion? come on!
std::string doubletostring(double d) {
  std::ostringstream output;
  output << d;
  return output.str();
}

std::string inttostring(int i) {
  std::ostringstream output;
  output << i;
  return output.str();
}

std::string printVector(const std::vector<int>& v) {
  std::ostringstream output;
  output << "[";
  for(unsigned int i = 0; i < v.size(); i++) {
    if(i > 0) output << ", ";
    output << v[i];
  }
  output << "]";
  return output.str();
}

std::string printVector(const Vector& v) {
  std::ostringstream output;
  output << "[";
  for(size_t i = 0; i < v.size(); i++) {
    if(i > 0) output << ",";
    output << v[i];
  }
  output << "]";
  return output.str();
}

std::string printPrediction(Prediction& p) {
  std::ostringstream output;
  Vector v = p.head;
  output << "[";
  for(size_t i = 0; i < v.size(); i++) {
    if(i > 0) output << ",";
    output << v[i];
  }
  v = p.body;
  if(v.size() > 0) {
    output << " +/- ";
    for(size_t i = 0; i < v.size(); i++) {
      if(i > 0) output << ",";
      output << v[i];
    }
  }
  output << "]";
  return output.str();
}


class Dataset {
private:
    int samplesRead;
    std::ifstream file;
    std::string filename;
    std::vector<int> inputCols;
    std::vector<int> outputCols;


public:
    Dataset() {
        this->inputCols.resize(1);
        this->outputCols.resize(1);

        this->inputCols.clear();
        this->outputCols.clear();
    }

    // manage input and output columns
    void addInputColumn(int col) {
        this->inputCols.push_back(col);
    }

    void addOutputColumn(int col) {
        this->outputCols.push_back(col);
    }

    std::vector<int> getInputColumns() {
        return this->inputCols;
    }

    std::vector<int> getOutputColumns() {
        return this->outputCols;
    }

    // gets the filename of the open dataset
    std::string getFilename() {
        return this->filename;
    }

    // sets the filename for the dataset
    void setFilename(std::string filename) {
        this->filename = filename;
    }

    // manage file datastream
    void open() {
        this->open(this->filename);
    }

    // open a datafile
    void open(std::string filename) {
        // close previous file
        if(this->file.is_open()) {
            this->file.close();
        }

        this->file.open(filename.c_str());
        if(!file.is_open() || file.fail()) {
            std::string msg("could not open file '" + filename + "'");
            this->setFilename("");
            throw std::runtime_error(msg);
            return; // will not be reached anyways...
        }
        this->setFilename(filename);
        this->reset();
    }

    bool hasNextSample() {
        return !this->file.eof() && this->file.good();
    }

    void reset() {
        this->samplesRead = 0;
        this->file.clear();
        this->file.seekg(0, std::ios::beg);
    }

    // retrieve new sample from datastream
    std::pair<Vector,Vector> getNextSample() {
        Vector input;
        Vector output;

        int currentCol = 1;
        double val = 0.0;

        std::string lineString;

        if(!this->hasNextSample()) {
           throw std::runtime_error("at end of dataset");
        }

        // find first valid string that does not start with #
        do {
            getline(file, lineString);
        } while(lineString[0] == '#' && !file.eof());

        if(!this->hasNextSample()) {
            throw std::runtime_error("at end of dataset");
        }

        // read string as stream buffer
        std::istringstream lineStream(lineString);
        while(!lineStream.eof()) {
            lineStream >> val;

            // add to input sample if appropriate
            std::vector<int>::iterator inputFind = std::find(this->inputCols.begin(), this->inputCols.end(), currentCol);
            if(inputFind != this->inputCols.end()) {
                input.push_back(val);
            }

            // add to output sample if appropriate
            std::vector<int>::iterator outputFind = std::find(this->outputCols.begin(), this->outputCols.end(), currentCol);
            if(outputFind != this->outputCols.end()) {
                output.push_back(val);
            }
            currentCol++;
        }

        std::pair<Vector,Vector> sample(input, output);
        return sample;
    }
};

/**
 * \ingroup icub_libLM_modules
 *
 * The MachineLearnerTestModule is used to feed datasets loaded from a file to
 * the learning machines and transformers.
 *
 * \author Arjan Gijsberts
 */

class MachineLearnerTestModule : public RFModule {
private:
    BufferedPort<PortablePair<Vector,Vector> > train_out;
    Port predict_inout;

    std::string portPrefix;

    Dataset dataset;

    int frequency; // in Hz

    void registerPort(Contactable& port, std::string name) {
        if(port.open(name.c_str()) != true) {
            std::string msg("could not register port ");
            msg+=name;
            throw std::runtime_error(msg);
        }
    }

    void registerAllPorts() {
        this->registerPort(this->train_out, this->portPrefix + "/train:o");
        this->train_out.setStrict();
        this->registerPort(this->predict_inout, this->portPrefix + "/predict:io");
    }

    void unregisterAllPorts() {
        this->train_out.close();
        this->predict_inout.close();
    }

public:
    MachineLearnerTestModule(std::string pp = "/lm/test") : portPrefix(pp) {

    }

    void printOptions(std::string error = "") {
        if(error != "") {
            std::cout << "Error: " << error << std::endl;
        }
        std::cout << "Available options" << std::endl;
        std::cout << "--help                 Display this help message" << std::endl;
        std::cout << "--trainport port       Data port for the training samples" << std::endl;
        std::cout << "--predictport port     Data port for the prediction samples" << std::endl;
        std::cout << "--datafile file        Filename containing the dataset" << std::endl;
        std::cout << "--inputs (idx1, ..)    List of indices to use as inputs" << std::endl;
        std::cout << "--outputs (idx1, ..)   List of indices to use as outputs" << std::endl;
        std::cout << "--port pfx             Prefix for registering the ports" << std::endl;
        std::cout << "--frequency f          Sampling frequency in Hz" << std::endl;
    }

    void printConfig() {
        std::cout << "* - Configuration -" << std::endl;
        std::cout << "* Datafile: " << this->dataset.getFilename() << std::endl;
        std::cout << "* Input columns: " << printVector(this->dataset.getInputColumns()) << std::endl;
        std::cout << "* Output columns: " << printVector(this->dataset.getOutputColumns()) << std::endl;
    }


    virtual bool configure(ResourceFinder& opt) {
        // read for the general specifiers:
        Value* val;

        // check for help request
        if(opt.check("help")) {
            this->printOptions();
            return false;
        }

        // check for port specifier: portSuffix
        if(opt.check("port", val)) {
            this->portPrefix = val->asString().c_str();
        }

        std::cout << "* Registering ports...";
        this->registerAllPorts();
        std::cout << "Done!" << std::endl;

        // check for train data port
        if(opt.check("trainport", val)) {
            Network::connect(this->train_out.where().getName().c_str(), val->asString().c_str());
        } else {
            // add message here if necessary
        }

        // check for predict data port
        if(opt.check("predictport", val)) {
            Network::connect(this->predict_inout.where().getName().c_str(), val->asString().c_str());
        } else {
            // add message here if necessary
        }

        // check for filename of the dataset
        if(opt.check("datafile", val)) {
            //this->dataset.setFilename(val->asString().c_str());
            this->dataset.open(val->asString().c_str());
        }

        // check for the columns of the dataset that should be used for inputs
        if(opt.check("inputs", val)) {
            // if it's a list,
            if(val->isList()) {
                Bottle* inputs = val->asList();
                for(int i = 0; i < inputs->size(); i++) {
                    if(inputs->get(i).isInt()) {
                        this->dataset.addInputColumn(inputs->get(i).asInt());
                    }
                }
            } else if(val->isInt()) {
                this->dataset.addInputColumn(val->asInt());
            }
        } else {
            this->dataset.addInputColumn(1);
        }

        // check for the columns of the dataset that should be used for outputs
        if(opt.check("outputs", val)) {
            // if it's a list,
            if(val->isList()) {
                Bottle* outputs = val->asList();
                for(int i = 0; i < outputs->size(); i++) {
                    if(outputs->get(i).isInt()) {
                        this->dataset.addOutputColumn(outputs->get(i).asInt());
                    }
                }
            } else if(val->isInt()) {
                this->dataset.addOutputColumn(val->asInt());
            }
        } else {
            this->dataset.addOutputColumn(2);
        }

        // check for port specifier: portSuffix
        if(opt.check("frequency", val)) {
            this->frequency = val->asInt();
        } else {
            this->frequency = 0;
        }

        this->printConfig();

        this->attachTerminal();

        return true;
    }

    void sendTrainSample(Vector input, Vector output) {
        PortablePair<Vector,Vector>& sample = this->train_out.prepare();
        sample.head = input;
        sample.body = output;
        this->train_out.writeStrict();
    }

    Prediction sendPredictSample(Vector input) {
        Prediction prediction;
        this->predict_inout.write(input, prediction);
        return prediction;
    }

    bool updateModule() {
        Time::delay(1.);
        return true;
    }

    bool respond(const Bottle& cmd, Bottle& reply) {
        bool success = false;

        try {
            switch(cmd.get(0).asVocab()) {
                case VOCAB4('h','e','l','p'): // print help information
                    success = true;
                    reply.add(Value::makeVocab("help"));

                    reply.addString("Testing module configuration options");
                    reply.addString("  help                  Displays this message");
                    reply.addString("  conf                  Print configuration");
                    reply.addString("  train [n]             Send training samples");
                    reply.addString("  predict [n]           Send testing samples");
                    reply.addString("  skip [n]              Skip samples");
                    reply.addString("  reset                 Reset dataset");
                    reply.addString("  shoot v1 ... vn       Shoot a single prediction sample");
                    reply.addString("  open fname            Opens a datafile");
                    reply.addString("  freq f                Sampling frequency in Hertz (0 for disabled)");
                    break;
                case VOCAB4('c','o','n','f'): // print the configuration
                    {
                    success = true;
                    this->printConfig();
                    break;
                    }

                case VOCAB4('s','k','i','p'): // skip some training sample(s)
                    {
                    success = true;
                    int noSamples = 1;
                    if(cmd.get(1).isInt()) {
                        noSamples = cmd.get(1).asInt();
                    }

                    for(int i = 0; i < noSamples; i++) {
                        std::pair<Vector,Vector> sample = this->dataset.getNextSample();
                        // now do nothing with it... poor sample!
                    }
                    reply.addString("Done!");

                    break;
                    }

                case VOCAB4('t','r','a','i'): // send training sample(s)
                    {
                    reply.add(Value::makeVocab("help"));
                    success = true;
                    int noSamples = 1;
                    if(cmd.get(1).isInt()) {
                        noSamples = cmd.get(1).asInt();
                    }

                    //double start = yarp::os::Time::now();
                    for(int i = 0; i < noSamples; i++) {
                        std::pair<Vector,Vector> sample = this->dataset.getNextSample();
                        this->sendTrainSample(sample.first, sample.second);

                        // sleep
                        if(this->frequency > 0)
                            yarp::os::Time::delay(1. / this->frequency);
                    }
                    // this timing only measures how fast the module can dump the samples on the network
                    //double end = yarp::os::Time::now();
                    //std::string reply_str = "Timing: " + doubletostring(noSamples / (end - start)) + " samples per second";
                    //reply.addString(reply_str.c_str());
                    reply.addString("Done!");

                    break;
                    }

                case VOCAB4('p','r','e','d'): // send prediction sample(s)
                    {
                    reply.add(Value::makeVocab("help"));
                    success = true;
                    int noSamples = 1;
                    if(cmd.get(1).isInt()) {
                        noSamples = cmd.get(1).asInt();
                    }

                    // initiate error vector (not completely failsafe!)
                    Vector error(this->dataset.getOutputColumns().size());
                    Vector nmlp(this->dataset.getOutputColumns().size());
                    error = 0.0;
                    nmlp = 0.0;

                    // make predictions and keep track of errors (MSE)
                    double start = yarp::os::Time::now();
                    for(int i = 0; i < noSamples; i++) {
                        std::pair<Vector,Vector> sample = this->dataset.getNextSample();
                        Prediction prediction = this->sendPredictSample(sample.first);

                        //std::cout << "head[" << prediction.head.size() << "]: " << prediction.head.toString() << std::endl;
                        //std::cout << "variance[" << prediction.body.size() << "]: " << prediction.body.toString() << std::endl;

                        Vector expected = prediction.head;
                        if(expected.size() != sample.second.size()) {
                            std::string msg("incoming prediction has incorrect dimension");
                            throw std::runtime_error(msg);
                        }
                        for(size_t j = 0; j < error.size(); j++) {
                            double dist = sample.second[j] - expected[j];
                            error[j] += (dist * dist);
                        }

                        Vector variance = prediction.body;
                        if(variance.size() > 0 && variance.size() != sample.second.size()) {
                            std::string msg("incoming predictive variance has invalid dimension");
                            throw std::runtime_error(msg);
                        }
                        for(size_t j = 0; j < nmlp.size(); j++) {
                            double normdist = (sample.second[j] - expected[j]) / variance[j];
                            nmlp[j] += 0.5 * ((normdist * normdist) + log(TWOPI) + 2 * log(variance[j]));
                        }

                        // sleep
                        if(this->frequency > 0)
                            yarp::os::Time::delay(1. / this->frequency);
                    }
                    double end = yarp::os::Time::now();
                    std::string reply_str = "Timing: " + doubletostring(noSamples / (end - start)) + " samples per second";
                    reply.addString(reply_str.c_str());

                    // take mean of cumulated errors
                    for(size_t i = 0; i < error.size(); i++) {
                        error[i] = error[i] / double(noSamples);
                    }
                    for(size_t i = 0; i < error.size(); i++) {
                        nmlp[i] = nmlp[i] / double(noSamples);
                    }

                    reply_str = "MSE: " + printVector(error);
                    reply.addString(reply_str.c_str());

                    reply_str = "NMLP: " + printVector(nmlp);
                    reply.addString(reply_str.c_str());

                    break;
                    }

                case VOCAB4('s','h','o','o'):
                    {
                    success = true;
                    Vector input;
                    for(int i = 1; i <= cmd.size(); i++) {
                        if(cmd.get(i).isDouble() || cmd.get(i).isInt()) {
                            input.push_back(cmd.get(i).asDouble());
                        }
                    }
                    Prediction prediction = this->sendPredictSample(input);
                    std::string reply_str = printVector(input) + " -> " + printPrediction(prediction);
                    reply.addString(reply_str.c_str());

                    break;
                    }

                case VOCAB4('o','p','e','n'):
                    success = true;
                    if(cmd.get(1).isString()) {
                    	std::cout << "Open..." << std::endl;
                        this->dataset.open(cmd.get(1).asString().c_str());
                    }
                    reply.addString((std::string("Opened dataset: ") + cmd.get(1).asString().c_str()).c_str());
                    break;

                case VOCAB4('r','e','s','e'):
                case VOCAB3('r','s','t'):
                    success = true;
                    this->dataset.reset();
                    reply.addString("Dataset reset to beginning");
                    break;

                case VOCAB4('f','r','e','q'): // set sampling frequency
                    {
                    if(cmd.size() > 1 && cmd.get(1).isInt()) {
                        success = true;
                        this->frequency = cmd.get(1).asInt();
                        reply.addString((std::string("Current frequency: ") + inttostring(this->frequency)).c_str());
                    }
                    break;
                    }

                case VOCAB3('s','e','t'): // set some configuration options
                    // implement some options
                    break;

                default:
                    break;
            }
        } catch(const std::exception& e) {
            success = true; // to make sure YARP prints the error message
            std::string msg = std::string("Error: ") + e.what();
            reply.addString(msg.c_str());
        } catch(...) {
            success = true; // to make sure YARP prints the error message
            std::string msg = std::string("Error. (something bad happened, but I wouldn't know what!)");
            reply.addString(msg.c_str());
        }

        return success;
    }

    bool close() {
        this->unregisterAllPorts();
        return true;
    }

};

} // test
} // learningmachine
} // iCub

using namespace iCub::learningmachine::test;

int main(int argc, char *argv[]) {
    Network yarp;
    int ret;

    ResourceFinder rf;
    rf.setDefaultContext("learningMachine");
    rf.configure("ICUB_ROOT", argc, argv);

    MachineLearnerTestModule module;
    try {
        ret = module.runModule(rf);
        //module.attachTerminal();
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        module.close();
        return 1;
    } catch(char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        module.close();
        return 1;
    }
    return ret;
}

