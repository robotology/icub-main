/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Executable for merging data from different ports into a single LM formatted
 * port (portablepair<vector>)
 *
 */

 // ./merge --config "(/foo:o 3 /foo:o 1 /bar:o (2,3,4) /baz:o ()) ()" --names "()"

#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <stdexcept>

#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {
namespace merge {

/*
SourceList:
  DataSource
   : PortSource

DataSelector
  CompositeSelector has DataSelector
  DataSelector
  : AllSelector
  : IndexSelector
*/


//class PortSource : public DataSource {
//private:
//    /**
//     * Copy constructor.
//     */
//    PortSource(const PortSource& other);
//
//    /**
//     * Assignment operator.
//     */
//    PortSource& operator=(const PortSource& other);
//
//protected:
//    /**
//     *
//     */
//    std::string portPrefix;
//
//public:
//    /**
//     * Default constructor.
//     */
//    PortSource() : portPrefix("/lm/merge/source") { }
//
//    /**
//     * Default destructor.
//     */
//    ~PortSource() { }
//
//    /**
//     * Retrieves new data from the ports.
//     * @param
//     * @return
//     */
//    void update() {
//        // for each in portmap, port.read, store in datamap
//        PortMap::iterator it;
//        for(it = this->portMap.begin(); it != this->portMap.end(); it++ ) {
//            std::cout << "Updating port " << (*it).first << std::endl;
//            (*it).second.read(this->dataMap[(*it).first]);
//            //cout << (*it).first << " => " << (*it).second << endl;
//        }
//    }
//
//    const Bottle& getData(std::string portName) {
//        return this->dataMap[portName];
//    }
//
//    bool addPort(std::string portName) {
//        // check if portName exists
//        if(!this->hasPort(portName)) {
//            this->portMap[portName] = Port();
//            std::string inputName = this->getInputName();
//            this->portMap[portName].open(inputName.c_str());
//            Network::connect(portName.c_str(), inputName.c_str());
//            return true;
//        } else {
//            return false;
//        }
//    }
//
//    bool hasPort(std::string portName) {
//        return (this->portMap.count(portName) > 0);
//    }
//
//    std::string getInputName() {
//        std::ostringstream buffer;
//        int i = 1;
//        do {
//            // standard prefix + i
//            buffer.str(""); // clear buffer
//            buffer << this->portPrefix << i++ << ":o";
//        } while(Network::queryName(buffer.str().c_str()).isValid());
//        return buffer.str();
//    }
//
//};


class PortSource {
protected:
    Bottle data;
    Port port;
    std::string getInputName(std::string prefix) {
        std::ostringstream buffer;
        int i = 1;
        do {
            // standard prefix + i
            buffer.str(""); // clear buffer
            buffer << prefix << i++ << ":o";
        } while(Network::queryName(buffer.str().c_str()).isValid());
        return buffer.str();
    }

public:
    /**
     * Default constructor.
     */
    PortSource(std::string name, std::string prefix = "/lm/merge/source") {
        this->initPort(prefix);
        this->connect(name);
    }

    /**
     * Default destructor.
     */
    ~PortSource();

    /**
     * Copy constructor.
     */
    PortSource(const PortSource& other);

    /**
     * Assignment operator.
     */
    PortSource& operator=(const PortSource& other);

    virtual void initPort(std::string prefix) {
        port.open(this->getInputName(prefix).c_str());
    }

    virtual void connect(std::string name) {
        Network::connect(name.c_str(), this->port.where().getName().c_str());
    }

    virtual void update() {
        this->port.read(data);
    }

    virtual Bottle& getData() {
        return this->data;
    }

};



typedef std::map<std::string, PortSource*> SourceMap;

class SourceList {
protected:
    SourceMap sourceMap;

public:
    /**
     * Default constructor.
     */
    SourceList();

    /**
     * Default destructor.
     */
    ~SourceList() {
        // destruct list!
    }

    /**
     * Copy constructor.
     */
    SourceList(const SourceList& other);

    /**
     * Assignment operator.
     */
    SourceList& operator=(const SourceList& other);

    virtual void update() {
        // for each in portmap, port.read, store in datamap
        SourceMap::iterator it;
        for(it = this->sourceMap.begin(); it != this->sourceMap.end(); it++ ) {
            (*it).second->update();
            //std::cout << "Updating port " << (*it).first << std::endl;

            //cout << (*it).first << " => " << (*it).second << endl;
        }
    }

    virtual bool hasSource(std::string name) {
        return (this->sourceMap.count(name) > 0);
    }

    virtual void addSource(std::string name) {
        if(!this->hasSource(name)) {
            this->sourceMap[name] = new PortSource(name);
        }
    }

    virtual PortSource& getSource(std::string name) {
        if(!this->hasSource(name)) {
            throw std::runtime_error("Attempt to retrieve inexistent source.");
        }
        return *(this->sourceMap[name]);
    }
};


/**
 * The DataSelector is an abstract base class for an object that selects a
 * part of data from one or more DataSources. The structure of DataSelector and
 * its subclasses follows the composite pattern.
 */
class DataSelector {
protected:

public:

    /*void select(Bottle& output, SourceList& sources) {

    }*/
    //std::string printStructure(int indent);
    virtual std::string toString(int indent = 0) = 0;

    virtual void declareSources(SourceList& sl) = 0;

    virtual void select(Bottle& bot, SourceList& sl) = 0;

};

class IndexSelector : public DataSelector{
protected:
    std::string name;
    int index;
public:
    /**
     * Default constructor.
     */
    IndexSelector(std::string n, int i) : name(n), index(i) { }

    std::string toString(int indent = 0) {
        std::ostringstream buffer;
        buffer << std::string(indent, ' ') << this->name << "(" << this->index << ")" << std::endl;
        return buffer.str();
    }

    virtual void declareSources(SourceList& sl) {
        sl.addSource(this->name);
    }

    virtual void select(Bottle& bot, SourceList& sl) {
        // to implement
    }
};

class AllSelector : public DataSelector {
protected:
    std::string name;
public:
    /**
     * Default constructor.
     */
    AllSelector(std::string n) : name(n) { }

    std::string toString(int indent = 0) {
        std::ostringstream buffer;
        buffer << std::string(indent, ' ') << this->name << std::endl;
        return buffer.str();
    }

    virtual void declareSources(SourceList& sl) {
        sl.addSource(this->name);
    }

    virtual void select(Bottle& bot, SourceList& sl) {
        // to implement
    }
};



class CompositeSelector : public DataSelector {
protected:
    std::vector<DataSelector*> children;
public:
    /**
     * Default constructor.
     */
    CompositeSelector(Bottle& format) {
        this->loadFormat(format);
    }

    /**
     * Default destructor.
     */
    ~CompositeSelector() {
        // clear children vector
        for(int i = 0; i < this->children.size(); i++) {
            delete this->children[i];
        }
        this->children.clear();
        this->children.resize(0);
    }

    /**
     * Copy constructor.
     */
    CompositeSelector(const CompositeSelector& other);

    /**
     * Assignment operator.
     */
    CompositeSelector& operator=(const CompositeSelector& other);

// ./merge --config "(/foo:o 3 /foo:o 1 /bar:o 2 3 4) /baz:o ()) ()" --names "()"
    void loadFormat(Bottle& format) {
        int i = 0;
        int len = format.size();
        while(i < len) {
            if(format.get(i).isString()) {
                std::string specifier = format.get(i).asString().c_str();
                i++;
                if(format.get(i).isInt()) {
                    do {
                        int index = format.get(i).asInt();
                        this->children.push_back(new IndexSelector(specifier, index));
                        i++;
                    } while(format.get(i).isInt());
                } else {
                    this->children.push_back(new AllSelector(specifier));
                }
            } else if(format.get(i).isList()) {
                    this->children.push_back(new CompositeSelector(*(format.get(i).asList())));
                    i++;
            }
        }
    }

    std::string toString(int indent = 0) {
        std::ostringstream buffer;
        buffer << std::string(indent, ' ') << "(" << std::endl;
        for(int i = 0; i < this->children.size(); i++) {
            buffer << this->children[i]->toString(indent + 2);
        }
        buffer << std::string(indent, ' ') << ")" << std::endl;
        return buffer.str();
    }

    virtual void declareSources(SourceList& sl) {
        for(int i = 0; i < this->children.size(); i++) {
            this->children[i]->declareSources(sl);
        }
    }

    virtual void select(Bottle& bot, SourceList& sl) {
        Bottle& bot2 = bot.addList();
        for(int i = 0; i < this->children.size(); i++) {
            this->children[i]->select(bot2, sl);
        }
    }
};

//typedef std::map<std::string, PortSource> SourceMap;
typedef std::map<std::string, Bottle> DataMap;

// have a paired structure of Port and Bottle, e.g. PortItem
// let PortSource use a map<string, PortItem>, this prevents to have
// double maps with identical keys




class MergeModule : public Module {
protected:
    /**
     * Desired period of the module updates.
     */
    double desiredPeriod;

    /**
     * Time spent in the last update routine.
     */
    double updateTiming;

    /**
     * The collecting resource for all data from the ports.
     */
    //PortSource portSource;

    /**
     *
     */
    DataSelector* dataSelector;

public:
    /**
     * Constructor.
     */
    MergeModule() : dataSelector((DataSelector*) 0) { }

    /**
     * Destructor.
     */
    ~MergeModule() {
        delete this->dataSelector;
    }

    /*
     * Inherited from yarp::os::Module
     */
    virtual double getPeriod() {
        return (this->updateTiming < this->desiredPeriod) ?
               (this->desiredPeriod - this->updateTiming) : 0.;
    }

    /*
     * Inherited from yarp::os::Module
     */
    virtual bool interruptModule() {
        return true;
    }

    /*
     * Inherited from yarp::os::Module
     */
    virtual bool open(Searchable& opt) {
        // read for the general specifiers:
        Value* val;
        bool success = false;

        if(opt.check("format", val)) {
            // prolly should check if it's a list...
            if(val->isList()) {
                this->dataSelector = new CompositeSelector(*(val->asList()));
                success = true;
            } else {
                throw std::runtime_error("The format must be a list!");
            }
        } else {
            // error, no format!
            //printErrorWithHelp("Please supply a format!");
        }


        return success;
    }

    /*
     * Inherited from yarp::os::Module
     */
    virtual bool updateModule() {
        double updateStart = yarp::os::Time::now();

        //this->portSource.update();
        //this->listeners.process(this->portSource);

        double updateEnd = yarp::os::Time::now();
        this->updateTiming = updateStart - updateEnd;
        return true;
    }

    bool respond(const Bottle& cmd, Bottle& reply) {
        bool success = false;

        try {
            switch(cmd.get(0).asVocab()) {
                case VOCAB4('h','e','l','p'): // print help information
                    success = true;
                    reply.add(Value::makeVocab("help"));

                    reply.addString("Merge module configuration options");
                    reply.addString("  help                  Displays this message");
                    reply.addString("  info                  Prints information");
                    reply.addString("  freq f                Sampling frequency in Hertz (0 for disabled)");
                    break;

                case VOCAB4('i','n','f','o'): // print information
                    {
                    reply.add(Value::makeVocab("help"));
                    success = true;
                    reply.addString(this->dataSelector->toString().c_str());
                    break;
                    }

                case VOCAB4('f','r','e','q'): // set sampling frequency
                    {
                    if(cmd.size() > 1 && (cmd.get(1).isInt() || cmd.get(1).isDouble())) {
                        success = true;
                        this->setDesiredPeriod(1. / cmd.get(1).asDouble());
                        //reply.addString((std::string("Current frequency: ") + cmd.get(1).toString().c_str()).c_str());
                    }
                    break;
                    }

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

    /**
     * Mutator for the desired period.
     * @param p  the desired period in seconds
     * @return
     */
    virtual void setDesiredPeriod(double p) {
        this->desiredPeriod = p;
    }

    /**
     * Accessor for the desired period.
     * @return the desired period in seconds
     */
    virtual double getDesiredPeriod() {
        return this->desiredPeriod;
    }
};

} // merge
} // learningmachine
} // iCub

using namespace iCub::learningmachine::merge;

int main(int argc, char *argv[]) {
    Network yarp;

    MergeModule module;
    try {
        module.runModule(argc,argv);
        //module.attachTerminal();
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch(char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }
    return 0;
}

