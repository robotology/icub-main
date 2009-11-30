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
#include <cassert>
#include <list>

#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
//#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/IOException.h>


using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {
namespace merge {


class PortSource {
protected:
    /**
     * Cached data.
     */
    Bottle data;

    /**
     * The port for incoming data.
     */
    Port port;

    /**
     * Returns the first free port given a prefix appended by an integer.
     *
     * @param prefix  the prefix for the port.
     * @return the string for the first available portname given the prefix
     */
    std::string getInputName(std::string prefix) {
        std::ostringstream buffer;
        int i = 1;
        do {
            // standard prefix + i
            buffer.str(""); // clear buffer
            buffer << prefix << i++ << ":i";
        } while (Network::queryName(buffer.str().c_str()).isValid());
        return buffer.str();
    }

    /**
     * Copy constructor (private and unimplemented on purpose).
     */
    PortSource(const PortSource& other);

    /**
     * Assignment operator (private and unimplemented on purpose).
     */
    PortSource& operator=(const PortSource& other);

public:
    /**
     * Default constructor.
     */
    PortSource(std::string name, std::string pp = "/lm/merge/source") {
        this->initPort(pp);
    }

    /**
     * Default destructor.
     */
    ~PortSource() {
        this->interrupt();
        this->close();
    }

    /**
     * Opens the first free incoming port with the given prefix.
     * @param prefix  the prefix for the name
     */
    virtual void initPort(std::string prefix) {
        port.open(this->getInputName(prefix).c_str());
    }

    /**
     * Connects the incoming port to the specified port.
     * @param dst  the destination port
     */
    virtual void connect(std::string dst) {
        if (Network::queryName(dst.c_str()).isValid()) {
            Network::connect(dst.c_str(), this->port.where().getName().c_str());
        } else {
            throw std::runtime_error("Cannot find requested port: " + dst);
        }
    }

    /**
     * Reads new data from the port and caches it locally.
     */
    virtual void update() {
        this->port.read(data);
    }

    /**
     * Returns the locally cached data.
     * @return a reference to the locally stored bottle
     */
    virtual Bottle& getData() {
        return this->data;
    }

    /**
     * Interrupts the port.
     */
    virtual void interrupt() {
        this->port.interrupt();
    }

    /**
     * Closes the port.
     */
    virtual void close() {
        this->port.close();
    }
};


class SourceList {
protected:
    typedef std::map<std::string, PortSource*> SourceMap;

    /**
     * Prefix for ports.
     */
    std::string portPrefix;

    /**
     * Map that links port names to the PortSource objects that are connected to
     * them.
     */
    SourceMap sourceMap;

public:
    /**
     * Default constructor.
     */
    SourceList(std::string pp = "/lm/merge/source") : portPrefix(pp) { }

    /**
     * Default destructor.
     */
    ~SourceList() {
        for (SourceMap::iterator it = this->sourceMap.begin(); it != this->sourceMap.end(); it++) {
            delete it->second;
        }
    }

    /**
     * Copy constructor.
     */
    SourceList(const SourceList& other);

    /**
     * Assignment operator.
     */
    SourceList& operator=(const SourceList& other);

    /**
     * Updates each registered port with new data.
     */
    virtual void update() {
        // for each in portmap, port.read, store in datamap
        SourceMap::iterator it;
        for (it = this->sourceMap.begin(); it != this->sourceMap.end(); it++ ) {
            it->second->update();
        }
    }

    /**
     * Returns true iff a PortSource has been registered for the given port name.
     */
    virtual bool hasSource(std::string name) {
        return (this->sourceMap.count(name) > 0);
    }

    /**
     *
     */
    virtual void addSource(std::string name) {
        if (!this->hasSource(name)) {
            this->sourceMap[name] = new PortSource(name, this->portPrefix);
            this->sourceMap[name]->connect(name);
        }
    }

    /**
     *
     */
    virtual PortSource& getSource(std::string name) {
        if (!this->hasSource(name)) {
            throw std::runtime_error("Attempt to retrieve inexistent source.");
        }
        return *(this->sourceMap[name]);
    }

    /**
     * Recursively interrupt all sources.
     */
    virtual void interrupt() {
        for (SourceMap::iterator it = this->sourceMap.begin(); it != this->sourceMap.end(); it++) {
            it->second->interrupt();
        }
    }

    /**
     * Recursively interrupt all sources.
     */
    virtual void close() {
        for (SourceMap::iterator it = this->sourceMap.begin(); it != this->sourceMap.end(); it++) {
            it->second->close();
        }
    }

    virtual std::string getPortPrefix() {
        return this->portPrefix;
    }

    virtual void setPortPrefix(std::string pp) {
        this->portPrefix = pp;
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

    virtual std::string toString(int indent = 0) = 0;

    virtual void declareSources(SourceList& sl) = 0;

    virtual void select(Bottle& bot, SourceList& sl) = 0;

};

class IndexSelector : public DataSelector {
protected:
    std::string name;
    std::list<std::list<int> > indices;

public:
    /**
     * Default constructor.
     */
    IndexSelector(std::string format) {
        this->loadFormat(format);
    }

    std::string toString(int indent = 0) {
        std::ostringstream buffer;
        //buffer << std::string(indent, ' ') << this->name << "(" << this->index << ")" << std::endl;
        buffer << std::string(indent, ' ') << this->name << std::endl;
        return buffer.str();
    }

    virtual void declareSources(SourceList& sl) {
        sl.addSource(this->name);
    }

    virtual void select(Bottle& bot, SourceList& sl) {
        //bot.add(sl.getSource(this->name).getData().get(this->index));
    }

    virtual void loadFormat(std::string format) {
        std::cout << "Parsing format: " << format << std::endl;
        // find indexing specifier
        std::string::size_type idxStart = format.find("[");
        this->name = format.substr(0, idxStart);

        std::string::size_type idxEnd;
        while (idxStart != std::string::npos) {
            idxEnd = format.find("]", idxStart);
            if (idxEnd == std::string::npos) {
                throw std::runtime_error("Missing closing bracket ']'");
            }
            this->loadIndices(format.substr(idxStart + 1, idxEnd - idxStart - 1));
            idxStart = format.find("[", idxStart + 1);
            if (idxStart != std::string::npos && (idxStart < idxEnd)) {
                throw std::runtime_error("Unexpected opening bracket '['");
            }
        }
    }

    virtual void loadIndices(std::string format) {
        std::cout << "Parsing index format: " << format << std::endl;

    }

    static std::vector<std::string> split(std::string input, std::string delimiter) {
        std::string::size_type start, end = 0;
        std::vector<std::string> output;
        while (end != std::string::npos) {
            end = input.find(delimiter, start);
            output.push_back(input.substr(start, (end == std::string::npos) ? end : end - start));
        }
        return output;
    }

    static int stringToInt(std::string str) {
        std::istringstream buffer(str);
        int ret;
        if (buffer >> ret) {
            return ret;
        } else {
            throw std::runtime_error("Could not read integer from " + str);
        }
    }
};

//class IndexSelector : public DataSelector{
//protected:
//    std::string name;
//    int index;
//public:
//    /**
//     * Default constructor.
//     */
//    IndexSelector(std::string n, int i) : name(n), index(i) { }
//
//    std::string toString(int indent = 0) {
//        std::ostringstream buffer;
//        buffer << std::string(indent, ' ') << this->name << "(" << this->index << ")" << std::endl;
//        return buffer.str();
//    }
//
//    virtual void declareSources(SourceList& sl) {
//        sl.addSource(this->name);
//    }
//
//    virtual void select(Bottle& bot, SourceList& sl) {
//        bot.add(sl.getSource(this->name).getData().get(this->index));
//    }
//};

//class AllSelector : public DataSelector {
//protected:
//    std::string name;
//public:
//    /**
//     * Default constructor.
//     */
//    AllSelector(std::string n) : name(n) { }
//
//    std::string toString(int indent = 0) {
//        std::ostringstream buffer;
//        buffer << std::string(indent, ' ') << this->name << std::endl;
//        return buffer.str();
//    }
//
//    virtual void declareSources(SourceList& sl) {
//        sl.addSource(this->name);
//    }
//
//    virtual void select(Bottle& bot, SourceList& sl) {
//        Bottle& src = sl.getSource(this->name).getData();
//        for(int i = 0; i < src.size(); i++) {
//            bot.add(src.get(i));
//        }
//    }
//};



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
        for (int i = 0; i < this->children.size(); i++) {
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

    void addChild(DataSelector* ds) {
        this->children.push_back(ds);
    }

    /**
     *
     */
    void loadFormat(Bottle& format) {
        int i = 0;
        int len = format.size();
        while (i < len) {
            if (format.get(i).isString()) {
                this->addChild(new IndexSelector(format.get(i).asString().c_str()));
                /*
                std::string name = format.get(i).asString().c_str();
                i++;
                if(format.get(i).isInt()) {
                    do {
                        int index = format.get(i).asInt();
                        this->addChild(new IndexSelector(name, index-1));
                        i++;
                    } while(format.get(i).isInt());
                } else {
                    this->addChild(new AllSelector(name));
                }*/
            } else if (format.get(i).isList()) {
                this->addChild(new CompositeSelector(*(format.get(i).asList())));
            } else {
                throw std::runtime_error(std::string("Unexpected token during parsing: ") +
                                         format.get(i).asString().c_str());
            }
            i++;
        }
    }

    std::string toString(int indent = 0) {
        std::ostringstream buffer;
        buffer << std::string(indent, ' ') << "(" << std::endl;
        for (int i = 0; i < this->children.size(); i++) {
            buffer << this->children[i]->toString(indent + 2);
        }
        buffer << std::string(indent, ' ') << ")" << std::endl;
        return buffer.str();
    }

    virtual void declareSources(SourceList& sl) {
        for (int i = 0; i < this->children.size(); i++) {
            this->children[i]->declareSources(sl);
        }
    }

    virtual void select(Bottle& bot, SourceList& sl) {
        Bottle& bot2 = bot.addList();
        for (int i = 0; i < this->children.size(); i++) {
            this->children[i]->select(bot2, sl);
        }
    }
};


class MergeModule : public RFModule {
protected:
    /**
     * Desired period of the module updates.
     */
    double desiredPeriod;

    /**
     * The collecting resource for all data from all sources.
     */
    SourceList sourceList;

    /**
     * Prefix for the ports.
     */
    std::string portPrefix;

    /**
     * A pointer to the root DataSelector.
     */
    DataSelector* dataSelector;

    /**
     * Output port.
     */
    Port output;

    void printOptions(std::string error = "") {
        int errorCode = 0;
        if (error != "") {
            std::cerr << "Error: " << error << std::endl;
        }
        std::cout << "Available options" << std::endl;
        std::cout << "--format               The format for the output (required)" << std::endl;
        std::cout << "--frequency f          Sampling frequency in Hz" << std::endl;
        std::cout << "--port pfx             Prefix for registering the ports" << std::endl;
    }

    void registerPort(Contactable& port, std::string name) {
        if (port.open(name.c_str()) != true) {
            std::string msg("could not register port ");
            msg+=name;
            throw std::runtime_error(msg);
        }
    }

    void registerAllPorts() {
        this->registerPort(this->output, this->portPrefix + "/output:o");
    }

    void unregisterAllPorts() {
        try {
            this->sourceList.close();
            this->output.close();
        } catch (yarp::IOException e) {
            printf("Exception happened while unregistering ports\n");
            printf("Message: %s\n", e.toString().c_str());
        }
    }


public:
    /**
     * Constructor.
     */
    MergeModule(std::string pp = "/lm/merge")
            : portPrefix(pp), dataSelector((DataSelector*) 0), desiredPeriod(0.1) { }

    /**
     * Destructor.
     */
    ~MergeModule() {
        delete this->dataSelector;
    }

    /*
     * Inherited from yarp::os::RFModule
     */
    virtual double getPeriod() {
        return this->desiredPeriod;
    }

    /*
     * Inherited from yarp::os::RFModule
     */
    virtual bool interruptModule() {
        this->sourceList.interrupt();
        this->output.interrupt();
        return true;
    }

    /*
     * Inherited from yarp::os::RFModule
     */
    virtual bool close() {
        this->unregisterAllPorts();
        return true;
    }

    /*
     * Inherited from yarp::os::RFModule
     */
    virtual bool configure(ResourceFinder& opt) {
        // read for the general specifiers:
        Value* val;
        bool success = false;

        if (opt.check("help")) {
            this->printOptions();
            return false;
        }

        // check for port specifier: portSuffix
        if (opt.check("port", val)) {
            this->portPrefix = val->asString().c_str();
            this->sourceList.setPortPrefix(this->portPrefix);
        }

        // read and parse format
        if (opt.check("format", val)) {
            if (val->isList()) {
                this->dataSelector = new CompositeSelector(*(val->asList()));
                this->dataSelector->declareSources(this->sourceList);
                success = true;
            } else {
                throw std::runtime_error("The format must be a list!");
            }
        } else {
            // error, no format!
            this->printOptions("Please supply a format!");
            return false;
        }


        if (opt.check("frequency", val)) {
            if (val->isDouble() || val->isInt()) {
                this->setFrequency(val->asDouble());
            }
        }

        this->registerAllPorts();

        this->attachTerminal();

        return success;
    }


    /*
     * Inherited from yarp::os::RFModule
     */
    virtual bool updateModule() {
        assert(this->dataSelector != (DataSelector*) 0);
        this->sourceList.update();
        Bottle out;
        this->dataSelector->select(out, this->sourceList);
        std::cout << "Bottle: " << out.toString().c_str() << std::endl;
        this->output.write(out);
        //this->listeners.process(this->portSource);
        return true;
    }

    /*
     * Inherited from yarp::os::RFModule
     */
    bool respond(const Bottle& cmd, Bottle& reply) {
        bool success = false;

        try {
            switch (cmd.get(0).asVocab()) {
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
                if (cmd.size() > 1 && (cmd.get(1).isInt() || cmd.get(1).isDouble())) {
                    success = true;
                    this->setDesiredPeriod(1. / cmd.get(1).asDouble());
                    //reply.addString((std::string("Current frequency: ") + cmd.get(1).toString().c_str()).c_str());
                }
                break;
            }

            default:
                break;
            }
        } catch (const std::exception& e) {
            success = true; // to make sure YARP prints the error message
            std::string msg = std::string("Error: ") + e.what();
            reply.addString(msg.c_str());
        } catch (...) {
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
     * Mutator for the desired period by means of setting the frequency.
     * @param f  the desired frequency
     * @return
     */
    virtual void setFrequency(double f) {
        if (f <= 0) {
            throw std::runtime_error("Frequency must be larger than 0");
        }
        this->setDesiredPeriod(1. / f);
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
    int ret;

    ResourceFinder rf;
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setDefaultContext("learningMachine");
    MergeModule module;
    try {
        ret = module.runModule(rf);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }
    return ret;
}

