/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * A template for wrapping abstract base classes in a concrete class.
 *
 */

#ifndef __ICUB_PORTABLE_TEMPLATE__
#define __ICUB_PORTABLE_TEMPLATE__

#include <stdexcept>
#include <string>
#include <cassert>
#include <fstream>
#include <sstream>

#include <yarp/os/Portable.h>
#include <yarp/os/Bottle.h>

#include "iCub/FactoryT.h"

using namespace yarp::os;

namespace iCub {
namespace learningmachine {

/**
 * A templated portable class intended to wrap abstract base classes. This 
 * template depends on an associated FactoryT for the specified type.
 *
 * \see iCub::learningmachine::MachinePortable
 * \see iCub::learningmachine::TransformerPortable
 *
 * \author Arjan Gijsberts
 *
 */

template<class T>
class PortableT : public Portable {
private:
    /**
     * A pointer to the actual wrapped base class.
     */
    T* wrapped;

public:
    /**
     * Constructor.
     *
     * @param w initial wrapped object
     */
    PortableT(T* w = (T*) 0) {
        this->setWrapped(w);
    }

    /**
     * Constructor.
     *
     * @param name name specifier of the wrapped object
     */
    PortableT(std::string name) {
        this->setWrapped(name);
    }

    /**
     * Destructor.
     */
    virtual ~PortableT() {
        delete(this->wrapped);
    }
    
    /**
     * Writes a wrapped object to a connection.
     *
     * @param connection the connection
     * @return true on success
     */
    bool write(ConnectionWriter& connection) {
        // return false directly if there is no machine. If not, we end up
        // up writing things on the port, after which an exception will be
        // thrown when accessing the machine.
        if(!this->hasWrapped()) {
            return false;
        }
        connection.appendInt(BOTTLE_TAG_LIST);
        connection.appendInt(2);
        Bottle nameBottle;
        nameBottle.addString(this->wrapped->getName().c_str());
        nameBottle.write(connection);
        this->getWrapped()->write(connection);
    
        // for text readers
        connection.convertTextMode();
        return true;
    }

    /**
     * Reads a wrapped object from a connection.
     *
     * @param connection the connection
     * @return true on success
     */
    bool read(ConnectionReader& connection) {
	    if(!connection.isValid()) {
        	return false;
	    }
	
	    connection.convertTextMode();
	    // check headers for the pair (name + actual object serialization)
	    int header = connection.expectInt();
	    int len = connection.expectInt();
	    if(header != BOTTLE_TAG_LIST || len != 2) {
	        return false;
	    }
	
	    // read machine identifier and use it to create object
	    Bottle nameBottle;
	    nameBottle.read(connection);
	    std::string name = nameBottle.get(0).asString().c_str();
	    this->setWrapped(name);
	    if(this->wrapped == (T *) 0) {
	        return false;
	    }
	
	    // call read method to construct specific object
	    bool ok = this->wrapped->read(connection);
	    return ok;
    }

    /**
     * Writes a wrapped object to a file.
     *
     * @param filename the filename
     * @return true on success
     */
    bool writeToFile(std::string filename) {
	    std::ofstream stream(filename.c_str());

	    if(!stream.is_open()) {
	        throw std::runtime_error(std::string("Could not open file '") + filename + "'");
	    }

	    stream << this->getWrapped()->getName() << std::endl;
	    stream << this->getWrapped()->toString();

	    stream.close();

	    return true;    
    }
    
    /**
     * Reads a wrapped object from a file.
     *
     * @param filename the filename
     * @return true on success
     */
    bool readFromFile(std::string filename) {
	    std::ifstream stream(filename.c_str());

	    if(!stream.is_open()) {
            throw std::runtime_error(std::string("Could not open file '") + filename + "'");
	    }

	    std::string name;
	    stream >> name;

	    this->setWrapped(name);
	    std::stringstream strstr;
	    strstr << stream.rdbuf();
	    this->getWrapped()->fromString(strstr.str());

	    return true;
    }

    /**
     * Returns true iff if there is a wrapped object.
     *
     * @return true iff there is a wrapped machine
     */
    bool hasWrapped() {
        return (this->wrapped != (T*) 0);
    }

    /**
     * The accessor for the wrapped object.
     *
     * @return a pointer to the wrapped object
     */
    T* getWrapped() {
        if(!this->hasWrapped()) {
            throw std::runtime_error("Attempt to retrieve inexistent wrapped object!");
        }
        return this->wrapped;
    }

    /**
     * The mutator for the wrapped object.
     *
     * @param w a pointer to object to wrap
     * @param wipe boolean whether the previous wrapped object has to be deleted
     */
    void setWrapped(T* w, bool wipe = true) {
        if(wipe && this->hasWrapped()) {
            delete this->wrapped;
            this->wrapped = (T*) 0;
        }
        this->wrapped = w;
    }

    /**
     * The mutator for the wrapped object.
     *
     * @param name The name specifier for the wrapped object
     * @param wipe boolean whether the previous wrapped object has to be deleted
     */
    void setWrapped(std::string name, bool wipe = true) {
        if(wipe && this->hasWrapped()) {
            delete this->wrapped;
            this->wrapped = (T*) 0;
        }
        this->wrapped = FactoryT<std::string, T>::instance().clone(name);
    }


};

} // learningmachine
} // iCub

#endif
