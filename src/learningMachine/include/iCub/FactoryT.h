/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * A singleton template implementation for the abstract factory pattern.
 *
 */

#ifndef LM_FACTORY_TEMPLATE__
#define LM_FACTORY_TEMPLATE__

#include <map>
#include <string>
#include <cassert>
#include <stdexcept>
#include <sstream>

namespace iCub {
namespace learningmachine {

/**
 *
 * A template class for the factory pattern. The factory should be used to
 * create concrete instances of abstract, registered classes.
 *
 * \author Arjan Gijsberts
 *
 */
template<class K, class T>
class FactoryT {
private:
    /**
     * The map that stores the key to object mapping.
     */
    std::map<K, T*> map;

    /**
     * Constructor (empty).
     */
    FactoryT() { }

    /**
     * Copy Constructor (private and unimplemented on purpose).
     */
    FactoryT(const FactoryT<K, T>& other);

    /**
     * The default destructor of the Factory template. This destructor takes
     * responsibility for deleting the prototype objects that have been
     * registered during its lifetime.
     */
    virtual ~FactoryT() {
        for(typename std::map<K, T* >::iterator it = this->map.begin(); it != this->map.end(); it++) {
            delete it->second;
        }
    }

    /**
     * Assignment operator (private and unimplemented on purpose).
     */
    FactoryT<K, T>& operator=(const FactoryT<K, T>& other);

public:
    /**
     * An instance retrieval method that follows the Singleton pattern.
     *
     * Note that this implementation should not be considered thread safe and
     * that problems may arise. However, due to the nature of the expected use
     * this will not be likely to result in problems.
     *
     * See http://www.oaklib.org/docs/oak/singleton.html for more information.
     *
     * @return the singleton factory instance
     */
    static FactoryT<K, T>& instance() {
        static FactoryT<K, T> instance;
        return instance;
    }

    /**
     * Registers a prototype object that can be used to create clones. Strictly
     * speaking, for this application the object only has to be able to return a
     * new object of its own type, regardless of the internal state of that
     * object.
     *
     * @param prototype the prototype object
     * @throw runtime error if the name of the prototype is empty
     * @throw runtime error if another object is registered with the same name
     */
    void registerPrototype(T* prototype) {
        assert(prototype != (T*) 0);

        if(prototype->getName() == "") {
            throw std::runtime_error("Cannot register prototype with empty key; please specify a unique key.");
        }

        if(this->map.count(prototype->getName()) > 0) {
            std::ostringstream buffer;
            buffer << "Prototype '" << prototype->getName()
                   << "' has already been registered; please specify a unique key.";
            throw std::runtime_error(buffer.str());
        }

        this->map[prototype->getName()] = prototype;
    }


    /**
     * Creates a new object given a specific type of key. The receiving end
     * takes ownership of the returned pointer.
     *
     * @param key a key that identifies the type of machine
     * @return a copied object of the specified type
     * @throw runtime error if no object has been registered with the same key
     */
    T* create(const K& key) {
        if(this->map.count(key) > 0) {
            std::ostringstream buffer;
            buffer << "Could not find prototype '" << key
                   << "'; please specify a valid key.";
            throw std::runtime_error(buffer.str());
        }

        return this->map.find(key)->second->clone();
    }
};

} // learningmachine
} // iCub

#endif
