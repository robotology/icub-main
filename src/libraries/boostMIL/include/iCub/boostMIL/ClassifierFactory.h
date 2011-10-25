/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Carlo Ciliberto
 * email:  carlo.ciliberto@iit.it
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



#include <string>
#include <cassert>
#include <stdexcept>
#include <sstream>

#include <map>
#include <deque>

#include "WeakClassifier.h"
#include "ClassifierInput.h"




#ifndef __CLASSIFIER_FACTORY__
#define __CLASSIFIER_FACTORY__

namespace iCub
{

namespace boostMIL
{

class ClassifierFactory {
protected:
    /**
     * The map that stores the key to object mapping.
     */
    std::map<std::string,WeakClassifier*> map;

private:

    /**
     * Constructor (empty).
     */
    ClassifierFactory() { }

    /**
     * Copy Constructor (private and unimplemented on purpose).
     */
    ClassifierFactory(const ClassifierFactory& other);

    /**
     * The default destructor of the Factory template. This destructor takes
     * responsibility for deleting the prototype objects that have been
     * registered during its lifetime.
     */
    virtual ~ClassifierFactory() {
        for(std::map<std::string,WeakClassifier*>::iterator it = this->map.begin(); it != this->map.end(); it++) {
            delete it->second;
        }
    }

    /**
     * Assignment operator (private and unimplemented on purpose).
     */
    ClassifierFactory& operator=(const ClassifierFactory& other);

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
    static ClassifierFactory &instance() {
        static ClassifierFactory instance;
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
    void registerClassifier(WeakClassifier *h) {
        assert(h != (WeakClassifier*) 0);

        if(h->getType() == "") {
            throw std::runtime_error("Cannot register classifier with empty type; please specify a unique key.");
        }

        if(this->map.count(h->getType()) > 0) {
            std::ostringstream buffer;
            buffer << "WeakClassifier of type '" << h->getType()
                   << "' has already been registered; please specify a unique key.";
            throw std::runtime_error(buffer.str());
        }

        this->map[h->getType()] = h;

        std::map<std::string,WeakClassifier*>::iterator itr = map.begin();
    }


    /**
     * Creates a new object given a specific type of key. The receiving end
     * takes ownership of the returned pointer.
     *
     * @param key a key that identifies the type of machine
     * @return a copied object of the specified type
     * @throw runtime error if no object has been registered with the same key
     */
    WeakClassifier* create(const std::string &type) const
    {
        if(this->map.count(type) == 0) 
        {
            std::ostringstream buffer;
            buffer << "Could not find prototype '" << type
                   << "'; please specify a valid key.";
            throw std::runtime_error(buffer.str());
        }

        return this->map.find(type)->second->create();
    }

    /*
    **
    * Extracts a deque of associated WeakClassifiers from each type of WeakClassifier 
    * registered to the Factory.
    *
    * @param input an input.
    * @param function_space a deque of WeakClassifiers
    */
    void pack(const Inputs *input, std::deque<WeakClassifier*> &function_space)
    {
        //for each type registered call the pack method of the WeakClassifier
        for(std::map<std::string,WeakClassifier*>::iterator itr = map.begin(); itr != map.end(); itr++)
        {
            std::list<WeakClassifier*> *list = itr->second->pack(input);
            if(list != NULL)
            {
                while(list->size())
                {
                    function_space.push_back(list->front());
                    list->pop_front();
                }
                delete list;
            }
        }
    }
};

}

}

#endif



