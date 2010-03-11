// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Stephen Hart
 * email:   stephen.hart@iit.it
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
#ifndef _CB_JACOBIAN_FACTORY__H__
#define _CB_JACOBIAN_FACTORY__H__

#include <ControlBasisJacobian.h>
#include <string>
#include <map>

namespace CB {
    
    /**
     * A class for creating Jacobian objects, with the type of object created based on a key
     * 
     * @param K the key
     * @param T the super class that all created classes derive from
     */
    class JacobianFactory { 

    private: 
        
        typedef ControlBasisJacobian* (*CreateObjectFunc)(std::string);
        
        /**
         * A map keys (strings) to functions (CreateObjectFunc)
         * When creating a new type, we simply call the function with the required key
         */
        std::map<std::string, CreateObjectFunc> mObjectCreator;

        /**
         * A vector of Jacobian prototypes that store the configuration information
         **/
        std::vector<ControlBasisJacobian *> prototypes;        

        /**
         * Pointers to this function are inserted into the map and called when creating objects
         *
         * @param S the type of class to create
         * @param Vin the vector of input names for the  class
         * @return a object with the type of S and inputs Vin
         */
        template<typename S> 
        static ControlBasisJacobian* createObject(std::string deviceName) {
            S* obj = new S();
            obj->setDevice(deviceName);
            return obj;
        }
        
    public:
        
        /**
         * Registers a class to that it can be created via createObject()
         *
         * @param S the type of class to create
         * @param pt a prototype of class S to register, this must be a subclass of T
         */ 
        template<typename S> 
        bool registerClass(S* pt){

            std::string id = pt->getInputSpace() + ":" + pt->getOutputSpace();

            if (mObjectCreator.find(id) != mObjectCreator.end()){ 
                //your error handling here
            }
            
            // store the information concerning this PF
            prototypes.push_back(pt);

            std::cout << "Jacobian Factory registering " << id << std::endl;
            // register the type of subclass
            mObjectCreator.insert( std::make_pair<std::string,CreateObjectFunc>(id, &createObject<S> ) ); 

            return true;
        }
        
        /**
         * Returns true if a given key exists
         *
         * @param id the id to check exists
         * @return true if the id exists
         */
        bool hasClass(std::string id){
            return mObjectCreator.find(id) != mObjectCreator.end();
        } 
        
        /**
         * Creates an object based on an id. It will return null if the key doesn't exist
         *
         * @param id the id of the object to create
         * @return the new object or null if the object id doesn't exist
         */
        ControlBasisJacobian* createObject(std::string inSpace, std::string outSpace, std::string deviceName){

            // we allow for users to get either the forward or inverse jacobian. 
            // we only store (aned return) the forwards, its up to users to query 
            // wehtehr they need the inverse or not via the needsInverse() function
            std::string id_fwd = inSpace + ":" + outSpace;
            std::string id_bwd = outSpace + ":" + inSpace;

            //Don't use hasClass here as doing so would involve two lookups
            std::map<std::string, CreateObjectFunc>::iterator iter = mObjectCreator.find(id_fwd); 
            if (iter == mObjectCreator.end()){ 
                iter = mObjectCreator.find(id_bwd); 
                if (iter == mObjectCreator.end()){ 
                    return NULL;
                }
            }
            //calls the required createObject() function
            return ((*iter).second(deviceName));
        }        

        /**
         * Constructor (empty)
         */
        JacobianFactory() { }

        /**
         * Destructor
         */
        ~JacobianFactory() { 
            for(int i=0; i<prototypes.size(); i++) 
                delete prototypes[i];
            prototypes.clear();
        }

        /**
         * instance operator to return the singleton
         **/
        static JacobianFactory& instance() {
            static JacobianFactory instance;
            return instance;
        }
                        
        /**
         * returns whether the thing asking for the jacobian really needs the inverse
         **/
        bool needsInverse(std::string inSpace, std::string outSpace) {
            bool r = false;
            
            //std::string id_fwd = inSpace + ":" + outSpace;
            std::string id_bwd = outSpace + ":" + inSpace;

            // if the backward key id exists in the factory, that means we need the inverse
            // of one of the registered Jacobians, so this returns true.  If it has the forward 
            //key, or doesnt have it at all, return false;
            return hasClass(id_bwd);

                
        }
    };
   

} // CB


#endif
