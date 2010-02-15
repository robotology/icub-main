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
#ifndef _CB_POTENTIAL_FUNCTION_FACTORY__H__
#define _CB_POTENTIAL_FUNCTION_FACTORY__H__

#include <ControlBasisPotentialFunction.h>
#include <string>
#include <vector>
#include <map>

namespace CB {
    
    /**
     * A class for creating PotentialFunction objects, with the type of object created based on a key
     * 
     * @param K the key
     * @param T the super class that all created classes derive from
     */
    class PotentialFunctionFactory { 

    private: 
        
        typedef ControlBasisPotentialFunction* (*CreateObjectFunc)(std::vector<std::string>);
        
        /**
         * A map keys (strings) to functions (CreateObjectFunc)
         * When creating a new type, we simply call the function with the required key
         */
        std::map<std::string, CreateObjectFunc> mObjectCreator;

        /**
         * A vector of PF prototypes that store the configuration information
         **/
        std::vector<ControlBasisPotentialFunction *> prototypes;        

        /**
         * Pointers to this function are inserted into the map and called when creating objects
         *
         * @param S the type of class to create
         * @param Vin the vector of input names for the  class
         * @return a object with the type of S and inputs Vin
         */
        template<typename S> 
        static ControlBasisPotentialFunction* createObject(std::vector<std::string> Vin) {
            S* obj = new S();
            obj->setInputs(Vin);
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

            std::string id = "/cb/" + pt->getSpace() + "/" + pt->getType();

            if (mObjectCreator.find(id) != mObjectCreator.end()){ 
                //your error handling here
            }
            
            // store the information concerning this PF
            prototypes.push_back(pt);

            std::cout << "PF Factory registering " << id << std::endl;
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
        ControlBasisPotentialFunction* createObject(std::string id, std::vector<std::string> Vin){
            //Don't use hasClass here as doing so would involve two lookups
            std::map<std::string, CreateObjectFunc>::iterator iter = mObjectCreator.find(id); 
            if (iter == mObjectCreator.end()){ 
                return NULL;
            }
            //calls the required createObject() function
            return ((*iter).second(Vin));
        }        

        /**                                                                                                                                                                                                        
         * Constructor (empty).                                                                                                                                                                                    
         */
        PotentialFunctionFactory() { }
        
        /**                                                                                                                                                                                                        
         * Destructor
         */
        ~PotentialFunctionFactory() { 
            for(int i=0; i<prototypes.size(); i++) 
                delete prototypes[i];
            prototypes.clear();
        }

        /**
         * instance operator to return the singleton
         **/
        static PotentialFunctionFactory& instance() {
            static PotentialFunctionFactory instance;
            return instance;
        }
        
        /**
         * gets the number of PFs registered
         **/
        int getNumRegisteredPotentialFunctions() {
            return prototypes.size();
        }
        
        /**
         * gets the key of the specified element
         **/
        std::string getName(int id) {
            if(id>=prototypes.size()) {
                std::cout<<"PotentialFunctionFactory::getName() -- element out of range!!"<<std::endl;
                return "";
            }
            std::string name = prototypes[id]->getSpace() + "/" + prototypes[id]->getType();
            return name;
        }
        
        /**
         * gets the info for the specified element
         **/
        PotentialFunctionInfo getPotentialFunctionInfo(int id) {
            
            PotentialFunctionInfo pf;
            pf.name = "";
            pf.space = "";
            pf.hasReference = false;
            
            if(id>=prototypes.size()) {
                std::cout<<"PotentialFunctionFactory::getPotentialFunctionInfo() -- element out of range!!"<<std::endl;
                return pf;
            }
            
            pf.space = prototypes[id]->getSpace();
            pf.name = pf.space + "/" + prototypes[id]->getType();
            pf.hasReference = prototypes[id]->needsReference();

            return pf;
        }
        
    };
   

} // CB


#endif
