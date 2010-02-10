// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-  
#ifndef _POTENTIAL_FUNCTION_MAP__H_
#define _POTENTIAL_FUNCTION_MAP__H_

// potential functions
#include <ConfigurationSquaredError.h>
#include <CartesianPositionSquaredError.h>
#include <CartesianPositionHarmonicFunction.h>
#include <CosineField.h>
#include <ManipulabilityField.h>

namespace CB {
    
    class PotentialFunctionMap {

    protected:

    public:

        PotentialFunctionMap() { }
        ~PotentialFunctionMap() { }
        
        ControlBasisPotentialFunction * getPotentialFunction(std::string pf, std::string sensorName, std::string referenceName) {
          
            ControlBasisPotentialFunction *potentialFunction;
            if(pf == "/cb/configuration/squared_error_pf") {
                potentialFunction = new ConfigurationSquaredError(sensorName, referenceName);
            } else if( pf == "/cb/cartesianposition/squared_error_pf") {
                potentialFunction = new CartesianPositionSquaredError(sensorName,referenceName);
            } else if( pf == "/cb/configuration/cosfield_pf") {
                potentialFunction = new CosineField(sensorName);
            } else if( pf == "/cb/configuration/manipulability_pf") {
                potentialFunction = new ManipulabilityField(sensorName);
            } else if( pf == "/cb/cartesianposition/harmonic_pf") {
                potentialFunction = new CartesianPositionHarmonicFunction(sensorName);
            } else {
                std::cout << "Controller got unknown potential function: " << pf.c_str() << std::endl;
                return NULL;
            }
            return potentialFunction;
            
        }
        
    };
}

#endif
