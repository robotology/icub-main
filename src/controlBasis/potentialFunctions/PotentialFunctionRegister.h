// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-  
#ifndef _POTENTIAL_FUNCTION_REGISTER__H_
#define _POTENTIAL_FUNCTION_REGISTER__H_

#include <PotentialFunctionFactory.h>

// potential functions
#include <CosineField.h>
#include <ManipulabilityField.h>
#include <ConfigurationSquaredError.h>
#include <CartesianPositionSquaredError.h>
#include <CartesianPositionHarmonicFunction.h>


namespace CB {

    void registerPotentialFunctions() {
        PotentialFunctionFactory::instance().registerClass<ConfigurationSquaredError>(new ConfigurationSquaredError());
        PotentialFunctionFactory::instance().registerClass<CartesianPositionSquaredError>(new CartesianPositionSquaredError());
        PotentialFunctionFactory::instance().registerClass<CosineField>(new CosineField());
        PotentialFunctionFactory::instance().registerClass<ManipulabilityField>(new ManipulabilityField());
        PotentialFunctionFactory::instance().registerClass<CartesianPositionHarmonicFunction>(new CartesianPositionHarmonicFunction());
    }

}

#endif
