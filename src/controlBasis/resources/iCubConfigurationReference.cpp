// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "iCubConfigurationReference.h"

CB::iCubConfigurationReference::iCubConfigurationReference(std::string name, int dofs) {
    
    // set configuration info
    deviceName = name + "/ref";
    numDOFs = dofs;
    numLinks = 1;
    size = dofs;
    
    moveable = false;
    
    values.resize(numDOFs); values.zero();
    desiredValues.resize(numDOFs); desiredValues.zero();
    maxLimits.resize(numDOFs); maxLimits.zero();
    minLimits.resize(numDOFs); minLimits.zero();
    
    DHParameters.resize(4,numLinks); DHParameters.zero();
    LinkTypes.resize(numLinks); LinkTypes.zero();
    
    std::cout << "Creating new iCubConfigurationReference(name=" << deviceName.c_str() << ",dof=" << numDOFs << ")" << std::endl;
    
    initPorts(); // mandatory init function
}

void CB::iCubConfigurationReference::setVals(yarp::sig::Vector ref) {
    if(ref.size() != values.size()) {
        std::cout << "Couldn't set reference values for " << deviceName.c_str() << "!!" << std::endl;
        return;
    } else {
        values = ref;        
    }
}
