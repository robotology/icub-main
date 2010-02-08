// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_POSITION__H_
#define _CARTESIAN_POSITION__H_

#include "ControlBasisResource.h"

namespace CB {

  /**
   * This class instantiates the abstract ControlBasisResource class for a CartesianPosition 
   * type resource. It is still abstract, in that it doesn't implement a runnable resource. 
   * This class must be extended for specific cartesian position device that implements the 
   * start, update, and stop functions.
   **/
  class CartesianPosition : public ControlBasisResource {

  protected:
    
  public:

      /**
       * returns the X coordinate of the position
       **/    
      double getX() { return values[0]; }

      /**
       * returns the Y coordinate of the position
       **/    
      double getY() { return values[1]; }

      /**
       * returns the Z coordinate of the position
       **/    
      double getZ() { return values[2]; }

      /**
       * Constructor
       **/
    CartesianPosition() {

        type = "cartesianposition";
        std::cout << "setting type of CartesianPosition to " << type.c_str() << std::endl;

        size = 3;
        values.resize(size);

        numInputs = 0;
        numOutputs = 1;
        outputName.push_back("data");

    }    
      
    /**
     * This is the function that posts the resource data to the output port.
     * it is type specific, so it is defined here.  it is automatically called 
     * after the update() function in the main thread loop.
     **/
    void postData() {
        yarp::os::Bottle &b = outputPort[0]->prepare();
        b.clear();
        b.addString(resourceName.c_str());
        b.addDouble(getX());
        b.addDouble(getY());
        b.addDouble(getZ());      
        outputPort[0]->write();
    }
  
  };

}

#endif


