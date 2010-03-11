// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
  * This is a simple helper class that enumertes the resource names to be used
  * in a control basis controller.  The resources include a sensor value,
  * a reference value (that is compared to the sensor value), a potential function
  * that evaluates a positive scalar "potential" value based on the sensor and
  * reference signals, and an effector that can be moved to reduce the potential
  * value.
  **/
#ifndef _CONTROLLER_PARAMETERS__H_
#define _CONTROLLER_PARAMETERS__H_

#include <iostream>

namespace CB {

  class ControllerParameters {
    
  public:

    /**
     * the sensor string
     **/
    std::string Sensor;
        
    /**
     * the reference string (leave as "" if there is no reference)
     **/
    std::string Reference;
        
    /**
     * the effector string
     **/
    std::string Effector;
    
    /**
     * the potential function string
     **/
    std::string PotentialFunction;
    
    /**
     * a (proportional) gain for the controller output
     **/
    double Gain;
    
    /**
     * Constructor
     **/
    ControllerParameters() {
      Sensor = "";
      Effector = "";
      Reference = "";
      PotentialFunction = "";
      Gain = 1.0;
    }
    
    /** 
     * Destructor
     **/
    ~ControllerParameters() { }
    
    /**
     * display function
     **/
    void display() {
      std::cout << std::endl << std::endl;
      std::cout << "Controller Info:" << std::endl << "------------------" << std::endl;
      std::cout << "Sensor: " << Sensor.c_str() << std::endl;
      if(Reference != "")  std::cout << "Reference: " << Reference.c_str() << std::endl;
      std::cout << "Potential Function: " << PotentialFunction.c_str() << std::endl;
      std::cout << "Effector: " << Effector.c_str() << std::endl;
      std::cout << "Gain: " << Gain << std::endl << std::endl;
    }
    
  };
  
}

#endif
