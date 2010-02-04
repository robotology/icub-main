// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CBAPI_HELPER__H_
#define _CBAPI_HELPER__H_

#include <RunnableControlLaw.h>

namespace CB {

    class CBAPIHelper {

    protected:
        
        RunnableControlLaw controlLaw;
        
        int numControllers;

    public:
        
        CBAPIHelper();
        ~CBAPIHelper() { }
      
        void addControllerToLaw(std::string sen, std::string ref, std::string pf, std::string eff, bool useTranspose, double gain);
        void clearControlLaw();
        void stopControlLaw();
        void runControlLaw();
        
        int getNumControllers() { return numControllers; }
        
        double getPotential(int n);
        double getPotentialDot(int n);
        int getState(int n);
        
        void useTranspose(bool b);
      
  };

}

#endif
