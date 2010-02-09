// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CBAPI_HELPER__H_
#define _CBAPI_HELPER__H_

#include <RunnableControlLaw.h>

namespace CB {

    /**
     * This class is a helper class for the CBAPI that stores the current control law.
     * You can start, stop, add, delete, and run controllers (adding them to a law in 
     * order of decreasing priority.  Can get their state and dyanmics too.
     **/ 
    class CBAPIHelper {

    protected:
        
        /**
         * The runnable control law class to add controllers to.
         **/
        RunnableControlLaw controlLaw;
        
        /**
         * The number of controllers in the law currently.
         **/
        int numControllers;

    public:
        
        /**
         * Constructor
         **/

        CBAPIHelper();

        /**
         * Destuctor
         **/
        ~CBAPIHelper() { }
      
        /**
         * Adds a controller to the law with the resources specified.
         * \param sen the sensor name
         * \param ref the reference (target) name, if there is one
         * \param pf the name of the potential function
         * \param eff the effector name
         * \param useTranspose  if true, uses J^T in the contrl law, if false, uses J^# (moore-penrose pseudoinverse)
         * \param gain the gain for the controller
         **/
        void addControllerToLaw(std::string sen, std::string ref, std::string pf, std::string eff, bool useTranspose, double gain);

        /**
         * clears the control law
         **/
        void clearControlLaw();

        /**
         * stops the control law
         **/
        void stopControlLaw();

        /**
         * runs the control law
         **/
        void runControlLaw();
        
        /**
         * gets the number of controllers in the law
         **/
        int getNumControllers() { return numControllers; }
        
        /**
         * gets the potential of controller n
         **/
        double getPotential(int n);

        /**
         * gets the estimated change in potential of controller dot
         **/
        double getPotentialDot(int n);

        /**
         * gets the state of controller n
         **/
        int getState(int n);
        
        /**
         * sets whether the controller is using the transpose of the Jacobian or the Moore-Penros pseudoinverse
         **/
        void useTranspose(bool b);
      
  };

}

#endif
