// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTROLLER__H_
#define _CONTROLLER__H_

#include "ControlBasisAction.h"
#include "ControlBasisResource.h"
#include "ControlBasisJacobian.h"
#include "ControlBasisPotentialFunction.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Port.h>
#include <vector>

#include <JacobianFactory.h>
#include <PotentialFunctionFactory.h>

namespace CB {
    
    /**
     * This class instantiates the abstract ControlBasisAction class 
     * for a primitive Controller.  It implements the control law
     * \delta \tau = J^# \phi(\sigma) \kappa, where \tau is the effector
     * resource (e.g., motor variables), \sigma is the input sensor 
     * resource information, and \phi is the control (potential) function.
     * The task Jacobian J captures sensitive of the potential evaluated
     * at the sensory values w.r.t., the output variables. that is,
     * J = \frac{ \partial \phi(\sigma) }{ \partial \tau}.  Also supports 
     * using J^T in place of J^#.
     *
     * A controller can be run in either "local" ir "remote" mode. In local mode,
     * it connects directly to ControlBasisResource classes specified in the
     * constructor.  If it runs in remote mode, it connects to these resources
     * over YARP ports, and the names of these ports must be specified in the
     * constructor.
     **/
    class Controller : public ControlBasisAction {
        
    protected:
    
        /**
         * inputSpace
         **/
        std::string inputSpace;
        
        /**
         * outputSpace
         **/
        std::string outputSpace;
        
        /**
         * potentialFunction
         **/
        std::string pfType;
    
        /**
         * control output
         **/
        yarp::sig::Vector Vout;

        /**
         * Potential
         **/
        double potential;

        /**
         * potential (last)
         **/
        double potentialLast;
    
        /**
         * Potential Dot
         **/
        double potentialDot;
        
        /**
         * whether an internal jacobian is necessary
         **/
        bool needsJacobian;

        /**
         * whether the inverse of the internal jacobian is needed
         **/
        bool needsJacobianInverse;
        
        /**
         * whether there is a reference sensory signal
         **/
        bool hasReference;

        /**
         * Controller (Task) Jacobian
         **/
        yarp::sig::Matrix Jc;
        
        /**
         * Controller Jacobian Inverse
         **/
        yarp::sig::Matrix JcInv;

        /**
         * a pointer to the sensor resource
         **/
        ControlBasisResource *sensor;

        /**
         * a pointer to the reference sensor resource (if necessary)
         **/
        ControlBasisResource *reference;

        /**
         * a pointer to the effector resource
         **/
        ControlBasisResource *effector;

        /**
         * a pointer to the potential function
         **/
        ControlBasisPotentialFunction *potentialFunction;

        /**
         * a pointer to the Jacobian
         **/
        ControlBasisJacobian *jacobian;

        /**
         * control gain
         **/
        double gain;    

        /**
         *  the name of the sensor
         **/
        std::string sensorName;
        
        /**
         *  the name of the sensor reference
         **/
        std::string referenceName;

        /**
         *  the name of the effector 
         **/
        std::string effectorName;

        /**
         * the name of the potential function
         **/
        std::string pfName;

        /**
         * flag for distributed sensor mode
         */
        bool distributedMode;

        /**
         * the name of the device the controller will control
         **/
        std::string deviceName;

        /**
         * storage for the potential as the controller runs
         */
        std::vector<double> potentialStore;

        /**
         * storage for the estimated potential derivative as the controller runs
         */
        std::vector<double> potentialDotStore;       

        /**
         * using transpose of Jacobian, rather than pseudoinverse, s.t. \delta \tau = -J^T \phi(\sigma)
         **/
        bool useJacobianTranspose;        

        /**
         * using a derivative term in the control law (in addition to a proportional term)
         **/
        bool useDerivativeTerm;

        /**
         * stores the bias towards convergence based on history
         **/
        double convergenceStore;

        /**
         * stores the previous convergence state
         **/
        double convergenceStoreLast;

        /**
         * stores the start time of the controller
         **/
        double t0;

        /**
         * stores the time of the last iteration of the controller
         **/
        double t_last;

        /**
         * stores an estimate of dt
         **/
        double dt;

    public:
        
        /** 
         * constructor for local set up (with reference)
         **/
        Controller(ControlBasisResource *sen,
                   ControlBasisResource *ref,
                   ControlBasisPotentialFunction *pf,
                   ControlBasisResource *eff);

        /** 
         * constructor for local set up
         **/
        Controller(ControlBasisResource *sen,
                   ControlBasisPotentialFunction *pf,
                   ControlBasisResource *eff);

        /**
         * constructor for remote set up (with reference sensoro)
         **/
        Controller(std::string sen, std::string ref, std::string pf, std::string eff);

        /**
         * constructor for remote set up
         **/
        Controller(std::string sen, std::string pf, std::string eff);
    
        /** 
         * destructor
         **/
        ~Controller() {
            std::cout << "Controller destructor..." << std::endl;
            resetController();
        }       
        
        /**
         * Inherited update fuction from ControlBasisAction class
         **/
        virtual bool updateAction();

        /**
         * Inherited start fuction from ControlBasisAction class
         **/
        virtual void startAction();

        /**
         * Inherited stop fuction from ControlBasisAction class
         **/
        virtual void stopAction();

        /**
         * Inherited postData fuction from ControlBasisAction class
         **/
        virtual void postData();  

        /**
         * reset funtion
         **/
        void resetController();  

        /**
         * gets the device name of the effector this controller moves
         * \returns deviceName
         **/
        std::string getOutputDeviceName() { 
            return deviceName; 
        }

        /**
         * gets the space of the effector 
         * \returns outputSpace
         **/
        std::string getOutputSpace() { 
            return outputSpace;
        }

        /**
         * gets the size of the effector signal 
         * \returns outputSize
         **/
        int getOutputSize() { 
            return Vout.size();
        }

        /**
         * gets the controller output \delta \tau = J^# \phi(\sigma)*\kappa         
         * \returns Vout the delta output
         **/
        yarp::sig::Vector getControlOutput() { return Vout; }

        /**
         * gets the controller's Task jacobian 
         * J = \frac{\partial \phi(\sigma)}{\partial \tau}         
         * \returns Jc the task jacobian
         **/
        yarp::sig::Matrix getTaskJacobian() { return Jc; }

        /**
         * determines if controller will be running by connecting
         * to resources over the YARP ports (distributed mode),
         * or not (local mode), where pointers to resources are 
         * maintained.
         * \return distributedMode in distributed mode
         **/
        bool inDistributedMode() { return distributedMode; }

        /**
         * sets the controller gain         
         * \param kappa the gain value
         **/
        void setGain(double kappa) { gain = kappa; }

        /**
         * returns current controller potential
         */
        double getControllerPotential() {
            return potential;
        }
        /**
         * returns current controller potential
         */
        double getControllerPotentialDot() {
            return potentialDot;
        }

        /** 
         * returns whether using Jacobian transpose mode in control
         **/
        bool usingJacobianTranspose() { 
            return useJacobianTranspose; 
        }

        /**
         * sets whether using jacobian transpose or not
         **/
        void useTranspose(bool b) {
            useJacobianTranspose = b;
        }

        /**
         * sets whether this controller is a PD or a P Controller 
         **/
        void usePDControl(bool b) {
            useDerivativeTerm = b;
        }

    protected:

        /**
         * creates a potential function of the specified type
         **/ 
        bool createPotentialFunction(std::string pf);

        /**
         * creates the necessary Jacobian to transform potential changes to effector changes.
         **/
        bool createJacobian();

        /**
         * determines the output device and space of effector resource
         **/
        void parseOutputResource();

  };
 

}

#endif
