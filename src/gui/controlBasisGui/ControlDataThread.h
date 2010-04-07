// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTROL_DATA_THREAD__H_
#define _CONTROL_DATA_THREAD__H_

#include "CBAPIHelper.h"
#include "CBAPIUtil.h"

#include <yarp/os/RateThread.h>
#include <string>

namespace CB {

    class ControlDataThread : yarp::os::RateThread {
      
    protected:
    
        CBAPIHelper *cbapi;
        CBAPITextWindow *cbapiText;
        int numControllers;
        
        std::string outputString;

        std::vector<double> potentials;
        std::vector<double> potentialDots;
        std::vector<int> states;

        int iteration;

        bool runningSequence;

    private:
        Glib::Dispatcher my_dispatcher_;
        sigc::signal<void> control_thread_update_finished_;
        
    public:
        
        explicit ControlDataThread() : yarp::os::RateThread(100) { 
            iteration=0;
            my_dispatcher_.connect(sigc::mem_fun(*this, &ControlDataThread::updateOutputString));    
        }

        virtual ~ControlDataThread() { } 
        
        void connectCBAPIObjects(CBAPIHelper *c, CBAPITextWindow *txt, bool sequence) {
            cbapi = c;
            cbapiText = txt;
            runningSequence = sequence;

            numControllers = cbapi->getNumControllers(runningSequence);

            for(int i=0; i<numControllers; i++) {
                potentials.push_back(0);
                potentialDots.push_back(0);
                states.push_back(0);
            }
            std::cout << "ControlDataThread::connectCBAPIObjects() finished" << std::endl;
        }
        
        sigc::signal<void>& control_thread_update_finished() {
            return control_thread_update_finished_;
        }
        
        void run() {
            my_dispatcher_();            
        }   
        
        void updateOutputString() {
            
            char c[64];
            std::string str;
            char *state_char;
            state_char = (char *)malloc(2);
            outputString = "";

            int printLimit; 
            if(runningSequence) {
                printLimit = cbapi->getNumberOfControllersInSequenceStep();
            } else {
                printLimit = numControllers;
            }
          
            for(int i=0; i<printLimit; i++) {
                potentials[i] = cbapi->getPotential(i, runningSequence);
                potentialDots[i] = cbapi->getPotentialDot(i, runningSequence);
                states[i] = cbapi->getState(i, runningSequence);
                getStateChar(states[i], (char *)state_char);
                if(!runningSequence) {
                    sprintf(c,"c[%d], phi: %.7f, phi_dot: %.7f, state=%s\n",
                            i, potentials[i], potentialDots[i], state_char);
                } else {
                    sprintf(c,"c[%d][%d], phi: %.7f, phi_dot: %.7f, state=%s\n",
                            cbapi->getSequenceControllerID(), i, potentials[i], potentialDots[i], state_char);
                }
                str = std::string(c);	
                for(int k=0; k<i;k++) str = "\t" + str;
                outputString += str;
            }
            
            control_thread_update_finished_();      
            
        }
        
        void startUpdateThread() { 
            if(numControllers != cbapi->getNumControllers(runningSequence)) {
                // if new controllers have been added since the last time
                // the thread ran, allocated storage for them
                numControllers = cbapi->getNumControllers(runningSequence);
                for(int i=0; i<numControllers; i++) {
                    potentials.push_back(0);
                    potentialDots.push_back(0);
                    states.push_back(0);
                }
            }
            start(); 
        }
        
        void stopUpdateThread() { 
            stop(); 
        }
        
        std::string getOutputString() {
            return outputString;
        }
        
        char *getStateChar(int s, char *c) {
            
            switch(s) {
            case 0:
                sprintf(c, "X");
                break;
            case 1:
                sprintf(c, "-");
                break;
            case 2:
                sprintf(c, "0");
                break;
            case 3:
                sprintf(c, "1");
                break;
            default:
                sprintf(c, "X");
                break;
            }
            return c;
        }
    };
    
}

#endif
