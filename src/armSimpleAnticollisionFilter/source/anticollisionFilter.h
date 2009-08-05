//Alexis Maldonado. 2007. GPLv3

#ifndef anticollisionFilter_h_
#define anticollisionFilter_h_

#include <robot.h>
#include <utils.h>


bool inCylinder(const ColumnVector &Pos);
bool behindBack(const ColumnVector &Pos);
bool wrongSide(const ColumnVector &Pos);
bool positionOk(const ColumnVector &Pos);
bool positionOk_withBack(const ColumnVector &Pos);
bool positionInBox(const ColumnVector &Pos);



extern bool go_on;

//To handle the signals
#include <ace/Signal.h>
#include <ace/Reactor.h>
#include <ace/Select_Reactor.h>


// Signal handler
class Sig_Handler : public ACE_Event_Handler {
public:
    Sig_Handler (ACE_Reactor &reactor) : reactor_ (reactor) {
        // Register the signal handlers.
        this->quit_sigkey_ = reactor.register_handler (SIGQUIT, this);
        this->int_sigkey_  = reactor.register_handler (SIGINT, this);
    }

    // Handle incoming signals
    virtual int handle_signal (int signum, siginfo_t *, ucontext_t *) {
        switch (signum) {
        case SIGQUIT: {
            printf ("SIGQUIT detected... exiting...\n");
            this->reactor_.remove_handler (SIGQUIT, 0, 0, this->quit_sigkey_);
            go_on = false;
            break;
        }
        case SIGINT: {
            printf ("SIGINT detected... exiting...\n");
            this->reactor_.remove_handler (SIGINT, 0, 0, this->int_sigkey_);
            go_on = false;
            break;
        }
        default:
            break;
        }
        return 0;
    }

protected:
    int int_sigkey_;
    int quit_sigkey_;
    ACE_Reactor &reactor_;
};




#endif // anticollisionFilter_h_ //~
