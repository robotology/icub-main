// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "mcEventDownsampler.h"

namespace mced {
    
    
    mcEventDownsampler::mcEventDownsampler() 
    {
        // mutex ctor   
    };
    
    mcEventDownsampler::~mcEventDownsampler()
    {
        
        stop();
        
        if(nullptr != timer)
        {
            delete timer;
            timer = nullptr;
        }
        // idem per le altre cose    
        
        // mutex dtor       
    }
    
    bool mcEventDownsampler::canprint()
    {
        // mutex->lock(?);
        // activate something.
        counter++;
        // boh
        
        bool cp = !isdownsampling;
        // mutex->unlock();
        
        return cp;
    }
    
    bool mcEventDownsampler::stop()
    {
        // controlli su valore di timer non null, se e'attivo etc....
        
        timer->stop();
        return true;
    }
    
    bool mcEventDownsampler::start(const Config &config)
    {
        if(nullptr != timer)
        {
            stop(); 
            // delete timer
        }
        
        yarp::os::TimerSettings ts(0.1);
        timer = new yarp::os::Timer(ts, &mcEventDownsampler::step, this, true);


        timer->start();
        // decidere dove costruire mutex. di sicuro deve essere costruito prima di timer
        // meglio nel ctor
        // construct timer etc
        
        // assegnare step() come callback al timer ....
        
        return true;
    }
    
    bool mcEventDownsampler::step(const yarp::os::YarpTimerEvent &event)
    {
        // qui dentro, deve esserci la logica della fsm e ci deve essere accesso ai dati privati della class Ticker
        // mutex->lock(?);        
        //fsmstep(fsm);
        

        if (counter < 5)
            isdownsampling = false;
        
        if (counter >= 5 && !isdownsampling)
            isdownsampling = true;

        if (isdownsampling)
        {
            if (print_countdown > 0)
            {
                print_countdown--;
            }
            else
            {
                printreport();
                print_countdown = 10;
                counter = 0;
            }
        }

        // genera ....
        
        // mutex->unlock();
        return true;
    }

    void mcEventDownsampler::printreport()
    {
        yError() <<  "Detected " << counter << " events on aggregate since the last message";
    }
    
    void mcEventDownsampler::fsmstep(void *fsm) {}
        
} // xxx


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------
