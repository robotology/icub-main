// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "mcEventDownsampler.h"

namespace mced {
    
    
    mcEventDownsampler::mcEventDownsampler() 
    {
        mutex = new std::mutex();
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
        bool cp = true;
        mutex->lock();
        // activate something.
        counter++;

        if(counter - latch_1 > 5)
        {
            cp = false;
        } else {
            cp = !isdownsampling;
        }

        mutex->unlock();
        
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
        expire_time = yarp::os::Time::now();        
        yarp::os::TimerSettings ts(0.01);
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
        mutex->lock();        
        //fsmstep(fsm);

        if (yarp::os::Time::now() - expire_time >= 1)
        {
            expire_time = yarp::os::Time::now();

            isdownsampling = (counter - latch_1 > 5);

            if (isdownsampling)
            {
                printreport();
                latch_2 = counter;
            }
            latch_1 = counter;
        }

        // genera ....
        
        mutex->unlock();
        return true;
    }

    void mcEventDownsampler::printreport()
    {
        yError() <<  "Detected " << (counter - latch_2) << " events on aggregate since the last message";
    }
    
    void mcEventDownsampler::fsmstep(void *fsm) {}
        
} // xxx


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------
