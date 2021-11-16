// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "mcEventDownsampler.h"

namespace mced {
    

    YARP_LOG_COMPONENT(MC_EVENT_DOWNSAMPLER, "mc.msgdownsampler.skipped-cmd-wrong-mode")


    mcEventDownsampler::mcEventDownsampler() 
    {
        mutex = new std::mutex();
    };
    
    mcEventDownsampler::~mcEventDownsampler()
    {
        
        stop();
        
        if(nullptr != timer)
        {
            delete timer;
            timer = nullptr;
        }

        delete mutex;
    }
    
    bool mcEventDownsampler::canprint()
    {
        mutex->lock();
        
        counter++;
        size_t diff = counter - latch_1;
        bool cp = !isdownsampling;

        mutex->unlock();

        if(diff > 5)
        {
            return false;
        } 
        else
        {
            return cp;
        }
    }
    
    bool mcEventDownsampler::stop()
    {

        if(nullptr != timer)
        {
            timer->stop();
            return true;
        }

        return false;
    }
    
    bool mcEventDownsampler::start(const Config &config)
    {
        if(nullptr != timer)
        {
            stop(); 
        }

        expire_time = yarp::os::Time::now();        
        yarp::os::TimerSettings ts(config.period);
        timer = new yarp::os::Timer(ts, &mcEventDownsampler::step, this, true, mutex);

        if (timer != nullptr) 
        {
            timer->start();
            return true;
        }
        else
        {
            return false;
        }
    }
    
    bool mcEventDownsampler::step(const yarp::os::YarpTimerEvent &event)
    {
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
        return true;
    }

    void mcEventDownsampler::printreport()
    {
        yCError(MC_EVENT_DOWNSAMPLER) <<  "Detected " << counter - latch_2 << " events on aggregate since the last message";
    }
            
} // mced
