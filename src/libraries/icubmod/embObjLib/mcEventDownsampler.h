
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef __TICKER_H_
#define __TICKER_H_

//#include <stdio.h>

#include <string>
#include <mutex>

#include <stddef.h>
#include <stdint.h>
#include <yarp/os/Timer.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>

namespace mced {
    
    class mcEventDownsampler
    {
    public:

        struct Config
        {
            double period {0};    // time between two ticks
            Config() = default;
            constexpr Config(double c) : period(c) {} 
            bool isvalid() const { return 0 != period; }
        };
      
        mcEventDownsampler();
        ~mcEventDownsampler();            
        bool start(const Config &config);              
        bool stop(); 
        bool canprint();
        Config config = 0.1;
        
    private: 
        bool step(const yarp::os::YarpTimerEvent &event); 
        void printreport();
        yarp::os::Timer* timer {nullptr};    
        double expire_time;
        bool isdownsampling {false};
        size_t counter {0};  // size_t should be enough for the next 5 billion years
        size_t latch_1{0};
        size_t latch_2{0};
        std::mutex *mutex {nullptr};
    };
    
} // mced

#endif  // include-guard
