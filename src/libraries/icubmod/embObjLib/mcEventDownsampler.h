
#ifndef __EVENT_DOWNSAMPLER_H_
#define __EVENT_DOWNSAMPLER_H_

#include <string>
#include <mutex>

#include <stddef.h>
#include <stdint.h>
#include <yarp/os/Timer.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>

namespace mced {

    class mcEventDownsampler
    {
    public:

        /**
         * @brief Contains the configurable parameters of the event downsampler
         * @var Config::period the period of the timer
         */
        struct Config
        {
            double period {0.};
            Config() = default;
            constexpr Config(double c) : period(c) {} 
            bool isvalid() const { return 0 != period; }
        };
      
        /**
         * @brief Construct a new Event Downsampler object and instantiates the mutex
         *        needed by the timer
         */
        mcEventDownsampler();

        /**
         * @brief Destroy the Event Downsampler object. It stops the timer then deletes it.
         * 
         */
        ~mcEventDownsampler();      

        /**
         * @brief Instantiates the yarp::os::Timer object and starts it.
         * @param config Structure containing the configuration parameters of the Event Downsampler
         * @return true if the instantiation was successful, false otherwise.
         */
        bool start(const Config &config);   

        /**
         * @brief Stops the timer
         * @return true if the operation was successful, false otherwise.
         */
        bool stop();

        /**
         * @brief Called by the object that implements the downsampler. Compares the difference between
         *        counter and latch_1 is and the threshold.
         * @return true if the difference is lower or equal to the threshold, false if it is higher. 
         */
        bool canprint();
        Config config = 0.1;
        
    private: 
        /**
         * @brief Callback function called by the timer at each tick. Implements the downsampling logic:
         *        the downsampling is activated if the number of events exceed a threshold, and a report
         *        showing the number of events since the last report is printed.
         *        The mutex is locked before entering the function, and unlocked right after returning.
         *        
         * @param event Structure containing data regarding the callback.
         * @return true 
         */
        bool step(const yarp::os::YarpTimerEvent &event); 

        /**
         * @brief Prints an error message in the logger with a cumulative number of events occurred since the
         *        last call.
         */
        void printreport();

        /** @brief Pointer to the timer object. */
        yarp::os::Timer* timer {nullptr};

        /**  @brief Number of seconds since epoch. Used to check when one second has elapsed. */
        double expire_time;
        
        /**  @brief Indicates when the downsampling is active. It is set to true when the number of events
         * exceeds a threshold
         */
        bool isdownsampling {false};

        /**  @brief Events counter. size_t should be enough for the next 5 billion years. */
        size_t counter {0};

        /**  @brief Support variable used to evaluate the difference with the counter. */
        size_t latch_1{0};

        /**  @brief Support variable used to evaluate the difference with the counter. */
        size_t latch_2{0};

        /**  @brief Mutex needed by the timer. */
        std::mutex *mutex {nullptr};
    };
    
} // mced

#endif  // __EVENT_DOWNSAMPLER_H_
