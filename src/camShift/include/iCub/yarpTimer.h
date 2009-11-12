#ifndef _YARPTIMER_
#define _YARPTIMER_

#include <yarp/os/Time.h>

class YarpTimer 
{
  private:
    double global_start, global_end, global_acc, lap_partial, lap_end, last_lap, lap_acc;
    int lap_counter;
  public:
    YarpTimer();
    ~YarpTimer();
    void reset();
    double now();
    void endlap();
    void start();
    void stop();

    // accessor methods
    int cyc();
    double lap();
    double lastlap();
    double tot();
    double avg();
};

#endif /* _YARPTIMER_ */
