#ifndef webots_common_h
#define webots_common_h

#include <yarp/os/Semaphore.h>

///corresponds to the number of device drivers
#define SEMAPHORE_NUM 6


class WebotsCommon
{
 private:
  WebotsCommon();
  ~WebotsCommon();
  
  static WebotsCommon *singleton;
  
  yarp::os::Semaphore sem_webots[SEMAPHORE_NUM],sem_control[SEMAPHORE_NUM];
  
 public:
  static WebotsCommon *getInstance();
  
  void sem_control_post();
  void sem_webots_wait();
  void sem_control_wait(int sem_num);
  void sem_webots_post(int sem_num);
};


#endif
