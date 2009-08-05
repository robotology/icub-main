#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef WIN32
  #include <windows.h>
#else
  #include <errno.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <fcntl.h>
  #include <unistd.h>
#endif



class Timer
{
protected:

#ifndef WIN32
  struct timeval m_TV;
#endif

  long           m_TimeToGo;
  long           m_StartTime;

public:
  Timer();

  void  Start (long TimeToGoMs);
  int   IsDone();
};

#endif
