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



/**
 * \class Timer
 * 
 * \brief A basic timer working on a milisecond time base
 */

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

  /// Start the timer with a given target time
  void  Start (long TimeToGoMs);
  /// Return true is the target time has been reached
  int   IsDone();
  
  static long GetTimeMs();
};

/**
 * \class Chrono
 * 
 * \brief A basic chrono working on a milisecond time base
 */
class Chrono
{
protected:

#ifndef WIN32
  struct timeval m_TV;
  struct timeval m_StartTime;
//  struct timeval m_TV;
#else
  long           m_StartTime;
#endif

public:
  Chrono();

  /// Start the chrono
  void  Start();
  /// Return the elapsed time in milliseconds
  long  ElapsedTimeMs();
  /// Useless
  long  ElapsedTimeUs();
};


#endif
