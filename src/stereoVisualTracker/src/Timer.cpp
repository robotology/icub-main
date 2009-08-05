#include "Timer.h"

Timer::Timer(){
  m_StartTime = 0;
  m_TimeToGo  = 0;
}

void Timer::Start(long TimeToGoMs){
#ifdef WIN32
  m_StartTime = GetTickCount();
#else
  gettimeofday(&m_TV,NULL);
  m_StartTime  = m_TV.tv_sec  * 1000;
  m_StartTime += m_TV.tv_usec / 1000;
#endif
  m_TimeToGo = TimeToGoMs;
}

int Timer::IsDone(){
  long currTime;
#ifdef WIN32
  currTime = GetTickCount();  
#else
  struct timeval mCTV;
  gettimeofday(&mCTV,NULL);
  currTime      = mCTV.tv_sec *1000;
  currTime     += mCTV.tv_usec/1000;
#endif
  if((currTime-m_StartTime)>m_TimeToGo)
    return TRUE;
  return FALSE;
}
