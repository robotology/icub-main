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
    return true;
  return false;
}

long Timer::GetTimeMs(){
#ifdef WIN32
  return GetTickCount();
#else
  struct timeval m_TV;
  gettimeofday(&m_TV,NULL);
  return (m_TV.tv_sec  * 1000) + (m_TV.tv_usec / 1000);
#endif  
}


Chrono::Chrono(){
  Start();
}

void Chrono::Start(){
#ifdef WIN32
  m_StartTime = GetTickCount();
#else
  gettimeofday(&m_StartTime,NULL);
  //m_StartTime  = m_TV.tv_sec  * 1000;
  //m_StartTime += m_TV.tv_usec / 1000;
#endif
}

long Chrono::ElapsedTimeMs(){
  long currTime;
#ifdef WIN32
  currTime = GetTickCount();  
  return (currTime-m_StartTime);
#else
  struct timeval mCTV;
  struct timeval mDTV;
  gettimeofday(&mCTV,NULL);
  timersub(&mCTV,&m_StartTime,&mDTV);
  currTime      = long(mDTV.tv_sec*1000) ;
  currTime     += long(mDTV.tv_usec/1000);
  return (currTime);
#endif
}

long Chrono::ElapsedTimeUs(){
  long currTime;
#ifdef WIN32
  currTime = GetTickCount()*1000;  
  return (currTime-m_StartTime)*1000;
#else
  struct timeval mCTV;
  struct timeval mDTV;
  gettimeofday(&mCTV,NULL);
  timersub(&mCTV,&m_StartTime,&mDTV);
  
  currTime      = long(mDTV.tv_sec *1000000);
  currTime     += long(mDTV.tv_usec);
  return (currTime);
#endif
}
