// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
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
  #include <string.h>
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


/**
 * Simple timer class, works in milliseconds
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
  Timer(long TimeToGoMs);

  /**
   * starts the timer 
   */
  void Start(){Start(m_TimeToGo);}
  void  Start (long TimeToGoMs);
  void SetDelay(long delay){m_TimeToGo = delay;}
  /**
   * @briefchecks whether m_TimeToGo ms have elapsed since the last call
   * of Start();
   * @return 1 if the time has elapsed, 0 otherwise 
   */ 
  int   IsDone();
};






class History
{
public:
  float   *m_History;
  int     m_DataSize;
  int     m_Size;
  int     m_Current;
  int     m_CurrentV;

public:
  History();
  ~History();

  void  Init(int dataSize, int size);
  void  Clear();
  void  AddData(float *data);
  void  AddVertData(float *data);
};

#endif
