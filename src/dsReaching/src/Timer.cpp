// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Eric Sauser,Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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
#include "iCub/Timer.h"

Timer::Timer(){
  m_StartTime = 0;
  m_TimeToGo  = 0;
}

Timer::Timer(long TimeToGoMs){
  m_StartTime = 0;
  m_TimeToGo  = TimeToGoMs;
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



History::History(){
  m_History   = NULL;
  m_DataSize  = 0;
  m_Size      = 0;
  m_Current   = 0;
  m_CurrentV  = 0;
}

History::~History(){
  if(m_History!=NULL) delete [] m_History;
  m_History = NULL;
}

void  History::Init(int dataSize, int size){
  if((dataSize>0)&&(size>0)){
    if(m_History!=NULL) delete [] m_History;    
    m_DataSize  = dataSize;
    m_Size      = size;
    m_History   = new float[dataSize*size];
    m_Current   = 0;
    m_CurrentV  = 0;
 }
  Clear();
}

void  History::Clear(){
  if(m_History!=NULL){
    m_Current = 0;
    memset(m_History,0,m_DataSize*m_Size*sizeof(float));
  }
}
void  History::AddData(float *data){
  if(m_History!=NULL){
    m_Current ++;   
    if(m_Current>=m_Size)
      m_Current = 0;

    memcpy(m_History+m_Current*m_DataSize,data,m_DataSize*sizeof(float));
  }
}

void  History::AddVertData(float *data){
  if(m_History!=NULL){
    m_CurrentV ++;   
    if(m_CurrentV>=m_DataSize)
      m_CurrentV = 0;


    int i;
    for(i=0;i<m_Size;i++){
      *(m_History + m_CurrentV + i*m_DataSize) = data[i];
    }
  }
}
