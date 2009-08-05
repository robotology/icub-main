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
#pragma warning( disable : 4786)

#include <GL/glut.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;



#include "GLConsole.h"
#include "GLTools.h"

GLConsole::GLConsole(pGLSubWindow parentWindow)
: Console(),GLSubWindow(parentWindow){
  m_FontHeight    = 13;
  m_CursorStatus  = 1;
  m_CursorTimer.Start(500);
  SetMaxLines(20);

}

GLConsole::~GLConsole(){
}

int  GLConsole::AutoCompletion(){
  int nMatches = Console::AutoCompletion();
  if(nMatches>1){
    string s = "-->";    
    for(int i=0;i<nMatches;i++){
      s.append(" ");
      s.append(m_Commands[m_AutoMatches[i]]->m_Name);
    }
    Print(s);
  }
  return nMatches;
}



void GLConsole::Render(){

  Update();

	glColor4f(0.0,0.0,0.0,1.0);
	glBegin(GL_LINE_LOOP);
		glVertex2i(   0, (m_MaxLines)*m_FontHeight+8);
		glVertex2i( m_ClientRect.m_Width, (m_MaxLines)*m_FontHeight+8);
		glVertex2i( m_ClientRect.m_Width, (m_MaxLines+1)*m_FontHeight+12);
		glVertex2i(   0, (m_MaxLines+1)*m_FontHeight+12);
	glEnd();

	glDisable(GL_DEPTH_TEST);

  int y = m_FontHeight;
	int i;
	string s;

  y += m_FontHeight*(m_MaxLines-m_Lines.size());
	for(i=0;i<m_Lines.size();i++){
	  s = "  ";
    s.append(m_Lines[i]);
    GLTools::DisplayText(1,y, s.c_str());
	  y += m_FontHeight;
	}

  y = (m_MaxLines+1) * m_FontHeight+8;
  s = "> ";
  s.append(m_CurrCmd);

  if(m_HasFocus){
    if(m_CursorStatus==1){
      s.append("_");
    }
    if(m_CursorTimer.IsDone()){
      m_CursorStatus = 1-m_CursorStatus;
      m_CursorTimer.Start(500);
    }
  }

	GLTools::DisplayText(1,y, s.c_str());

	glEnable(GL_DEPTH_TEST);

}

void  GLConsole::OnNormalKey(char key){
  if(key==3) // Ctrl+C
    m_CurrCmd = "";
  
  if((key>=' ') && (key<'~')){
    AddChar(key);
  }
  if((key==8)||(key==127))
    EraseChar();

  if(key==13)
    Accept();
  
  if(key=='\t')
    AutoCompletion();
  
}
void  GLConsole::OnSpecialKey(int key){
  switch (key) {
  case GLUT_KEY_UP : 
    HistoryPrev(); break;
  case GLUT_KEY_DOWN : 
    HistoryNext(); break;
  }
}

void  GLConsole::Resize(int w, int h){
  int mlines = (h-12)/(m_FontHeight)-1;
  mlines = (mlines>1?mlines:1);
  SetMaxLines(mlines);
}
