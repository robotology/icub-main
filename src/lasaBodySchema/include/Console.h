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
#ifndef __CONSOLE_H__
#define __CONSOLE_H__

//#pragma warning( disable : 4786)

#include <vector>
#include <string>
#include <sstream>

using namespace std;

typedef vector<string> String_List;

class Command {
public:
  string  m_Name;
public:
                Command(string name);
  virtual       ~Command();
  virtual int   Execute(string args);
};
typedef Command *pCommand;
typedef vector<pCommand> Command_List;


class Console
{
private:
  static streambuf *    m_Stdout;


protected:
  Command_List          m_Commands;

  String_List           m_Lines;
  int                   m_MaxLines;
  String_List           m_History;
  int                   m_MaxHistory;
  int                   m_CurrHistory;

  string                m_CurrCmd;

  vector<int>           m_AutoMatches;


  int                   m_CurrX;
  int                   m_CurrY;

  ostringstream         m_oStream;

public:
            Console();
  virtual  ~Console();

  void      Free();
  void      Clear();
  void      SetMaxLines(int size);
  void      SetMaxHistory(int size);

  void      AddLine(string line);
  void      AddHistory(string line);

  void      AddCommand(pCommand cmd);
  pCommand  FindCommand(string name);

  void      HistoryPrev();
  void      HistoryNext();

  void      AddChar(char c);
  void      EraseChar();
  void      Accept();
  virtual   int       AutoCompletion();

  void      Print(string line);
  void      Update();

  streambuf *GetStreamBuf();
  void      SetStdout();

};


#endif
