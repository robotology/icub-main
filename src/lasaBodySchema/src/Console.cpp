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
#include "Console.h"

#include <iostream>
using namespace std;

Command::Command(string name){
  m_Name = name;
}
Command::~Command(){
}
int Command::Execute(string args){
  return 0;
}

streambuf * Console::m_Stdout = NULL;

Console::Console(){
  if(m_Stdout == NULL)
    m_Stdout = cout.rdbuf();

  Free();
}
Console::~Console(){
  Free();

  if(cout.rdbuf() == GetStreamBuf())
    cout.rdbuf(m_Stdout);
}

void Console::Clear(){
  m_Lines.clear();
  m_History.clear();
  m_CurrY       = 0;
  m_CurrX       = 0;
  m_CurrCmd     = "";
  m_CurrHistory = 0;
  m_oStream.str("");
}

void Console::Free(){
  m_Commands.clear();
  m_MaxLines    = 20;
  m_MaxHistory  = 20;
  Clear();
}

void Console::SetMaxLines(int size){
  if(size<=0)
    return;

  if(size>m_MaxLines)
    m_MaxLines = size;


  while(size<m_MaxLines){
    if(m_Lines.size()>size)
      m_Lines.erase(m_Lines.begin());
    m_MaxLines--;
  }
}

void Console::SetMaxHistory(int size){
  if(size<=0)
    return;
  while(size<m_MaxHistory){
    m_History.erase(m_History.begin());
    m_CurrHistory--;
  }
}

void Console::AddLine(string line){
  if(m_Lines.size() >= m_MaxLines){
    m_Lines.erase(m_Lines.begin());
    m_Lines.push_back(line);
  }else{
    m_Lines.push_back(line);
  }
}

void Console::AddHistory(string line){
  if(m_History.size() >= m_MaxHistory){
    m_History.erase(m_History.begin());
    m_History.push_back(line);
  }else{
    m_History.push_back(line);
  }
  m_CurrHistory = m_History.size()-1;
}

void Console::AddChar(char c){
  string s(1,c);
  m_CurrCmd.append(s);
}

void Console::EraseChar(){
  if(m_CurrCmd.size()>0)
    m_CurrCmd = m_CurrCmd.substr(0,m_CurrCmd.size()-1);
}

void Console::Accept(){
  if(m_CurrCmd.size()>0)
    AddHistory(m_CurrCmd);  

  int cmdEnd = m_CurrCmd.find_first_of(" ");
  string cmd;
  string args;

  if(cmdEnd == string::npos){
    cmd   = m_CurrCmd;
    args  = "";
  }
  else{
    cmd   = m_CurrCmd.substr(0,cmdEnd);
    args  = m_CurrCmd.substr(cmdEnd+1);
  }

  pCommand pCmd = FindCommand(cmd);
  if(pCmd!=NULL){
    string s = "> ";
    s.append(m_CurrCmd);
    Print(s);
    pCmd->Execute(args);    
  }else{
    if(cmd.size()==0){
      Print(cmd);
    }else{
      string s = "\"";
      s.append(cmd);
      s = s.append("\": Command not found");
      Print(s);
    }
  }

  m_CurrCmd = "";
}

pCommand  Console::FindCommand(string name){
  int i;
  for(i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.compare(name)==0)
      return m_Commands[i];
  }
  return NULL;
}

int Console::AutoCompletion(){
  int i;
  int len = m_CurrCmd.size();

  m_AutoMatches.clear();

  for(i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.substr(0,len).compare(m_CurrCmd)==0)
      m_AutoMatches.push_back(i);
  }

  if(m_AutoMatches.size()==1){
    m_CurrCmd = m_Commands[m_AutoMatches[0]]->m_Name;
    m_CurrCmd.append(" ");
  }

  return m_AutoMatches.size();
}

void Console::AddCommand(pCommand cmd){
  if(cmd==NULL)
    return;
  m_Commands.push_back(cmd);
}

void Console::HistoryPrev(){
  if(m_History.size()>0){
    m_CurrCmd = m_History[m_CurrHistory];
    m_CurrHistory--;
    if(m_CurrHistory<0)
      m_CurrHistory=0;
  }
}
void Console::HistoryNext(){
  if(m_History.size()>0){
    m_CurrHistory++;
    if(m_CurrHistory<m_History.size()){
      m_CurrCmd = m_History[m_CurrHistory];
    }else{
      m_CurrCmd = "";
      m_CurrHistory--;
    }
  }
}

void Console::Print(string line){
  AddLine(line);  
}

void Console::Update(){
  string s = m_oStream.str();
  
  int cpos = 0;
  int opos = 0;

  if(s.size()>0){
    //while(cpos!=string::npos){
    for(int i=0;i<s.size();i++){
      opos = cpos;
      cpos = s.find("\n",opos);
      string ss = s.substr(opos,cpos-opos);
      if(ss.size()>0)
        Print(ss);        
      if(cpos==string::npos)
        break;
      cpos++;
    }
    m_oStream.str("");
  }
}
streambuf *Console::GetStreamBuf(){
  return m_oStream.rdbuf();
}

void Console::SetStdout(){
  cout.rdbuf(GetStreamBuf());  
}
/*
ofstream outFile("some_file_name.dat");//create output file object
streambuf * cout_backup=cout.rdbuf();//create backup of standard out
cout.rdbuf(outFile.rdbuf());//assign cout stream to outFile stream
cout<<"hello!"<<endl;//call any function that calls cout
cout.rdbuf(cout_backup);//restore the standard stream
outFile.close();//best to be neat and tidy about these things
*/
