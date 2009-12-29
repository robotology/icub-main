#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#ifdef WIN32
#pragma warning( disable : 4786)
#endif

#include <vector>
#include <string>
#include <sstream>
using namespace std;

typedef vector<string> String_List;

class Command {
public:
  string  m_Name;
  string  m_FilePath;
public:
                Command(string name);
                Command(string name, string filepath);
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
  int                   m_CursorPos;

  ostringstream         m_oStream;

public:
            Console();
  virtual   ~Console();

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

  void      MoveRight(bool bSkipWord=false);
  void      MoveLeft(bool bSkipWord=false);


  void      AddChar(char c);
  void      EraseChar(bool bkw = true);
  void      ClearLine();
  void      Accept(bool bToHistory = true);
  virtual   int       AutoCompletion();
  vector<string>       AutoCompleteFilename();

  void      Execute(string cmd, bool bToHistory = true);

  void      Print(string line);
  void      Update();

  ostream   *GetStream();

  streambuf *GetStreamBuf();
  void      SetStdout();

};
#define SVNTEST_V123


#endif
