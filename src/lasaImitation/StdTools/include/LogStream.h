#ifndef LOGSTREAM_H_
#define LOGSTREAM_H_


#include "XmlTree.h"

#define LOGSTREAM_ALL           -1
#define LOGSTREAM_DEFAULTENTRY  "Default"
#define LOGSTREAM_MAXINDENT     32

typedef vector<string> StringVector;

class LogStream
{
protected:
  pXmlTree        mLogTree;
  pXmlTree        mOptionsTree;
  
  pXmlTree        mCurrentEntry;
  pXmlTree        mCurrentOptions;
  
  StringVector    mStringList;
 
  char            mSpaces[LOGSTREAM_MAXINDENT+1];
 
public:
          LogStream();
  virtual ~LogStream();  
  
          void            ClearAll();
  
          void            SetCurrentEntry(const string entry);

          void            Clear();
          void            Clear(const string entry);
          
          void            Append(const string data);  
          void            Append(const string entry, const string data);
          
          StringVector&   GetStrings(int nbLines = LOGSTREAM_ALL);
          StringVector&   GetStrings(const string entry, int nbLines = LOGSTREAM_ALL);
          
          string          GetLastString();
          string          GetLastString(const string entry);

          void            SetAutoPrint(bool bAuto=true);
          void            SetAutoPrint(const string entry, bool bAuto=true);
          void            SetDeltaIndent(int dIndent);
          void            SetDeltaIndent(const string entry, int dIndent);
                    
          void            Print(int nbLines = LOGSTREAM_ALL);
          void            Print(const string entry, int nbLines = LOGSTREAM_ALL);
          
          int             Save(const string filename);
          int             Load(const string filename);
          
          void            PrintAll();
};


#endif /*LOGSTREAM_H_*/
