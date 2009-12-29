#include "LogStream.h"

#include "Various.h"

LogStream::LogStream(){
  mLogTree      = new XmlTree();
  mOptionsTree  = new XmlTree();
  for(int i=0;i<LOGSTREAM_MAXINDENT;i++) mSpaces[i] = ' ';
  mSpaces[LOGSTREAM_MAXINDENT] = 0;
  ClearAll();  
}

LogStream::~LogStream(){
  ClearAll();
  delete mLogTree;  
  delete mOptionsTree;
}
  
void            LogStream::ClearAll(){
  mStringList.clear();
  
  mLogTree->Clear();
  mLogTree->SetName("Log");

  mOptionsTree->Clear();
  mOptionsTree->SetName("Log");

  mCurrentEntry   = new XmlTree(LOGSTREAM_DEFAULTENTRY);
  mCurrentOptions = new XmlTree(LOGSTREAM_DEFAULTENTRY);

  mLogTree->AddSubTree(mCurrentEntry);
  mOptionsTree->AddSubTree(mCurrentOptions);
}

void            LogStream::SetCurrentEntry(const string entry){
  mCurrentEntry   = mLogTree->Find(entry);
  mCurrentOptions = mOptionsTree->Find(entry);
  if(mCurrentEntry == NULL){
    mCurrentEntry   = new XmlTree(entry);
    mCurrentOptions = new XmlTree(entry);
    mLogTree->AddSubTree(mCurrentEntry);
    mOptionsTree->AddSubTree(mCurrentOptions);
  }
}


void            LogStream::Clear(){
  string entryName = mCurrentEntry->GetName(); 
  mCurrentEntry->Clear();
  mCurrentOptions->Clear(); 
  mCurrentEntry->SetName(entryName);
  mCurrentOptions->SetName(entryName);
}
void            LogStream::Clear(const string entry){
  SetCurrentEntry(entry);
  Clear(); 
}

void            LogStream::Append(const string data){
  int cIndent = mCurrentOptions->CGet("Indent",0);
  mSpaces[cIndent] = 0;
  string text(mSpaces);
  text.append(data);
  mCurrentEntry->AddSubTree(new XmlTree("Entry",text));
  if(mCurrentOptions->CGet("AutoPrint",false))
    Print(1);  
  mSpaces[cIndent] = ' ';
}  

void            LogStream::Append(const string entry, const string data){
  SetCurrentEntry(entry);
  Append(data); 
}

StringVector&   LogStream::GetStrings(int nbLines){
  pXmlTreeList list = mCurrentEntry->GetSubTrees();
  unsigned int size = list->size();  

  unsigned int start = 0;
  if(nbLines>0){
    if(nbLines>int(size)){
      nbLines = int(size);  
    }
  }
  start = list->size() - ((unsigned int)nbLines); 
  
  mStringList.resize(size-start);
  for(unsigned int i=start;i<size;i++){
    mStringList[i-start] = list->at(i)->GetData();
  }
  return mStringList;
}

StringVector&   LogStream::GetStrings(const string entry, int nbLines){
  SetCurrentEntry(entry);
  return GetStrings(nbLines);  
}


string          LogStream::GetLastString(){
  pXmlTreeList list = mCurrentEntry->GetSubTrees();
  unsigned int size = list->size();
  if(size>0){
    return list->at(size-1)->GetData();
  }else{
    return "";  
  }  
}

string          LogStream::GetLastString(const string entry){
  SetCurrentEntry(entry);
  return GetLastString();
}


void            LogStream::Print(int nbLines){
  pXmlTreeList list = mCurrentEntry->GetSubTrees();
  unsigned int size = list->size();  

  unsigned int start = 0;
  if(nbLines>0){
    if(nbLines>int(size)){
      nbLines = int(size);  
    }
    start = list->size() - ((unsigned int)nbLines); 
  }else{
    start = 0;
  }
  
  for(unsigned int i=start;i<size;i++){
    cout << list->at(i)->GetData() << endl;
  }
}

void            LogStream::Print(const string entry, int nbLines){
  SetCurrentEntry(entry);
  Print(nbLines);
}

void            LogStream::SetAutoPrint(bool bAuto){
  mCurrentOptions->Set("AutoPrint",bAuto);
}
void            LogStream::SetAutoPrint(const string entry, bool bAuto){
  SetCurrentEntry(entry);
  SetAutoPrint(bAuto);
}
void            LogStream::SetDeltaIndent(int dIndent){
  int cIndent = mCurrentOptions->CGet("Indent",0) + dIndent;
  if(cIndent<0) cIndent = 0;
  if(cIndent>LOGSTREAM_MAXINDENT) cIndent = LOGSTREAM_MAXINDENT;
  mCurrentOptions->Set("Indent",cIndent);
}
void            LogStream::SetDeltaIndent(const string entry, int dIndent){
  SetCurrentEntry(entry);
  SetDeltaIndent(dIndent);
}


void            LogStream::PrintAll(){
  pXmlTreeList list = mLogTree->GetSubTrees();
  for(unsigned int i=0;i<list->size();i++){
    mCurrentEntry = list->at(i);
    pXmlTreeList list2 = mCurrentEntry->GetSubTrees();
    if(list2->size()>0){
      cout << "********************"<<endl;
      cout << "* ENTRY: <"<< mCurrentEntry->GetName() <<">"<<endl;
      cout << "*-------------------"<<endl;
      Print();
    }
  }
  cout << "********************"<<endl;
  
}


int             LogStream::Save(const string filename){
  return mLogTree->SaveToFile(filename);  
}

int             LogStream::Load(const string filename){
  return mLogTree->LoadFromFile(filename);    
}

