#include "DataLogger.h"

#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

#include <yarp/os/Time.h>
using namespace yarp::os;

#define DATALOGGER_CR   "\n"

DataLogger* DataLogger::mGlobalDataLogger = NULL;

DataLogger* DataLogger::GetDataLogger(){
    return mGlobalDataLogger;    
}

void DataLogger::SetCurrent(){   
    mGlobalDataLogger = this;
}

DataLogger::DataLogger(){
    strcpy(mFilename,"./data/log.txt");
    mBufferPtr[0]   = NULL;
    mBufferPtr[1]   = NULL;
    mCurrBufferPtr  = NULL;
    mEntryBuffer    = NULL;
    Free();
    
    if(mGlobalDataLogger==NULL) 
        SetCurrent();
}

DataLogger::~DataLogger(){
    if(mGlobalDataLogger==this) mGlobalDataLogger=NULL;
    Free();
}

void DataLogger::Free(){
    if(mBufferPtr[0]) delete mBufferPtr[0]; 
    if(mBufferPtr[1]) delete mBufferPtr[1]; 
    if(mEntryBuffer)  delete mEntryBuffer;

    mBufferPtr[0]   = NULL;
    mBufferPtr[1]   = NULL;
    mEntrySize      = 0;
    mBufferSize     = 0;
    mEntryBuffer    = NULL;    
    mCurrBufferPtr  = NULL;
    mCurrBufferId   = 0;
    mCurrEntry      = 0;
    bSaveRequested  = false;
    bAppendToFile   = false;
    bReady          = false;
    bStarted        = false;
    mCRSize         = strlen(DATALOGGER_CR);
}

bool DataLogger::Init(unsigned int dataSize, unsigned int bufferSize){
    Free();
    
    mEntrySize      = dataSize;
    mBufferSize     = bufferSize;
    
    mBufferPtr[0]   = new char[mEntrySize*mBufferSize];
    mBufferPtr[1]   = new char[mEntrySize*mBufferSize];
    
    mCurrBufferPtr  = mBufferPtr[0];

    mEntryBuffer    = new char[mEntrySize];
    
    mCurrBufferId   = 0;
    mCurrEntry      = 0;
    bAppendToFile   = false;

    bReady          = true;
    bStarted        = false;
    
    if((mBufferPtr[0]==NULL)||(mBufferPtr[1]==NULL)){
        Free();
        return false;           
    }
    cout << "Data logging ready"<<endl;
    return true;
}

void    DataLogger::Reset(){
    mCurrBufferPtr  = mBufferPtr[0];
    mCurrBufferId   = 0;
    mCurrEntry      = 0;
    bAppendToFile   = false;    
    bStarted        = false;
}

void    DataLogger::Start(const char *filename){
    if(!bReady) return;
    
    DataLogger::Reset();
    if(filename!=NULL){        
        strcpy(mFilename,filename);
    }
    cout << "DataLogger: Starting recording data to file "<< mFilename<<endl;
    bStarted = true;        
}

void    DataLogger::Stop(){
    if(!bStarted) return;
    DataLogger::Close();
    bStarted = false;
}


bool DataLogger::AddTextEntry(const char *entryStr, bool wait){
    if(!bStarted)        return false;   
    if(entryStr == NULL) return false;

    int len = strlen(entryStr);
    
    if(len>=mEntrySize-mCRSize){
        memcpy(mEntryBuffer,entryStr,mEntrySize-mCRSize);
        strcpy(mEntryBuffer+(mEntrySize-mCRSize),DATALOGGER_CR);
    }else{
        memcpy(mEntryBuffer,entryStr,len);
        memset(mEntryBuffer+len,' ',mEntrySize-mCRSize-len);
        strcpy(mEntryBuffer+(mEntrySize-mCRSize),DATALOGGER_CR);
    } 
    return DataLogger::AddEntry(mEntryBuffer,wait);
}


bool DataLogger::AddEntry(const void *entryPtr, bool wait){
    if(!bStarted)        return false;   
    if(entryPtr == NULL) return false;

    memcpy(mCurrBufferPtr,entryPtr,mEntrySize);
    mCurrEntry++;
    if(mCurrEntry >= mBufferSize){
        mCurrBufferId   = 1-mCurrBufferId;
        mCurrBufferPtr  = mBufferPtr[mCurrBufferId];
        mCurrEntry      = 0;
        bSaveRequested  = true;
    }else{
        mCurrBufferPtr += mEntrySize;        
    }
    return true;
}

void DataLogger::Update(){
    if(!bStarted) return;
        
    if(mCurrBufferPtr!=NULL){ 
        if(bSaveRequested){
            ofstream file;
            if(bAppendToFile){
                file.open(mFilename,ios_base::out|ios_base::app|ios_base::binary);
            }else{
                file.open(mFilename,ios_base::out|ios_base::binary);
                bAppendToFile = true;
            }
                
            if(file.is_open()){
                file.write(mBufferPtr[1-mCurrBufferId],mEntrySize*mBufferSize);                        
                file.close();
            }        
            bSaveRequested = false;    
        }
    }
}
int  DataLogger::GetEntrySize(){
    return mEntrySize;    
}
void DataLogger::Close(){
    if(!bReady) return;
    
    if(mCurrBufferPtr!=NULL){ 
        if(bSaveRequested){
            ofstream file;
            if(bAppendToFile){
                file.open(mFilename,ios_base::out|ios_base::app|ios_base::binary);
            }else{
                file.open(mFilename,ios_base::out|ios_base::binary);
                bAppendToFile = true;
            }
                
            if(file.is_open()){
                file.write(mBufferPtr[1-mCurrBufferId],mEntrySize*mBufferSize);                        
                file.close();
            }        
            bSaveRequested = false;    
        }
    }
    if(mCurrEntry>0){
        ofstream file;
        if(bAppendToFile){
            file.open(mFilename,ios_base::out|ios_base::app|ios_base::binary);
        }else{
            file.open(mFilename,ios_base::out|ios_base::binary);
            bAppendToFile = true;
        }
        if(file.is_open()){
            cout << "DataLogger: Closing data log to file "<< mFilename<<endl;
            file.write(mBufferPtr[mCurrBufferId],mEntrySize*mCurrEntry);                        
            file.close();
        }                    
    }
    DataLogger::Reset();
}





DataLoggerThread::DataLoggerThread(unsigned int periodMs) : 
                                   RateThread(periodMs){
}

bool DataLoggerThread::threadInit(){
    return true;
}

void DataLoggerThread::afterStart(bool s){
  if (s)
    cout << "Data Logger thread started" << endl;
  else
    cout << "Data Logger did not start" << endl;
}

void DataLoggerThread::run(){
    Update();
}

void DataLoggerThread::threadRelease()
{
    Close();
}

bool    DataLoggerThread::Init(unsigned int entrySize, unsigned int bufferSize){
    mMutex.wait();
    bool ret = DataLogger::Init(entrySize,bufferSize);
    mMutex.post();
    return ret;
}
void    DataLoggerThread::Start(const char *filename){
    mMutex.wait();
    DataLogger::Start(filename);
    mMutex.post();    
}
void    DataLoggerThread::Stop(){
    mMutex.wait();
    DataLogger::Stop();
    mMutex.post();    
}
void    DataLoggerThread::Reset(){
    mMutex.wait();
    DataLogger::Reset();
    mMutex.post();    
}

bool    DataLoggerThread::AddEntry(const void *entryPtr, bool wait){
    mMutex.wait();
    bool ret = DataLogger::AddEntry(entryPtr,wait);
    mMutex.post();
    return ret;
}
bool    DataLoggerThread::AddTextEntry(const char *entryPtr, bool wait){
    mMutex.wait();
    bool ret = DataLogger::AddTextEntry(entryPtr,wait);
    mMutex.post();
    return ret;
}
void    DataLoggerThread::Update(){
    mMutex.wait();
    DataLogger::Update();
    mMutex.post();
}
