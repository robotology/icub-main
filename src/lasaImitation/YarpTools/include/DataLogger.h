#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <stdlib.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
using namespace yarp::os;

class DataLogger
{
private:
    static DataLogger *mGlobalDataLogger; 
          
private:
    char            *mBufferPtr[2];
    char            *mCurrBufferPtr;
    
    int             mCurrBufferId;
    int             mCurrEntry;

    char            *mEntryBuffer;
    int             mCRSize;
    
    int             mEntrySize;
    int             mBufferSize;
    
    bool            bAppendToFile;
    bool            bSaveRequested;
    bool            bReady;
    bool            bStarted;
        
    char            mFilename[512];
    
public:
            DataLogger();
    virtual ~DataLogger();
    
    virtual bool    Init(unsigned int entrySize, unsigned int bufferSize = 512);
        
    virtual void    Start(const char *filename = NULL);
    virtual void    Stop();
    virtual void    Reset();
    
    virtual bool    AddEntry(const void *entryPtr, bool wait = true);
    virtual bool    AddTextEntry(const char *entryPtr, bool wait = true);
        
    virtual void    Update();
        
    virtual void    SetCurrent();
    virtual int     GetEntrySize();
    
    static DataLogger* GetDataLogger();

protected:    
            void    Close();
            void    Free();
};


class DataLoggerThread : public RateThread, public DataLogger
{
private:
    Semaphore mMutex;

public:
    DataLoggerThread(unsigned int periodMs);

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();



    virtual bool    Init(unsigned int entrySize, unsigned int bufferSize = 512);

    virtual void    Start(const char *filename = NULL);
    virtual void    Stop();
    virtual void    Reset();
    
    virtual bool    AddEntry(const void *entryPtr, bool wait = true);
    virtual bool    AddTextEntry(const char *entryPtr, bool wait = true);
        
    virtual void    Update();
    
};



#endif /*DATALOGGER_H_*/
