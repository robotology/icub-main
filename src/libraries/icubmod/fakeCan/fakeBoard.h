#ifndef __FAKEBOARD__
#define __FAKEBOARD__

#include <yarp/os/RateThread.h>
#include "msgList.h"

class FakeBoard: public yarp::os::RateThread
{
    int canId;
    MsgList inMessages;
    MsgList *outMessages;

public:
    FakeBoard(int id=0, int p=100);

    ~FakeBoard();

    bool threadInit();

    void threadRelease();

    void setId(int id)
    {
        canId=id;
    }

    void setReplyFifo(MsgList *outBuffer)
    {
        outMessages=outBuffer;
    }

    void pushMessage(const FCMSG &msg)
    {
        inMessages.lock();
        inMessages.push_back(msg);
        inMessages.unlock();
    }

    void run();
};

#endif
