#include "fakeBoard.h"
#define CAN_BCAST_POSITION 0x001

#include <iostream>
#include <stdlib.h>

using namespace std;

FakeBoard::FakeBoard(int id, int p):yarp::os::RateThread(p)
{
    canId=id;
    outMessages=0;
}

FakeBoard::~FakeBoard()
{

}

void FakeBoard::run()
{
    //pop from list of messages
    inMessages.lock();
    MsgIt it=inMessages.begin();

    while(it!=inMessages.end())
    {
        FCMSG &m=(*it);
        if ((m.id&0x0f)==canId)
        {
           // fprintf(stderr, "%d rec %d %d\n", canId, (*it).id, (*it).data[0]);
            //got one message
            FCMSG reply;
            // int ch = (m.data[0]&&0x80)?1:0;

            reply.id=0;
            reply.id=(canId<<4);
            reply.len=8;

            reply.data[0]=m.data[0];

#if 0
            if ((m.data[0]&&0xef)==CAN_GET_P_GAIN)
                {

                }
            if ((m.data[0]&&0xef)==CAN_GET_D_GAIN)
                {

                }
            if ((m.data[0]&&0xef)==CAN_GET_I_GAIN)
                {

                }
            if ((m.data[0]&&0xef)==CAN_GET_ILIM_GAIN)
                {

                }
            if ((m.data[0]&&0xef)==CAN_GET_SCALE_GAIN)
                {

                }
            if ((m.data[0]&&0xef)==CAN_GET_OFFSET_GAIN)
                {

                }
            if ((m.data[0]&&0xef)==CAN_GET_TLIM_GAIN)
                {

                }
#endif 

            outMessages->lock();
            outMessages->push_back(reply);
            outMessages->unlock();
        }
        it++;

        //TODO:push replies
    }

    inMessages.clear();
    inMessages.unlock();

    
    // bcast
    FCMSG reply;
    int c=0;
    outMessages->lock();

    reply.id=canId<<4;
    reply.id=reply.id|0x100;   //bcast
    reply.id=reply.id|CAN_BCAST_POSITION;   //position
    reply.data[0]=static_cast<char>((100*(1-0.5*rand()/(double)RAND_MAX)+0.5));
    reply.data[1]=0;
    reply.data[2]=0;
    reply.data[3]=0;

    reply.data[4]=static_cast<char>((100*(1-0.5*rand()/(double)RAND_MAX)+0.5));
    reply.data[5]=0;
    reply.data[6]=0;
    reply.data[7]=0;
    reply.len=8;
      
    outMessages->push_back(reply);

    //status
    reply.id=0;
    reply.id=canId<<4;
    reply.id=reply.id|0x100;   //bcast
    reply.id=reply.id|0x003;    //status message
    reply.len=8;
    for(int k=0;k<8;k++)
        reply.data[k]=0;
    outMessages->push_back(reply);
    //fprintf(stderr, "%d %.4x\n", canId, reply.id);
    outMessages->unlock();
}

bool FakeBoard::threadInit()
{
    fprintf(stderr, "Starting board %d\n", canId);
    return true;
}

void FakeBoard::threadRelease()
{
    fprintf(stderr, "Stopping board %d\n", canId);
}
