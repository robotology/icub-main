// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Marco Accame
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
#include "driver.h"
#include <stdio.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

#include "EoUpdaterProtocol.h"

using namespace yarp::os;
using namespace yarp::dev;


// marco.accame: the new cDriver2 is just a copy of old cDriver but with different APIs

cDriver2::cDriver2()
{
    yarp::os::Time::turboBoost();
}


int cDriver2::init (Searchable &config)
{
    bool ret;

    ret=dd.open(config);
    if (!ret)
        return -1;

    dd.view(iCanBus);
    dd.view(iFactory);

    if (iCanBus==0)
        return -1;
    if (iFactory==0)
        return -1;

    int i;
    for (i = 0x700; i < 0x7FF; i++) iCanBus->canIdAdd (i);
    for (i = 0x200; i < 0x2FF; i++) iCanBus->canIdAdd (i); //for strain board (polling messages used for calibration)

    iCanBus->canSetBaudRate(0); //0=1Mbit/s

    yarp::os::Time::delay(2.0);

    return 0;
}


int cDriver2::uninit ()
{
    dd.close();
    return true;
}


int cDriver2::receive_message(vector<CanPacket> &canpackets, int howMany, double TIMEOUT)
{
    bool  ret;

    int maxreadmsg = MAX_READ_MSG;

    // or even better:
    maxreadmsg = canpackets.size();

    if (howMany>maxreadmsg)
        return 0;

    unsigned int how_many_messages=0;
    int read=0;
//    int count=0;

    double start=Time::now();
    double now=start;
    bool done=false;

    // creating a temporary buffer just for this transaction .... hmm maybe better using a permanent class member.
    CanBuffer tmpBuff=createCanBuffer(maxreadmsg);

    while(!done)
        {

            ret=iCanBus->canRead(tmpBuff, maxreadmsg, &how_many_messages, false);

            if (read+how_many_messages>maxreadmsg)
                {
                    how_many_messages=(maxreadmsg-read);
                }

            for(unsigned int k=0;k<how_many_messages;k++)
                {
                    // convert a yarp::dev::CanMessage into a CanPacket. we use operator = of CanPacket.
                    canpackets[read] = tmpBuff[k];
                    read++;
                }

            now=Time::now();

            if (read>=howMany)
                {
                    done=true;
                    read=howMany;
                }

            if ( (now-start)>TIMEOUT)
                done=true;


            //  Time::delay(0.0);
        }

    destroyCanBuffer(tmpBuff);
    if(!ret)
        return 0;

    return read;
}


int cDriver2::send_message(vector<CanPacket> &canpackets, int n)
{
    unsigned int sent=0;

    if(n > canpackets.size())
    {   // just a control
        n = canpackets.size();
    }

    CanBuffer tmpBuff=createCanBuffer(n);

    for(int i=0; i<n; i++)
    {
        // in here it would be nice having operator = in class yarp::dev::CanMessage or in class CanPacket so that we can just use following line
        // tmpBuff[0] = canpackets[0];
        tmpBuff[i].setId(canpackets[i].getId());
        tmpBuff[i].setLen(canpackets[i].getLen());
        tmpBuff[i].setBuffer(canpackets[i].getData());
        // or ... memcpy(tmpBuff[0].getData(), canpackets[0].getData(), canpackets[0].getLen());
    }

    bool ret = iCanBus->canWrite(tmpBuff, n, &sent);

    destroyCanBuffer(tmpBuff);

    if(!ret)
        return 0;
    else
        return sent;
}


yarp::dev::CanBuffer cDriver2::createCanBuffer(int m)
{
    return iFactory->createBuffer(m);
}


void cDriver2::destroyCanBuffer(yarp::dev::CanBuffer &buff)
{
    iFactory->destroyBuffer(buff);
}


// types used by eDriver


// - class eDriver2

eDriver2::eDriver2()
{
    mSocket = new CanSocket;
}

eDriver2::~eDriver2()
{
    delete mSocket;
}

#undef USE_LEGACY_0X20_MESSAGE

int eDriver2::init(yarp::os::Searchable &config)
{
    ACE_UINT32 local=(ACE_UINT32)config.find("local").asInt();

    if (!mSocket->create(3334,local))
    {
        yError("invalid address\n");
        return -1;
    }

    mBoardAddr=(ACE_UINT32)config.find("remote").asInt();

//        mCanBusId=config.check("canid")?config.find("canid").asInt():0;
    ////////////////////////////////////

    timestart = yarp::os::Time::now();

    // marco.accame on 23 may 16: review this value.
    // if we remove the delay the risk is that the ETH link is not up yet if we send the connect as soon as we power motors on.
    // but ... is it still true?
    // on the other hand, we could just reduce it to 1 sec. is it enough?
    // NOTE: on cDriver::init() there is the same delay(2.0), hence it is better keep it the same.... or make it 1.750

    //yarp::os::Time::delay(0.5); // avoids that the user sends a message too soon (2.0 is a good value)

    // removed the command to avoid a bootstrap
    // unsigned char CMD_JMP_UPD=0x0C;
    // mSocket->sendTo(&CMD_JMP_UPD,1,3333,mBoardAddr);

    yarp::os::Time::delay(2.0); // (was 3)



#if defined(USE_LEGACY_0X20_MESSAGE)

    ////////////////////////////////////
    //unsigned char CMD_CANGTW_START=0x20;
    //mSocket->sendTo(&CMD_CANGTW_START, 1, 3333, mBoardAddr);
    static unsigned char cmd_cangtw_start[8] = {0x20, 0, 0, 0, 0, 0, 0, 0};
    yDebug() << "byte is ...." << cmd_cangtw_start[0];

    mSocket->sendTo(cmd_cangtw_start, 1, 3333, mBoardAddr);

    // marco.accame on 23 may 16: review this value. 250 ms is ok.

    // version 2.2 of the eUpdater waits for 2 seconds after reception of GTW_START before forwarding udp packets to can.
    // we wait for 1.5 sec so that max wait of the udp packet is 0.5 sec and the timeout is not triggered.
    // for next versions of eUpdater we may reduce this time to 0.5 or even entirely remove the delay.
    yarp::os::Time::delay(1.5);

#else

    yWarning("sending request to start can gateway mode");

    const uint16_t t_can_stabilisation = 900; // ms
    const uint8_t b_send_ff_after_can_stabilisation = 0;
    const uint16_t t_wait_for_can_reply = 0; // ms
    const uint8_t b_clear_can_buffers_on_start_gtw = 0;
    const uint8_t b_send_ack = 1;

//    unsigned char cmd_cangtw_start[8] =
//    {
//        0x20,
//        t_can_stabilisation & 0xff,
//        t_can_stabilisation >> 8,
//        b_send_ff_after_can_stabilisation,
//        t_wait_for_can_reply & 0xff,
//        t_wait_for_can_reply >> 8,
//        b_clear_can_buffers_on_start_gtw,
//        b_send_ack
//    };

    eOuprot_cmd_CANGATEWAY_t command = {0};
    command.opc = uprot_OPC_LEGACY_CANGATEWAY;
    command.sendcanbroadcast = b_send_ff_after_can_stabilisation;
    command.time4canstable = t_can_stabilisation;
    command.time2waitcanreply = t_wait_for_can_reply;
    command.rxcanbufferclear = b_clear_can_buffers_on_start_gtw;
    command.ackrequired = b_send_ack;


    mSocket->sendTo(&command, sizeof(eOuprot_cmd_CANGATEWAY_t), 3333, mBoardAddr);


    ACE_UINT16 port;
    ACE_UINT32 address;

    const double waitingTime = 1.5; // in seconds


    if(1 == b_send_ack)
    {
        eOuprot_cmdREPLY_t cmdreply = {0};

        yDebug("waiting for an ack/nak from board");

        // we wait at least waitingTime sec because if the remote board does not run the new protocol, then it will not send an ack back.
        // and we must wait the rigth amout of time
        int mswait = 1000.0f * waitingTime;
        int nrec = mSocket->receiveFrom(&cmdreply, sizeof(cmdreply), address, port, mswait);

        if(-1 == nrec)
        {
            yWarning("remote board did not sent any ack to command 0x20. it may have a version of eUpdater older than 6.6");
        }
        else if(sizeof(cmdreply) == nrec)
        {
            if((uprot_OPC_CANGATEWAY == cmdreply.opc) && (uprot_RES_OK == cmdreply.res))
            {
                yDebug("REMOTE BOARD is in CAN gateway now");
            }
            else if((uprot_OPC_CANGATEWAY == cmdreply.opc) && (uprot_RES_OK != cmdreply.res))
            {
                yWarning("REMOTE BOARD tells that it is not in CAN gateway mode... why?");
            }
            else
            {
                uint8_t *bb = (uint8_t*) &cmdreply;
                yWarning("REMOTE BOARD sends an unknown reply[%d] = {%x, %x, %x, %x}.", nrec, bb[0], bb[1], bb[2], bb[3]);
            }
        }
        else
        {
            uint8_t *bb = (uint8_t*) &cmdreply;
            yWarning("REMOTE BOARD sends an unknown reply[%d] = {%x, %x, %x, %x}.", nrec, bb[0], bb[1], bb[2], bb[3]);
        }
    }
    else
    {
        // we must wait the value specified by board to enter in GTW: 1 sec but we make 1.5
        yarp::os::Time::delay(waitingTime);
    }


#endif

    //printf("@ %f sec from start: exit init phase\n", yarp::os::Time::now() - timestart);

    return 0;
}


int eDriver2::uninit()
{
//    static char CMD_CANGTW_STOP = 0x21;

//    mSocket->sendTo(&CMD_CANGTW_STOP,1,3334,mBoardAddr);

    mSocket->close();

    return 0;
}


int eDriver2::receive_message(vector<CanPacket> &canpackets, int howMany, double TIMEOUT)
{
    CanPkt_t canPkt;
    ACE_UINT16 port;
    ACE_UINT32 address;

    double tstart=yarp::os::Time::now();

    int nread=0;

    while (true)
    {
        int nrec=mSocket->receiveFrom(&canPkt,sizeof(CanPkt_t),address,port,1);

        if (nrec==sizeof(CanPkt_t))
        {
            printf(">> received a packet of size %d\n", nrec);

            if (address==mBoardAddr && port==3334)
            {
                int nframes=canPkt.header.canFrameNumOf;

                for (int f=0; f<nframes; ++f)
                {
                    //if (!mCanBusId || canPkt.frames[f].canBus==mCanBusId)
                    int canbus = canPkt.frames[f].canBus;
                    {
                        //printf(">>> (RX) Len=%d ID=%x Data=",canPkt.frames[0].len,canPkt.frames[0].canId);
                        //for (int l=0; l<canPkt.frames[0].len; ++l) printf("%x ",canPkt.frames[0].data[l]);
                        //printf("<<<\n");

                        canpackets[nread].setCanBus(canbus);

                        canpackets[nread].setLen(canPkt.frames[f].len);
                        canpackets[nread].setId(canPkt.frames[f].canId);
                        memcpy(canpackets[nread].getData(), canPkt.frames[f].data, canPkt.frames[f].len);

                        if (++nread>=howMany) return nread;
                    }
                }
            }
        }

        if (yarp::os::Time::now()-tstart>TIMEOUT) break;
    }

    return nread;
}


int eDriver2::send_message(vector<CanPacket> &canpackets, int n)
{
    CanPkt_t canPkt;
    static ACE_UINT32 prognumber = 0;


    for (int i=0; i<n; ++i)
    {
        canPkt.header.signature=0x12;
        canPkt.header.canFrameNumOf=1;
#ifdef USE_PROG_ID
        canPkt.header.progressive = prognumber++;
#endif

        canPkt.frames[0].canBus = canpackets[i].getCanBus();
        canPkt.frames[0].canId = canpackets[i].getId();
        canPkt.frames[0].len = canpackets[i].getLen();
        memcpy(canPkt.frames[0].data, canpackets[i].getData(), canpackets[i].getLen());

        //printf("<<< (TX) Len=%d ID=%x Data=",message[i].getLen(),message[i].getId());
        //for (int l=0; l<message[i].getLen(); ++l) printf("%x ",message[i].getData()[l]);
        //printf(">>>\n");

        if(CanPacket::everyCANbus == canpackets[i].getCanBus())
        {   // send to both can1 and can2
            canPkt.frames[0].canBus = 1;
            mSocket->sendTo(&canPkt,sizeof(CanPkt_t), 3334, mBoardAddr);
            yarp::os::Time::delay(0.001);

            canPkt.frames[0].canBus = 2;
            mSocket->sendTo(&canPkt,sizeof(CanPkt_t), 3334, mBoardAddr);
            yarp::os::Time::delay(0.001);
        }
        else
        {   // send it to the relevant bus
            mSocket->sendTo(&canPkt,sizeof(CanPkt_t), 3334, mBoardAddr);
            yarp::os::Time::delay(0.001);
        }

        //printf("@ %f sec from start: sent udp packet\n", yarp::os::Time::now() - timestart);

        // limit througput so that the ems board can safely receive the packet.
        // without this delay, the ems receives many udp packets withing a few micro-seconds, and
        // its dma buffer goes in overflow because the eth isr cannot execute at such speed.
        //yarp::os::Time::delay(0.001);
    }

    return n;
}


// in here we have the old cDriver

#if     defined(DRIVER_KEEP_LEGACY_IDRIVER)


cDriver::cDriver ()
{
	yarp::os::Time::turboBoost();
}


int cDriver::init (Searchable &config)
{
    bool ret;

    ret=dd.open(config);
    if (!ret)
        return -1;

    dd.view(iCanBus);
    dd.view(iFactory);

    if (iCanBus==0)
        return -1;
    if (iFactory==0)
        return -1;

    int i;
    for (i = 0x700; i < 0x7FF; i++) iCanBus->canIdAdd (i);
    for (i = 0x200; i < 0x2FF; i++) iCanBus->canIdAdd (i); //for strain board (polling messages used for calibration)

    iCanBus->canSetBaudRate(0); //0=1Mbit/s

    yarp::os::Time::delay(2.0);

    return 0;
}


int cDriver::uninit ()
{
    dd.close();
    return true;
}


int cDriver::receive_message(CanBuffer &messages, int howMany, double TIMEOUT)
{
    bool  ret;

    if (howMany>MAX_READ_MSG)
        return 0;

    unsigned int how_many_messages=0;
    int read=0;
    int count=0;

    double start=Time::now();
    double now=start;
    bool done=false;

    CanBuffer tmpBuff=createBuffer(MAX_READ_MSG);
    while(!done)
        {

            ret=iCanBus->canRead(tmpBuff, MAX_READ_MSG, &how_many_messages, false);

            if (read+how_many_messages>MAX_READ_MSG)
                {
                    how_many_messages=(MAX_READ_MSG-read);
                }

            for(unsigned int k=0;k<how_many_messages;k++)
                {
                    messages[read]=tmpBuff[k];
                    read++;
                }

            now=Time::now();

            if (read>=howMany)
                {
                    done=true;
                    read=howMany;
                }

            if ( (now-start)>TIMEOUT)
                done=true;


            //  Time::delay(0.0);
        }

    destroyBuffer(tmpBuff);
    if(!ret)
        return 0;

    return read;
}


int cDriver::send_message(CanBuffer &message, int messages)
{
    unsigned int sent=0;

    bool ret = iCanBus->canWrite(message, messages, &sent);

    if(!ret)
        return 0;
    else
        return sent;
}


yarp::dev::CanBuffer cDriver::createBuffer(int m)
{
    return iFactory->createBuffer(m);
}


void cDriver::destroyBuffer(yarp::dev::CanBuffer &buff)
{
    iFactory->destroyBuffer(buff);
}






    int eDriver::init(yarp::os::Searchable &config)
    {
        ACE_UINT32 local=(ACE_UINT32)config.find("local").asInt();

        if (!mSocket.create(3334,local))
        {
            yError("invalid address\n");
            return -1;
        }

        mBoardAddr=(ACE_UINT32)config.find("remote").asInt();

        mCanBusId=config.check("canid")?config.find("canid").asInt():0;
        ////////////////////////////////////

        timestart = yarp::os::Time::now();

        // marco.accame on 23 may 16: review this value.
        // if we remove the delay the risk is that the ETH link is not up yet if we send the connect as soon as we power motors on.
        // but ... is it still true?
        // on the other hand, we could just reduce it to 1 sec. is it enough?
        // NOTE: on cDriver::init() there is the same delay(2.0), hence it is better keep it the same.... or make it 1.750

        //yarp::os::Time::delay(2.0); // avoids that the user sends a message too soon (2.0 is a good value)
        yarp::os::Time::delay(1.75);

        // removed the command to avoid a bootstrap
        // unsigned char CMD_JMP_UPD=0x0C;
        // mSocket.sendTo(&CMD_JMP_UPD,1,3333,mBoardAddr);

        // yarp::os::Time::delay(0.5); // (was 3)

        ////////////////////////////////////
        unsigned char CMD_CANGTW_START=0x20;

        mSocket.sendTo(&CMD_CANGTW_START,1,3333,mBoardAddr);

        // marco.accame on 23 may 16: review this value. 250 ms is ok.

        // version 2.2 of the eUpdater waits for 2 seconds after reception of GTW_START before forwarding udp packets to can.
        // we wait for 1.5 sec so that max wait of the udp packet is 0.5 sec and the timeout is not triggered.
        // for next versions of eUpdater we may reduce this time to 0.5 or even entirely remove the delay.
        //yarp::os::Time::delay(1.5);
        yarp::os::Time::delay(0.25);

        //printf("@ %f sec from start: exit init phase\n", yarp::os::Time::now() - timestart);

        return 0;
    }

    int eDriver::uninit()
    {
        static char CMD_CANGTW_STOP = 0x21;

        mSocket.sendTo(&CMD_CANGTW_STOP,1,3334,mBoardAddr);

        mSocket.close();

        return 0;
    }

    int eDriver::receive_message(yarp::dev::CanBuffer &messages, int howMany, double TIMEOUT)
    {
        CanPkt_t canPkt;
        ACE_UINT16 port;
        ACE_UINT32 address;

        double tstart=yarp::os::Time::now();

        int nread=0;

        while (true)
        {
            int nrec=mSocket.receiveFrom(&canPkt,sizeof(CanPkt_t),address,port,1);

            if (nrec==sizeof(CanPkt_t))
            {
                if (address==mBoardAddr && port==3334)
                {
                    int nframes=canPkt.header.canFrameNumOf;

                    for (int f=0; f<nframes; ++f)
                    {
                        if (!mCanBusId || canPkt.frames[f].canBus==mCanBusId)
                        {
                            //printf(">>> (RX) Len=%d ID=%x Data=",canPkt.frames[0].len,canPkt.frames[0].canId);
                            //for (int l=0; l<canPkt.frames[0].len; ++l) printf("%x ",canPkt.frames[0].data[l]);
                            //printf("<<<\n");

                            messages[nread].setLen(canPkt.frames[f].len);
                            messages[nread].setId(canPkt.frames[f].canId);
                            memcpy(messages[nread].getData(),canPkt.frames[f].data,canPkt.frames[f].len);

                            if (++nread>=howMany) return nread;
                        }
                    }
                }
            }

            if (yarp::os::Time::now()-tstart>TIMEOUT) break;
        }

        return nread;
    }

    int eDriver::send_message(yarp::dev::CanBuffer &message, int n)
    {
        CanPkt_t canPkt;
        static ACE_UINT32 prognumber = 0;


        for (int i=0; i<n; ++i)
        {
            canPkt.header.signature=0x12;
            canPkt.header.canFrameNumOf=1;
#ifdef USE_PROG_ID
            canPkt.header.progressive = prognumber++;
#endif

            canPkt.frames[0].canBus=mCanBusId;
            canPkt.frames[0].canId=message[i].getId();
            canPkt.frames[0].len=message[i].getLen();
            memcpy(canPkt.frames[0].data,message[i].getData(),message[i].getLen());

            //printf("<<< (TX) Len=%d ID=%x Data=",message[i].getLen(),message[i].getId());
            //for (int l=0; l<message[i].getLen(); ++l) printf("%x ",message[i].getData()[l]);
            //printf(">>>\n");

            mSocket.sendTo(&canPkt,sizeof(CanPkt_t),3334,mBoardAddr);

            //printf("@ %f sec from start: sent udp packet\n", yarp::os::Time::now() - timestart);

            // limit througput so that the ems board can safely receive the packet.
            // without this delay, the ems receives many udp packets withing a few micro-seconds, and
            // its dma buffer goes in overflow because the eth isr cannot execute at such speed.
            yarp::os::Time::delay(0.001);
        }

        return n;
    }

    yarp::dev::CanBuffer eDriver::createBuffer(int m)
    {
        return yarp::dev::ImplementCanBufferFactory<EthCanMessage,ECMSG>::createBuffer(m);
    }

    void eDriver::destroyBuffer(yarp::dev::CanBuffer &buff)
    {
        yarp::dev::ImplementCanBufferFactory<EthCanMessage,ECMSG>::destroyBuffer(buff);
    }


#endif//defined(DRIVER_KEEP_LEGACY_IDRIVER)


// eof


