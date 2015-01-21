#ifndef __ETH_DRIVER_H__
#define __ETH_DRIVER_H__

#include "driver.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

//#include <string>

#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>
#include <ace/Time_Value.h>
#include <ace/OS_NS_sys_socket.h>

//#include "stdint.h"

class CanSocket
{
public:
     CanSocket()
    {
        ACE_OS::socket_init(2,2);
        mSocket=NULL;
    }

    ~CanSocket()
    {
        close();
        ACE_OS::socket_fini();
    }

    bool create(ACE_UINT16 port,ACE_UINT32 address)
    {
        mSocket=new ACE_SOCK_Dgram_Bcast(ACE_INET_Addr(port,address));
        return mSocket!=NULL;
    }

    void sendTo(void* data,size_t len,ACE_UINT16 port,ACE_UINT32 address)
    {
        mSocket->send(data,len,ACE_INET_Addr(port,address));
    }

    ssize_t receiveFrom(void* data,size_t len,ACE_UINT32 &address,ACE_UINT16 &port,int wait_msec)
    {
        ACE_Time_Value tv(wait_msec/1000,(wait_msec%1000)*1000);
        ACE_INET_Addr ace_addr;
        ssize_t nrec=mSocket->recv(data,len,ace_addr,0,&tv);

        if (nrec>0)
        {
            address=ace_addr.get_ip_address();
            port=ace_addr.get_port_number();
        }
        else
        {
            address=0;
            port=0;
        }

        return nrec;
    }

    void close()
    {
        if (mSocket)
        {
            mSocket->close();
            delete mSocket;
            mSocket=NULL;
        }
    }

protected:
    ACE_SOCK_Dgram_Bcast* mSocket;
};

/////////////////////////////////////////////////////////////////

struct ECMSG
{
    int id;
    unsigned char data[8];
    int len;
};

class EthCanMessage : public yarp::dev::CanMessage
{
public:
    ECMSG *msg;

public:
    EthCanMessage(){ msg=0; }
    virtual ~EthCanMessage(){}

    virtual CanMessage &operator=(const CanMessage &l)
    {
        const EthCanMessage &tmp=dynamic_cast<const EthCanMessage &>(l);
        memcpy(msg, tmp.msg, sizeof(ECMSG));
        return *this;
    }

    virtual unsigned int getId() const { return msg->id; }
    virtual unsigned char getLen() const { return msg->len; }
    virtual void setLen(unsigned char len) { msg->len=len; }
    virtual void setId(unsigned int id){ msg->id=id; }
    virtual const unsigned char *getData() const { return msg->data; }
    virtual unsigned char *getData(){ return msg->data; }
    virtual unsigned char *getPointer(){ return (unsigned char *) msg; }
    virtual const unsigned char *getPointer() const { return (const unsigned char *) msg; }
    virtual void setBuffer(unsigned char *b){ if (b) msg=(ECMSG *)(b); }
};

///////////////////////////////////////////////////

#define USE_PROG_ID
#undef USE_EXTRA64

struct CanPktHeader_t
{
    unsigned char signature;
    unsigned char canFrameNumOf;
#ifdef USE_PROG_ID   
    ACE_UINT8         dummy[2];    
    ACE_UINT32        progressive;
#else 
	ACE_UINT8         dummy[6]; 
#endif
};

struct CanPktFrame_t
{
    unsigned char  canBus;
    unsigned char  len;
    unsigned short canId;
    unsigned char  dummy[4];
    unsigned char  data[8];
};

struct CanPkt_t
{
    CanPktHeader_t header;
    CanPktFrame_t frames[1];
#ifdef USE_EXTRA64
	uint8_t	      extra64[64];
#endif
};

///////////////////////////////////////////////////

class eDriver : public yarp::dev::ImplementCanBufferFactory<EthCanMessage,ECMSG>,public iDriver
{
private:
    CanSocket mSocket;

    char mCanBusId;
    ACE_UINT32 mBoardAddr;

	double timestart;

    yarp::dev::ICanBufferFactory *iFactory;
public:
    eDriver(){}
   ~eDriver(){}

    int init(yarp::os::Searchable &config)
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

		yarp::os::Time::delay(2.0); // avoids that the user sends a message too soon (2.0 is a good value)
		
		// removed the command to avoid a bootstrap
        // unsigned char CMD_JMP_UPD=0x0C;
        // mSocket.sendTo(&CMD_JMP_UPD,1,3333,mBoardAddr);

        // yarp::os::Time::delay(0.5); // (was 3)

        ////////////////////////////////////
        unsigned char CMD_CANGTW_START=0x20;

        mSocket.sendTo(&CMD_CANGTW_START,1,3333,mBoardAddr);

		// version 2.2 of the eUpdater waits for 2 seconds after reception of GTW_START before forwarding udp packets to can.
		// we wait for 1.5 sec so that max wait of the udp packet is 0.5 sec and the timeout is not triggered.
		// for next versions of eUpdater we may reduce this time to 0.5 or even entirely remove the delay.
        yarp::os::Time::delay(1.5); 	

		//printf("@ %f sec from start: exit init phase\n", yarp::os::Time::now() - timestart);

        return 0;
    }

    int uninit()
    {
        static char CMD_CANGTW_STOP = 0x21;

        mSocket.sendTo(&CMD_CANGTW_STOP,1,3334,mBoardAddr);

        mSocket.close();

        return 0;
    }

    int receive_message(yarp::dev::CanBuffer &messages, int howMany = MAX_READ_MSG, double TIMEOUT = 1.0)
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

    int send_message(yarp::dev::CanBuffer &message, int n)
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

    yarp::dev::CanBuffer createBuffer(int m)
    {
        return yarp::dev::ImplementCanBufferFactory<EthCanMessage,ECMSG>::createBuffer(m);
    }

    void destroyBuffer(yarp::dev::CanBuffer &buff)
    {
        yarp::dev::ImplementCanBufferFactory<EthCanMessage,ECMSG>::destroyBuffer(buff);
    }
};

#endif