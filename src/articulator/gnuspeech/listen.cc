// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>

#include <fstream>
using namespace std;

#include "listen.h"

#include "yarpy.h"
#include "control.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Sound.h>
#include <yarp/sig/SoundFile.h>
#include <yarp/dev/PolyDriver.h>

#include <fstream>

#define MAX_LEN 10000

//#define SAMPLE_RATE	22000
#define SAMPLE_RATE	19750

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

Semaphore moveOn(0);
Semaphore saveMutex(1);
double saveTime = -1000;
double saveModeTime = -1000;
Bottle saveBottle;
int modeStartStop = 0;
int saveSkip = 0;
Port siggy;



#define VOL 4

class ExtCmd : public BufferedPort<Bottle> {
public:
    Semaphore mutex;
    BufferedPort<Sound> pout;

    ExtCmd() : mutex(0) {
        useCallback();
    }

    virtual void onRead(Bottle& bot) {
        if (bot.size()==3) {
            printf("got a bottle: %s\n", bot.toString().c_str());
            setParamTarget(bot.get(0).asInt(), 
                           bot.get(1).asInt(), 
                           bot.get(2).asDouble());
        } else {
            int v = bot.get(0).asInt();
            if (v==999||v==998||v==997) {
                printf("got a large bottle: %s\n", bot.toString().c_str());
                for (int i=1; i<bot.size(); i+=3) {
                    setParamTarget(bot.get(i+0).asInt(), 
                                   bot.get(i+1).asInt(), 
                                   bot.get(i+2).asDouble());
                }
                if (v==998||v==997) {
                    //printf("should save\n");
                    saveMutex.wait();
                    saveTime = Time::now();
                    saveModeTime = saveTime;
                    saveBottle = bot;
                    saveMutex.post();
                    moveOn.post();
                }
            }
        }
    }
};

ExtCmd *extCmd = NULL;

PolyDriver driver;

void init_listen(const char *name) {
    Network::init();
    printf("Opening ports ...\n");
    extCmd = new ExtCmd;
    char buf[1000];
    sprintf(buf,"%s/audio", name);
    extCmd->pout.open(buf);
    sprintf(buf,"%s/cmd", name);
    extCmd->open(buf);
    sprintf(buf,"%s/sig", name);
    siggy.open(buf);


    Drivers::factory().add(new 
                           DriverCreatorOf<ArticulatorMotor>("articulator",
                                                             "controlboard",
                                                             "ArticulatorMotor"));


    Property options;
    options.put("device", "controlboard");
    options.put("subdevice", "articulator");
    options.put("name", name);
    driver.open(options);
    
    printf("... ports opened\n");
    extCmd->mutex.post();
}


void port_audio(unsigned char *sample, int len) {
    int16_t *sample16 = (int16_t*)sample;
    extCmd->mutex.wait();
    Sound& sound = extCmd->pout.prepare();
    sound.resize(len,1);
    sound.setFrequency(SAMPLE_RATE);
    static int ct = 0;
    ct++;
    for (int i=0; i<len; i++) {
        sound.set(sample16[i]*VOL,i);
        //sound.set(i+ct%10,i);
    }

    double now = Time::now();
    saveMutex.wait();
    if (now-saveTime<1) {
        saveSkip++;
        if (saveSkip>=5) {
            static int ct = 0;
            char buf[256];
            char prefix[] = "/scratch/articulate";
            sprintf(buf,"%s/log-%09d.wav",prefix,ct);
            printf("Saving to %s\n",buf);
            yarp::sig::file::write(sound,buf);
            sprintf(buf,"%s/log-%09d.txt",prefix,ct);
            ofstream fout(buf);
            fout << saveBottle.toString().c_str() << endl;
            fout.close();
            ct++;
            saveTime = 0;
            saveSkip = 0;
            Bottle ack;
            ack.addInt(1);
            siggy.write(ack);
            saveMutex.post();
            printf("Waiting for next command...\n");
            moveOn.wait();
            saveMutex.wait();
        }
    }
    saveMutex.post();

    extCmd->pout.write(true);  //strict write
    extCmd->mutex.post();
  

    if (now - saveModeTime>10) {
        static double first = now;
        static double target = 0;
        target += ((double)len)/SAMPLE_RATE;
        double dt = target-(now-first);
        if (dt>0) {
            Time::delay(dt);
            //printf("wait for %g\n", dt);
        }
    }
}


// I think this is useless now, should remove...
void plisten(unsigned char *sample, int len) {
    //printf("got %d entities\n",len);
    //printf("missing port_audio call\n");
    port_audio(sample,len);
    return;

    static ofstream fout("/tmp/snd.txt");
    static ofstream fout2("/tmp/mot.txt");


    for (int i=0; i<len; i++) {
        fout << ((int)(sample[i])) << " ";
    }
    fout << endl;
    fout.flush();

    fout2 << getTime() << " ";
    fout2 << getParam(0,0) << " ";
    fout2 << getParam(0,1) << " ";
    fout2 << getParam(1,0) << " ";
    fout2 << getParam(2,0) << " ";
    fout2 << getParam(2,1) << " ";
    fout2 << getParam(2,2) << " ";
    fout2 << getParam(2,3) << " ";
    for (int i=0; i<TOTAL_REGIONS; i++) {
        fout2 << getParam(3,i) << " ";
    }
    fout2 << getParam(4,0);
    fout2 << endl;
    fout2.flush();
    //printf("sum %g\n", sum);

}

