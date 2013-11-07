#include <stdio.h>

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>

class Stats
{
private:

    double totalUsed;      //total time taken iterations
    unsigned int count;    //number of iterations from last reset
    unsigned int estPIt;   //number of useful iterations for period estimation
    double totalT;         //time bw run, accumulated
    double sumTSq;         //cumulative sum sq of estimated period dT
    double sumUsedSq;      //cumulative sum sq of estimated thread tun
    double previousRun;    //time when last iteration started
    double currentRun;     //time when this iteration started
    // AC
    double elapsed;        //time between start and end
    bool scheduleReset;

    void _resetStat() {
        totalUsed=0;
        count=0;
        estPIt=0;
        totalT=0;
        sumUsedSq=0;
        sumTSq=0;
    }

public:

    Stats(){
        _resetStat();
    }

    void resetStat() {
        _resetStat();
    }

    double getEstPeriod() {
        double ret;

        if (estPIt==0)
            ret=0;
        else
            ret=totalT/estPIt;

        return ret;
    }

    void getEstPeriod(double &av, double &std) {

        if (estPIt==0) {
            av=0;
            std=0;
        } else {
            av=totalT/estPIt;
            if (estPIt>1) {
                std=sqrt(((1.0/(estPIt-1))*(sumTSq-estPIt*av*av)));
            } else {
                std=0;
            }
        }

    }

    unsigned int getIterations() {
        unsigned int ret=count;

        return ret;
    }

    double getEstUsed() {
        double ret;

        if (count<1)
            ret=0.0;
        else
            ret=totalUsed/count;

        return ret;
    }

    void getEstUsed(double &av, double &std) {

        if (count<1) {
            av=0;
            std=0;
        } else {
            av=totalUsed/count;
            if (count>1) {
                std=sqrt((1.0/(count-1))*(sumUsedSq-count*av*av));
            } else {
                std=0;
            }
        }

    }


    void tickStart() {
        currentRun=yarp::os::Time::now();

        if (count>0) {
            //double saved=adaptedPeriod;
            double dT=(currentRun-previousRun)*1000;
            sumTSq+=dT*dT;
            totalT+=dT;
            estPIt++;
        }

        previousRun=currentRun;
        count++;
    }

    void tickEnd()
    {
        elapsed=(yarp::os::Time::now()-currentRun) * 1000;

        //save last
        totalUsed+=elapsed;
        sumUsedSq+=elapsed*elapsed;
    }

    //AC
    double getElapsed()
    {
    	return elapsed;
    }
};
