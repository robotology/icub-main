// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 * 
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#ifndef THREADSIFT_H
#define THREADSIFT_H

// Show thread basic functionalities, you may want to have a look at the
// ratethread example.

// added initThread/releaseThread example -nat

#include <stdio.h>

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>

#include "sift.hpp"
#include "SIFT_disparity.h"
#include <iostream>
#include <yarp/os/Semaphore.h>

using namespace std;

using namespace yarp::os;

class ThreadSift : public Thread {
public:
    IplImage * img;
    SIFT_Image sift_im;
    Semaphore * sem;

///I'm not sure how to guarantee that the constructor of "Thread" is still called while using a new constructor
//     ThreadSift(Semaphore * semaphore){
//         this->sem = semaphore;
//     }
    void set_Semaphore(Semaphore * semaphore){
        this->sem = semaphore;
    }

    virtual bool threadInit() {
/*        img = 0;
        sem = NULL;*/
//         printf("Starting thread1\n");
        return true;
    }

    //called by start after threadInit, s is true iff the thread started
    //successfully
//     virtual void afterStart(bool s) {
//         if (s)
//             printf("Thread1 started successfully\n");
//         else
//             printf("Thread1 did not start\n");
//     }

    virtual void run() {
        //         while (!isStopping()) {
//         printf("Hello, from thread1\n");
        //             Time::delay(1);
        //         }
        if(img!=0){
            //sift_im.restart(); //clean the SIFT_Image first, because the load() function doesn't do that
            sift_im.load(img);
        }else
            cout << "Inside thread, and img is ==0..." << endl;
        if(sem == NULL)
            cout<<"sem == NULL RANÇO!!"<<endl;

//         cout << "exiting thread." << endl;
    }

    virtual void threadRelease() {
        sem->post();
//         printf("Goodbye from thread1\n");
        //sift_im.~SIFT_Image();
    }
};

#endif //THREADSIFT_H
