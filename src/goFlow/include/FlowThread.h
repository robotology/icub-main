// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// 
// FlowThread.cpp
// Thread for computing flow using full image stack
// ratethread example.

// added initThread/releaseThread example -nat

#include <stdio.h>

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include "Flow.h"
#include "FlowJob.h"
#include "ImagePair.h"
#include "Defn.h"

using namespace yarp::os;

template <class T>
class FlowThread : public Thread {

	private: ImageStack<T> *is;
				FlowJob *job;
				Flow *f;
//				ImageOf<PixelFloat> *u_vels;
//				ImageOf<PixelFloat> *v_vels;
				//BufferedPort<BinPortable <ImagePair> > port;
				Port port;

	public:

	 FlowThread(ConstString portName)
	 {
		this->port.open(portName);
		printf("FlowThread: created - IS and Job required\n"); 
	 }

	 FlowThread(ConstString portName, ImageStack<T> *is)
	 {
		this->port.open(portName);
		this->is = is;
		printf("FlowThread: created - Have IS - need Job\n");
	 }

	 FlowThread(ConstString portName, ImageStack<T> *is, FlowJob *job)
	 {
		 this->port.open(portName); 
		 this->job = job;
		 this->is = is;
		 printf("FlowThread: Have IS and Job - fire when ready\n"); 
	 }

	 virtual bool setImageStack(ImageStack<T> *is)
	 {
		this->is = is;
		if(this->is == NULL) return false;
	   return true;
	 }

	 virtual bool setFlowJob(FlowJob *job)
    {
      this->job = job;
      if(this->job == NULL) return false;
      return true;
    }

    virtual bool threadInit()
	 {
		printf("Starting thread1\n");

	   // first some sanity checks	
		if(f == NULL){
         printf("flowThread: ERROR -> no image stack set\n");
         return false;
      }
      
		if(job == NULL){
         printf("flowThread: ERROR -> no flow rob set\n");
         return false;
      }


		// now some objects that might be useful
		f = new Flow();
 //     u_vels = new ImageOf<PixelFloat>();
  //    v_vels = new ImageOf<PixelFloat>();
	
	//	u_vels->resize(320, 240);
	 //  v_vels->resize(320, 240);	
		return true;
	 }

    virtual void run() {
			double before, after;
	
        while (!isStopping()) {
			   before = Time::now();	
				ImageOf<PixelFloat> u_vels;
				ImageOf<PixelFloat> v_vels;
				u_vels.resize(PIC_W,PIC_H);
				v_vels.resize(PIC_W,PIC_H);
				if(is->isFull()){
					// compute flow
					double before = Time::now();
					f->calc_lucas_optical_flow(is, job, &u_vels, &v_vels);
					// prepare send flow on port
				/*	
					  for(int i = 0; i < u_vels.width(); i++){
						  for(int j = 0; j < v_vels.height(); j++){
							  printf("%f %f\n", u_vels.pixel(i,j), v_vels.pixel(i,j));
					 		}
					}
		 			*/			
					ImagePair pair; // = port.prepare();
					pair.u_vels.copy(u_vels, PIC_W, PIC_H); //copy(u_vels, PIC_W, PIC_H);
					pair.v_vels.copy(v_vels, PIC_W, PIC_H);
					port.write(pair);
				}
				after = Time::now();
				printf("OF updates (Hz): %.2lf\n", 1.0/(after - before));
        }
    }

    virtual void threadRelease()
	{
		printf("FlowThread finished\n");
	}
};
