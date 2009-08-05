// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// 
// FlowDisplayThread.cpp
// Thread for creating and displaying flow vectors (images send on port)


#include <stdio.h>

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>

#include <Flow.h>
#include <FlowJob.h>
#include <GoFlowView.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw; 

FlowView::FlowView(ConstString portInName, ConstString portOutName, 
					 															int skip, int scale)
{
	this->portInName = portInName;
	this->portOutName = portOutName;
	this->skip = skip;
	this->scale = scale;
}

bool FlowView::threadInit()
{
	printf("Initialising connection\n");

	//this->portIn = new BufferedPort<BinPortable<ImagePair> >;
	this->portIn = new Port;
	//this->portOut = new BufferedPort<BinPortable<ImageOf<PixelRgb> > >;
	this->portOut = new Port;	
	this->portIn->open(portInName);
	this->portOut->open(portOutName);
	Network::connect("/yarpFlow/flow", portIn->getName());
	
	return true;
}

void FlowView::run() 
{
	ImageOf<PixelFloat> u_vels, v_vels;
	u_vels.resize(PIC_W,PIC_H);
	v_vels.resize(PIC_W,PIC_H);
	printf("FlowView is running\n");
   
	//portIn->close();
	//portIn->open("/test");
   while (true) {
		//check if new flow exists and grab it if it does
		//BinPortable<ImagePair> *b = 
		ImagePair pair;
	  	portIn->read(pair);
		u_vels = pair.u_vels;
		v_vels = pair.v_vels; 

		ImageOf<PixelRgb> img;// = portOut->prepare();
		img.resize(u_vels.width(),u_vels.height());
		img.zero();
		makeVectorImage(img, pair.u_vels, pair.v_vels);
		portOut->write(img);
	}
}


// draw flow vector image
void FlowView::makeVectorImage(ImageOf<PixelRgb> &img, 
				ImageOf<PixelFloat> u_vels, ImageOf<PixelFloat> v_vels)
{
	int width = u_vels.width();
	int height = u_vels.height();
   int skip = this->skip;
	int scale = this->scale;  

	for(int i = skip/2; i < width; i+=skip){
		 for(int j = skip/2; j < height; j+=skip){
			  int di = (int) ( scale * u_vels.pixel(i,j) + 0.5);
		     int dj = (int) ( scale * v_vels.pixel(i,j) + 0.5);
			   addCircle(img, PixelRgb(255,255,255), i, j, 1);
		  		addSegment(img, PixelRgb(255,255,255), i, j, i+di, j+dj);
			 }
	}
	return;
}


void FlowView::threadRelease()
{
	printf("Goodbye from FlowView\n");
}


// run as stand alone
int main()
{
	FlowView flowViewThread("/yarpFlow/yarpViewIn", "/yarpFlow/yarpViewOut", 10, 4);

	flowViewThread.start();
	
	return 0;
}
