// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <Flow.h>
#include <ImageStack.h>
#include <FlowThread.h>
#include <Defn.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

int main()
{
	//yarp::sig::ImageOf<yarp::sig::PixelMono> ypImg;
	int i = 0;
	int count = 0;

	Network::init();
	//ImageStack<IplImage> is(5, 10);
	ImageStack<yarp::sig::ImageOf<yarp::sig::PixelMono> > is(5,5);	

	FlowThread<yarp::sig::ImageOf<yarp::sig::PixelMono> > flowThread("/yarpFlow/flow");
//	FlowDisplayThread flowDisplayThread(&flowBank);

	Port p;
	Port p2;
	//Port p;
	p.open("/yarpFlow/imgIn");
	p2.open("/yarpFlow/imgOut");
	//Network::connect("/icub/cam/left", "/yarpFlow/imgIn");
	Network::connect("/cam", "/yarpFlow/imgIn");	
	Network::connect("/yarpFlow/imgOut", "/chris/view");

	printf("yarpFlow: starting flowThread\n");
	flowThread.setImageStack(&is);
	flowThread.setFlowJob(new FlowJob(PIC_W, PIC_H));
	flowThread.start();
//	flowDisplayThread.start();

	while (true) {
		//printf("yarpFlow: reading\n");
		ImageOf<PixelMono> *ypImgMono = new ImageOf<PixelMono>;	
		//printf("reading images\n");
		p.read(*ypImgMono);
		ypImgMono->copy(*ypImgMono,PIC_W,PIC_H);
		//ypImgMono.resize(PIC_W,PIC_H);
	   //ypRgbImg.copy(*p.read());
		p2.write(*ypImgMono);
		yarp::os::Time::delay(0.1); ///10.0);
		//p2.write(imgRgbImg);

		//sprintf(backFile, "file%d.ppm", count);	
		//yarp::sig::file::write(*ypImg, backFile);
		//cvImage = (IplImage *) input.getIplImage();
		while(is.isLocked()){
			yarp::os::Time::delay(0.1);
		}
		is.Lock();
		is.addSlice(ypImgMono);	
		is.UnLock();
	}
	//Time::delay(0.1);
	count++;
}
