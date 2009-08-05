// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// 
// FlowView.h

#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <Flow.h>
#include <FlowJob.h>
#include <ImagePair.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

class FlowView : public Thread {

	private:
		ImageOf<PixelFloat> u_vels;
		ImageOf<PixelFloat> v_vels;
		bool newFlow;
		//BufferedPort<BinPortable <ImagePair> > *portIn;
		Port *portIn;
		Port *portOut;
		ConstString portInName;
		ConstString portOutName;
		int skip;	// vector skip in display
		int scale; // vector scale in display

	public:

		FlowView(ConstString portInName, ConstString portOutName, int skip=7, int scale=1);

		bool setFlow(ImageOf<PixelFloat> u_vels, ImageOf<PixelFloat> v_vels);

		virtual bool threadInit();

		virtual void run();

		virtual void threadRelease();

		void makeVectorImage(ImageOf<PixelRgb> &img, ImageOf<PixelFloat> u_vels,
													 ImageOf<PixelFloat> v_vels);
};
