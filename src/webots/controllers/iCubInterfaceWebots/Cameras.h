#pragma once

#include <yarp/os/all.h>
using namespace yarp::os;
#include <yarp/sig/all.h>
using namespace yarp::sig;
#include <webots/camera.h>

//Class to enable the cameras and send the images to a yarp port
class Cameras
{
public:
    Cameras(void);
    ~Cameras(void);
    void SendImages(void);

    enum Cam {LEFT, RIGHT};

protected:
	bool left, right;
    BufferedPort<ImageOf<PixelRgb> > pLCam, pRCam;
    WbDeviceTag leftCam, rightCam;

private:
    void SendImage(Cam cam);
    
};
