#include "Cameras.h"
#include <webots/robot.h>

#include <iostream> 
using namespace std;

Cameras::Cameras(void)
{
    Property options;
    options.fromConfigFile("configFiles/cams.ini");

	left = options.find("left").asInt();
	right = options.find("right").asInt();

	string strLeft = (left == true) ? "yes" : "no";
	string strRight = (right == true) ? "yes" : "no";
	printf("Enabled cameras : \nleft : %s\nright : %s\n", strLeft.c_str(), strRight.c_str()); 

	if(left)
	{
		pLCam.open(options.find("left_cam_port").asString().c_str());
		leftCam = wb_robot_get_device("LEFT_CAM");
		wb_camera_enable(leftCam, 10);
	}
	if(right)
	{
		pRCam.open(options.find("right_cam_port").asString().c_str());
		rightCam = wb_robot_get_device("RIGHT_CAM");
		wb_camera_enable(rightCam, 10);
	}
}

Cameras::~Cameras(void)
{
	if(left)
	{
		pLCam.close();
	}
	if(right)
	{
		pRCam.close();
	}
}

void Cameras::SendImages(void)
{
	if(left)
	{
		const unsigned char *leftImageWebots = wb_camera_get_image(leftCam);
		int leftWidth = wb_camera_get_width(leftCam);
		int leftHeight = wb_camera_get_height(leftCam);
		ImageOf<PixelRgb> &leftImageYarp = pLCam.prepare();
		leftImageYarp.setExternal((char*)leftImageWebots, leftWidth, leftHeight);
		pLCam.write();
	}
	if(right)
	{
		const unsigned char *rightImageWebots = wb_camera_get_image(rightCam);
		int rightWidth = wb_camera_get_width(rightCam);
		int rightHeight = wb_camera_get_height(rightCam);
		ImageOf<PixelRgb> &rightImageYarp= pRCam.prepare();
		rightImageYarp.setExternal((char*)rightImageWebots, rightWidth, rightHeight);
		pRCam.write();
	}
}
