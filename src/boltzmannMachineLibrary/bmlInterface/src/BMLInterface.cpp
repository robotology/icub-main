// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/BMLInterface.h>

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//within Project Include
//#include <iCub/ImageProcessor.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

#define _imgRecv (*(ptr_imgRecv))
static YARPImgRecv *ptr_imgRecvLayer0;
static YARPImgRecv *ptr_imgRecvLayer1;
static YARPImgRecv *ptr_imgRecvLayer2;
static YARPImgRecv *ptr_imgRecvLayer3;
static YARPImgRecv *ptr_imgRecvLayer4;
static YARPImgRecv *ptr_imgRecvLayer5;
static YARPImgRecv *ptr_imgRecvLayer6;
static YARPImgRecv *ptr_imgRecvLayer7;
static YARPImgRecv *ptr_imgRecvLayer8;
#define _imgRecvLayer0 (*(ptr_imgRecvLayer0))
#define _imgRecvLayer1 (*(ptr_imgRecvLayer1))
#define _imgRecvLayer2 (*(ptr_imgRecvLayer2))
#define _imgRecvLayer3 (*(ptr_imgRecvLayer3))
#define _imgRecvLayer4 (*(ptr_imgRecvLayer4))
#define _imgRecvLayer5 (*(ptr_imgRecvLayer5))
#define _imgRecvLayer6 (*(ptr_imgRecvLayer6))
#define _imgRecvLayer7 (*(ptr_imgRecvLayer7))
#define _imgRecvLayer8 (*(ptr_imgRecvLayer8))




BMLInterface::BMLInterface(){
	this->command=new string("");
	//---
	/*meanColour_flag=true;
	contrastLP_flag=false;
	blobCataloged_flag=true;*/
	
}




// try to interrupt any communications or resource usage
bool BMLInterface::interruptModule() {
    port_in.interrupt();
	port_out.interrupt();
    return true;
}

bool BMLInterface::close() {
		port_in.close();
		port_out.close();
		closePorts();
		return true;
	}

void BMLInterface::setOptions(yarp::os::Property opt){
	options	=opt;
}

bool BMLInterface::updateModule() {    
   
    return true;
}




void BMLInterface::setRowDim(int number){
	this->rowDim=number;
}

void BMLInterface::setColDim(int number){
	this->colDim=number;
}

bool BMLInterface::outPorts(){
	bool ret = true;
	//IplImage *cvImage = cvCreateImage(cvSize(320,240),8, 3);
	//cvCvtColor((IplImage*)outMeanColourLP->getIplImage(), cvImage, CV_BGR2RGB);
	//image_out->wrapIplImage(cvImage);
	/*int psb;
	IppiSize srcsize={320,240};
	//printf("Entered in outPorts \n");
	Ipp8u* im_out = ippiMalloc_8u_C1(320,240,&psb);
	Ipp8u* im_tmp[3];
	Ipp8u* im_tmp_tmp= ippiMalloc_8u_C1(320,240,&psb);
	im_tmp[0]=im_out;
	im_tmp[1]=im_out;
	im_tmp[2]=im_out;
	ippiCopy_8u_C3P3R(this->outMeanColourLP->getPixelAddress(0,0),320*3,im_tmp,psb,srcsize);
	ippiCopy_8u_C1R(im_tmp[2],psb,im_tmp_tmp,psb,srcsize);
	ippiCopy_8u_C1R(im_tmp[0],psb,im_tmp[2],psb,srcsize);
	ippiCopy_8u_C1R(im_tmp[2],psb,blobFov->getPixelAddress(0,0),psb,srcsize);*/
	//ippiCopy_8u_P3C3R(im_tmp,psb,image_out->getPixelAddress(0,0),320*3,srcsize);
	//this->_pOutPort2->prepare()=*(this->image_out);
	this->_pOutPort2->prepare()=*(this->image_out);
	//this->_pOutPort2->prepare()=*(this->processor2->portImage);
	//this->_pOutPort3->prepare()=*(this->processor3->portImage);
	//printf("After prepares \n");
	this->_pOutPort2->write();
	//this->_pOutPort2->write();
	//this->_pOutPort3->write();
	//ippiFree(im_out);
	//ippiFree(im_tmp_tmp);
	//ippiFree(im_tmp);
	if(strcmp(command->c_str(),"")){
		Bottle& outBot1=_pOutPort->prepare();
		//bOptions.addString("to");
		//bOptions.addString("Layer0");
		outBot1.fromString(command->c_str());
		outBot1.addList()=bOptions;
		this->_pOutPort->writeStrict();
		command->clear();
		bOptions.clear();
	}
	return ret;
}



/*void BMLInterface::deleteObjects() {
    delete ptr_imgRecv;
    delete ptr_inputImg;
    delete ptr_semaphore;
}*/

bool BMLInterface::openPorts(){
	bool ret = false;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	printf("Registering port %s on network %s...\n", "/rea/BMLInterface/in","default");
	ret = _imgRecv.Connect("/rea/BMLInterface/in","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	//--------
	ret = _imgRecvLayer0.Connect("/rea/BMLInterface/inLayer0","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer1.Connect("/rea/BMLInterface/inLayer1","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer2.Connect("/rea/BMLInterface/inLayer2","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	//--------
	ret = _imgRecvLayer3.Connect("/rea/BMLInterface/inLayer3","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer4.Connect("/rea/BMLInterface/inLayer4","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer5.Connect("/rea/BMLInterface/inLayer5","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer6.Connect("/rea/BMLInterface/inLayer6","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer7.Connect("/rea/BMLInterface/inLayer7","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvLayer8.Connect("/rea/BMLInterface/inLayer8","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	//-------------
	if (true)
        {		
            _pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
            printf("Registering port %s on network %s...\n", "/rea/BMLInterface/out","dafult");
            bool ok = _pOutPort->open("/rea/BMLInterface/out");
            if  (ok)
                printf("Port registration succeed!\n");
            else 
                {
                    printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
                    return false;
                }
			_pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;
            printf("Registering port %s on network %s...\n", "/rea/BMLInterface/out","dafult");
            ok = _pOutPort2->open("/rea/BMLInterface/outBlobs");
            if  (ok)
                printf("Port registration succeed!\n");
            else 
                {
                    printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
                    return false;
                }

        }

	return true;
}

bool BMLInterface::closePorts(){
	bool ret = false;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	printf("Closing port %s on network %s...\n", "/rea/BMLInterface/in","default");
	_imgRecv.Disconnect();//("/rea/BMLInterface/in","default");
	//--------
	ret = _imgRecvLayer0.Disconnect();//("/rea/BMLInterface/inLayer0","default");
	ret = _imgRecvLayer1.Disconnect();//("/rea/BMLInterface/inLayer1","default");
	ret = _imgRecvLayer2.Disconnect();//("/rea/BMLInterface/inLayer2","default");
	//--------
	ret = _imgRecvLayer3.Disconnect();//("/rea/BMLInterface/inLayer3","default");
	ret = _imgRecvLayer4.Disconnect();//("/rea/BMLInterface/inLayer4","default");
	ret = _imgRecvLayer5.Disconnect();//("/rea/BMLInterface/inLayer5","default");
	ret = _imgRecvLayer6.Disconnect();//("/rea/BMLInterface/inLayer6","default");
	ret = _imgRecvLayer7.Disconnect();//("/rea/BMLInterface/inLayer7","default");
	ret = _imgRecvLayer8.Disconnect();//("/rea/BMLInterface/inLayer8","default");
	//-------------
	if (true)
        {		
            //_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
            printf("Closing port %s on network %s...\n", "/rea/BMLInterface/out","dafult");
            _pOutPort->close();//open("/rea/BMLInterface/out");
			//_pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;
            printf("Closing port %s on network %s...\n", "/rea/BMLInterface/out","dafult");
            _pOutPort2->close();//open("/rea/BMLInterface/outBlobs");
        }
	return true;
}







//static GtkWidget *mainWindow = NULL;
bool BMLInterface::open(Searchable& config) {
    //ct = 0;
    //port_in.open(getName("in"));
	//ConstString portName2 = options.check("name",Value("/worker2")).asString();
	//port_out.open(getName("out"));
	//port_Blobs.open(getName("outBlobs"));
    //cmdPort.open(getName("cmd")); // optional command port
    //attach(cmdPort); // cmdPort will work just like terminal

	/

	return true;	
}



