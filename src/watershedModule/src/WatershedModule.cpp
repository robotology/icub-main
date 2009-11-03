// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/WatershedModule.h>

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


#define BLOB_MAXSIZE 4096
#define BLOB_MINSIZE 100


static GtkWidget *menubar;
static GtkWidget *fileMenu, *imageMenu, *helpMenu;
static GtkWidget *fileItem, *imageItem, *helpItem;
static GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
static GtkWidget *mainWindow = NULL;
//status bar of the windows
static GtkWidget *statusbar;
//drawing area
static GtkWidget *da;
// Current frame
static GdkPixbuf *frame = NULL;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;

static YARPImgRecv *ptr_imgRecvRed;
static YARPImgRecv *ptr_imgRecvGreen;
static YARPImgRecv *ptr_imgRecvBlue;
static YARPImgRecv *ptr_imgRecvRG;
static YARPImgRecv *ptr_imgRecvGR;
static YARPImgRecv *ptr_imgRecvBY;

// Image to Display
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;

static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgRed;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgGreen;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgBlue;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgRG;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgGR;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgBY;

static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_middleImg;
static yarp::sig::ImageOf<yarp::sig::PixelInt> *ptr_tagged;
static yarp::sig::ImageOf<yarp::sig::PixelMono>* _outputImage;
static yarp::sig::ImageOf<yarp::sig::PixelRgb>* _outputImage3;

static ImageOf<PixelMonoSigned> rgs;
static ImageOf<PixelMonoSigned> grs;
static ImageOf<PixelMonoSigned> bys;
static ImageOf<PixelMono> r2;
static ImageOf<PixelMono> g2;
static ImageOf<PixelMono> b2;


// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
// Timeout ID
static guint timeout_ID;
// Watershed operator
static WatershedOperator *_wOperator;
static SalienceOperator *_salience;
static WatershedModule *wModule;

#define _imgRecv (*(ptr_imgRecv))

#define _imgRecvRed (*(ptr_imgRecvRed))
#define _imgRecvGreen (*(ptr_imgRecvGreen))
#define _imgRecvBlue (*(ptr_imgRecvBlue))
#define _imgRecvRG (*(ptr_imgRecvRG))
#define _imgRecvGR (*(ptr_imgRecvGR))
#define _imgRecvBY (*(ptr_imgRecvBY))

#define _inputImg (*(ptr_inputImg))

#define _inputImgRed (*(ptr_inputImgRed))
#define _inputImgGreen (*(ptr_inputImgGreen))
#define _inputImgBlue (*(ptr_inputImgBlue))
#define _inputImgRG (*(ptr_inputImgRG))
#define _inputImgGR (*(ptr_inputImgGR))
#define _inputImgBY (*(ptr_inputImgBY))

#define _middleImg (*(ptr_middleImg))
#define _tagged (*(ptr_tagged))
#define _semaphore (*(ptr_semaphore))

WatershedModule::WatershedModule(){
	meanColour_flag=true;
	contrastLP_flag=false;
	blobCataloged_flag=true;
	foveaBlob_flag=false;
	colorVQ_flag=false;
	blobList_flag=false;
	maxSaliencyBlob_flag=false;
	tagged_flag=false;
	watershed_flag=false;
	bluePlane_flag=false;
	redPlane_flag=false;
	greenPlane_flag=false;
	RG_flag=false;
	GR_flag=false;
	BY_flag=false;
	//----
	this->wOperator=new WatershedOperator(false,320,240,320,10);
	_wOperator=this->wOperator;
	this->salience=new SalienceOperator(320,240);
	_salience=this->salience;
	maxSalienceBlob_img=new ImageOf<PixelMono>;
	maxSalienceBlob_img->resize(320,240);
	outContrastLP=new ImageOf<PixelMono>;
	outContrastLP->resize(320,240);
	outMeanColourLP=new ImageOf<PixelBgr>;
	outMeanColourLP->resize(320,240);
	wModule=this;

	max_boxes = new YARPBox[3];
	//initializing the image plotted out int the drawing area
	image_out=new ImageOf<PixelRgb>;
	image_out->resize(320,240);
	_outputImage3=new ImageOf<PixelRgb>;
	_outputImage3->resize(320,240);
	_outputImage=new ImageOf<PixelMono>;
	_outputImage->resize(320,240);
	ptr_inputRed=new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the red plane
	ptr_inputRed->resize(320,240);
	ptr_inputGreen= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the green plane
	ptr_inputGreen->resize(320,240);
	ptr_inputBlue= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the blue plane
	ptr_inputBlue->resize(320,240);
	ptr_inputRG= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the R+G- colour opponency
	ptr_inputRG->resize(320,240);
	ptr_inputGR= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the G+R- colour opponency
	ptr_inputGR->resize(320,240);
	ptr_inputBY= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the B+Y- colour opponency
	ptr_inputBY->resize(320,240);
	_inputImgRGS=new ImageOf<PixelMonoSigned>;
	_inputImgGRS=new ImageOf<PixelMonoSigned>;
	_inputImgBYS=new ImageOf<PixelMonoSigned>;
	_inputImgRGS->resize(320,240);
	_inputImgGRS->resize(320,240);
	_inputImgBYS->resize(320,240);
	blobFov=new ImageOf<PixelMono>;
	blobFov->resize(320,240);

	salienceBU=10;
	salienceTD=10;
}


bool WatershedModule::open(Searchable& config) {
    //ct = 0;
    //port_in.open(getName("in"));
	//ConstString portName2 = options.check("name",Value("/worker2")).asString();
	//port_out.open(getName("out"));
	//port_Blobs.open(getName("outBlobs"));
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal

	// create a new window
	this->createObjects();
	this->setUp();
    mainWindow = this->createMainWindow();
	

	// Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
	gtk_window_move(GTK_WINDOW(mainWindow), 10,10);
	// All GTK applications must have a gtk_main(). Control ends here
	// and waits for an event to occur (like a key press or
	// mouse event).

	gtk_main ();
	gtk_widget_destroy(mainWindow);
    yarp::os::Network::fini();
	
    return true;
}

// try to interrupt any communications or resource usage
bool WatershedModule::interruptModule() {
    /**
	* a port for reading the edge image 
	*/
		port_in.interrupt(); // 
	/**
	* a port for reading the input Image of the Red Plane
	*/
		portRedPlane.interrupt(); // 
	/**
	* a port for reading the input Image of the Green Plane
	*/
		portGreenPlane.interrupt();
	/**
	* a port for reading the input Image of the Blue Plane 
	*/
		portBluePlane.interrupt(); 
	/**
	* a port for reading the R+G- colour opponency Image
	*/
		portRG.interrupt(); // 
	/**
	* a port for reading the G+R- colour opponency Image
	*/
		portGR.interrupt(); // 
	/**
	* a port for reading the B+Y- colour opponency Image 
	*/
	    portBY.interrupt(); // 
	/**
	* port where the processed image is buffered out
	*/
		port_out.interrupt(); //
	/**
	* port where the image of the found blob is put
	*/
	port_Blobs.interrupt(); //
    return true;
}

bool WatershedModule::close() {
		
	/**
	* a port for reading the edge image 
	*/
		port_in.close(); // 
	/**
	* a port for reading the input Image of the Red Plane
	*/
		portRedPlane.close(); // 
	/**
	* a port for reading the input Image of the Green Plane
	*/
		portGreenPlane.close();
	/**
	* a port for reading the input Image of the Blue Plane 
	*/
		portBluePlane.close(); 
	/**
	* a port for reading the R+G- colour opponency Image
	*/
		portRG.close(); // 
	/**
	* a port for reading the G+R- colour opponency Image
	*/
		portGR.close(); // 
	/**
	* a port for reading the B+Y- colour opponency Image 
	*/
	    portBY.close(); // 
	/**
	* port where the processed image is buffered out
	*/
		port_out.close(); //
	/**
	* port where the image of the found blob is put
	*/
	port_Blobs.close(); //
	cmdPort.close();
	closePorts();
	return true;
	}

void WatershedModule::setOptions(yarp::os::Property opt){
	options	=opt;
}

bool WatershedModule::updateModule() {    
    return true;
}


bool getPlanes(){
	bool ret = false;
	ret = _imgRecvRed.Update();
	if (ret == false){
		return false;
	}
	ret = _imgRecvGreen.Update();
	ret = _imgRecvBlue.Update();

	

	_semaphore.wait();
	ret = _imgRecvRed.GetLastImage(&_inputImgRed);
	wModule->ptr_inputRed=&_inputImgRed;
	_semaphore.post();
	_semaphore.wait();
	ret = _imgRecvGreen.GetLastImage(&_inputImgGreen);
	wModule->ptr_inputGreen=&_inputImgGreen;
	_semaphore.post();
	_semaphore.wait();
	ret = _imgRecvBlue.GetLastImage(&_inputImgBlue);
	wModule->ptr_inputBlue=&_inputImgBlue;
	_semaphore.post();
	
	//printf("GetImage: out of the semaphore \n");
	return ret;
}

bool getOpponencies(){
	bool ret = false;
	ret = _imgRecvRG.Update();
	ret = _imgRecvGR.Update();
	ret = _imgRecvBY.Update();

	if (ret == false){
		return false;
	}

	_semaphore.wait();
	ret = _imgRecvGR.GetLastImage(&_inputImgGR);
	wModule->ptr_inputGR=&_inputImgGR;
	_semaphore.post();
	_semaphore.wait();
	ret = _imgRecvRG.GetLastImage(&_inputImgRG);
	wModule->ptr_inputRG=&_inputImgRG;
	_semaphore.post();
	_semaphore.wait();
	ret = _imgRecvBY.GetLastImage(&_inputImgBY);
	wModule->ptr_inputBY=&_inputImgBY;
	_semaphore.post();
	//printf("GetImage: out of the semaphore \n");
	return ret;
}

void rain(){
		printf("Rain1");
		wModule->max_tag=wModule->wOperator->apply(*_outputImage,_tagged);
		printf("MAX_TAG=%d",wModule->max_tag);
		bool ret=getPlanes();
		if(ret==false){
			printf("No Planes! \n");
			//return;
		}
		ret=getOpponencies();
		if(ret==false){
			printf("No Opponency! \n");
			//return;
		}
		int psb32s;
		IppiSize srcsize={320,240};
		Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
		Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
		Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
		/*ImageOf<PixelMonoSigned> *_inputImgRGS=new ImageOf<PixelMonoSigned>;
		ImageOf<PixelMonoSigned> *_inputImgGRS=new ImageOf<PixelMonoSigned>;
		ImageOf<PixelMonoSigned> *_inputImgBYS=new ImageOf<PixelMonoSigned>;
		_inputImgRGS->resize(320,240);
		_inputImgGRS->resize(320,240);
		_inputImgBYS->resize(320,240);*/
		if(ptr_inputImgRG!=NULL){
			ippiScale_8u32s_C1R(_inputImgRG.getPixelAddress(0,0),320,_inputImgRGS32,psb32s,srcsize);
			ippiConvert_32s8s_C1R(_inputImgRGS32,psb32s,(Ipp8s*)wModule->_inputImgRGS->getPixelAddress(0,0),320,srcsize);
			//_inputImgRGS->copy(_inputImgRG,320,240);
		}
		else
			return;
		if(ptr_inputImgGR!=NULL){
			ippiScale_8u32s_C1R(_inputImgGR.getPixelAddress(0,0),320,_inputImgGRS32,psb32s,srcsize);
			ippiConvert_32s8s_C1R(_inputImgGRS32,psb32s,(Ipp8s*)wModule->_inputImgGRS->getPixelAddress(0,0),320,srcsize);
			//_inputImgGRS->copy(_inputImgGR,320,240);
		}
		else
			return;
		if(ptr_inputImgBY!=NULL){
			ippiScale_8u32s_C1R(_inputImgBY.getPixelAddress(0,0),320,_inputImgBYS32,psb32s,srcsize);
			ippiConvert_32s8s_C1R(_inputImgBYS32,psb32s,(Ipp8s*)wModule->_inputImgBYS->getPixelAddress(0,0),320,srcsize);
			//_inputImgBYS->copy(_inputImgBY,320,240);
		}
		else
			return;
		wModule->salience->blobCatalog(_tagged, *wModule->_inputImgRGS, *wModule->_inputImgGRS, *wModule->_inputImgBYS,
			_inputImgBlue, _inputImgGreen, _inputImgRed, wModule->max_tag);
		wModule->blobCataloged_flag=true;
		//istruction to set the ptr_tagged in the Watershed Module with the static variable _tagged
		wModule->tagged=ptr_tagged; //ptr_tagged is the pointer to _tagged
		ippiFree(_inputImgRGS32); //Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
		ippiFree(_inputImgGRS32); //Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
		ippiFree(_inputImgBYS32); //Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
}


//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------

/* usual callback function */
static void callback( GtkWidget *widget,gpointer   data ){
    g_print ("Hello again - %s was pressed \n", (char *) data);
	
	if(!strcmp((char *)data,"Rain1")){
		rain();
	}
	else if(!strcmp((char *)data,"Rain2")){
		printf("Rain2");
		wModule->max_tag=wModule->wOperator->apply(*_outputImage,_tagged);
		wModule->salience->blobCatalog(_tagged, rgs, grs, bys, r2, g2, b2, wModule->max_tag);
	}
	else if(!strcmp((char *)data,"Rain3")){
		printf("Rain3");
		wModule->max_tag=wModule->wOperator->apply(*_outputImage,_tagged);
		wModule->salience->blobCatalog(_tagged, rgs, grs, bys, r2, g2, b2, wModule->max_tag);
	}
	else if(!strcmp((char *)data,"DrawAllBlobs1")){
		printf("DrawAllBlobs1");
		wModule->drawAllBlobs(false);
	}
	else if(!strcmp((char *)data,"DrawAllBlobs2")){
		printf("DrawAllBlobs2");
		wModule->drawAllBlobs(false);
	}
	else if(!strcmp((char *)data,"DrawAllBlobs3")){
		printf("DrawAllBlobs3");
		wModule->drawAllBlobs(false);
	}
	else if(!strcmp((char *)data,"drawFoveaBlob1")){
		printf("drawFoveaBlob1");
		wModule->salience->drawFoveaBlob(*wModule->salience->foveaBlob,*wModule->tagged);
	}
	else if(!strcmp((char *)data,"drawFoveaBlob2")){
		printf("drawFoveaBlob2");
		
	}
	else if(!strcmp((char *)data,"drawFoveaBlob3")){
		printf("drawFoveaBlob3");
		
	}
	else if(!strcmp((char *)data,"drawVQColor1")){
		printf("drawColorVQ1 function");
		wModule->salience->DrawVQColor(*wModule->salience->colorVQ_img,*wModule->tagged);
	}
	else if(!strcmp((char *)data,"drawVQColor2")){
		printf("drawFoveaBlob2");
		
	}
	else if(!strcmp((char *)data,"drawVQColor3")){
		printf("drawFoveaBlob3");
		
	}
	else if(!strcmp((char *)data,"ContrastLP1")){
		printf("ContrastLP1 function");
		wModule->salience->DrawVQColor(*wModule->salience->colorVQ_img,*wModule->tagged);
	}
	else if(!strcmp((char *)data,"ContrastLP2")){
		printf("ContrastLP2");
		
	}
	else if(!strcmp((char *)data,"ContrastLP3")){
		printf("ContrastLP3");
		
	}
	else if(!strcmp((char *)data,"maxSalienceBlob1")){
		printf("drawColorVQ1 function");
		wModule->salience->maxSalienceBlob(*wModule->tagged, wModule->max_tag,wModule->max_boxes[0]);
	}
	else if(!strcmp((char *)data,"maxSalienceBlob2")){
		printf("drawFoveaBlob2");
		
	}
	else if(!strcmp((char *)data,"maxSalienceBlob3")){
		printf("drawFoveaBlob3");
		
	}
	
}

void cleanExit(){
	/*g_source_remove (timeout_ID);
	timeout_ID = 0;
	//closePorts();
	if (_options.saveOnExit != 0)
		saveOptFile(_options.fileName);
	if (frame)
		g_object_unref(frame);*/
	// Exit from application
	gtk_main_quit ();
    //deleteObjects();
}


static gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
	//printf("entering expose_CB \n");
	if(frame){
		//printf("frame not null");
		if ( mainWindow){
				//printf("frame and mainWindow present \n");
				guchar *pixels;
				unsigned int rowstride;
				unsigned int imageWidth,imageHeight,areaWidth, areaHeight;
				IppiSize srcsize={320,240};
				
				bool ret=getPlanes();
				if(ret==false){
					printf("No Planes! \n");
					//return TRUE;
				}
				ret=getOpponencies();
				if(ret==false){
					printf("No Opponency! \n");
					//return TRUE;
				}
				//=new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
				//_outputImage->resize(320,240);
				bool conversion=true;
				_outputImage=_wOperator->getPlane(&_inputImg); 
				rain();
				
				if(wModule->foveaBlob_flag){
					wModule->salience->drawFoveaBlob(*wModule->salience->foveaBlob,*wModule->tagged);
					ippiCopy_8u_C1R(wModule->salience->foveaBlob->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->redPlane_flag){
					ippiCopy_8u_C1R(wModule->ptr_inputRed->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->greenPlane_flag){
					ippiCopy_8u_C1R(wModule->ptr_inputGreen->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->bluePlane_flag){
					ippiCopy_8u_C1R(wModule->ptr_inputBlue->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->RG_flag){
					ippiCopy_8u_C1R(wModule->ptr_inputRG->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->GR_flag){
					ippiCopy_8u_C1R(wModule->ptr_inputGR->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->BY_flag){
					ippiCopy_8u_C1R(wModule->ptr_inputBY->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->watershed_flag){
					ippiCopy_8u_C1R(wModule->wOperator->tSrc.getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->tagged_flag){
					printf("dimension of the tagged image %d,%d \n", wModule->tagged->width(), wModule->tagged->height());
					/*for(int r=0; r<wModule->tagged->height(); r++)
						for (int c=0; c<wModule->tagged->width(); c++)
							*wModule->tagged->getPixelAddress(r,c)=255-*wModule->tagged->getPixelAddress(r,c);*/
					ippiCopy_8u_C1R(wModule->tagged->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->blobList_flag){
					wModule->drawAllBlobs(false);
					/*if(wModule->blobList!=""){
						ippiCopy_8u_C1R((unsigned char*)wModule->blobList,320,_outputImage->getPixelAddress(0,0),320,srcsize);
						conversion=true;
					}*/
				}
				else if(wModule->maxSaliencyBlob_flag){
					wModule->salience->DrawMaxSaliencyBlob(*wModule->maxSalienceBlob_img,wModule->max_tag,*wModule->tagged);
					ippiCopy_8u_C1R(wModule->maxSalienceBlob_img->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->contrastLP_flag){
					wModule->drawAllBlobs(false);
					ippiCopy_8u_C1R(wModule->outContrastLP->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
					conversion=true;
				}
				else if(wModule->colorVQ_flag){
					wModule->salience->DrawVQColor(*wModule->salience->colorVQ_img,*wModule->tagged);
					ippiCopy_8u_C3R(wModule->salience->colorVQ_img->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
					conversion=false;
				}
				else if(wModule->blobCataloged_flag){
					if(wModule->contrastLP_flag){
						ippiCopy_8u_C1R(wModule->outContrastLP->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
						conversion=true;
					}
					else if(wModule->meanColour_flag){
						//_outputImage=_wOperator->getPlane(&_inputImg); 
						//rain();
						wModule->drawAllBlobs(false);
						ippiCopy_8u_C3R(wModule->outMeanColourLP->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);	
						conversion=false;
					}
					else
					_outputImage=_wOperator->getPlane(&_inputImg); //the input is a RGB image, whereas the watershed is working with a mono image
				}
				else
				_outputImage=_wOperator->getPlane(&_inputImg); //the input is a RGB image, whereas the watershed is working with a mono image

				//-------
				
				if(conversion){
					int psb,width=320,height=240;
					Ipp8u* im_out = ippiMalloc_8u_C1(width,height,&psb);
					Ipp8u* im_tmp[3];
					//two copies in order to have 2 conversions
					//the first transform the yarp mono into a 4-channel image
					ippiCopy_8u_C1R(_outputImage->getPixelAddress(0,0), width,im_out,psb,srcsize);
					im_tmp[0]=im_out;
					im_tmp[1]=im_out;
					im_tmp[2]=im_out;
					//the second transforms the 4-channel image into colorImage for yarp
					ippiCopy_8u_P3C3R(im_tmp,psb,wModule->image_out->getPixelAddress(0,0),width*3,srcsize);
					ippiFree(im_out);
					ippiFree(im_tmp); //throws exception for heap corruption if done for every position of the vector
					//ippiFree(im_tmp[1]);
					//ippiFree(im_tmp[2]);
				}
				else
					ippiCopy_8u_C3R(_outputImage3->getPixelAddress(0,0),320*3,wModule->image_out->getPixelAddress(0,0),320*3,srcsize);
				
				//----------
				_semaphore.wait();
				bool result=yarpImage2Pixbuf(wModule->image_out, frame);
				//bool result=yarpImage2Pixbuf(&_inputImg, frame);
				imageWidth = _inputImg.width();
				imageHeight = _inputImg.height();
				_semaphore.post();

				
				
	            
				if (imageWidth==0||imageHeight==0) {
					printf("exit for dimension nil \n");
					return TRUE;
				}
	 
				areaWidth = event->area.width;
				areaHeight = event->area.height;

				unsigned int pixbufWidth=gdk_pixbuf_get_width(frame);
				unsigned int pixbufHeight=gdk_pixbuf_get_height(frame);

				if ((imageWidth!=pixbufWidth) || (imageHeight!=pixbufHeight))
					{
						g_object_unref(frame);
						//printf("unreferencing frame \n");
						frame=gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, imageWidth, imageHeight);
					}

				if ( (areaWidth != imageWidth) || (areaHeight != imageHeight) )
					{
						GdkPixbuf *scaledFrame;
						//printf("scaling image... \n");
						scaledFrame = gdk_pixbuf_scale_simple(	frame,
																areaWidth,
																areaHeight,
																GDK_INTERP_BILINEAR); // Best quality
						//GDK_INTERP_NEAREST); // Best speed

						pixels = gdk_pixbuf_get_pixels (scaledFrame);
						rowstride = gdk_pixbuf_get_rowstride(scaledFrame);
						gdk_draw_rgb_image (widget->window,
											widget->style->black_gc,
											event->area.x, event->area.y,
											event->area.width, event->area.height,
											GDK_RGB_DITHER_NORMAL,
											pixels,
											rowstride);
						g_object_unref(scaledFrame);
				
					}
				else
					{
						pixels = gdk_pixbuf_get_pixels (frame);
						rowstride = gdk_pixbuf_get_rowstride(frame);
						//printf("drawing image... \n");
						gdk_draw_rgb_image (widget->window,
											widget->style->black_gc,
											event->area.x, event->area.y,
											event->area.width, event->area.height,
											GDK_RGB_DITHER_NORMAL,
											pixels,
											rowstride);
					}
		}
		else{
			//printf("mainWindow results nil");
		}
	
	}
	return TRUE;
}

static void cb_draw_value( GtkToggleButton *button )
{
    /* Turn the value display on the scale widgets off or on depending
     *  on the state of the checkbutton */
    printf("callbacks from draw value %s \n",button->button.label_text);
	if(!strcmp(button->button.label_text,"<--RedPlane")){
		if(button->active)
			wModule->redPlane_flag=true;
		else
			wModule->redPlane_flag=false;
	}
	else if(!strcmp(button->button.label_text,"<--GreenPlane")){
		if(button->active)
			wModule->greenPlane_flag=true;
		else
			wModule->greenPlane_flag=false;
	}
	else if(!strcmp(button->button.label_text,"<--BluePlane")){
		if(button->active)
			wModule->bluePlane_flag=true;
		else
			wModule->bluePlane_flag=false;
	}
	if(!strcmp(button->button.label_text,"<--RGImage")){
		if(button->active)
			wModule->RG_flag=true;
		else
			wModule->RG_flag=false;
	}
	else if(!strcmp(button->button.label_text,"<--GRImage")){
		if(button->active)
			wModule->GR_flag=true;
		else
			wModule->GR_flag=false;
	}
	else if(!strcmp(button->button.label_text,"<--BYImage")){
		if(button->active)
			wModule->BY_flag=true;
		else
			wModule->BY_flag=false;
	}
	else if(!strcmp(button->button.label_text,"Red1-->")){
		if(button->active){
			//imageProcessModule->processor1->redPlane_flag=1;
			//imageProcessModule->processor1->greenPlane_flag=0;
			//imageProcessModule->processor1->bluePlane_flag=0;
		}
	}
	else if(!strcmp(button->button.label_text,"Green1-->")){
		if(button->active){
			//imageProcessModule->processor1->redPlane_flag=0;
			//imageProcessModule->processor1->greenPlane_flag=1;
			//imageProcessModule->processor1->bluePlane_flag=0;
		}
	}
	else if(!strcmp(button->button.label_text,"Blue1-->")){
		if(button->active){
			//imageProcessModule->processor1->redPlane_flag=0;
			//imageProcessModule->processor1->greenPlane_flag=0;
			//imageProcessModule->processor1->bluePlane_flag=1;
		}
	}
	if(!strcmp(button->button.label_text,"Red2-->")){
		if(button->active){
			//imageProcessModule->processor2->redPlane_flag=1;
			//imageProcessModule->processor2->greenPlane_flag=0;
			//imageProcessModule->processor2->bluePlane_flag=0;
		}
	}
	else if(!strcmp(button->button.label_text,"Green2-->")){
		if(button->active){
			//imageProcessModule->processor2->redPlane_flag=0;
			//imageProcessModule->processor2->greenPlane_flag=1;
			//imageProcessModule->processor2->bluePlane_flag=0;
		}
	}
	else if(!strcmp(button->button.label_text,"Blue2-->")){
		if(button->active){
			//imageProcessModule->processor2->redPlane_flag=0;
			//imageProcessModule->processor2->greenPlane_flag=0;
			//imageProcessModule->processor2->bluePlane_flag=1;
		}
	}
	if(!strcmp(button->button.label_text,"Red3-->")){
		if(button->active){
			//imageProcessModule->processor3->redPlane_flag=1;
			//imageProcessModule->processor3->greenPlane_flag=0;
			//imageProcessModule->processor3->bluePlane_flag=0;
		}
	}
	else if(!strcmp(button->button.label_text,"Green3-->")){
		if(button->active){
			//imageProcessModule->processor3->redPlane_flag=0;
			//imageProcessModule->processor3->greenPlane_flag=1;
			//imageProcessModule->processor3->bluePlane_flag=0;
		}

	}
	else if(!strcmp(button->button.label_text,"Blue3-->")){
		if(button->active){
			//imageProcessModule->processor3->redPlane_flag=0;
			//imageProcessModule->processor3->greenPlane_flag=0;
			//imageProcessModule->processor3->bluePlane_flag=1;
		}
	}
	else if(!strcmp(button->button.label_text,"ContrastLP1-->")){
		if(button->active)
			wModule->contrastLP_flag=true;
		else
			wModule->contrastLP_flag=false;
	}
	else if(!strcmp(button->button.label_text,"ContrastLP2-->")){
		if(button->active)
			wModule->contrastLP_flag=true;
		else
			wModule->contrastLP_flag=false;
		
	}
	else if(!strcmp(button->button.label_text,"ContrastLP3-->")){
		if(button->active)
			wModule->contrastLP_flag=true;
		else
			wModule->contrastLP_flag=false;	
	}
	else if(!strcmp(button->button.label_text,"MeanColoursLP1-->")){
		if(button->active)
			wModule->meanColour_flag=true;
		else
			wModule->meanColour_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"MeanColoursLP2-->")){
		if(button->active)
			wModule->meanColour_flag=true;
		else
			wModule->meanColour_flag=false;
	}
	else if(!strcmp(button->button.label_text,"MeanColoursLP3-->")){
		if(button->active)
			wModule->meanColour_flag=true;
		else
			wModule->meanColour_flag=false;
	}
	else if(!strcmp(button->button.label_text,"MaxSaliencyBlob1-->")){
		if(button->active)
			wModule->maxSaliencyBlob_flag=true;
		else
			wModule->maxSaliencyBlob_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"MaxSaliencyBlob2-->")){
		if(button->active)
			wModule->maxSaliencyBlob_flag=true;
		else
			wModule->maxSaliencyBlob_flag=false;
	}
	else if(!strcmp(button->button.label_text,"MaxSaliencyBlob3-->")){
		if(button->active)
			wModule->maxSaliencyBlob_flag=true;
		else
			wModule->maxSaliencyBlob_flag=false;
	}
	else if(!strcmp(button->button.label_text,"FoveaBlob1-->")){
		if(button->active)
			wModule->foveaBlob_flag=true;
		else
			wModule->foveaBlob_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"FoveaBlob2-->")){
		if(button->active)
			wModule->foveaBlob_flag=true;
		else
			wModule->foveaBlob_flag=false;
	}
	else if(!strcmp(button->button.label_text,"FoveaBlob3-->")){
		if(button->active)
			wModule->foveaBlob_flag=true;
		else
			wModule->foveaBlob_flag=false;
	}
	else if(!strcmp(button->button.label_text,"ColorVQ1-->")){
		if(button->active)
			wModule->colorVQ_flag=true;
		else
			wModule->colorVQ_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"ColorVQ2-->")){
		if(button->active)
			wModule->colorVQ_flag=true;
		else
			wModule->colorVQ_flag=false;
	}
	else if(!strcmp(button->button.label_text,"ColorVQ3-->")){
		if(button->active)
			wModule->colorVQ_flag=true;
		else
			wModule->colorVQ_flag=false;
	}
	else if(!strcmp(button->button.label_text,"BlobList1-->")){
		if(button->active)
			wModule->blobList_flag=true;
		else
			wModule->blobList_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"BlobList2-->")){
		if(button->active)
			wModule->blobList_flag=true;
		else
			wModule->blobList_flag=false;
	}
	else if(!strcmp(button->button.label_text,"BlobList3-->")){
		if(button->active)
			wModule->blobList_flag=true;
		else
			wModule->blobList_flag=false;
	}
	else if(!strcmp(button->button.label_text,"Tagged1-->")){
		if(button->active)
			wModule->tagged_flag=true;
		else
			wModule->tagged_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"Tagged2-->")){
		if(button->active)
			wModule->tagged_flag=true;
		else
			wModule->tagged_flag=false;
	}
	else if(!strcmp(button->button.label_text,"Tagged3-->")){
		if(button->active)
			wModule->tagged_flag=true;
		else
			wModule->tagged_flag=false;
	}
	else if(!strcmp(button->button.label_text,"Watershed1-->")){
		if(button->active)
			wModule->watershed_flag=true;
		else
			wModule->watershed_flag=false;
			
	}
	else if(!strcmp(button->button.label_text,"Watershed2-->")){
		if(button->active)
			wModule->watershed_flag=true;
		else
			wModule->watershed_flag=false;
	}
	else if(!strcmp(button->button.label_text,"Watershed3-->")){
		if(button->active)
			wModule->watershed_flag=true;
		else
			wModule->watershed_flag=false;
	}
}

bool WatershedModule::outPorts(){
	bool ret = true;

	/*IplImage *cvImage = cvCreateImage(cvSize(320,240),8, 3);
	cvCvtColor((IplImage*)outMeanColourLP->getIplImage(), cvImage, CV_BGR2RGB);
	image_out->wrapIplImage(cvImage);*/

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
	return ret;
}

bool getImage(){
	bool ret = false;
	ret = _imgRecv.Update();

	if (ret == false){
		return false;
	}

	_semaphore.wait();
	ret = _imgRecv.GetLastImage(&_inputImg);
	_semaphore.post();
	
	//printf("GetImage: out of the semaphore \n");
	return ret;
}


static gint menuFileQuit_CB(GtkWidget *widget, gpointer data)
{
	cleanExit();
	return TRUE;
}

static gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
	if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);
            //gtk_widget_show_all (saveSingleDialog);
		
        } 
	else 
        {
            //gtk_widget_hide (saveSingleDialog);	
        }

	return TRUE;
}

static void cb_digits_scale2( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
	wModule->salienceTD=adj->value;
	printf("salienceTD: %f",wModule->salienceTD);
}

static void cb_digits_scale( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
	wModule->salienceBU=adj->value;
	printf("salienceBU: %f",wModule->salienceBU);
}


static gint timeout_CB (gpointer data){
	if (getImage()){
            //             int imageWidth, imageHeight, pixbufWidth, pixbufHeight;
            //             _semaphore.wait();
            //            imageWidth = _inputImg.width();
            //            imageHeight = _inputImg.height();
            //            _semaphore.post();
            //            pixbufWidth = gdk_pixbuf_get_width(frame);
            //            pixbufHeight = gdk_pixbuf_get_height(frame);
            //            if ( (imageWidth != pixbufWidth) || (imageHeight != pixbufHeight) )
            //                            {
            //                    g_object_unref(frame);
            //                    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, imageWidth, imageHeight);
            //            }
            //            _frameN++;
			gtk_widget_queue_draw (da);
            //            if (_savingSet)
            //                saveCurrentFrame();
    }

	wModule->outPorts();
	return TRUE;
}




//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
static void updateStatusbar (GtkStatusbar  *statusbar)
{
    gchar *msg;
    float fps;
    fps = 1000 / float(50);
 
    gtk_statusbar_pop (statusbar, 0); // clear any previous message, underflow is allowed 
				    
    msg = g_strdup_printf ("%s - %.1f fps","/rea/imageProcess", fps);

    gtk_statusbar_push (statusbar, 0, msg);

    g_free (msg);
}

//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* WatershedModule::createMenubar(void)
{
    GtkWidget *menubar;

	menubar =  gtk_menu_bar_new ();
	GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    imageItem = gtk_menu_item_new_with_label ("Image");
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    fileSingleItem = gtk_check_menu_item_new_with_label ("Save single image..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSingleItem);
    gtk_signal_connect( GTK_OBJECT(fileSingleItem), "toggled", GTK_SIGNAL_FUNC(menuFileSingle_CB), mainWindow);
    fileSetItem = gtk_check_menu_item_new_with_label ("Save a set of images..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSetItem);
//    gtk_signal_connect( GTK_OBJECT(fileSetItem), "toggled", GTK_SIGNAL_FUNC(menuFileSet_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    // Submenu: Image  
    /*imageMenu = gtk_menu_new();
    imageSizeItem = gtk_menu_item_new_with_label ("Original size");
    gtk_menu_append( GTK_MENU(imageMenu), imageSizeItem);
    gtk_signal_connect( GTK_OBJECT(imageSizeItem), "activate", GTK_SIGNAL_FUNC(menuImageSize_CB), mainWindow);
    imageRatioItem = gtk_menu_item_new_with_label ("Original aspect ratio");
    gtk_menu_append( GTK_MENU(imageMenu), imageRatioItem);
    gtk_signal_connect( GTK_OBJECT(imageRatioItem), "activate", GTK_SIGNAL_FUNC(menuImageRatio_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    imageFreezeItem = gtk_check_menu_item_new_with_label ("Freeze");
    gtk_menu_append( GTK_MENU(imageMenu), imageFreezeItem);
    gtk_signal_connect( GTK_OBJECT(imageFreezeItem), "toggled", GTK_SIGNAL_FUNC(menuImageFreeze_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    imageFramerateItem = gtk_menu_item_new_with_label ("Change refresh interval..");
    gtk_menu_append( GTK_MENU(imageMenu), imageFramerateItem);
    gtk_signal_connect( GTK_OBJECT(imageFramerateItem), "activate", GTK_SIGNAL_FUNC(menuImageFramerate_CB), mainWindow);
    imageIntervalItem = gtk_menu_item_new_with_label ("Show Interval..");
    gtk_menu_append( GTK_MENU(imageMenu), imageIntervalItem);
    gtk_signal_connect( GTK_OBJECT(imageIntervalItem), "activate", GTK_SIGNAL_FUNC(menuImageInterval_CB), mainWindow);*/
    // Submenu: Help
    /*helpMenu = gtk_menu_new();	
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);*/

    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(imageItem), imageMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), imageItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
	return menubar;
}

//-------------------------------------------------
// Service Fuctions
//-------------------------------------------------



void WatershedModule::createObjects() {
    ptr_imgRecv = new YARPImgRecv;
	ptr_imgRecvRed = new YARPImgRecv;
	ptr_imgRecvGreen = new YARPImgRecv;
	ptr_imgRecvBlue = new YARPImgRecv;
	ptr_imgRecvRG = new YARPImgRecv;
	ptr_imgRecvGR = new YARPImgRecv;
	ptr_imgRecvBY = new YARPImgRecv;

	ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
	ptr_tagged->resize(320,240);

    ptr_inputImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
	ptr_inputImgRed = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
	ptr_inputImgGreen = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
	ptr_inputImgBlue = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
	ptr_inputImgRG = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
	ptr_inputImgGR = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
	ptr_inputImgBY = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    
	ptr_semaphore = new yarp::os::Semaphore;
}

bool WatershedModule::openPorts(){
	bool ret = false;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	g_print("Registering port %s on network %s...\n", "/rea/Watershed/in","default");
	ret = _imgRecv.Connect("/rea/Watershed/in","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	//--------
	ret = _imgRecvRed.Connect("/rea/Watershed/inRed","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvGreen.Connect("/rea/Watershed/inGreen","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvBlue.Connect("/rea/Watershed/inBlue","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	//--------
	ret = _imgRecvRG.Connect("/rea/Watershed/inRG","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvGR.Connect("/rea/Watershed/inGR","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	ret = _imgRecvBY.Connect("/rea/Watershed/inBY","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            g_print("Port registration succeed!\n");
        }
	else
        {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
	//-------------
	if (true)
        {		
            _pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
            g_print("Registering port %s on network %s...\n", "/rea/Watershed/out","default");
            bool ok = _pOutPort->open("/rea/Watershed/out");
            if  (ok)
                g_print("Port registration succeed!\n");
            else 
                {
                    g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
                    return false;
                }
			_pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;
            g_print("Registering port %s on network %s...\n", "/rea/Watershed/outBlobs","default");
            ok = _pOutPort2->open("/rea/Watershed/outBlobs");
            if  (ok)
                g_print("Port registration succeed!\n");
            else 
                {
                    g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
                    return false;
                }

        }

	return true;
}

bool WatershedModule::closePorts(){
	bool ret = false;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	g_print("Closing port %s on network %s...\n", "/rea/Watershed/in","default");
	ret = _imgRecv.Disconnect();
	//--------
	ret = _imgRecvRed.Disconnect(); //("/rea/Watershed/inRed","default");
	ret = _imgRecvGreen.Disconnect();//("/rea/Watershed/inGreen","default");
	ret = _imgRecvBlue.Disconnect(); //("/rea/Watershed/inBlue","default");
	//--------
	ret = _imgRecvRG.Disconnect();//("/rea/Watershed/inRG","default");
	ret = _imgRecvGR.Disconnect();//("/rea/Watershed/inGR","default");
	ret = _imgRecvBY.Disconnect();//("/rea/Watershed/inBY","default");
	//-------------
	if (true)
        {		
			_pOutPort->close();
            g_print("Closing port %s on network %s...\n", "/rea/Watershed/out","default");
			_pOutPort2->close();
            g_print("Closing port %s on network %s...\n", "/rea/Watershed/outBlobs","default");
        }

	return true;
}


void WatershedModule::setUp()
{
	if (true)
		_imgRecv.SetLogopolar(false);
	else
		_imgRecv.SetLogopolar(true);
	
	if (true)
		_imgRecv.SetFovea(false);
	else
		_imgRecv.SetFovea(true);
	
	if (openPorts() == false)
		ACE_OS::exit(1);
	
	_inputImg.resize(320,240);
	_inputImgRed.resize(320,240);
	_inputImgGreen.resize(320,240);
	_inputImgBlue.resize(320,240);
	_inputImgRG.resize(320,240);
	_inputImgGR.resize(320,240);
	_inputImgBY.resize(320,240);


}

static void scale_set_default_values( GtkScale *scale )
{
    gtk_range_set_update_policy (GTK_RANGE (scale),
                                 GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 0);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}


static GtkWidget *xpm_label_box( gchar     *xpm_filename,
                                 gchar     *label_text )
{
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    /* Create box for image and label */
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    /* Now on to the image stuff */
	if(xpm_filename!=NULL)
		image = gtk_image_new_from_file (xpm_filename);

    /* Create a label for the button */
    label = gtk_label_new (label_text);

    /* Pack the image and label into the box */
    if(xpm_filename!=NULL)
		gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
		gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* WatershedModule::createMainWindow(void)
{
	//Module=this; //it is necessary to synchronise the static function with this class
	
	GtkRequisition actualSize;
	GtkWidget* window;
	GtkWidget *label;
	GtkWidget *separator;
	
    //gtk_init (&argc, &argv);
	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "Watershed and Saliency Module");
	gtk_window_set_default_size(GTK_WINDOW (window), 320, 700); 
	gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
	g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (gtk_main_quit),
                      NULL);

    
	// When the window is given the "delete_event" signal (this is given
	// by the window manager, usually by the "close" option, or on the
	// titlebar), we ask it to call the delete_event () function
	// as defined above. The data passed to the callback
	// function is NULL and is ignored in the callback function.
    //g_signal_connect (G_OBJECT (window), "delete_event", G_CALLBACK (delete_event), NULL);

    // Box for main window in a ordered list
	GtkWidget *box,*box2,*boxA,*box3,*box4;
	box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
	// MenuBar for main window
	menubar = createMenubar();
	gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
	//gtk_widget_size_request(menubar, &actualSize);
	// Drawing Area : here the image will be drawed
	da = gtk_drawing_area_new ();
	
    g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
	/*if (_options.outputEnabled == 1)
        {
            g_signal_connect (da, "button_press_event", G_CALLBACK (clickDA_CB), NULL);
            // Ask to receive events the drawing area doesn't normally subscribe to
            gtk_widget_set_events (da, gtk_widget_get_events (da) | GDK_BUTTON_PRESS_MASK);
        }*/
	gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);
	//Toolbox area
	//creates the area as collection of port processes sequence
	box2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box2);
	GtkWidget *button,*button2,*buttonCheck;
    GtkWidget *boxButton,*boxButton2;
	GtkWidget *boxButtons;
	GtkWidget *boxSliders;
	boxButtons = gtk_hbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
	gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
	boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
	gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
	 /* Create a new button */
    button = gtk_button_new ();
	button2 = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "output1");
	g_signal_connect (G_OBJECT (button2), "clicked",G_CALLBACK (callback), (gpointer) "output2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "output1");
	boxButton2= xpm_label_box (NULL, "output2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_widget_show (boxButton2);
    gtk_container_add (GTK_CONTAINER (button2), boxButton2);
    gtk_widget_show (button2);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button);
	//gtk_container_add (GTK_CONTAINER (boxButtons), button2);

	label = gtk_label_new ("CheckList:");
	gtk_box_pack_start (GTK_BOX (boxButtons), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheckGreen = gtk_check_button_new_with_label("<--GreenPlane");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheckGreen), FALSE);
    g_signal_connect (G_OBJECT (buttonCheckGreen), "toggled",G_CALLBACK (cb_draw_value), (gpointer) "GreenPlane");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheckGreen, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheckGreen);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--RedPlane");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gpointer) "RedPlane");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--BluePlane");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gpointer) "BluePlane");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--RGImage");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "<--RGImage");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--GRImage");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "<--GRImage");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--BYImage");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gpointer) "<--BYImage");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
	
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--WatershedRain");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gpointer) "<--WatershedRain");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--MeanColorsImage");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "<--MeanColorsImage");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("<--Blue1");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gpointer) "<--Blue1");
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
	
	gtk_container_add (GTK_CONTAINER (box2), boxButtons);
	
	//---- vSeparator
	separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);

	
	//-----main section
	GtkWidget *scrollbar;
    
    
    GtkWidget *scale;
    GtkObject *adj1, *adj2;
	GtkWidget *hscale, *vscale;


	adj1 = gtk_adjustment_new (0.0, 0.0, 101.0, 0.1, 1.0, 1.0);
    vscale = gtk_vscale_new (GTK_ADJUSTMENT (adj1));
    scale_set_default_values (GTK_SCALE (vscale));
    gtk_box_pack_start (GTK_BOX (boxSliders), vscale, TRUE, TRUE, 0);
    gtk_widget_show (vscale);

	/*separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    gtk_widget_show (separator);*/

	

	//----------BOXA SECTION:1
	//boxA is the area that contains the two subsection for watershed and saliency operators
	boxA = gtk_hbox_new (FALSE, 0);
	
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
	gtk_widget_show (boxA);
	
	//---- vSeparator
	separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxA), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
	
	//--box3 section A
	box3 = gtk_hbox_new (FALSE, 0);
	/*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

	label = gtk_label_new ("Options:");
	gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


	scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

	//-----Check Buttons
	//box4=  gtk_vbox_new (FALSE, 0);
	
	//gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    //gtk_box_pack_start (GTK_BOX (box2), box3, FALSE, FALSE, 0);
    //gtk_widget_show (box3);

	box4 = gtk_vbox_new (FALSE, 0);
	gtk_container_set_border_width (GTK_CONTAINER (box4), 0);

	label = gtk_label_new ("SalienceBU Percentage:");
	gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

	double maxAdj=100;
	double minAdj=1;
	double stepAdj=1;
	
	adj1 = gtk_adjustment_new(1, minAdj,maxAdj,stepAdj, 1, 1);
	hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
	g_signal_connect (G_OBJECT (adj1), "value_changed",
                      G_CALLBACK (cb_digits_scale), NULL);


	label = gtk_label_new ("SalienceTD Percentage:");
	gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

	adj2 = gtk_adjustment_new (1, minAdj,maxAdj,stepAdj, 1, 1);
	hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj2));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
	gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
	g_signal_connect (G_OBJECT (adj2), "value_changed",
                      G_CALLBACK (cb_digits_scale2), NULL);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

	//-----box4
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Option1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gpointer)"ColourOpponency11");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Option2-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "FindEdges1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Option3-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Normalize1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
	//---box 4

	//-------run button
	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Rain1");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Rain1");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

	//---- vSeparator
	separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
	
	//--box3 section B
	//box3 = gtk_hbox_new (FALSE, 0);
	/*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

	label = gtk_label_new ("PlotsSelectors:");
	gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


	scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

	//-----Check Buttons
	box4=  gtk_vbox_new (FALSE, 0);
		gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

	//-----box4
	box4=  gtk_vbox_new (FALSE, 0);
	
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Watershed1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Watershed1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
	
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Tagged1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Tagged1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("BlobList1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "BlobList1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("ContrastLP1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ContrastLP1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MeanColoursLP1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("FoveaBlob1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "FoveaBlob1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("ColorVQ1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ColorVQ1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("ContrastLP1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ContrastLP1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("MaxSaliencyBlob1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MaxSaliencyBlob1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);


	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
	//---box 4

	//-------run button
	

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs1");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "DrawAllBlobs1");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob1");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "drawFoveaBlob1");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor1");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "drawVQColor1");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "maxSalienceBlob1");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "maxSalienceBlob1");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	//------ HSEPARATOR ---------------
	separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

	//----------BOXA SECTION:2
	//boxA is the area that contains the two subsection for watershed and saliency operators
	boxA = gtk_hbox_new (FALSE, 0);
	
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
	gtk_widget_show (boxA);
	gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);
	
	//--box3 section A
	box3 = gtk_hbox_new (FALSE, 0);
	/*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

	label = gtk_label_new ("Options:");
	gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


	scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

	//-----Check Buttons
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

	//-----box4
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("ColourOpponency1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ColourOpponency11");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MeanColoursLP1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Normalize1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
	//---box 4

	//-------run button
	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Rain2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Rain2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

	//---- vSeparator
	separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
	
	//--box3 section B
	//box3 = gtk_hbox_new (FALSE, 0);
	/*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

	label = gtk_label_new ("Options:");
	gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


	scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

	//-----Check Buttons
	box4=  gtk_vbox_new (FALSE, 0);
	
	/*-------------
	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
	---------------*/

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

	//-----box4
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("ContrastLP2-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ContrastLP2");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP2-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MeanColoursLP2");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Normalize1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
	//---box 4

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "DrawAllBlobs2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "drawFoveaBlob2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "drawVQColor2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	//------ HSEPARATOR ---------------
	separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

	//----------BOXA SECTION:3
	//boxA is the area that contains the two subsection for watershed and saliency operators
	boxA = gtk_hbox_new (FALSE, 0);
	box3 = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
	gtk_widget_show (boxA);
	gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);
	
	//--box3 section A
	/*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

	label = gtk_label_new ("Options:");
	gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


	scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

	//-----Check Buttons
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

	//-----box4
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("ColourOpponency1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ColourOpponency11");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("FindEdges1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "FindEdges1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Normalize1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
	//---box 4

	//-------run button
	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Rain3");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Rain3");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

	//---- vSeparator
	separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
	
	//--box3 section B
	/*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

	label = gtk_label_new ("OCheckList:");
	gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


	scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

	//-----Check Buttons
	box4=  gtk_vbox_new (FALSE, 0);

	/*--------------------
	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	// A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	-----*/


	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

	//-----box4
	box4=  gtk_vbox_new (FALSE, 0);
	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("ContrastLP3-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ContrastLP3");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP3-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MeanColoursLP3");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	/* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Normalize1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

	gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
	//---box 4

	//-------run button
	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs3");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "DrawAllBlobs3");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob3");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "drawFoveaBlob3");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	button = gtk_button_new ();
	/* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor3");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "drawVQColor3");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
	gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
	gtk_widget_show (button);

	//------ HSEPARATOR ---------------
	separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

	//gtk_container_add (GTK_CONTAINER (box2), boxSliders);
	gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
	// StatusBar for main window
	statusbar = gtk_statusbar_new ();
	updateStatusbar(GTK_STATUSBAR (statusbar));
	gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
	gtk_widget_size_request(statusbar, &actualSize);
	//_occupiedHeight += 2*(actualSize.height);

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
	// TimeOut used to refresh the screen
    timeout_ID = gtk_timeout_add (100, timeout_CB, NULL);

	mainWindow=window;

	return window;
}



void WatershedModule::drawAllBlobs(bool stable)
{
	salience->ComputeSalienceAll(this->max_tag,this->max_tag);
	//extracts the PixelBgr color of a particular blob identified by the id (last parameter)
	PixelBgr varFoveaBlob = salience->varBlob(*tagged, *wModule->ptr_inputRG, *wModule->ptr_inputGR, *wModule->ptr_inputBY, 1);

	salience->drawFoveaBlob(*blobFov, *tagged);
	//__OLD//salience.drawBlobList(blobFov, tagged, blobList, max_tag, 127);
	
	//list of boolean whether is present or not a blob
	blobList = new char [320*240+1];
	memset(blobList, 0, sizeof(char)*(max_tag+1));
	// - faster
	// - it considers also "lateral" pixels
	// - it doesn't add pixels iteratively
	wOperator->findNeighborhood(*tagged, 0, 0, blobList);
	salience->fuseFoveaBlob3(*tagged, blobList, varFoveaBlob,max_tag);

	// alternative method
	//__OLD//rain.fuseFoveaBlob(tagged, blobList, max_tag);
	
	//__OLD//blobList[1]=2; // so the fovea blob is eliminated by the removeBlobList
	//__OLD//salience.statBlobList(tagged, blobList, max_tag, fovBox);
	YARPBox fovBox;
	fovBox=salience->getBlobNum(1);
	//__OLD//salience.removeBlobList(blobList, max_tag);
	salience->removeFoveaBlob(*tagged);
	//__OLD//salience.updateFoveaBlob(tagged, blobList, max_tag);

	if (stable) {
		for (int i=0; i<2; i++) {
			memset(blobList, 0, sizeof(char)*(max_tag+1));
			wOperator->findNeighborhood(*tagged, 0, 0, blobList);
			const int minBoundingArea=15*15;
			int count=salience->countSmallBlobs(*tagged, blobList, max_tag, minBoundingArea);
			printf("Count of small blobs: %d \n",count);
			blobList[1]=0;
			salience->mergeBlobs(*tagged, blobList, max_tag, 1);
		}

		/*__OLD//while (num!=0) {
			blobList[1]=0;
			salience.mergeBlobs(tagged, blobList, max_tag, 1);
			memset(blobList, 0, sizeof(char)*max_tag);
			rain.findNeighborhood(tagged, 0, 0, blobList);
			num = salience.checkSmallBlobs(tagged, blobList, max_tag, minBoundingArea);
		}*/
	}
		
	//__OLD//salience.drawFoveaBlob(blobFov, tagged);
	//__OLD//salience.drawBlobList(blobFov, tagged, blobList, max_tag, 127);
	
	// Comment the following line to disable the elimination of non valid blob
	salience->RemoveNonValidNoRange(max_tag, BLOB_MAXSIZE, BLOB_MINSIZE);
	
	//__OLD//salience.DrawContrastLP(rg, gr, by, tmp1, tagged, max_tag, 0, 1, 30, 42, 45); // somma coeff pos=3 somma coeff neg=-3
	//__OLD//salience.checkIOR(tagged, IORBoxes, num_IORBoxes);
	//__OLD//salience.doIOR(tagged, IORBoxes, num_IORBoxes);
	//float salienceBU=1.0,salienceTD=0.0;
	IppiSize srcsize={320,240};
	PixelMonoSigned searchTD=0,searchRG=30,searchGR=42,searchBY=45;
	int psb32s;
	Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
	Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
	Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
	//_inputImgGR
	if(ptr_inputImgRG!=NULL){
		//_inputImgRGS->copy(*ptr_inputImgRG,320,240);
		ippiScale_8u32s_C1R(_inputImgRG.getPixelAddress(0,0),320,_inputImgRGS32,psb32s,srcsize);
		ippiConvert_32s8s_C1R(_inputImgRGS32,psb32s,(Ipp8s*)_inputImgRGS->getPixelAddress(0,0),320,srcsize);
		//ippiCopy_8u_C1R(_inputImgRG.getPixelAddress(0,0),320,_inputImgRGS->getPixelAddress(0,0),320,srcsize);
	}
	else
		return;
	//_inputImgGR
	if(ptr_inputImgGR!=NULL){
		//_inputImgGRS->copy(*ptr_inputImgGR,320,240);
		ippiScale_8u32s_C1R(_inputImgGR.getPixelAddress(0,0),320,_inputImgGRS32,psb32s,srcsize);
		ippiConvert_32s8s_C1R(_inputImgGRS32,psb32s,(Ipp8s*)_inputImgGRS->getPixelAddress(0,0),320,srcsize);
		//ippiCopy_8u_C1R(_inputImgGR.getPixelAddress(0,0),320,_inputImgGRS->getPixelAddress(0,0),320,srcsize);
	}
	else
		return;
	//_inputImgBY
	if(ptr_inputImgBY!=NULL){
		//_inputImgBYS->copy(*ptr_inputImgBY,320,240);
		ippiScale_8u32s_C1R(_inputImgBY.getPixelAddress(0,0),320,_inputImgBYS32,psb32s,srcsize);
		ippiConvert_32s8s_C1R(_inputImgBYS32,psb32s,(Ipp8s*)_inputImgBYS->getPixelAddress(0,0),320,srcsize);
		//ippiCopy_8u_C1R(_inputImgBY.getPixelAddress(0,0),320,_inputImgBYS->getPixelAddress(0,0),320,srcsize);
	}
	else
		return;
	int nBlobs=salience->DrawContrastLP2(*_inputImgGRS, *_inputImgGRS, *_inputImgBYS, *outContrastLP, *tagged, max_tag, salienceBU, salienceTD, searchRG, searchGR, searchBY, 255); // somma coeff pos=3 somma coeff neg=-3
	printf("The number of blobs: %d",nBlobs);
	salience->ComputeMeanColors(max_tag); //compute for every box the mean Red,Green and Blue Color.
	salience->DrawMeanColorsLP(*outMeanColourLP,*tagged);
	
	//__OLD//meanOppCol.Zero();
	//__OLD//salience.DrawMeanOpponentColorsLP(meanOppCol, tagged);

	/*__OLD//blobFinder.DrawGrayLP(tmp1, tagged, 200);
	//__OLD//ACE_OS::sprintf(savename, "./rain.ppm");
	//__OLD//YARPImageFile::Write(savename, tmp1);*/

	//__OLD//rain.tags2Watershed(tagged, oldWshed);

	delete(blobList);
	ippiFree(_inputImgRGS32); //Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
	ippiFree(_inputImgGRS32); //Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
	ippiFree(_inputImgBYS32); //Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
}


