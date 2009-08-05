// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/fingerImageThread.h>

const int SNIFFER_THREAD_RATE=50;
const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=512;
BufferedPort<Bottle> port;
int sensor [12];


/**
* initialise the thread
*/
bool fingerImageThread::threadInit(){
	redraw = false;
	for (int i=0; i<12; i++) sensor[i] = 0;

	cvImage= cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 ); //200, 200
	image2=new ImageOf<PixelRgb>;
	image2->resize(320,240);
	printf("Image Thread initialising.....");

	Property prop;
    prop.put("device", "ecan");
    prop.put("CanTxTimeout", 500);
    prop.put("CanRxTimeout", 500);
    prop.put("CanDeviceNum", 0);
    prop.put("CanMyAddress", 0);
    prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    driver.open(prop);
    if (!driver.isValid())
    {
        fprintf(stderr, "Error opening PolyDriver check parameters\n");
        return false;
    }
    driver.view(iCanBus);
    driver.view(iBufferFactory);
    iCanBus->canSetBaudRate(0); //default 1MB/s
	iCanBus->canIdAdd(0x300);
	iCanBus->canIdAdd(0x301);
    readBuffer=iBufferFactory->createBuffer(localBufferSize);
    return true;
}

/**
* code that is executed after the thread starts
* @param s is true if the thread started
*/
void fingerImageThread::afterStart(bool s){
	PixelRgb black(0,0,0);
	addRectangle(*image2,black,0,0,320,240); //background //200, 200
}

/* running code of the thread */
void fingerImageThread::run(){
	
	unsigned int messages=localBufferSize;
    unsigned int readMessages=0;
    bool res=iCanBus->canRead(readBuffer, messages, &readMessages);

	for (unsigned int i=0; i<readMessages; i++)
	{
		if (readBuffer[i].getId()==0x300)
		{
			sensor[0]=readBuffer[i].getData()[1];
			sensor[1]=readBuffer[i].getData()[2];
			sensor[2]=readBuffer[i].getData()[3];
			sensor[3]=readBuffer[i].getData()[4];
			sensor[4]=readBuffer[i].getData()[5];
			sensor[5]=readBuffer[i].getData()[6];
			sensor[6]=readBuffer[i].getData()[7];

			for (int i=0; i<7; i++) {
				sensor[i] =  256 - 2 * sensor[i];
				if (sensor[i] < 0) sensor[i] = 0;
			}
		}
		else if (readBuffer[i].getId()==0x301)
		{
			sensor[7]= readBuffer[i].getData()[1];
			sensor[8]= readBuffer[i].getData()[2];
			sensor[9]= readBuffer[i].getData()[3];
			sensor[10]=readBuffer[i].getData()[4];
			sensor[11]=readBuffer[i].getData()[5];

			for (int i=7; i<12; i++) {
				sensor[i] =  256 - 2 * sensor[i];
				if (sensor[i] < 0) sensor[i] = 0;
			}
		}
	}

	for (int i=1; i<12; i++)	
		printf("%d ", sensor[i]);			
	printf("\n");

	PixelRgb black(0,0,0);
	int medPointX=160;	
	redraw = false;
	addCircle(*image2,PixelRgb(sensor[4],sensor[4],0),medPointX-41,100,80);
	addCircle(*image2,PixelRgb(sensor[7],sensor[7],0),medPointX+41,100,80);
	addRectangle(*image2,black,medPointX,61,42,160);
	addRectangle(*image2,black,medPointX,123,120,21);
	addRectangle(*image2,PixelRgb(sensor[5],sensor[5],0),medPointX-21,61,20,40);
	addRectangle(*image2,PixelRgb(sensor[6],sensor[6],0),medPointX+21,61,20,40);
	addRectangle(*image2,PixelRgb(sensor[3],sensor[3],0),medPointX-61,123,60,20);
	addRectangle(*image2,PixelRgb(sensor[8],sensor[8],0),medPointX+61,123,60,20);
	addRectangle(*image2,PixelRgb(sensor[0],sensor[0],0),medPointX-21,186,20,41);
	addRectangle(*image2,PixelRgb(sensor[11],sensor[11],0),medPointX+21,186,20,41);
	addRectangle(*image2,PixelRgb(sensor[2],sensor[2],0),medPointX-82,165,39,20);
	addRectangle(*image2,PixelRgb(sensor[9],sensor[9],0),medPointX+82,165,39,20);
	addRectangle(*image2,PixelRgb(sensor[1],sensor[1],0),medPointX-82,207,39,20);
	addRectangle(*image2,PixelRgb(sensor[10],sensor[10],0),medPointX+82,207,39,20);
	redraw = true;
}

/**
* code executed when the thread is released
*/
void fingerImageThread::threadRelease(){
	printf("Image Thread releasing.....");	
	iBufferFactory->destroyBuffer(readBuffer);
    driver.close();
}
