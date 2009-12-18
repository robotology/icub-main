#include <iCub/rgbProcessorThread.h>


rgbProcessorThread::rgbProcessorThread():RateThread(THREAD_RATE)
{
    reinit_flag=false;
    
    redPlane=0;    
    greenPlane=0;
    bluePlane=0;

    img=0;
}

rgbProcessorThread::~rgbProcessorThread()
{
    delete redPlane;
    delete greenPlane;
    delete bluePlane;    
}

/*processorThread::processorThread(Property &op):processorThread(){
        
        
}*/

void rgbProcessorThread::reinitialise(){
    
    shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);
    redPlane=new ImageOf<PixelMono>;
    redPlane->resize(width,height);
    greenPlane=new ImageOf<PixelMono>;
    greenPlane->resize(width,height);
    bluePlane=new ImageOf<PixelMono>;
    bluePlane->resize(width,height);
}

bool rgbProcessorThread::threadInit(){
    printf("Thread initialisation.. \n");
    
    return true;
}


void rgbProcessorThread::run(){
    if(img==0){
        return;	
    }
    /*width=img->width();
    height=img->height();*/
    /*if(!reinit_flag){
        reinitialise();
        reinit_flag=true;
    }*/
    extractPlanes(img);

}
   


void rgbProcessorThread::threadRelease(){
    printf("Thread releasing.. \n");
}



void rgbProcessorThread::setInputImage(ImageOf<PixelRgb>* inputImage){
    this->img=inputImage;
    this->width=inputImage->width();
    this->height=inputImage->height();
    reinitialise();
}

/*
 * extracts the plane red for the input image
 */
void rgbProcessorThread::getRedPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp){
    
	
	//int width=inputImage->width();
	//int height=inputImage->height();
   
	IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();

	//Ipp8u* shift[3];
	/*shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);*/
	//ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
	ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
    ippiCopy_8u_C1R(shift[0],psb,tmp->getRawImage(),tmp->getRowSize(),srcsize);
	/*ippiFree(shift[0]);
	ippiFree(shift[1]);
	ippiFree(shift[2]);*/
}

/*
 * extracts the plane green for the input image
 */
void rgbProcessorThread::getGreenPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp){
    
	
	//int width=inputImage->width();
	//int height=inputImage->height();
    
	
	
	IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();

	//Ipp8u* shift[3];
	/*shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);*/
	//ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
	ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
	ippiCopy_8u_C1R(shift[1],psb,tmp->getRawImage(),tmp->getRowSize(),srcsize);
	/*ippiFree(shift[0]);
	ippiFree(shift[1]);
	ippiFree(shift[2]);*/
}

/*
 * extracts the plane blue for the input image
 */
void rgbProcessorThread::getBluePlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp){
    
	
	//int width=inputImage->width();
	//int height=inputImage->height();
   		
	IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();

	//Ipp8u* shift[3];
	/*shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);*/
	//ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
	ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
	ippiCopy_8u_C1R(shift[2],psb,tmp->getRawImage(),tmp->getRowSize(),srcsize);
	/*ippiFree(shift[0]);
	ippiFree(shift[1]);
	ippiFree(shift[2]);*/
}

void rgbProcessorThread::extractPlanes(ImageOf<PixelRgb>* src){
    IppiSize srcsize ={width,height};

	//tmp=new ImageOf<PixelMono>;
	//tmp->resize(width,height);

    //1.get the red,blue and green planes
	this->getBluePlane(src,bluePlane);	
	//ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),bluePlane->getRawImage(),bluePlane->getRowSize(),srcsize);
	

	this->getRedPlane(src,redPlane);
	//ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),redPlane->getRawImage(),redPlane->getRowSize(),srcsize);

	this->getGreenPlane(src,greenPlane);
	//ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),greenPlane->getRawImage(),greenPlane->getRowSize(),srcsize);
}
