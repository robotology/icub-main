#include <iCub/rgbProcessorThread.h>
#include <iCub/convert_bitdepth.h>
#include <ipps.h>

void conv_32f_to_8u(Ipp32f*im_i,int p4_,Ipp8u*im_o,int p1_,IppiSize srcsize_)
{
  Ipp32f min,max;
  ippiMinMax_32f_C1R(im_i,p4_,srcsize_,&min,&max);
  if (max==min){max=255.0;min=0.0;}
  ippiScale_32f8u_C1R(im_i,p4_,im_o,p1_,srcsize_,min,max);
}


rgbProcessorThread::rgbProcessorThread():RateThread(THREAD_RATE)
{
    reinit_flag=false;
    
    redPlane=0;    
    greenPlane=0;
    bluePlane=0;

    redGreen_yarp=0;
    greenRed_yarp=0;
    blueYellow_yarp=0;

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
    srcsize.width=width;
    srcsize.height=height;

    shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);

    redPlane=new ImageOf<PixelMono>;
    redPlane->resize(width,height);
    greenPlane=new ImageOf<PixelMono>;
    greenPlane->resize(width,height);
    bluePlane=new ImageOf<PixelMono>;
    bluePlane->resize(width,height);

    redGreen_yarp=new ImageOf<PixelMono>;
    redGreen_yarp->resize(width,height);
	greenRed_yarp=new ImageOf<PixelMono>;
    greenRed_yarp->resize(width,height);
	blueYellow_yarp=new ImageOf<PixelMono>;
    blueYellow_yarp->resize(width,height);
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
    colourOpponency();
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

void rgbProcessorThread::colourOpponency(){
    //1. Initialisation
    Ipp8u* redPlane_ippi= ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* greenPlane_ippi= ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* bluePlane_ippi= ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* yellowPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);

    Ipp8u* redGreen_ippi= ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* greenRed_ippi= ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* blueYellow_ippi= ippiMalloc_8u_C1(width,height,&psb);

    //2.1 shifts the 3 planes by one pixel on the right 
	//ippiRShiftC_8u_C1IR(1,greenPlane->getPixelAddress(0,0),width,srcsize);
	//ippiRShiftC_8u_C1IR(1,redPlane->getPixelAddress(0,0),width,srcsize);
	//ippiRShiftC_8u_C1IR(1,yellowPlane->getPixelAddress(0,0),width,srcsize);

	/*DEPRECATED bluePlane_tmp=this->RShiftC(bluePlane);
	redPlane_tmp=this->RShiftC(redPlane);
	greenPlane_tmp=this->RShiftC(greenPlane);*/
	
	/*ippiRShiftC_8u_C1R(bluePlane->getPixelAddress(0,0),width,1,bluePlane_tmp->getPixelAddress(0,0),width,srcsize);
	ippiRShiftC_8u_C1R(redPlane->getPixelAddress(0,0),width,1,redPlane_tmp->getPixelAddress(0,0),width,srcsize);
	ippiRShiftC_8u_C1R(greenPlane->getPixelAddress(0,0),width,1,greenPlane_tmp->getPixelAddress(0,0),width,srcsize);*/

	
	//2.2 convert the shifted planes to the IPP Image (unsigned 8bit)
	Ipp8u* bluePlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* redPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* yellowPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* greenPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);

	

    ippiCopy_8u_C1R(bluePlane->getRawImage(),bluePlane->getRowSize(),bluePlane_ippi,psb,srcsize);
	ippiCopy_8u_C1R(redPlane->getRawImage(),redPlane->getRowSize(),redPlane_ippi,psb,srcsize);
    ippiCopy_8u_C1R(greenPlane->getRawImage(),greenPlane->getRowSize(),greenPlane_ippi,psb,srcsize);
	
	//3. convolve with a gaussian in order to filter image planes red and green
	
	/*ippiFilterGauss_8u_C1R(bluePlane_ippi,psb,bluePlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(redPlane_ippi,psb,redPlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(greenPlane_ippi,psb,greenPlane_ippi_f,psb,srcsize,ippMskSize5x5);*/

	int psb2;
	Ipp32f* redPlane_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* bluePlane_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* greenPlane_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);

	Ipp32f* redGreen_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* blueYellow_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* greenRed_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);

	Ipp32f* redPlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* bluePlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* yellowPlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	Ipp32f* greenPlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	
    /*ippiConvert_8u32f_C1R(greenPlane->getRawImage(),greenPlane->getRowSize(),greenPlane_ippi32,psb2,srcsize);
    ippiConvert_8u32f_C1R(bluePlane->getRawImage(),bluePlane->getRowSize(),bluePlane_ippi32,psb2,srcsize);
    ippiConvert_8u32f_C1R(redPlane->getRawImage(),redPlane->getRowSize(),redPlane_ippi32,psb2,srcsize);*/

	ippiConvert_8u32f_C1R(greenPlane_ippi,psb,greenPlane_ippi32,psb2,srcsize);
	ippiConvert_8u32f_C1R(bluePlane_ippi,psb,bluePlane_ippi32,psb2,srcsize);
	ippiConvert_8u32f_C1R(redPlane_ippi,psb,redPlane_ippi32,psb2,srcsize);

	
	int pBufferSize;
	ippiFilterGaussGetBufferSize_32f_C1R(srcsize, 7, &pBufferSize);
	Ipp8u *pBufferBlue;
	pBufferBlue = ippsMalloc_8u(pBufferSize);
	Ipp8u *pBufferRed;
	pBufferRed = ippsMalloc_8u(pBufferSize);
	Ipp8u *pBufferGreen;
	pBufferGreen = ippsMalloc_8u(pBufferSize);
	
	//ippiFilter possible variables for border ippBorderConst,ippBorderRepl,ippBorderWrap,ippBorderMirror,ippBorderMirrorR
	Ipp32f value=1;
	Ipp32f sigma=2;
	int kernelSize=7;
    ippiFilterGaussBorder_32f_C1R(greenPlane_ippi32,psb2,greenPlane_ippi32_f,psb2,srcsize,kernelSize,sigma,ippBorderConst,value,pBufferGreen);
	ippiFilterGaussBorder_32f_C1R(bluePlane_ippi32,psb2,bluePlane_ippi32_f,psb2,srcsize,kernelSize,sigma,ippBorderConst,value,pBufferBlue);
	ippiFilterGaussBorder_32f_C1R(redPlane_ippi32,psb2,redPlane_ippi32_f,psb2,srcsize,kernelSize,sigma,ippBorderConst,value,pBufferRed);
	
	

	//4.1 subtract the red from the green to obtain R+G-
	ippiSub_32f_C1R(redPlane_ippi32,psb2,greenPlane_ippi32_f,psb2,redGreen_ippi32,psb2,srcsize);
	//ippiSub_8u_C1RSfs(redGreen_ippi,psb,greenPlane_ippi_f,psb,redGreen_ippi,psb,srcsize,-1);
	//4.2 subtract the green from the red to obtain G+R-
	ippiSub_32f_C1R(greenPlane_ippi32,psb2,redPlane_ippi32_f,psb2,greenRed_ippi32,psb2,srcsize);
	//ippiSub_8u_C1RSfs(greenRed_ippi,psb,redPlane_ippi_f,psb,greenRed_ippi,psb,srcsize,-1);
	//4.3 risht shift;
	//ippiRShiftC_32s_C1IR(1,greenPlane_ippi32_f,psb2,srcsize);
	//ippiRShiftC_32s_C1IR(1,redPlane_ippi32_f,psb2,srcsize);
	//4.4 sum the filtered red and filtered green to get the filtered yellow
	ippiAdd_32f_C1R(redPlane_ippi32_f,psb2,greenPlane_ippi32_f,psb2,yellowPlane_ippi32_f,psb2,srcsize);
	//ippiAdd_8u_C1RSfs(redPlane_ippi_f,psb,greenPlane_ippi_f,psb,yellowPlane_ippi_f,psb,srcsize,-1);
	//4.5. subtract the yellow from the blue in order to get B+Y-
	ippiSub_32f_C1R(bluePlane_ippi32,psb2,yellowPlane_ippi32_f,psb2,blueYellow_ippi32,psb2,srcsize);
	//ippiSub_8u_C1RSfs(yellowPlane_ippi_f,psb,blueYellow_ippi,psb,blueYellow_ippi,psb,srcsize,-1);

	//5. convesion back to 8u images 
	conv_32f_to_8u(blueYellow_ippi32,psb2,blueYellow_yarp->getRawImage(),blueYellow_yarp->getRowSize(),srcsize);
    conv_32f_to_8u(redGreen_ippi32,psb2,redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize);
    conv_32f_to_8u(greenRed_ippi32,psb2,greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),srcsize);

	/*int thresholdHigh=140;
	int thresholdLow=1;
	for (int i=0;i<320*240;i++){
			if(greenRed_ippi[i]==255) 
				greenRed_ippi[i]=0;
			else if((greenRed_ippi[i]>thresholdLow)&&(greenRed_ippi[i]<thresholdHigh)) 
				greenRed_ippi[i]=0;
			else if(greenRed_ippi[i]<=thresholdLow) 
				greenRed_ippi[i]=255;

			if(blueYellow_ippi[i]==255)
				blueYellow_ippi[i]=0;
			else if((blueYellow_ippi[i]>thresholdLow)&&(blueYellow_ippi[i]<thresholdHigh)) 
				blueYellow_ippi[i]=0;
			else if(blueYellow_ippi[i]<=thresholdLow) 
				blueYellow_ippi[i]=255;

			if(redGreen_ippi[i]==255) 
				redGreen_ippi[i]=0;	
			else if((redGreen_ippi[i]>thresholdLow)&&(redGreen_ippi[i]<thresholdHigh)) 
				redGreen_ippi[i]=0;
			else if(redGreen_ippi[i]<=thresholdLow) 
				redGreen_ippi[i]=255;		
	}*/


	
	
	//delete tmp;
	
	/*
	//4.1 subtract the red from the green to obtain R+G-
	ippiSub_8u_C1RSfs(redPlane_ippi,psb,greenPlane_ippi_f,psb,redGreen_ippi,psb,srcsize,0);
	//ippiSub_8u_C1RSfs(redGreen_ippi,psb,greenPlane_ippi_f,psb,redGreen_ippi,psb,srcsize,-1);
	//4.2 subtract the green from the red to obtain G+R-
	ippiSub_8u_C1RSfs(greenPlane_ippi,psb,redPlane_ippi_f,psb,greenRed_ippi,psb,srcsize,0);
	//ippiSub_8u_C1RSfs(greenRed_ippi,psb,redPlane_ippi_f,psb,greenRed_ippi,psb,srcsize,-1);
	//4.3 risht shift;
	ippiRShiftC_8u_C1IR(1,greenPlane_ippi_f,psb,srcsize);
	ippiRShiftC_8u_C1IR(1,redPlane_ippi_f,psb,srcsize);
	//4.4 sum the filtered red and filtered green to get the filtered yellow
	ippiAdd_8u_C1RSfs(redPlane_ippi_f,psb,greenPlane_ippi_f,psb,yellowPlane_ippi_f,psb,srcsize,1);
	ippiCopy_8u_C1R(yellowPlane_ippi_f,psb,this->yellowPlane->getPixelAddress(0,0),320,srcsize);
	//ippiAdd_8u_C1RSfs(redPlane_ippi_f,psb,greenPlane_ippi_f,psb,yellowPlane_ippi_f,psb,srcsize,-1);
	//4.5. subtract the yellow from the blue in order to get B+Y-
	ippiSub_8u_C1RSfs(bluePlane_ippi,psb,yellowPlane_ippi_f,psb,blueYellow_ippi,psb,srcsize,0);
	//ippiSub_8u_C1RSfs(yellowPlane_ippi_f,psb,blueYellow_ippi,psb,blueYellow_ippi,psb,srcsize,-1);

	*/
	
	//7. back conversion to YarpImages
	//ImageOf<PixelMono> *redGreen_yarp=new ImageOf<PixelMono>;
	//ImageOf<PixelMono> *greenRed_yarp=new ImageOf<PixelMono>;
	//ImageOf<PixelMono> *blueYellow_yarp=new ImageOf<PixelMono>;
	//redGreen_yarp->resize(width,height);
	//greenRed_yarp->resize(width,height);
	//blueYellow_yarp->resize(width,height);
	
    /*ippiCopy_8u_C1R(redGreen_ippi,psb,redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize);
	ippiCopy_8u_C1R(greenRed_ippi,psb,greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),srcsize);
	ippiCopy_8u_C1R(blueYellow_ippi,psb,blueYellow_yarp->getRawImage(),blueYellow_yarp->getRowSize(),srcsize);*/

	/*ippiCopy_8u_C1R(redPlane->getPixelAddress(0,0),psb,redGreen_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(greenPlane->getPixelAddress(0,0),psb,greenRed_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(bluePlane->getPixelAddress(0,0),psb,blueYellow_yarp->getPixelAddress(0,0),width,srcsize);*/
    
	/*blueYellow_yarp=bluePlane;
	redGreen_yarp=bluePlane;
	greenRed_yarp=bluePlane;*/

	/*this->blueYellow_flag=1;
	this->redGreen_flag=1;
	this->greenRed_flag=1;*/

    //6. free memory space

    ippiFree(bluePlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(redPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree (greenPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
    ippiFree(yellowPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);

    ippiFree(redGreen_ippi);
	ippiFree(blueYellow_ippi);
	ippiFree(greenRed_ippi);

    ippiFree(bluePlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(redPlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(yellowPlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(greenPlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);

	ippiFree(redPlane_ippi32_f);
	ippiFree(bluePlane_ippi32_f);
	ippiFree(greenPlane_ippi32_f);
	ippiFree(yellowPlane_ippi32_f);

	ippiFree(redGreen_ippi32);
	ippiFree(blueYellow_ippi32);
	ippiFree(greenRed_ippi32);
	
	ippiFree(redPlane_ippi32);
	ippiFree(bluePlane_ippi32);
	ippiFree(greenPlane_ippi32);
	
	ippsFree(pBufferBlue);
	ippsFree(pBufferRed);
	ippsFree(pBufferGreen);
}
