// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/ImageProcessor.h>



// available methods for edges detection
//#define CONVMAX
//#define CONVFILTER
//#define IPPISOBEL
#define OPENCVSOBEL
//#define CONVSEQ

ImageProcessor::ImageProcessor(){
	this->inImage=NULL;
	portImage=NULL;
	inputImage_flag=0;
	redPlane_flag=1;
	bluePlane_flag=0;
	greenPlane_flag=0;
	yellowPlane_flag=0;
	blueYellow_flag=0;
	greenRed_flag=0;
	redGreen_flag=0;

	colourOpponency_flag=1;
	findEdges_flag=1;
	normalize_flag=0;
	combineMax_flag=1;
	width=320;
	height=240;
	int psb;
	IppiSize srcsize={320,240};

	portImage=new ImageOf<PixelRgb>;
	this->portImage->resize(320,240);

	edges_yarp=new ImageOf<PixelMono>;
	edges_yarp->resize(320,240);

	redPlane=new ImageOf<PixelMono>;
	redPlane->resize(320,240);
	greenPlane=new ImageOf<PixelMono>;
	greenPlane->resize(320,240);
	bluePlane=new ImageOf<PixelMono>;
	bluePlane->resize(320,240);
	yellowPlane=new ImageOf<PixelMono>;
	yellowPlane->resize(320,240);

	redPlane_tmp=new ImageOf<PixelMono>;
	redPlane_tmp->resize(320,240);
	greenPlane_tmp=new ImageOf<PixelMono>;
	greenPlane_tmp->resize(320,240);
	bluePlane_tmp=new ImageOf<PixelMono>;
	bluePlane_tmp->resize(320,240);
	
	
	red_yarp=new ImageOf<PixelMono>;
	red_yarp->resize(320,240);
	green_yarp=new ImageOf<PixelMono>;
	green_yarp->resize(320,240);
	blue_yarp=new ImageOf<PixelMono>;
	blue_yarp->resize(320,240);

	redGreenEdges_yarp=new ImageOf<PixelMono>;
	redGreenEdges_yarp->resize(320,240);
	greenRedEdges_yarp=new ImageOf<PixelMono>;
	greenRedEdges_yarp->resize(320,240);
	blueYellowEdges_yarp=new ImageOf<PixelMono>;
	blueYellowEdges_yarp->resize(320,240);

	edgesBlue=new ImageOf<PixelMono>;
	//edgesBlue->resize(width,height);
	edgesRed=new ImageOf<PixelMono>;
	//edgesRed->resize(width,height);
	edgesGreen=new ImageOf<PixelMono>;
	//edgesGreen->resize(width,height);

	redGreen_ippi = ippiMalloc_8u_C1(width,height,&psb);
	greenRed_ippi = ippiMalloc_8u_C1(width,height,&psb);
	blueYellow_ippi = ippiMalloc_8u_C1(width,height,&psb);
	bluePlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
	redPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
	greenPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);

	redGreen_yarp=new ImageOf<PixelMono>;
	redGreen_yarp->resize(320,240);
	greenRed_yarp=new ImageOf<PixelMono>;
	greenRed_yarp->resize(320,240);
	blueYellow_yarp=new ImageOf<PixelMono>;
	blueYellow_yarp->resize(320,240);

	tmp=new ImageOf<PixelMono>;
	tmp->resize(320,240);
	image_out=new ImageOf<PixelRgb>;
	image_out->resize(320,240);
	image_tmp=new ImageOf<PixelMono>;
	image_tmp->resize(320,240);
	outputImage=new ImageOf<PixelMono>;
	outputImage->resize(320,240);

	cvImage= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,1);

	this->cannyOperator=new CANNY(srcsize);
	
	src0[0]= 1;src0[1]= 2;src0[2]=1;
	src0[3]= 0;src0[4]= 0;src0[5]=0;
	src0[6]=-1;src0[7]=-2;src0[8]=1;
	
	src1[0]=2;src1[1]= 1;src1[2]= 0;
	src1[3]=1;src1[4]= 0;src1[5]=-1;
	src1[6]=0;src1[7]=-1;src1[8]=-2;	

	src2[0]=1;src2[1]=0;src2[2]=-1;
	src2[3]=2;src2[4]=0;src2[5]=-2;
	src2[6]=1;src2[7]=0,src2[8]=-1;

	src3[0]=0;src3[1]=-1;src3[2]=-2;
	src3[3]=1;src3[4]= 0;src3[5]=-1;
	src3[6]=2;src3[7]= 1,src3[8]= 0;

	src4[0]=-1;src4[1]=-2;src4[2]=-1;
	src4[3]= 0;src4[4]= 0;src4[5]= 0;
	src4[6]= 1;src4[7]= 2;src4[8]= 1;

	src5[0]=-2;src5[1]=-1;src5[2]=0;
	src5[3]=-1;src5[4]= 0;src5[5]=1;
	src5[6]= 0;src5[7]= 1;src5[8]=2;

	src6[0]=-1;src6[1]=0;src6[2]=1;
	src6[3]=-2;src6[4]=0;src6[5]=2;
	src6[6]=-1;src6[5]=0;src6[8]=1;

	src7[0]= 0;src7[1]= 1;src7[2]=2;
	src7[3]=-1;src7[4]= 0;src7[5]=1;
	src7[6]=-2;src7[7]=-1;src7[8]=0;
		
}

ImageProcessor::ImageProcessor(ImageOf<PixelRgb>* inputImage){
	this->inImage=inputImage;
	this->portImage=portImage;
	redPlane_flag=1;
	bluePlane_flag=0;
	greenPlane_flag=0;
	blueYellow_flag=0;
	greenRed_flag=0;
	redGreen_flag=0;

	colourOpponency_flag=0;
	findEdges_flag=0;
	normalize_flag=0;
	combineMax_flag=0;

	IppiSize srcsize={320,240};
	
	portImage=new ImageOf<PixelRgb>;
	this->portImage->resize(320,240);

	redGreen_yarp=new ImageOf<PixelMono>;
	redGreen_yarp->resize(320,240);
	greenRed_yarp=new ImageOf<PixelMono>;
	greenRed_yarp->resize(320,240);
	blueYellow_yarp=new ImageOf<PixelMono>;
	blueYellow_yarp->resize(320,240);

	this->cannyOperator=new CANNY(srcsize);
}

/*function that gets the 3 images in the colourOpponency space R+G-,G+R-,B+Y-
	the standard indicates that redGreen means R+G-, whereas greenRed means G+R-;
*/
void ImageProcessor::colourOpponency(ImageOf<PixelRgb> *src){
	int width=src->width();
	int height=src->height();
	IppiSize srcsize ={320,240};
	
	/*if((unsigned int)bluePlane==0xcdcdcdcd){
		bluePlane=new ImageOf<PixelMono>;
		bluePlane->resize(width,height);
	}
	else	
		printf("bluePlane1: 0x%08x\n", bluePlane);

	if((unsigned int)redPlane==0xcdcdcdcd){
		redPlane=new ImageOf<PixelMono>;
		redPlane->resize(width,height);
	}
	else
		printf("redPlane1: 0x%08x\n", redPlane);

	if((unsigned int)greenPlane==0xcdcdcdcd){
		greenPlane=new ImageOf<PixelMono>;
		greenPlane->resize(width,height);
	}
	else
		printf("greenPlane1: 0x%08x\n", greenPlane);*/
	
	//bluePlane->resize(width,height);
	//redPlane->resize(width,height);
	//greenPlane->resize(width,height);

	//tmp=new ImageOf<PixelMono>;
	//tmp->resize(width,height);
	
	/*redPlane_tmp=new ImageOf<PixelMono>;
	redPlane_tmp->resize(width,height);
	greenPlane_tmp=new ImageOf<PixelMono>;
	greenPlane_tmp->resize(width,height);*/

	int psb;
	int psb4;
	
	
	//1.get the red,blue and green planes
	this->getBluePlane(src,tmp);	
	//printf("tmp: 0x%08x\n", tmp);
	ippiCopy_8u_C1R(tmp->getPixelAddress(0,0),width,bluePlane->getPixelAddress(0,0),width,srcsize);
	//printf("bluePlane2: 0x%08x\n", bluePlane);

	this->getRedPlane(src,tmp);
	//printf("tmp: 0x%08x\n", tmp);
	ippiCopy_8u_C1R(tmp->getPixelAddress(0,0),width,redPlane->getPixelAddress(0,0),width,srcsize);
	//printf("redPlane2: 0x%08x\n", redPlane);

	this->getGreenPlane(src,tmp);
	//printf("tmp: 0x%08x\n", tmp);
	ippiCopy_8u_C1R(tmp->getPixelAddress(0,0),width,greenPlane->getPixelAddress(0,0),width,srcsize);
	//printf("greenPlane2: 0x%08x\n", greenPlane);
	

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
	Ipp8u* yellowPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* greenPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	

	ippiCopy_8u_C1R(bluePlane->getPixelAddress(0,0),width,bluePlane_ippi,psb,srcsize);
	ippiCopy_8u_C1R(redPlane->getPixelAddress(0,0),width,redPlane_ippi,psb,srcsize);
	ippiCopy_8u_C1R(greenPlane->getPixelAddress(0,0),width,greenPlane_ippi,psb,srcsize);
	
	//3. convolve with a gaussian in order to filter image planes red and green
	
	/*ippiFilterGauss_8u_C1R(bluePlane_ippi,psb,bluePlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(redPlane_ippi,psb,redPlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(greenPlane_ippi,psb,greenPlane_ippi_f,psb,srcsize,ippMskSize5x5);*/

	int psb2;
	redPlane_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	bluePlane_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	greenPlane_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);

	redGreen_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	blueYellow_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);
	greenRed_ippi32 = ippiMalloc_32f_C1(width,height,&psb2);

	redPlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	bluePlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	yellowPlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	greenPlane_ippi32_f = ippiMalloc_32f_C1(width,height,&psb2);
	
	
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
	conv_32f_to_8u(blueYellow_ippi32,psb2,blueYellow_ippi,psb,srcsize);
	conv_32f_to_8u(redGreen_ippi32,psb2,redGreen_ippi,psb,srcsize);
	conv_32f_to_8u(greenRed_ippi32,psb2,greenRed_ippi,psb,srcsize);

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


	//6. free memory space
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
	
	ippiCopy_8u_C1R(redGreen_ippi,psb,redGreen_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(greenRed_ippi,psb,greenRed_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(blueYellow_ippi,psb,blueYellow_yarp->getPixelAddress(0,0),width,srcsize);
	/*ippiCopy_8u_C1R(redPlane->getPixelAddress(0,0),psb,redGreen_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(greenPlane->getPixelAddress(0,0),psb,greenRed_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(bluePlane->getPixelAddress(0,0),psb,blueYellow_yarp->getPixelAddress(0,0),width,srcsize);*/

	//ippiFree(bluePlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	//ippiFree(redPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	//ippiFree (greenPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	
	ippiFree(bluePlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(redPlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(yellowPlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(yellowPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(greenPlane_ippi_f); // ippiMalloc_8u_C1(width,height,&psb);
	
    
	/*blueYellow_yarp=bluePlane;
	redGreen_yarp=bluePlane;
	greenRed_yarp=bluePlane;*/
}

/*function that gets the 3 images in the colourOpponency space R+G-,G+R-,B+Y-
	the standard indicates that redGreen means R+G-, whereas greenRed means G+R-;
*/
void ImageProcessor::colourOpponency(ImageOf<PixelRgb> *src,ImageOf<PixelMono> *redGreen_yarp,ImageOf<PixelMono> *greenRed_yarp,ImageOf<PixelMono> *blueYellow_yarp){
	/*int width=src->width();
	int height=src->height();
	IppiSize srcsize ={width,height};
	
	ImageOf<PixelMono> *bluePlane=new ImageOf<PixelMono>;
	ImageOf<PixelMono> *redPlane=new ImageOf<PixelMono>;
	ImageOf<PixelMono> *greenPlane=new ImageOf<PixelMono>;
	bluePlane->resize(width,height);
	redPlane->resize(width,height);
	greenPlane->resize(width,height);

	ImageOf<PixelMono> *bluePlane_tmp=new ImageOf<PixelMono>;
	ImageOf<PixelMono> *redPlane_tmp=new ImageOf<PixelMono>;
	ImageOf<PixelMono> *greenPlane_tmp=new ImageOf<PixelMono>;
	bluePlane_tmp->resize(width,height);
	redPlane_tmp->resize(width,height);
	greenPlane_tmp->resize(width,height);

	int psb;
	int psb4;
	
	//1.get the red,blue and green planes
	this->getBluePlane(src,bluePlane);
	if(bluePlane==NULL){
		printf("BluePlane NULL!");
		return;
	}
	this->getRedPlane(src,redPlane);
	if(redPlane==NULL){
		printf("redPlane NULL!");
		return;
	}
	this->getGreenPlane(src,greenPlane);
	if(greenPlane==NULL){
		printf("greenPlane NULL!");
		return;
	}
	

	//2.1 shifts the 3 planes by one pixel on the right ????
	//bluePlane_tmp=this->RShiftC(bluePlane);
	//redPlane_tmp=this->RShiftC(redPlane);
	//greenPlane_tmp=this->RShiftC(greenPlane);
	
	
	//2.2 convert the shifted planes to the IPP Image (unsigned 8bit)
	Ipp8u* bluePlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* redPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* greenPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* bluePlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* redPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* yellowPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* greenPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	redGreen_ippi = ippiMalloc_8u_C1(width,height,&psb);
	greenRed_ippi = ippiMalloc_8u_C1(width,height,&psb);
	blueYellow_ippi = ippiMalloc_8u_C1(width,height,&psb);

	ippiCopy_8u_C1R(bluePlane->getPixelAddress(0,0),width,bluePlane_ippi,psb,srcsize);
	ippiCopy_8u_C1R(redPlane->getPixelAddress(0,0),width,redPlane_ippi,psb,srcsize);
	ippiCopy_8u_C1R(greenPlane->getPixelAddress(0,0),width,greenPlane_ippi,psb,srcsize);

	
	//3. convolve with a gaussian in order to filter image planes red and green
	ippiFilterGauss_8u_C1R(bluePlane_ippi,psb,bluePlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(redPlane_ippi,psb,redPlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(greenPlane_ippi,psb,greenPlane_ippi_f,psb,srcsize,ippMskSize5x5);
	
	
	//4.1 subtract the red from the green to obtain G+R-
	ippiSub_8u_C1RSfs(redPlane_ippi,psb,greenPlane_ippi_f,psb,redGreen_ippi,psb,srcsize,0.5);
	//4.2 subtract the green from the red to obtain R+G-
	ippiSub_8u_C1RSfs(greenPlane_ippi_f,psb,redPlane_ippi,psb,greenRed_ippi,psb,srcsize,0.5);
	//4.3 sum the filtered red and filtered green to get the filtered yellow
	ippiAdd_8u_C1RSfs(redPlane_ippi_f,psb,greenPlane_ippi_f,psb,yellowPlane_ippi_f,psb,srcsize,0.5);
	//4.4. subtract the yellow from the blue in order to get B+Y-
	ippiSub_8u_C1RSfs(bluePlane_ippi_f,psb,yellowPlane_ippi_f,psb,blueYellow_ippi,psb,srcsize,0.5);
	
	//5. back conversion to YarpImages
	//ImageOf<PixelMono> *redGreen_yarp=new ImageOf<PixelMono>;
	//ImageOf<PixelMono> *greenRed_yarp=new ImageOf<PixelMono>;
	//ImageOf<PixelMono> *blueYellow_yarp=new ImageOf<PixelMono>;
	//redGreen_yarp->resize(width,height);
	//greenRed_yarp->resize(width,height);
	//blueYellow_yarp->resize(width,height);
	
	ippiCopy_8u_C1R(redGreen_ippi,psb,redGreen_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(greenRed_ippi,psb,greenRed_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(blueYellow_ippi,psb,blueYellow_yarp->getPixelAddress(0,0),width,srcsize);*/
}


/*ImageOf<PixelRgb>* ImageProcessor::findEdges(ImageOf<PixelRgb>* inputImage,int x_order,int y_order){
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=inputImage->width();
	int height=inputImage->height();
	ImageOf<PixelRgb>* outputImage=new ImageOf<PixelRgb>;
	outputImage->resize(width,height);
	IppiSize srcsize ={width,height};
	//memory allocation	
	int psb;
	int psb4;
	Ipp8u* colour_in = ippiMalloc_8u_C4(width,height,&psb4);
	Ipp8u* colour_tmp = ippiMalloc_8u_C4(width,height,&psb4);
	ippiCopy_8u_C3AC4R(inputImage->getPixelAddress(0,0),width*3,colour_in,psb4,srcsize);
	ippiCopy_8u_AC4C3R(colour_in,psb4,outputImage->getPixelAddress(0,0),width*3,srcsize);
	return outputImage;
}*/

ImageOf<PixelMono>* ImageProcessor::findEdges(){
	int psb;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->redGreen_yarp->width();
	int height=this->redGreen_yarp->height();
	IppiSize srcsize ={width,height};
	//2. convolve every colour opponency with the sobel in order to get edges
	Ipp8u src2[3*3]={0, 1 ,2,-1,0,1,-2,-1,0};
	Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputGreenRed = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputBlueYellow = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 4, 4 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));
	ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//5. save the output into the outputImage

	ImageOf<PixelMono>* outputEdges=new ImageOf<PixelMono>;
	outputEdges->resize(width,height);
	ippiCopy_8u_C1R(outputRedGreen,width,redGreenEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(outputGreenRed,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(outputBlueYellow,width,blueYellowEdges_yarp->getPixelAddress(0,0),width,srcsize);
	return outputEdges;
}

/*ImageOf<PixelMono>* ImageProcessor::findEdgesBlueOpponency(){
	int psb;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->redGreen_yarp->width();
	int height=this->redGreen_yarp->height();
	IppiSize srcsize ={width,height};
	//2. convolve every colour opponency with the sobel in order to get edges
	Ipp8u src2[3*3]={0, 1 ,2,-1,0,1,-2,-1,0};
	//Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	//Ipp8u* outputGreenRed = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* output = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 4, 4 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//------------------------------
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));
	//ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	//--------->   ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//5. save the output into the outputImage
	//--------------------
	
	this->cannyOperator->proc(blueYellow_ippi,width,40,50);
	output=this->cannyOperator->get_edgemap();
	IppiMorphState* ppState;
	Ipp8u pMask5x5[5*5] =
	   {1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1};
	Ipp8u pMask3x3[3*3] =
	   {0, 1, 0,
	    1, 1, 1,
		0, 1, 0,};
	IppiSize maskSize5x5 = {5, 5};
	IppiSize maskSize3x3 = {3, 3};
	IppiPoint anchor = {1, 1};
	ippiMorphologyInitAlloc_8u_C1R( srcsize.width, pMask3x3, maskSize3x3, anchor,&ppState);
	//ippiDilate3x3_8u_C1IR(output,psb,srcsize);
	//ippiDilate_8u_C1IR(output,psb,srcsize,pMask,maskSize,anchor);
	//ippiDilateBorderReplicate_8u_C1R(output,psb,outputImage->getPixelAddress(0,0),width,srcsize,ippBorderRepl, ppState);
	//ImageOf<PixelMono>* edges=new ImageOf<PixelMono>;
	//edges->resize(width,height);
	//ippiCopy_8u_C1R(outputRedGreen,width,redGreenEdges_yarp->getPixelAddress(0,0),width,srcsize);
	//ippiCopy_8u_C1R(outputGreenRed,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(output,width,outputImage->getPixelAddress(0,0),width,srcsize);
	return outputImage;
}*/

ImageOf<PixelMono>* ImageProcessor::findEdgesBlueOpponency(){
	int psb,psb32;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->blueYellow_yarp->width();
	int height=this->blueYellow_yarp->height();
	IppiSize srcsize ={width,height};
	IppiSize dstsize ={width-2,height-2};
	//2. convolve every colour opponency with the sobel in order to get edges
	/*Ipp32s src0[3*3]={1, 2 ,1,
					 0, 0, 0,
					-1,-2, 1};
	Ipp32s src1[3*3]={2, 1 ,0,
					 1, 0, -1,
					 0,-1, -2};	
	Ipp32s src2[3*3]={1, 0 ,-1,
					 2, 0, -2,
					 1, 0, -1};
	Ipp32s src3[3*3]={0, -1 ,-2,
					 1,  0, -1,
					 2,  1,  0};
	Ipp32s src4[3*3]={-1,-2,-1,
					  0, 0, 0,
					  1, 2, 1};
	Ipp32s src5[3*3]={-2, -1,0,
					 -1, 0, 1,
				 	  0, 1, 2};
	Ipp32s src6[3*3]={-1, 0 ,1,
					 -2, 0, 2,
					 -1, 0, 1};
	Ipp32s src7[3*3]={0, 1 ,2,
					-1, 0, 1,
					-2,-1, 0};*/
	
	Ipp32f* inputBlueYellow32 = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp32f* outputBlueYellow32 = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp8u* outputBlueYellow = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputBlueYellow2 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputBlueYellow3 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp32f* outputBlueYellow32B = ippiMalloc_32f_C1(width,height,&psb32);

	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 5, 5 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	IppiPoint anchor={0,0};
	//3.Convolute with the sobel operator
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));

	
	//conversion of the input image into 32f image
	//conv_8u_to_32f(blueYellow_yarp->getPixelAddress(0,0),width,inputBlueYellow32,psb32,srcsize);
	//ippiConvert_8u32f_C1R(blueYellow_yarp->getPixelAddress(0,0),width,inputBlueYellow32,psb32,srcsize);

#ifdef CONVSEQ
	//convolution of the previous convolution
	int CONVSEQ_TH=700;
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src0,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src1,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src2,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src3,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src4,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src5,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src6,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src7,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
#endif

#ifdef CONVMAX
	int CONVMAX_TH=700;
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src0,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src1,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src2,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src3,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src4,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src5,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src6,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
	ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src7,3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow32[i]<outputBlueYellow32B[i])
			outputBlueYellow32[i]=outputBlueYellow32B[i];
	}
#endif

#ifdef CONVFILTER
	int CONVFILTER_TH=20;
	//ippiFilterSobelHoriz_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	//ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen,psb,srcsize);
	//ippiFilter_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,dstsize,src1,src2Size,anchor);
	//ippiFilterGauss_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize5x5);
	//ippiFilterGauss_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize,ippMskSize5x5);
	//ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize);

	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow,psb,dstsize,src0,src2Size,anchor,CONVFILTER_TH);
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src1,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src2,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src3,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src4,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src5,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src6,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}
	ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,src7,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}

	//ippiCopy_32f_C1R(inputRedGreen32,psb32,outputBlueYellow32B,psb32,srcsize);
#endif
#ifdef IPPISOBEL
	

	ippiFilterSobelHoriz_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow3,psb,srcsize);
	ippiFilterSobelVert_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,srcsize);
	
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow3==NULL){
			printf("outputBlueYellow3 NULL");
			break;
		}
		if(outputBlueYellow2==NULL){ 
			printf("outputBlueYellow2 NULL");
			break;
		}
		if(outputBlueYellow3[i]<outputBlueYellow2[i])
			outputBlueYellow3[i]=outputBlueYellow2[i];
	}

	IppiSize msksize={3,3};
	Ipp8u src[3*3]={1,1,1,1,1,1,1,1,1};
	ippiConvValid_8u_C1R(outputBlueYellow3,psb,srcsize,src,3,msksize,&outputBlueYellow[1 + width*1],psb,10);
	/*conv_8u_to_32f(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow32,psb32,srcsize);
	ippiFilterSobelCross_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize,ippMskSize3x3);
	conv_32f_to_8u(outputBlueYellow32B,psb32,outputBlueYellow2,psb,srcsize);
	for(int i=0; i<320*240;i++){
		if(outputBlueYellow[i]<outputBlueYellow2[i])
			outputBlueYellow[i]=outputBlueYellow2[i];
	}*/
#endif
#ifdef OPENCVSOBEL
	IppiSize msksize={3,3};
	if(blueYellow_flag)
		cvSobel(blueYellow_yarp->getIplImage(),cvImage,1,1,3);
	else
		cvImage=new IplImage();
	ippiCopy_8u_C1R((unsigned char*)cvImage->imageData,width,outputBlueYellow2,psb,srcsize);
	Ipp8u src[3*3]={1,4,1,
					4,20,4,
					1,4,1};
	ippiConvValid_8u_C1R(outputBlueYellow2,psb,srcsize,src,3,msksize,&outputBlueYellow[1 + width*1],psb,1);
	//ippiFilterMedian_8u_C1R((unsigned char*)cvImage->imageData,width,outputBlueYellow,psb,srcsize,msksize,anchor);
#endif

	

	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//4. thresholding the images
	//redGreenEdges=new ImageOf<PixelMono>;
	
	//redGreenEdges->resize(width,height);
	//if(blueYellowEdges==0xcdcdcdcd){
		blueYellowEdges=new ImageOf<PixelMono>;
		blueYellowEdges->resize(width,height);
	//}

	//conv_32f_to_8u(outputBlueYellow32,psb32,greenRedEdges->getPixelAddress(0,0),width,srcsize);


	//ippiDilate3x3_8u_C1IR(outputRedGreen,psb,srcsize);
	//ippiErode_8u_C1IR(outputRedGreen,psb,srcsize,&src1[0],src1Size,anchor);
	//ippiErode3x3_8u_C1IR(outputRedGreen,psb,srcsize);
	//ippiFilterMedian_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,src1Size,anchor);
	//ippiThreshold_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,(Ipp8u) 50 ,ippCmpGreater);
	//5. save the output into the outputImage
	//ippiThreshold_8u_C1IR(outputRedGreen,psb,srcsize,100,ippCmpGreater);
	ippiCopy_8u_C1R(outputBlueYellow,width,blueYellowEdges->getPixelAddress(0,0),width,srcsize);
	
	//ippiAdd_8u_C1IRSfs(redGreenEdges->getPixelAddress(0,0),width,redGreenEdges->getPixelAddress(0,0),width,srcsize,0.5);
	//ippiCopy_8u_C1R(outputBlueYellow,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
	//ippiCopy_8u_C1R(outputBlueYellow,width,blueYellowEdges_yarp->getPixelAddress(0,0),width,srcsize);

	ippiFree(outputBlueYellow);
	ippiFree(outputBlueYellow2);
	ippiFree(outputBlueYellow3);
	ippiFree(outputBlueYellow32);
	ippiFree(outputBlueYellow32B);
	ippiFree(inputBlueYellow32);


	return blueYellowEdges;
}


ImageOf<PixelMono>* ImageProcessor::findEdgesBlue(){
	int psb;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->bluePlane->width();
	int height=this->bluePlane->height();
	IppiSize srcsize ={width,height};
	//2. convolve every colour opponency with the sobel in order to get edges
	Ipp8u src2[3*3]={0, 1 ,2,-1,0,1,-2,-1,0};
	//Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	//Ipp8u* outputGreenRed = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* output = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 4, 4 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//------------------------------
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));
	//ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	//--------->   ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//5. save the output into the outputImage
	//--------------------
	
	this->cannyOperator->proc(bluePlane_ippi,width,40,50);
	output=this->cannyOperator->get_edgemap();


	//ImageOf<PixelMono>* edges=new ImageOf<PixelMono>;
	//edges->resize(width,height);
	//ippiCopy_8u_C1R(outputRedGreen,width,redGreenEdges_yarp->getPixelAddress(0,0),width,srcsize);
	//ippiCopy_8u_C1R(outputGreenRed,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(output,width,outputImage->getPixelAddress(0,0),width,srcsize);
	return outputImage;
}

/*ImageOf<PixelMono>* ImageProcessor::findEdgesGreenOpponency(){
	int psb;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->redGreen_yarp->width();
	int height=this->redGreen_yarp->height();
	IppiSize srcsize ={width,height};
	//2. convolve every colour opponency with the sobel in order to get edges
	Ipp8u src2[3*3]={0, 1 ,2,-1,0,1,-2,-1,0};
	//Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* output = ippiMalloc_8u_C1(width,height,&psb);
	//Ipp8u* outputBlueYellow = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 4, 4 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//------------
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));
	//ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	//----------->  ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//5. save the output into the outputImage
	//--------------------
	this->cannyOperator->proc(greenRed_ippi,width,40,50);
	output=this->cannyOperator->get_edgemap();
	
	//ippiCopy_8u_C1R(outputRedGreen,width,redGreenEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(output,width,outputImage->getPixelAddress(0,0),width,srcsize);
	return outputImage;
}*/

ImageOf<PixelMono>* ImageProcessor::findEdgesGreenOpponency(){
	int psb,psb32;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->greenRed_yarp->width();
	int height=this->greenRed_yarp->height();
	IppiSize srcsize ={width,height};
	IppiSize dstsize ={width-2,height-2};
	//2. convolve every colour opponency with the sobel in order to get edges
	/*Ipp32s src0[3*3]={1, 2 ,1,
					 0, 0, 0,
					-1,-2, 1};
	Ipp32s src1[3*3]={2, 1 ,0,
					 1, 0, -1,
					 0,-1, -2};	
	Ipp32s src2[3*3]={1, 0 ,-1,
					 2, 0, -2,
					 1, 0, -1};
	Ipp32s src3[3*3]={0, -1 ,-2,
					 1,  0, -1,
					 2,  1,  0};
	Ipp32s src4[3*3]={-1,-2,-1,
					  0, 0, 0,
					  1, 2, 1};
	Ipp32s src5[3*3]={-2, -1,0,
					 -1, 0, 1,
				 	  0, 1, 2};
	Ipp32s src6[3*3]={-1, 0 ,1,
					 -2, 0, 2,
					 -1, 0, 1};
	Ipp32s src7[3*3]={0, 1 ,2,
					-1, 0, 1,
					-2,-1, 0};*/
	
	Ipp32f* outputGreenRed32 = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp32f* inputGreenRed32 = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp8u* outputGreenRed = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputGreenRed2 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputGreenRed3 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp32f* outputGreenRed32B = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 5, 5 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	IppiPoint anchor={1,1};
	//3.Convolute with the sobel operator
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));

	//conversion of the input image into 32f image
	//conv_8u_to_32f(greenRed_yarp->getPixelAddress(0,0),width,inputGreenRed32,psb32,srcsize);
	//ippiConvert_8u32f_C1R(greenRed_yarp->getPixelAddress(0,0),width,inputGreenRed32,psb32,srcsize);

#ifdef CONVSEQ
	//convolution of the previous convolution
	ippiConvValid_32f_C1R(inputGreenRed32,psb32,srcsize,src0,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src1,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src2,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src3,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src4,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src5,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src6,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src7,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
#endif

#ifdef CONVMAX
	int CONVMAX_TH=700;
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src0,3,src2Size,&outputGreenRed32[1 + width*1],psb32);
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src1,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src2,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src3,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src4,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src5,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src6,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
	ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,src7,3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed32[i]<outputGreenRed32B[i])
			outputGreenRed32[i]=outputGreenRed32B[i];
	}
#endif
#ifdef CONVFILTER
	int CONVFILTER_TH=20;
	//ippiFilterSobelHoriz_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	//ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen,psb,srcsize);
	//ippiFilter_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,dstsize,src1,src2Size,anchor);
	//ippiFilterGauss_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize5x5);
	//ippiFilterGauss_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize,ippMskSize5x5);
	

	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed,psb,dstsize,src0,src2Size,anchor,CONVFILTER_TH);
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src1,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src2,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src3,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src4,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src5,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src6,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,src7,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}
	//ippiCopy_32f_C1R(inputRedGreen32,psb32,outputGreenRed32B,psb32,srcsize);
#endif

#ifdef IPPISOBEL
	ippiFilterSobelHoriz_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed3,psb,srcsize);
	ippiFilterSobelVert_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,srcsize);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed==NULL){
			printf("outputGreenRed NULL");
			break;
		}
		if(outputGreenRed2==NULL){
			printf("outputGreenRed2 NULL");
			break;
		}
		if(outputGreenRed3[i]<outputGreenRed2[i])
			outputGreenRed3[i]=outputGreenRed2[i];
	}

	IppiSize msksize={3,3};
	Ipp8u src[3*3]={1,1,1,1,1,1,1,1,1};
	ippiConvValid_8u_C1R(outputGreenRed3,psb,srcsize,src,3,msksize,&outputGreenRed[1 + width*1],psb,5);
	
	/*conv_8u_to_32f(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed32,psb32,srcsize);
	ippiFilterSobelCross_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize,ippMskSize3x3);
	conv_32f_to_8u(outputGreenRed32B,psb32,outputGreenRed2,psb,srcsize);
	for(int i=0; i<320*240;i++){
		if(outputGreenRed[i]<outputGreenRed2[i])
			outputGreenRed[i]=outputGreenRed2[i];
	}*/

#endif

#ifdef OPENCVSOBEL
	IppiSize msksize={3,3};
	if(greenRed_flag)
		cvSobel(greenRed_yarp->getIplImage(),cvImage,1,1,3);
	else
		cvImage=new IplImage();
	ippiCopy_8u_C1R((unsigned char*)cvImage->imageData,width,outputGreenRed2,psb,srcsize);
	Ipp8u src[3*3]={1,4,1,
					4,20,4,
					1,4,1};
	ippiConvValid_8u_C1R(outputGreenRed2,psb,srcsize,src,3,msksize,&outputGreenRed[1 + width*1],psb,1);
	//ippiFilterMedian_8u_C1R((unsigned char*)cvImage->imageData,width,outputGreenRed,psb,srcsize,msksize,anchor);
#endif



	//ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//4. thresholding the images
	//redGreenEdges=new ImageOf<PixelMono>;
	
	//redGreenEdges->resize(width,height);
	//if((unsigned int)greenRedEdges==0xcdcdcdcd){
		greenRedEdges=new ImageOf<PixelMono>;
		greenRedEdges->resize(width,height);
	//}

	//convesion back to 8u
	//conv_32f_to_8u(outputGreenRed32,psb32,greenRedEdges->getPixelAddress(0,0),width,srcsize);

	//ippiDilate3x3_8u_C1IR(outputRedGreen,psb,srcsize);
	//ippiErode_8u_C1IR(outputRedGreen,psb,srcsize,&src1[0],src1Size,anchor);
	//ippiErode3x3_8u_C1IR(outputRedGreen,psb,srcsize);
	//ippiFilterMedian_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,src1Size,anchor);
	//ippiThreshold_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,(Ipp8u) 50 ,ippCmpGreater);
	//5. save the output into the outputImage
	//ippiThreshold_8u_C1IR(outputRedGreen,psb,srcsize,100,ippCmpGreater);
	ippiCopy_8u_C1R(outputGreenRed,width,greenRedEdges->getPixelAddress(0,0),width,srcsize);
	//ippiAdd_8u_C1IRSfs(redGreenEdges->getPixelAddress(0,0),width,redGreenEdges->getPixelAddress(0,0),width,srcsize,0.5);
	//ippiCopy_8u_C1R(outputGreenRed,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
	//ippiCopy_8u_C1R(outputBlueYellow,width,blueYellowEdges_yarp->getPixelAddress(0,0),width,srcsize);

	ippiFree(outputGreenRed);
	ippiFree(outputGreenRed2);
	ippiFree(outputGreenRed3);
	ippiFree(outputGreenRed32);
	ippiFree(outputGreenRed32B);
	ippiFree(inputGreenRed32);

	return greenRedEdges;
}

ImageOf<PixelMono>* ImageProcessor::findEdgesGreen(){
	int psb;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->greenPlane->width();
	int height=this->greenPlane->height();
	IppiSize srcsize ={width,height};
	//2. convolve every colour opponency with the sobel in order to get edges
	Ipp8u src2[3*3]={0, 1 ,2,-1,0,1,-2,-1,0};
	//Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* output = ippiMalloc_8u_C1(width,height,&psb);
	//Ipp8u* outputBlueYellow = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 4, 4 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//------------
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));
	//ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	//----------->  ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//5. save the output into the outputImage
	//--------------------
	this->cannyOperator->proc(greenPlane_ippi,width,40,50);
	output=this->cannyOperator->get_edgemap();
	
	//ippiCopy_8u_C1R(outputRedGreen,width,redGreenEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(output,width,outputImage->getPixelAddress(0,0),width,srcsize);
	return outputImage;
}

ImageOf<PixelMono>* ImageProcessor::findEdgesRedOpponency(){
	int psb,psb32;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->redGreen_yarp->width();
	int height=this->redGreen_yarp->height();
	//printf("width:%d /n",width);
	IppiSize srcsize ={width,height};
	IppiSize dstsize ={width-2,height-2};
	IppiPoint anchor={1,1};
	//2. convolve every colour opponency with the sobel in order to get edges
	/*Ipp32s src0[3*3]={1, 2 ,1,
					 0, 0, 0,
					-1,-2, 1};
	Ipp32s src1[3*3]={2, 1 ,0,
					 1, 0, -1,
					 0,-1, -2};	
	Ipp32s src2[3*3]={1, 0 ,-1,
					 2, 0, -2,
					 1, 0, -1};
	Ipp32s src3[3*3]={0, -1 ,-2,
					 1,  0, -1,
					 2,  1,  0};
	Ipp32s src4[3*3]={-1,-2,-1,
					  0, 0, 0,
					  1, 2, 1};
	Ipp32s src5[3*3]={-2, -1,0,
					 -1, 0, 1,
				 	  0, 1, 2};
	Ipp32s src6[3*3]={-1, 0 ,1,
					 -2, 0, 2,
					 -1, 0, 1};
	Ipp32s src7[3*3]={0, 1 ,2,
					-1, 0, 1,
					-2,-1, 0};*/
	
	Ipp32f* inputRedGreen32 = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp32f* outputRedGreen32 = ippiMalloc_32f_C1(width,height,&psb32);
	Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputRedGreen2 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* outputRedGreen3 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp32f* outputRedGreen32B = ippiMalloc_32f_C1(width,height,&psb32);
	//printf("psb:%d /n",psb);
	Ipp8u dst2[2*2];
	//Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 5, 5 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//3.Convolute with the sobel operator
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));

	//conversion of the input image into 32f image
	//ippiConvert_8u32f_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen32,psb32,srcsize);
	//conv_8u_to_32f(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen32,psb32,srcsize);

#ifdef CONVSEQ
	int CONVSEQ_TH=600;
	//convolution of the previous convolution
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src0,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src1,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src2,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src3,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src4,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src5,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src6,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src7,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
#endif

#ifdef CONVMAX
	int CONVMAX_TH=700;
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src0,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src1,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	/*for(int i=0; i<320*240;i++){
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}*/
	/*ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src2,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src3,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src4,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src5,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src6,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}
	ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src7,3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
	
		if(outputRedGreen32[i]<outputRedGreen32B[i])
			outputRedGreen32[i]=outputRedGreen32B[i];
	}*/
#endif

#ifdef CONVFILTER
	int CONVFILTER_TH=5;
	//ippiFilterSobelHoriz_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
	//ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen,psb,srcsize);
	//ippiFilter_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,dstsize,src1,src2Size,anchor);
	//ippiFilterGauss_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize5x5);
	//ippiFilterGauss_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize,ippMskSize5x5);
	//ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize);

	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,dstsize,src0,src2Size,anchor,CONVFILTER_TH);
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src1,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src2,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src3,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src4,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src5,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src6,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}
	ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,src7,src2Size,anchor,CONVFILTER_TH);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}

	//ippiCopy_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
#endif

#ifdef IPPISOBEL
	ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen3,psb,srcsize);
	ippiFilterSobelVert_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,srcsize);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen==NULL){
			printf("outputRedGreen NULL");
			break;
		}
		if(outputRedGreen2==NULL){
			printf("outputRedGreen2 NULL");
			break;
		}
		if(outputRedGreen3[i]<outputRedGreen2[i])
			outputRedGreen3[i]=outputRedGreen2[i];
	}

	IppiSize msksize={3,3};
	Ipp8u src[3*3]={1,1,1,1,1,1,1,1,1};
	ippiConvValid_8u_C1R(outputRedGreen3,psb,srcsize,src,3,msksize,&outputRedGreen[1 + width*1],psb,5);
	/*conv_8u_to_32f(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen32,psb32,srcsize);
	ippiFilterSobelCross_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize3x3);
	conv_32f_to_8u(outputRedGreen32B,psb32,outputRedGreen2,psb,srcsize);
	for(int i=0; i<320*240;i++){
		if(outputRedGreen[i]<outputRedGreen2[i])
			outputRedGreen[i]=outputRedGreen2[i];
	}*/
#endif

#ifdef OPENCVSOBEL
	IppiSize msksize={3,3};
	if(redGreen_flag)
		cvSobel(redGreen_yarp->getIplImage(),cvImage,1,1,3);
	else
		cvImage=new IplImage();
	ippiCopy_8u_C1R((unsigned char*)cvImage->imageData,width,outputRedGreen2,psb,srcsize);
    Ipp8u src[3*3]={1,4,1,
					4,20,4,
					1,4,1};
	ippiConvValid_8u_C1R(outputRedGreen2,psb,srcsize,src,3,msksize,&outputRedGreen[1 + width*1],psb,1);
	//ippiFilterMedian_8u_C1R((unsigned char*)cvImage->imageData,width,outputRedGreen,psb,srcsize,msksize,anchor);
#endif


	//ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	
	//4. thresholding the images
	//redGreenEdges=new ImageOf<PixelMono>;
	//redGreenEdges->resize(width,height);
	//if((unsigned int)redGreenEdges==0xcdcdcdcd){
		redGreenEdges=new ImageOf<PixelMono>;
		redGreenEdges->resize(width,height);
	//}

	//convesion back to 8u
	//conv_32f_to_8u(outputRedGreen32B,psb32,outputRedGreen,psb,srcsize);

	//ippiDilate3x3_8u_C1IR(outputRedGreen,psb,srcsize);
	//ippiErode_8u_C1IR(outputRedGreen,psb,srcsize,&src1[0],src1Size,anchor);
	//ippiErode3x3_8u_C1IR(outputRedGreen,psb,srcsize);
	//ippiFilterMedian_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,src1Size,anchor);
	//ippiThreshold_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,(Ipp8u) 50 ,ippCmpGreater);
	//5. save the output into the outputImage
	//ippiThreshold_8u_C1IR(outputRedGreen,psb,srcsize,100,ippCmpGreater);
	ippiCopy_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),width,srcsize);
	
	//ippiAdd_8u_C1IRSfs(redGreenEdges->getPixelAddress(0,0),width,redGreenEdges->getPixelAddress(0,0),width,srcsize,0.5);
	
	ippiFree(outputRedGreen);
	ippiFree(outputRedGreen2);
	ippiFree(outputRedGreen3);
	ippiFree(outputRedGreen32);
	ippiFree(outputRedGreen32B);
	ippiFree(inputRedGreen32);

	return redGreenEdges;
}



/*ImageOf<PixelMono>* ImageProcessor::findEdgesRedOpponency(){
	int psb;
	//1.gets the redGreen, greenRed and blueYellow from the this class
	int width=this->redPlane->width();
	int height=this->redPlane->height();
	IppiSize srcsize ={width,height};
	//2. convolve every colour opponency with the sobel in order to get edges
	Ipp8u src2[3*3]={0, 1 ,2,-1,0,1,-2,-1,0};
	//Ipp8u* outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* output = ippiMalloc_8u_C1(width,height,&psb);
	//Ipp8u* outputBlueYellow = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u dst2[2*2];
	Ipp8u *pointer=NULL;
	//*pointer=src1[0];
	IppiSize src1Size = { 4, 4 };
	IppiSize src2Size = { 3, 3 };
	int divisor = 2;
	int sign = 1;
	//------------
	//ippiGenSobelKernel_16s ((Ipp16s *) src2, 9, 1, sign); // using "Sobel" kernel
	//printf("the sizeof(IPP8u) is %d  while psb= %d\n",sizeof(Ipp8u),psb);
	//ippiConvFull_16s_C1R (pointer, 8*sizeof(Ipp16s), src1Size, src2,3*sizeof(Ipp16s), src2Size, dst2, 2*sizeof(Ipp16s), 2 );
	//ippiConvFull_8u_C1R(pointer,8*sizeof(Ipp8u),src1Size,src2,3*sizeof(Ipp8u),src2Size,dst,2*sizeof(Ipp8u),2);
	//printf("redGreen_yarp: 0x%08x\n", redGreen_yarp->getPixelAddress(0,0));
	//ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
	//----------->  ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
	//ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
	//ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
	//printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
	//5. save the output into the outputImage
	//--------------------
	this->cannyOperator->proc(redPlane_ippi,width,40,50);
	output=this->cannyOperator->get_edgemap();
	
	//ippiCopy_8u_C1R(outputRedGreen,width,redGreenEdges_yarp->getPixelAddress(0,0),width,srcsize);
	ippiCopy_8u_C1R(output,width,outputImage->getPixelAddress(0,0),width,srcsize);
	return outputImage;
}*/

/*DEPRECATED ImageOf<PixelRgb>* ImageProcessor::findEdges(ImageOf<PixelRgb>* inputImage,int x_order,int y_order){
	IplImage *cvImage = cvCreateImage(cvSize(inputImage->width(),  
											inputImage->height()), 
											IPL_DEPTH_8U, 1 );
	cvCvtColor((IplImage*)inputImage->getIplImage(), cvImage, CV_RGB2GRAY);
	//apply the operation
	IplImage* dst = cvCreateImage( cvSize(cvImage->width,cvImage->height), 8, 1 );
	cvSobel(cvImage,dst, x_order, y_order, 3 );
	//revert back to YarpImage
	IplImage* dstColor = cvCreateImage( cvGetSize(cvImage), 8, 3 );
	cvCvtColor(cvImage,dstColor, CV_GRAY2RGB);
	//ImageOf<PixelBgr> yarpReturnImage;
	portImage.wrapIplImage(dstColor);
	return &portImage;
	
}*/

/*normalise the image between the max and min value*/
ImageOf<PixelMono>* ImageProcessor::normalize ( ImageOf<PixelMono>* src ){
	int min=0, max=255;
	int psb32;
	Ipp32f* src32=ippiMalloc_32f_C1(320,240,&psb32);
	IppiSize srcsize={320,240};
	conv_8u_to_32f(src->getPixelAddress(0,0),320,src32,psb32,srcsize);
	conv_32f_to_8u(src32,psb32,outputImage->getPixelAddress(0,0),320,srcsize);
	
	//1.search for min and max
	//this->minMax(src,&max,&min);
	//printf("min Value=%d ; max value=%d",min,max);
	//2.stretch features map
	//outputImage=this->fullRange(src,&max,&min);

	return outputImage;
}

void ImageProcessor::minMax(ImageOf<PixelMono> *src,int* maxValue,int* minValue){
	int psb;
	int width=src->width();
	int height=src->height();
	IppiSize srcsize ={width,height};
	Ipp8u minValue_ippi=0,maxValue_ippi=0;
	Ipp8u* src_ippi = ippiMalloc_8u_C1(width,height,&psb);
	ippiCopy_8u_C1R(src->getPixelAddress(0,0),width,src_ippi,psb,srcsize);
	ippiMin_8u_C1R(src_ippi,psb,srcsize,&minValue_ippi);
	ippiMax_8u_C1R(src_ippi,psb,srcsize,&maxValue_ippi);
	*minValue=minValue_ippi;
	*maxValue=maxValue_ippi;
	//ippFree(src_ippi);
}


ImageOf<PixelMono>* ImageProcessor::fullRange(ImageOf<PixelMono> *src,int *mx, int *mn){
	int psb;
	int width=src->width();
	int height=src->height();
	IppiSize srcsize ={width,height};
	Ipp8u* minValue_ippi;
	minValue_ippi=(Ipp8u*)mn;
	Ipp8u* out2_ippi = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* out_ippi = ippiMalloc_8u_C1(width,height,&psb);
	int a;
	if (mx!=mn) {
		a = (int)((long)255 * (1<<9) / (mx - mn)); /// factor to extend range
		//1.substract from the img the mn in order to force to zero the minumum pixel intensity
		//iplSubtractS(img, out, mn, false);
		ippiSubC_8u_C1RSfs(src->getPixelAddress(0,0),width,*minValue_ippi,out_ippi,psb,srcsize,0.5);
		//ippiSub_8u_C1RSfs(src->getPixelAddress(0,0),width,&src1[0],width,out_ippi,psb,srcsize,1);
		//2.extend the range starting from the factor a
		//*(scala->values)=a;
		Ipp8u src2[3*3]={0,1,0,
						1,a,1,
						0,1,0};
		IppiSize src2Size={2,3};
		//iplConvolve2D(out, out, &scala, 1, IPL_SUM);
		ippiConvValid_8u_C1R(out_ippi,width,srcsize,src2,3,src2Size,&out2_ippi[1 + width*1],psb,2);
	}
	ippiCopy_8u_C1R(out2_ippi,psb,outputImage->getPixelAddress(0,0),width,srcsize);

	ippiFree(out_ippi);
	ippiFree(out2_ippi);
	return outputImage;
}

ImageOf<PixelRgb>* ImageProcessor::getOutputImage(){
	return portImage;
}

void ImageProcessor::setInputImage(ImageOf<PixelRgb> *src){
	this->inImage=src;
	this->portImage=src;
}
void ImageProcessor::setInputImage(ImageOf<PixelMono> *src){
	
}

/* function that processes the input image (passed as parameter)
	accordingly to the processing options selected for this processor
*/

ImageOf<PixelRgb>* ImageProcessor::process (ImageOf<PixelRgb> *src){
	//ImageOf<PixelMono> *image_tmp=new ImageOf<PixelMono>;
	//ImageOf<PixelRgb> *image_out=new ImageOf<PixelRgb>;
	int width=src->width();
	int height=src->height();
	//image_tmp->resize(width,height);
	//image_out->resize(width,height);
	IppiSize srcsize;
	srcsize.height=src->height();
	srcsize.width=src->width();
	/*if(image_tmp==NULL){
		printf("process:image_tmp NULL");
		image_tmp=new ImageOf<PixelMono>;
		image_tmp->resize(width,height);
	}
	if(image_out==NULL){
		printf("process:image_out NULL");
		image_out=new ImageOf<PixelRgb>;
		image_out->resize(width,height);
	}*/
	
	int image_tmp_flag=0;

	if(this->redPlane_flag){
		if(this->colourOpponency_flag){
			this->colourOpponency(src);
			if(this->findEdges_flag){
				if(this->combineMax_flag){
					image_tmp=this->combineMax();
				}
				else{
					image_tmp=this->findEdgesRedOpponency();
				}
			}
			else
				image_tmp=this->redGreen_yarp;
		}
		else{
			this->colourOpponency(src);
			if(this->findEdges_flag){
				image_tmp=this->findEdgesRedOpponency();
			}
			else{
				//image_tmp=this->getRedPlane(src);
				image_tmp=redPlane;
			}
				
		}
	}
	else if(bluePlane_flag)
		if(this->colourOpponency_flag){
			this->colourOpponency(src);
			if(this->findEdges_flag){
				if(this->combineMax_flag){
					image_tmp=this->combineMax();
				}
				else{
					image_tmp=this->findEdgesBlueOpponency();
				}
			}
			else
				image_tmp=this->blueYellow_yarp;
		}
		else{
			this->colourOpponency(src);
			if(this->findEdges_flag){
				image_tmp=this->findEdgesBlueOpponency();
			}
			else{
				//image_tmp=this->getBluePlane(src);
				image_tmp=bluePlane;
			}
		}
	else if(greenPlane_flag)
		if(this->colourOpponency_flag){
			this->colourOpponency(src);
			if(this->findEdges_flag){
				if(this->combineMax_flag){
					image_tmp=this->combineMax();
				}
				else{
					image_tmp=this->findEdgesGreenOpponency();
				}
			}
			else
				image_tmp=this->greenRed_yarp;

		}
		else{
			this->colourOpponency(src);
			if(this->findEdges_flag){
				image_tmp=this->findEdgesGreenOpponency();
			}
			else{
				//image_tmp=this->getGreenPlane(src);
				image_tmp=greenPlane;
			}
		}
	else if(this->yellowPlane_flag){
			this->colourOpponency(src);
			ippiCopy_8u_C1R(yellowPlane->getPixelAddress(0,0),320,image_tmp->getPixelAddress(0,0),320,srcsize);
	}

	if(this->normalize_flag){
		image_tmp=this->normalize(image_tmp);
	}
	
	//-----------
	/*if(inputImage_flag==0){
		printf("process:image_tmp NULL");
		return NULL;
	}*/
	
	if(this->inputImage_flag){
		ippiCopy_8u_C3R(src->getPixelAddress(0,0),width*3,image_out->getPixelAddress(0,0),width*3,srcsize);
	}
	else{
		int psb;
		im_out = ippiMalloc_8u_C1(width,height,&psb);
		Ipp8u* im_tmp[3];
		//two copies in order to have 2 conversions
		//the first transform the yarp mono into a 4-channel image
		ippiCopy_8u_C1R(image_tmp->getPixelAddress(0,0), width,im_out,psb,srcsize);
		im_tmp[0]=im_out;
		im_tmp[1]=im_out;
		im_tmp[2]=im_out;
		//the second transforms the 4-channel image into colorImage for yarp
		ippiCopy_8u_P3C3R(im_tmp,psb,image_out->getPixelAddress(0,0),width*3,srcsize);
		this->portImage=image_out;
		ippiFree(im_out);
		ippiFree(im_tmp);
	}
	return image_out;
}

/*
 * load the red plane from ImageOf<PixelRgb>
 */
ImageOf<PixelMono>* ImageProcessor::getRedPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp)
{
	/*if((unsigned char)inputImage==0xffffffff){
		return NULL;
	}
	else{
		//printf("inputImage: 0x%08x \n",inputImage);
	}*/
	unsigned char c = 0;
    int x, y, z;
    int Offset;
	int psb;
	int width=inputImage->width();
	int height=inputImage->height();
	//printf("RedPlane-outputImage: 0x%08x \n",outputImage);
	/*if((unsigned int)outputImage==0xcdcdcdcd){
		outputImage=new ImageOf<PixelMono>;
		outputImage->resize(width,height);
	}*/
	
	IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();
	Ipp8u* shift[3];
	shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);
	ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
	ippiCopy_8u_C1R(shift[0],psb,tmp->getPixelAddress(0,0),width,srcsize);
	ippiFree(shift[0]);
	ippiFree(shift[1]);
	ippiFree(shift[2]);
	return outputImage;
}

/*
 * load the green plane from ImageOf<PixelRgb>
 */
ImageOf<PixelMono>* ImageProcessor::getGreenPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp){
	/*if((unsigned char)inputImage==0xffffffff){
		return NULL;
	}
	else{
		//printf("inputImage: 0x%08x \n",inputImage->getPixelAddress(0,0));
	}*/
	unsigned char c = 0;
    int x, y, z;
    int Offset;
	int psb;
	int width=inputImage->width();
	int height=inputImage->height();
	//printf("GreenPlane-outputImage: 0x%08x \n",outputImage);
	/*if((unsigned int)outputImage==0xcdcdcdcd){
		outputImage=new ImageOf<PixelMono>;
		outputImage->resize(width,height);
	}*/
	
	IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();
	Ipp8u* shift[3];
	shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);
	ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
	ippiCopy_8u_C1R(shift[1],psb,tmp->getPixelAddress(0,0),width,srcsize);
	ippiFree(shift[0]);
	ippiFree(shift[1]);
	ippiFree(shift[2]);
	return outputImage;
}

/*
 * load the blue plane from ImageOf<PixelRgb>
 */
ImageOf<PixelMono>* ImageProcessor::getBluePlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp){
	/*if((unsigned char)inputImage==0xffffffff){
		return NULL;
	}
	else{
		//printf("inputImage: 0x%08x \n",inputImage);
	}*/
	unsigned char c = 0;
    int x, y, z;
    int Offset;
	int psb;
	int width=inputImage->width();
	int height=inputImage->height();
	//printf("BluePlane-outputImage: 0x%08x \n",outputImage);
	/*if((unsigned int)outputImage==0xcdcdcdcd){
		outputImage=new ImageOf<PixelMono>;
		outputImage->resize(width,height);
	}*/
	
	
	IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();
	Ipp8u* shift[3];
	shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);
	ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
	ippiCopy_8u_C1R(shift[2],psb,tmp->getPixelAddress(0,0),width,srcsize);
	ippiFree(shift[0]);
	ippiFree(shift[1]);
	ippiFree(shift[2]);
	return outputImage;
}

/*
 * load the red plane from ImageOf<PixelRgb>
 */
/*DEPRECATED ImageOf<PixelMono>* ImageProcessor::getRedPlane(ImageOf<PixelRgb>* src){
    
	if(src==NULL){
		printf("getRedPlane NULL \n");
		return NULL;
	}
	else if((unsigned int)src==0xffffffff){
		printf("getRedPlane 0xffffffff \n");
		return NULL;
	}
	else{
		printf("getRedPlane correct \n");
	}

	unsigned char *image;
    unsigned char c = 0;
    int x, y, z;
    int Offset;


	int *X_Size=new int ;
	*X_Size=src->width();
	int *Y_Size=new int;
	*Y_Size= src->height();
    int *planes=new int;
	*planes=1;
	
    image = new unsigned char[*X_Size * *Y_Size * *planes];
	ImageOf<PixelMono>* imageMono=new ImageOf<PixelMono>;
    if (image == NULL)
    {
        exit (-1);
    }
    Offset = (4 - ((*X_Size * *planes) % 4)) % 4;
    //if (*planes == 1)
    //    fread (&palette, sizeof (RGBQUAD), 256, fin);
	unsigned char* pointerImageMono=new unsigned char;
    for (y = *Y_Size - 1; y >= 0; y--)
    {
        for (x = 0; x < *X_Size; x++)
            for (z = *planes - 1; z >= 0; z--)
            {
                //fread (&c, 1, 1, fin);
				PixelRgb pRgb=src->pixel(x,y);
				c=pRgb.r;
				image[y ** planes * *X_Size + *planes * x + z] =(unsigned char) (c);
				//image[x ** planes * *Y_Size + *planes * y + z] =(unsigned char) (c);
            }
			//for (x = 0; x < Offset; x++){
            //fread (&c, 1, 1, fin);
			//}
    }
    //fclose (fin);
	imageMono->setExternal((unsigned char*)image,*X_Size,*Y_Size);
    return imageMono;
}*/


/*
 * load the green plane from ImageOf<PixelRgb>
 */
/*DEPRECATED ImageOf<PixelMono>* ImageProcessor::getGreenPlane(ImageOf<PixelRgb>* src)
{
	if(src==NULL){
		printf("getGreenPlane NULL \n");
		return NULL;
	}
	else if((unsigned int)src==0xffffffff){
		printf("getGreenPlane 0xffffffff \n");
		return NULL;
	}
	else{
		printf("getGreenPlane correct \n");
	}

    unsigned char *image;
    unsigned char c = 0;
    int x, y, z;
    int Offset;

	//ImageOf<PixelRgb>* src=inputImage;

	int *X_Size=new int ;
	*X_Size=src->width();
	int *Y_Size=new int;
	*Y_Size= src->height();
    int *planes=new int;
	*planes=1;
	
    image = new unsigned char[*X_Size * *Y_Size * *planes];
	ImageOf<PixelMono>* imageMono=new ImageOf<PixelMono>;
	imageMono->resize(*X_Size,*Y_Size);
    if (image == NULL)
    {
        exit (-1);
    }
    Offset = (4 - ((*X_Size * *planes) % 4)) % 4;
    //if (*planes == 1)
    //    fread (&palette, sizeof (RGBQUAD), 256, fin);
	unsigned char* pointerImageMono=new unsigned char;
    for (y = *Y_Size - 1; y >= 0; y--)
    {
        for (x = 0; x < *X_Size; x++)
            for (z = *planes - 1; z >= 0; z--)
            {
                //fread (&c, 1, 1, fin);
				PixelRgb pRgb=src->pixel(x,y);
				c=pRgb.g;
				image[y ** planes * *X_Size + *planes * x + z] =(unsigned char) (c);
				//image[x ** planes * *Y_Size + *planes * y + z] =(unsigned char) (c);
            }
			//for (x = 0; x < Offset; x++){
			//fread (&c, 1, 1, fin);
			//}
    }
    //fclose (fin);
	imageMono->setExternal((unsigned char*)image,*X_Size,*Y_Size);
    return imageMono;
}*/

/*
 * load the blue plane from ImageOf<PixelRgb>
 */
/*DEPRECATED ImageOf<PixelMono>* ImageProcessor::getBluePlane(ImageOf<PixelRgb>* src){
    
	if(src==NULL){
		printf("getBluePlane NULL \n");
		return NULL;
	}
	else if((unsigned int)src==0xffffffff){
		printf("getBluePlane 0xffffffff \n");
		return NULL;
	}
	else{
		printf("getBluePlane correct \n");
	}

	unsigned char *image;
    unsigned char c = 0;
    int x, y, z;
    int Offset;

	

	int *X_Size=new int ;
	*X_Size=src->width();
	int *Y_Size=new int;
	*Y_Size= src->height();
    int *planes=new int;
	*planes=1;
	
    image = new unsigned char[*X_Size * *Y_Size * *planes];
	ImageOf<PixelMono>* imageMono=new ImageOf<PixelMono>;
    if (image == NULL)
    {
        exit (-1);
    }
    Offset = (4 - ((*X_Size * *planes) % 4)) % 4;
    //if (*planes == 1)
    //fread (&palette, sizeof (RGBQUAD), 256, fin);
	unsigned char* pointerImageMono=new unsigned char;
    for (y = *Y_Size - 1; y >= 0; y--)
    {
        for (x = 0; x < *X_Size; x++)
            for (z = *planes - 1; z >= 0; z--)
            {
                //fread (&c, 1, 1, fin);
				PixelRgb pRgb=src->pixel(x,y);
				c=pRgb.b;
				image[y ** planes * *X_Size + *planes * x + z] =(unsigned char) (c);
				//image[x ** planes * *Y_Size + *planes * y + z] =(unsigned char) (c);
            }
			//for (x = 0; x < Offset; x++){

            //fread (&c, 1, 1, fin);
			//}
    }
    //fclose (fin);
	imageMono->setExternal((unsigned char*)image,*X_Size,*Y_Size);
    return imageMono;
}*/

ImageOf<PixelMono>* ImageProcessor::RShiftC (ImageOf<PixelMono> *src1){
	int width=src1->width();
	int height=src1->height();
	IppiSize size ={width,height};
	//memory allocation
	ImageOf<PixelMono>* outputImage=new ImageOf<PixelMono>;
	outputImage->resize(width,height);
	int psb;
	int psb2;
	Ipp8u* im_in = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* im_out = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* shift[4];
	shift[0]=ippiMalloc_8u_C1(10,1,&psb2); 
	shift[1]=ippiMalloc_8u_C1(10,1,&psb2);
	shift[2]=ippiMalloc_8u_C1(10,1,&psb2);
	shift[3]=ippiMalloc_8u_C1(10,1,&psb2);
	//image type conversion
	//appling the operator on the image
	Ipp32u* shift32=(Ipp32u*)shift;
	ippiRShiftC_8u_C1R(im_in, psb,*shift32, im_out, psb, size);
	ippiCopy_8u_C1R(im_out,psb,outputImage->getPixelAddress(0,0),width,size);
	return outputImage;
}

ImageOf<PixelMono>*  ImageProcessor::LShiftC ( ImageOf<PixelMono> *src1){
	int width=src1->width();
	int height=src1->height();
	IppiSize size ={width,height};
	//memory allocation
	ImageOf<PixelMono>* outputImage=new ImageOf<PixelMono>;
	outputImage->resize(width,height);
	int psb;
	int psb2;
	Ipp8u* im_in = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* im_out = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* shift[4];
	shift[0]=ippiMalloc_8u_C1(10,1,&psb2); 
	shift[1]=ippiMalloc_8u_C1(10,1,&psb2);
	shift[2]=ippiMalloc_8u_C1(10,1,&psb2);
	shift[3]=ippiMalloc_8u_C1(10,1,&psb2);
	//image type conversion
	//appling the operator on the image
	Ipp32u* shift32= (Ipp32u*) shift;
	ippiLShiftC_8u_C1R(im_in, psb,*shift32, im_out, psb, size);
	ippiCopy_8u_C1R(im_out,psb,outputImage->getPixelAddress(0,0),width,size);
	return outputImage;
}

ImageOf<PixelMono>* ImageProcessor::combineMax(){
	int height= 240;
	int width= 320;
	IppiSize srcsize ={width,height};
	int psb;

	Ipp8u* edgesBlue_ippi=ippiMalloc_8u_C1(width,height,&psb); 
	Ipp8u* edgesGreen_ippi=ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* edgesRed_ippi=ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* edgesOutput_ippi=ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* edgesOutput2_ippi=ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* edgesMask_ippi=ippiMalloc_8u_C1(width,height,&psb);

 
	edgesBlue=this->findEdgesBlueOpponency();
	if(edgesBlue==NULL)
		return NULL;
	ippiCopy_8u_C1R(edgesBlue->getPixelAddress(0,0),width,edgesBlue_ippi,psb,srcsize);

	edgesGreen=this->findEdgesGreenOpponency();
	if(edgesGreen==NULL)
		return NULL;
	ippiCopy_8u_C1R(edgesGreen->getPixelAddress(0,0),width,edgesGreen_ippi,psb,srcsize);

	edgesRed=this->findEdgesRedOpponency();
	if(edgesRed==NULL)
		return NULL;
	ippiCopy_8u_C1R(edgesRed->getPixelAddress(0,0),width,edgesRed_ippi,psb,srcsize);


	int i=0,r=0,c=0;
	
	//if((unsigned int)edgesOutput==0xcdcdcdcd){
		edgesOutput=new ImageOf<PixelMono>;
		edgesOutput->resize(width,height);
	//}
	
	
	/*unsigned char* pointerRed=edgesOutput->getRawImage();
	unsigned char* pointerBlue=edgesBlue->getRawImage();
	unsigned char* pointerGreen=edgesGreen->getRawImage();
	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++)	{
			int value=pointerRed[i];
			if(pointerRed[i]<pointerBlue[i])
				pointerRed[i]=pointerBlue[i];
			if(pointerRed[i]<pointerGreen[i])
				pointerRed[i]=pointerGreen[i];
		}
		i++;
	}*/

	int thresholdHigh=140;
	int thresholdLow=1;
	if(edgesOutput_ippi==NULL)
		return NULL;
	for(int i=0; i<320*240;i++){
		//edgesOutput_ippi[i]=edgesBlue_ippi[i];

		if(edgesGreen_ippi[i]<edgesRed_ippi[i])
			if(edgesRed_ippi[i]<edgesBlue_ippi[i])
				edgesOutput_ippi[i]=edgesBlue_ippi[i];
			else
				edgesOutput_ippi[i]=edgesRed_ippi[i];
		else
			if(edgesGreen_ippi[i]<edgesBlue_ippi[i])
				edgesOutput_ippi[i]=edgesBlue_ippi[i];
			else
				edgesOutput_ippi[i]=edgesGreen_ippi[i];


		/*if(edgesOutput_ippi[i]==255) 
				greenRed_ippi[i]=0;
			else if((edgesOutput_ippi[i]>thresholdLow)&&(edgesOutput_ippi[i]<thresholdHigh)) 
				greenRed_ippi[i]=0;
			else if(edgesOutput_ippi[i]<=thresholdLow) 
				greenRed_ippi[i]=255;*/
	}

	/*ippiThreshold_8u_C1R(edgesOutput_ippi,psb,edgesMask_ippi,psb,srcsize,100,ippCmpLess);
	for (int i=0;i<320*240;i++){
		if(edgesMask_ippi[i]!=255)
			edgesOutput_ippi[i]=0;
	}*/

	/*ippiAdd_8u_C1IRSfs(edgesGreen_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize,1);
	ippiAdd_8u_C1IRSfs(edgesRed_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize,1);
	ippiAdd_8u_C1IRSfs(edgesBlue_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize,1);*/

	ippiCopy_8u_C1R(edgesOutput_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize);
	//edgesOutput->setExternal((unsigned char*)pointerRed,height,width);

	ippiFree(edgesBlue_ippi);
	ippiFree(edgesRed_ippi);
	ippiFree(edgesGreen_ippi);
	ippiFree(edgesOutput_ippi);
	ippiFree(edgesOutput2_ippi);
	ippiFree(edgesMask_ippi);
    return edgesOutput;
}

ImageOf<PixelMono>* ImageProcessor::subtract(ImageOf<PixelMono> *src1, ImageOf<PixelMono> *src2){
	int width=src1->width();
	int height=src1->height();
	ImageOf<PixelMono>* outputImage=new ImageOf<PixelMono>;
	outputImage->resize(width,height);
	IppiSize srcsize ={width,height};
	//memory allocation	
	int psb;
	int psb4;
	Ipp8u* gray_in1 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* gray_in2 = ippiMalloc_8u_C1(width,height,&psb);
	Ipp8u* gray_out = ippiMalloc_8u_C1(width,height,&psb);
	ippiCopy_8u_C1R(src1->getPixelAddress(0,0),width,gray_in1,psb,srcsize);
	ippiCopy_8u_C1R(src2->getPixelAddress(0,0),width,gray_in2,psb,srcsize);
	ippiSub_8u_C1RSfs(gray_in1,psb,gray_in2,psb,gray_out,psb,srcsize,0.3);
	ippiCopy_8u_C1R(gray_out,psb,outputImage->getPixelAddress(0,0),width,srcsize);
	return outputImage;
}

//*convert pixel order rgba to planar order yuv channels*/
void ImageProcessor::convertRGB(IppiSize sze,Ipp8u * rgba_orig, int psb_o){
	/*Ipp8u** pyuva=(Ipp8u**) malloc (4*sizeof(Ipp8u*));
	
	int width=sze.width;
	int height=sze.height;
	IppiSize srcsize;
	srcsize.height=height;
	srcsize.width=width;
	int psb_4,psb;
	
	
	Ipp8u *yuva_orig=ippiMalloc_8u_C1(width*4,height,&psb_4);
	Ipp8u *y_orig=ippiMalloc_8u_C1(width*4,height,&psb);
	Ipp8u *u_orig=ippiMalloc_8u_C1(width*4,height,&psb);
	Ipp8u *v_orig=ippiMalloc_8u_C1(width*4,height,&psb);
	Ipp8u *tmp=ippiMalloc_8u_C1(width*4,height,&psb);
	//QImage *qrgb;
	//qrgb= new QImage(width,height,32,256);
	//ippiRGBToYUV_8u_AC4R(rgba_orig,psb_o,yuva_orig,psb_4,srcsize);
	pyuva[0]=y_orig;
	pyuva[1]=u_orig;
	pyuva[2]=v_orig;
	pyuva[3]=tmp;
	ippiCopy_8u_C4P4R(yuva_orig,psb_4,pyuva,psb,srcsize);
	//ippiCopy_8u_C4R(rgba_orig,psb_o,qrgb->bits(),width*4,srcsize);*/
} 