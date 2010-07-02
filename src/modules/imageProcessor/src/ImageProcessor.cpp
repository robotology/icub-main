// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/ImageProcessor.h>



#include <ipp.h>
#include <ipps.h>
//#include <qimage.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


// available methods for edges detection
//#define CONVMAX
//#define CONVFILTER
//#define IPPISOBEL
//#define OPENCVSOBEL
//#define CONVSEQ

ImageProcessor::ImageProcessor():RateThread(THREAD_RATE)
{
    this->inImage=NULL;
    portImage=NULL;

    maskSeed=4;
    maskTop=20;

    canProcess_flag=0;
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
    combineMax_flag=0;
    resized_flag=false;
    //width=320;
    //height=240;
    //int psb;
    //IppiSize srcsize={320,240};

    portImage=new ImageOf<PixelRgb>;
    //this->portImage->resize(320,240);

    edges_yarp=new ImageOf<PixelMono>;
    edgesOutput=new ImageOf<PixelMono>;    

    redPlane=new ImageOf<PixelMono>;
    greenPlane=new ImageOf<PixelMono>;
    bluePlane=new ImageOf<PixelMono>;
    yellowPlane=new ImageOf<PixelMono>;
    

    redPlane_tmp=new ImageOf<PixelMono>;
    greenPlane_tmp=new ImageOf<PixelMono>;
    bluePlane_tmp=new ImageOf<PixelMono>;
    
    
    
    red_yarp=new ImageOf<PixelMono>;
    green_yarp=new ImageOf<PixelMono>;
    blue_yarp=new ImageOf<PixelMono>;
    

    redGreenEdges_yarp=new ImageOf<PixelMono>;
    greenRedEdges_yarp=new ImageOf<PixelMono>;
    blueYellowEdges_yarp=new ImageOf<PixelMono>;

    redGreenEdges=new ImageOf<PixelMono>;
    greenRedEdges=new ImageOf<PixelMono>;
    blueYellowEdges=new ImageOf<PixelMono>;	

    redGreen_ippi = 0;
    greenRed_ippi = 0;
    blueYellow_ippi = 0;
    bluePlane_ippi = 0;
    redPlane_ippi = 0;
    greenPlane_ippi = 0;


    edgesBlue=new ImageOf<PixelMono>;
    //edgesBlue->resize(width,height);
    edgesRed=new ImageOf<PixelMono>;
    //edgesRed->resize(width,height);
    edgesGreen=new ImageOf<PixelMono>;
    //edgesGreen->resize(width,height);
    
    

    tmp=new ImageOf<PixelMono>;
    
    image_out=new ImageOf<PixelRgb>;
    image_tmp=new ImageOf<PixelMono>;
    outputImage=new ImageOf<PixelMono>;
    
    
    src0s[0]= 1;src0s[1]= 2;src0s[2]=1;
    src0s[3]= 0;src0s[4]= 0;src0s[5]=0;
    src0s[6]=-1;src0s[7]=-2;src0s[8]=1;
    
    src1s[0]=2;src1s[1]= 1;src1s[2]= 0;
    src1s[3]=1;src1s[4]= 0;src1s[5]=-1;
    src1s[6]=0;src1s[7]=-1;src1s[8]=-2;	

    src2s[0]=1;src2s[1]=0;src2s[2]=-1;
    src2s[3]=2;src2s[4]=0;src2s[5]=-2;
    src2s[6]=1;src2s[7]=0,src2s[8]=-1;

    src3s[0]=0;src3s[1]=-1;src3s[2]=-2;
    src3s[3]=1;src3s[4]= 0;src3s[5]=-1;
    src3s[6]=2;src3s[7]= 1,src3s[8]= 0;

    src4s[0]=-1;src4s[1]=-2;src4s[2]=-1;
    src4s[3]= 0;src4s[4]= 0;src4s[5]= 0;
    src4s[6]= 1;src4s[7]= 2;src4s[8]= 1;

    src5s[0]=-2;src5s[1]=-1;src5s[2]=0;
    src5s[3]=-1;src5s[4]= 0;src5s[5]=1;
    src5s[6]= 0;src5s[7]= 1;src5s[8]=2;

    src6s[0]=-1;src6s[1]=0;src6s[2]=1;
    src6s[3]=-2;src6s[4]=0;src6s[5]=2;
    src6s[6]=-1;src6s[5]=0;src6s[8]=1;

    src7s[0]= 0;src7s[1]= 1;src7s[2]=2;
    src7s[3]=-1;src7s[4]= 0;src7s[5]=1;
    src7s[6]=-2;src7s[7]=-1;src7s[8]=0;

    src0f[0]= 1;src0f[1]= 2;src0f[2]=1;
    src0f[3]= 0;src0f[4]= 0;src0f[5]=0;
    src0f[6]=-1;src0f[7]=-2;src0f[8]=1;
    
    src1f[0]=2;src1f[1]= 1;src1f[2]= 0;
    src1f[3]=1;src1f[4]= 0;src1f[5]=-1;
    src1f[6]=0;src1f[7]=-1;src1f[8]=-2;	

    src2f[0]=1;src2f[1]=0;src2f[2]=-1;
    src2f[3]=2;src2f[4]=0;src2f[5]=-2;
    src2f[6]=1;src2f[7]=0,src2f[8]=-1;

    src3f[0]=0;src3f[1]=-1;src3f[2]=-2;
    src3f[3]=1;src3f[4]= 0;src3f[5]=-1;
    src3f[6]=2;src3f[7]= 1,src3f[8]= 0;

    src4f[0]=-1;src4f[1]=-2;src4f[2]=-1;
    src4f[3]= 0;src4f[4]= 0;src4f[5]= 0;
    src4f[6]= 1;src4f[7]= 2;src4f[8]= 1;

    src5f[0]=-2;src5f[1]=-1;src5f[2]=0;
    src5f[3]=-1;src5f[4]= 0;src5f[5]=1;
    src5f[6]= 0;src5f[7]= 1;src5f[8]=2;

    src6f[0]=-1;src6f[1]=0;src6f[2]=1;
    src6f[3]=-2;src6f[4]=0;src6f[5]=2;
    src6f[6]=-1;src6f[5]=0;src6f[8]=1;

    src7f[0]= 0;src7f[1]= 1;src7f[2]=2;
    src7f[3]=-1;src7f[4]= 0;src7f[5]=1;
    src7f[6]=-2;src7f[7]=-1;src7f[8]=0;

    CONVMAX=false;
    CONVFILTER=false;
    IPPISOBEL=false;
    OPENCVSOBEL=false;
    CONVSEQ=false;
        
}

ImageProcessor::~ImageProcessor(){
    printf("Destructor /n");
    delete inImage;
    delete portImage;
    delete edges_yarp;

    delete redPlane;
    delete greenPlane;
    delete bluePlane;	
    delete yellowPlane;

    delete redPlane_tmp;
    delete greenPlane_tmp;
    delete bluePlane_tmp;
    
    delete red_yarp;
    delete green_yarp;
    delete blue_yarp;
    
    delete redGreenEdges_yarp;
    delete greenRedEdges_yarp;
    delete blueYellowEdges_yarp;
    delete edgesBlue;
    delete edgesGreen;
    
    ippiFree(redGreen_ippi);
    ippiFree(greenRed_ippi);
    ippiFree(blueYellow_ippi);
    ippiFree(bluePlane_ippi);
    ippiFree(redPlane_ippi);
    ippiFree(greenPlane_ippi);

    delete redGreen_yarp;
    delete greenRed_yarp;
    delete blueYellow_yarp;	

    delete tmp;
    delete image_out;
    delete image_tmp;
    delete outputImage;
    //---->cvImage16
    //---->cvImage8
}

ImageProcessor::ImageProcessor(ImageOf<PixelRgb>* inputImage):RateThread(THREAD_RATE)
{
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

    //IppiSize srcsize={320,240};
    
    edgesOutput=new ImageOf<PixelMono>;
    portImage=new ImageOf<PixelRgb>;
    
    tmp=new ImageOf<PixelMono>;
    
    redGreen_yarp=new ImageOf<PixelMono>;
    greenRed_yarp=new ImageOf<PixelMono>;
    blueYellow_yarp=new ImageOf<PixelMono>;
    
    
}
/**
* 
*/
void ImageProcessor::resizeImages(int width,int height){
    this->height=height;
    this->width=width;

    
    IppiSize srcsize={width,height};
    tmp->resize(width,height);
    portImage->resize(width,height);
    /*redGreen_yarp->resize(width,height);
    greenRed_yarp->resize(width,height);
    blueYellow_yarp->resize(width,height);*/

    edgesOutput->resize(width,height);
    edges_yarp->resize(width,height);
    redPlane->resize(width,height);
    greenPlane->resize(width,height);
    bluePlane->resize(width,height);
    yellowPlane->resize(width,height);
    
    redPlane_tmp->resize(width,height);
    greenPlane_tmp->resize(width,height);
    bluePlane_tmp->resize(width,height);
    
    red_yarp->resize(width,height);
    green_yarp->resize(width,height);
    blue_yarp->resize(width,height);

    redGreenEdges_yarp->resize(width,height);
    greenRedEdges_yarp->resize(width,height);
    blueYellowEdges_yarp->resize(width,height);

    redGreenEdges->resize(width,height);
    greenRedEdges->resize(width,height);
    blueYellowEdges->resize(width,height);

    


    if(redGreen_ippi==0){
        redGreen_ippi = ippiMalloc_8u_C1(width,height,&psb);
        greenRed_ippi = ippiMalloc_8u_C1(width,height,&psb);
        blueYellow_ippi = ippiMalloc_8u_C1(width,height,&psb);
        bluePlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
        redPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
        greenPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);
    }

    tmp->resize(width,height);
    image_out->resize(width,height);
    image_tmp->resize(width,height);
    outputImage->resize(width,height);

    cvImage16= cvCreateImage(cvSize(width,height),IPL_DEPTH_16S,1);
    cvImage8= cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
    this->cannyOperator=new CANNY(srcsize);
    

    //redGreen opponency
    inputRedGreen32 = ippiMalloc_32f_C1(width,height,&psb32);
    outputRedGreen2 = ippiMalloc_8u_C1(width,height,&psb_border);
    outputRedGreen = ippiMalloc_8u_C1(width,height,&psb);
    outputRedGreen3 = ippiMalloc_8u_C1(width,height,&psb_border);
    outputRedGreen32B = ippiMalloc_32f_C1(width,height,&psb32); 
    outputRedGreen32 = ippiMalloc_32f_C1(width,height,&psb32);
    redGreenBorder=ippiMalloc_8u_C1(width+2,height+2,&psb_border);

    //greenRed opponency
    inputGreenRed32 = ippiMalloc_32f_C1(width,height,&psb32);
    outputGreenRed2 = ippiMalloc_8u_C1(width,height,&psb);
    outputGreenRed = ippiMalloc_8u_C1(width,height,&psb);
    outputGreenRed3 = ippiMalloc_8u_C1(width,height,&psb);
    outputGreenRed32 = ippiMalloc_32f_C1(width,height,&psb32);
    outputGreenRed32B = ippiMalloc_32f_C1(width,height,&psb32);
    greenRedBorder=ippiMalloc_8u_C1(width+2,height+2,&psb_border);
   
    //blueYellow opponency temporary images
    outputBlueYellow32 = ippiMalloc_32f_C1(width,height,&psb32);
    outputBlueYellow32B = ippiMalloc_32f_C1(width,height,&psb32);
    inputBlueYellow32 = ippiMalloc_32f_C1(width,height,&psb32);
    outputBlueYellow = ippiMalloc_8u_C1(width,height,&psb);
    outputBlueYellow2 = ippiMalloc_8u_C1(width,height,&psb);
    outputBlueYellow3 = ippiMalloc_8u_C1(width,height,&psb);
    blueYellowBorder=ippiMalloc_8u_C1(width+2,height+2,&psb_border);

    resized_flag=true;
}

/**
*	initialization of the thread 
*/
bool ImageProcessor::threadInit(){
    printf("Thread initialization .... \n");
    return true;
}
/**
* active loop of the thread
*/
void ImageProcessor::run(){

    if(*redGreen_flag)
        findEdgesRedOpponency(redGreenEdges_yarp);
    if(*greenRed_flag)
        findEdgesGreenOpponency(greenRedEdges_yarp);
    if(*blueYellow_flag)
        findEdgesBlueOpponency(blueYellowEdges_yarp);

    if((*redGreen_flag)&&(*greenRed_flag)&&(*blueYellow_flag)){
        combineMax(edges_yarp);
        //lineMax();
    }
    
}
/**
*	releases the thread
*/
void ImageProcessor::threadRelease(){
    printf("Thread realeasing .... \n");
    
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
    //Ipp8u dst2[2*2];
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



void ImageProcessor::findEdgesBlueOpponency(ImageOf<PixelMono>*  blueYellowEdges){
   
    //1.gets the redGreen, greenRed and blueYellow from the this class
    int width=this->blueYellow_yarp->width();
    int height=this->blueYellow_yarp->height();
    IppiSize srcsize ={width,height};
    IppiSize dstsize ={width-2,height-2};
    IppiSize bordersize={width+2,height+2};
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
    
    
    

    //Ipp8u dst2[2*2];
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
    conv_8u_to_32f(blueYellow_yarp->getPixelAddress(0,0),width,inputBlueYellow32,psb32,srcsize);
    //ippiConvert_8u32f_C1R(blueYellow_yarp->getPixelAddress(0,0),width,inputBlueYellow32,psb32,srcsize);

    if(CONVSEQ){
        
        //convolution of the previous convolution
        int CONVSEQ_TH=700;
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,src0f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src1f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src2f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src3f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src4f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src5f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src6f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputBlueYellow32B,psb32,srcsize,src7f,3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiConvert_32f8u_C1R(outputBlueYellow32,psb32,outputBlueYellow,psb,srcsize,ippRndZero);
        //ippiFree(&outputBlueYellow32[1 + psb32*1]);
        //ippiFree(outputBlueYellow32B);
        
    }

    else if(CONVMAX){
        
        int CONVMAX_TH=700;
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src0f[0],3,src2Size,&outputBlueYellow32[1 + psb32*1],psb32);
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src1f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src2f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src3f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src4f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src5f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src6f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvValid_32f_C1R(inputBlueYellow32,psb32,srcsize,&src7f[0],3,src2Size,&outputBlueYellow32B[1 + psb32*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputBlueYellow32[i]<outputBlueYellow32B[i])
                outputBlueYellow32[i]=outputBlueYellow32B[i];
        }*/
        ippiConvert_32f8u_C1R(outputBlueYellow32,psb32,outputBlueYellow,psb,srcsize,ippRndZero);
        /*ippiFree(&outputBlueYellow32[1 + psb32*1]);
        ippiFree(outputBlueYellow32B);*/
    }

    else if (CONVFILTER){
        
        
        int CONVFILTER_TH=20;
        //ippiFilterSobelHoriz_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        //ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen,psb,srcsize);
        //ippiFilter_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,dstsize,src1,src2Size,anchor);
        //ippiFilterGauss_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize5x5);
        //ippiFilterGauss_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize,ippMskSize5x5);
        //ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize);

        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow,psb,dstsize,&src0s[0],src2Size,anchor,CONVFILTER_TH);
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src1s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src2s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src3s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src4s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src5s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src6s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFilter_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,outputBlueYellow2,psb,dstsize,&src7s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
        ippiFree(outputBlueYellow2);
        //ippiCopy_32f_C1R(inputRedGreen32,psb32,outputBlueYellow32B,psb32,srcsize);
    }
         

    else if (IPPISOBEL){
        ippiCopyConstBorder_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),blueYellow_yarp->getRowSize(),srcsize,blueYellowBorder,psb_border,bordersize,1,1,0);
        ippiFilterSobelHoriz_8u_C1R(blueYellowBorder,psb_border,outputBlueYellow3,psb,srcsize);
        ippiFilterSobelVert_8u_C1R(blueYellowBorder,psb_border,outputBlueYellow2,psb,srcsize);
        
        for(int i=0; i<psb*height;i++){
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
            outputBlueYellow[i]=outputBlueYellow3[i];
        }

        /*ippiAdd_8u_C1IRSfs(outputBlueYellow2,psb,outputBlueYellow3,psb,srcsize,0.5);
        ippiCopy_8u_C1R(outputBlueYellow3,psb,outputBlueYellow,psb,srcsize);*/

        /*IppiSize msksize={3,3};
        Ipp8u src[3*3]={1,1,1,1,1,1,1,1,1};
        ippiConvValid_8u_C1R(outputBlueYellow3,psb,srcsize,src,3,msksize,&outputBlueYellow[1 + width*1],psb,10);*/

        
        //ippiFree(outputBlueYellow2);
        //ippiFree(outputBlueYellow3);
    }
    else if(IPPICROSS){
        conv_8u_to_32f(blueYellow_yarp->getRawImage(),blueYellow_yarp->getRowSize(),outputBlueYellow32,psb32,srcsize);
        ippiFilterSobelCross_32f_C1R(outputBlueYellow32,psb32,outputBlueYellow32B,psb32,srcsize,ippMskSize3x3);
        conv_32f_to_8u(outputBlueYellow32B,psb32,outputBlueYellow2,psb,srcsize);
        for(int i=0; i<psb*width;i++){
            if(outputBlueYellow[i]<outputBlueYellow2[i])
                outputBlueYellow[i]=outputBlueYellow2[i];
        }
    }
    else if(OPENCVSOBEL){
        
        IppiSize msksize={3,3};
        if(blueYellow_flag){
            cvSobel(blueYellow_yarp->getIplImage(),cvImage16,1,1,3);
            cvConvertScale(cvImage16,cvImage8);
        }
        else
            cvImage8=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);

        ippiCopy_8u_C1R((unsigned char*)cvImage8->imageData,cvImage8->widthStep,outputBlueYellow2,psb,srcsize);
        Ipp8u src[3*3]={1,maskSeed,1,
                        maskSeed,maskTop,maskSeed,
                        1,maskSeed,1};
        ippiConvValid_8u_C1R(outputBlueYellow2,psb,srcsize,src,3,msksize,&outputBlueYellow[1 + width*1],psb,1);
        //ippiFilterMedian_8u_C1R((unsigned char*)cvImage->imageData,width,outputBlueYellow,psb,srcsize,msksize,anchor);
        //ippiFree(outputBlueYellow2);
    }

    

    //ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
    //ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
    //ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
    //printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
    //4. thresholding the images
    //redGreenEdges=new ImageOf<PixelMono>;
    
    //redGreenEdges->resize(width,height);
    //if(blueYellowEdges==0xcdcdcdcd){
    //	blueYellowEdges=new ImageOf<PixelMono>;
    //	blueYellowEdges->resize(width,height);
    //}

    //conv_32f_to_8u(outputBlueYellow32,psb32,greenRedEdges->getPixelAddress(0,0),width,srcsize);


    //ippiDilate3x3_8u_C1IR(outputRedGreen,psb,srcsize);
    //ippiErode_8u_C1IR(outputRedGreen,psb,srcsize,&src1[0],src1Size,anchor);
    //ippiErode3x3_8u_C1IR(outputRedGreen,psb,srcsize);
    //ippiFilterMedian_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,src1Size,anchor);
    //ippiThreshold_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,(Ipp8u) 50 ,ippCmpGreater);
    //5. save the output into the outputImage
    //ippiThreshold_8u_C1IR(outputRedGreen,psb,srcsize,100,ippCmpGreater);
    //blankBorder(outputBlueYellow);
    ippiCopy_8u_C1R(outputBlueYellow,psb,blueYellowEdges->getRawImage(),psb,srcsize);
    
    //ippiAdd_8u_C1IRSfs(redGreenEdges->getPixelAddress(0,0),width,redGreenEdges->getPixelAddress(0,0),width,srcsize,0.5);
    //ippiCopy_8u_C1R(outputBlueYellow,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
    //ippiCopy_8u_C1R(outputBlueYellow,width,blueYellowEdges_yarp->getPixelAddress(0,0),width,srcsize);

    //ippiFree(inputBlueYellow32);
    //ippiFree(outputBlueYellow);
    //ippiFree(outputBlueYellow2);
    //ippiFree(outputBlueYellow3);
    //ippiFree(outputBlueYellow32);
    //ippiFree(outputBlueYellow32B);
    


   // return blueYellowEdges;
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
    //Ipp8u dst2[2*2];
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



void ImageProcessor::findEdgesGreenOpponency(ImageOf<PixelMono>* greenRedEdges){
    
    //1.gets the redGreen, greenRed and blueYellow from the this class
    int width=this->greenRed_yarp->width();
    int height=this->greenRed_yarp->height();
    IppiSize srcsize ={width,height};
    IppiSize bordersize={width+2,height+2};
    IppiSize dstsize ={width-2,height-2};
    //2. convolve every colour opponency with the sobel in order to get edges
    
    
    
    //Ipp8u dst2[2*2];
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
    conv_8u_to_32f(greenRed_yarp->getPixelAddress(0,0),width,inputGreenRed32,psb32,srcsize);
    //ippiConvert_8u32f_C1R(greenRed_yarp->getPixelAddress(0,0),width,inputGreenRed32,psb32,srcsize);

    if(CONVSEQ){
        
        //convolution of the previous convolution
        ippiConvValid_32f_C1R(inputGreenRed32,psb32,srcsize,src0f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src1f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src2f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src3f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src4f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src5f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src6f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputGreenRed32B,psb32,srcsize,src7f,3,src2Size,&outputGreenRed32[1 + psb32*1],psb32);
        ippiConvert_32f8u_C1R(outputGreenRed32,psb32,outputGreenRed,psb,srcsize,ippRndZero);
        /* typedef enum {
        ippRndZero,
        ippRndNear,
        ippRndFinancial
        } IppRoundMode; 
        */
        //ippiFree(&outputGreenRed32[1 + psb32*1]);
        //ippiFree(outputGreenRed32B);
    }

    else if(CONVMAX){
        
        int CONVMAX_TH=700;
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src0f[0],3,src2Size,&outputGreenRed32[1 + width*1],psb32);
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src1f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src2f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src3f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src4f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src5f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src6f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvValid_32f_C1R(inputGreenRed32,width,srcsize,&src7f[0],3,src2Size,&outputGreenRed32B[1 + width*1],psb32);
        /*for(int i=0; i<320*240;i++){
            if(outputGreenRed32[i]<outputGreenRed32B[i])
                outputGreenRed32[i]=outputGreenRed32B[i];
        }*/
        ippiConvert_32f8u_C1R(outputGreenRed32,psb32,outputGreenRed,psb,srcsize,ippRndZero);
        //ippiFree(&outputGreenRed32[1 + psb32*1]);
        //ippiFree(outputGreenRed32B);
    }
    else if (CONVFILTER){
        
        int CONVFILTER_TH=20;
        //ippiFilterSobelHoriz_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        //ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen,psb,srcsize);
        //ippiFilter_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,dstsize,src1,src2Size,anchor);
        //ippiFilterGauss_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize5x5);
        //ippiFilterGauss_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize,ippMskSize5x5);
        

        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed,psb,dstsize,&src0s[0],src2Size,anchor,CONVFILTER_TH);
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src1s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src2s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src3s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src4s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src5s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src6s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        ippiFilter_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,outputGreenRed2,psb,dstsize,&src7s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
        //ippiCopy_32f_C1R(inputRedGreen32,psb32,outputGreenRed32B,psb32,srcsize);
        ippiFree(outputGreenRed2);
    }

    else if (IPPISOBEL){
        
        ippiCopyConstBorder_8u_C1R(greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),srcsize,greenRedBorder,psb_border,bordersize,1,1,0);
        ippiFilterSobelHoriz_8u_C1R(greenRedBorder,psb_border,outputGreenRed3,psb,srcsize);
        ippiFilterSobelVert_8u_C1R(greenRedBorder,psb_border,outputGreenRed2,psb,srcsize);

        for(int i=0; i<psb*height;i++){
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
            outputGreenRed[i]=outputGreenRed3[i];
        }

        /*ippiAdd_8u_C1IRSfs(outputGreenRed2,psb,outputGreenRed3,psb,srcsize,0.5);
        ippiCopy_8u_C1R(outputGreenRed3,psb,outputGreenRed,psb,srcsize);*/
            
        /*IppiSize msksize={3,3};
        Ipp8u src[3*3]={1,1,1,1,1,1,1,1,1};
        ippiConvValid_8u_C1R(outputGreenRed3,psb,srcsize,src,3,msksize,&outputGreenRed[1 + width*1],psb,5);*/
        
        
        //ippiFree(outputGreenRed2);
        //ippiFree(outputGreenRed3);

    }
    else if(IPPICROSS){
        conv_8u_to_32f(greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),outputGreenRed32,psb32,srcsize);
        ippiFilterSobelCross_32f_C1R(outputGreenRed32,psb32,outputGreenRed32B,psb32,srcsize,ippMskSize3x3);
        conv_32f_to_8u(outputGreenRed32B,psb32,outputGreenRed2,psb,srcsize);
        for(int i=0; i<psb*width;i++){
            if(outputGreenRed[i]<outputGreenRed2[i])
                outputGreenRed[i]=outputGreenRed2[i];
        }
    }
    else if (OPENCVSOBEL){
        
        IppiSize msksize={3,3};
        if(greenRed_flag){
            cvSobel(greenRed_yarp->getIplImage(),cvImage16,1,1,3);
            cvConvertScale(cvImage16,cvImage8);
        }
        else
            cvImage8=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
        ippiCopy_8u_C1R((unsigned char*)cvImage8->imageData,cvImage8->widthStep,outputGreenRed2,psb,srcsize);
        Ipp8u src[3*3]={1,maskSeed,1,
                        maskSeed,maskTop,maskSeed,
                        1,maskSeed,1};
        ippiConvValid_8u_C1R(outputGreenRed2,psb,srcsize,src,3,msksize,&outputGreenRed[1 + width*1],psb,1);
        //ippiFilterMedian_8u_C1R((unsigned char*)cvImage->imageData,width,outputGreenRed,psb,srcsize,msksize,anchor);
        //ippiFree(outputGreenRed2);
    }



    //ippiConvValid_8u_C1R(greenRed_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputGreenRed[1 + width*1],psb,10);
    //ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
    //ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
    //printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);

    //4. thresholding the images
    //redGreenEdges=new ImageOf<PixelMono>;
    
    //redGreenEdges->resize(width,height);
    //if((unsigned int)greenRedEdges==0xcdcdcdcd){
    //	greenRedEdges=new ImageOf<PixelMono>;
    //	greenRedEdges->resize(width,height);
    //}


    //5. convesion back to 8u
    //conv_32f_to_8u(outputGreenRed32,psb32,greenRedEdges->getPixelAddress(0,0),width,srcsize);
    //ippiDilate3x3_8u_C1IR(outputRedGreen,psb,srcsize);
    //ippiErode_8u_C1IR(outputRedGreen,psb,srcsize,&src1[0],src1Size,anchor);
    //ippiErode3x3_8u_C1IR(outputRedGreen,psb,srcsize);
    //ippiFilterMedian_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,src1Size,anchor);
    //ippiThreshold_8u_C1R(outputRedGreen,psb,redGreenEdges->getPixelAddress(0,0),psb,srcsize,(Ipp8u) 50 ,ippCmpGreater);
    
    //6. save the output into the outputImage
    //ippiThreshold_8u_C1IR(outputRedGreen,psb,srcsize,100,ippCmpGreater);
    //blankBorder(outputGreenRed);
    ippiCopy_8u_C1R(outputGreenRed,psb,greenRedEdges->getRawImage(),psb,srcsize);
    //ippiAdd_8u_C1IRSfs(redGreenEdges->getPixelAddress(0,0),width,redGreenEdges->getPixelAddress(0,0),width,srcsize,0.5);
    //ippiCopy_8u_C1R(outputGreenRed,width,greenRedEdges_yarp->getPixelAddress(0,0),width,srcsize);
    //ippiCopy_8u_C1R(outputBlueYellow,width,blueYellowEdges_yarp->getPixelAddress(0,0),width,srcsize);

    //ippiFree(outputGreenRed);
    //ippiFree(outputGreenRed2);
    //ippiFree(outputGreenRed3);
    
    //ippiFree(inputGreenRed32);

    //return greenRedEdges;
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
    //Ipp8u dst2[2*2];
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

 void ImageProcessor::findEdgesRedOpponency(ImageOf<PixelMono>* redGreenEdges){
    
    //1.gets the redGreen, greenRed and blueYellow from the this class
    int width=this->redGreen_yarp->width();
    int height=this->redGreen_yarp->height();
    //printf("width:%d /n",width);
    IppiSize srcsize ={width,height};
    IppiSize dstsize ={width-2,height-2};
    IppiSize bordersize={width+2,height+2};
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
    
    
   
    
    //printf("psb:%d /n",psb);
    //Ipp8u dst2[2*2];
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

    if(CONVSEQ){
        
        int CONVSEQ_TH=600;
        //convolution of the previous convolution
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,src0f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src1f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src2f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src3f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src4f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src5f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src6f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiCopy_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        ippiConvValid_32f_C1R(outputRedGreen32B,psb32,srcsize,src7f,3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);

        ippiConvert_32f8u_C1R(outputRedGreen32,psb32,outputRedGreen,psb,srcsize,ippRndZero);
        //ippiFree(&outputRedGreen32[1 + psb32*1]);
        //ippiFree(outputRedGreen32B);
    }

    else if(CONVMAX){
        int CONVMAX_TH=700;
        
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src0f[0],3,src2Size,&outputRedGreen32[1 + psb32*1],psb32);
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src1f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src2f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src3f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src4f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src5f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src6f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvValid_32f_C1R(inputRedGreen32,psb32,srcsize,&src7f[0],3,src2Size,&outputRedGreen32B[1 + psb32*1],psb32);
        /*for(int i=0; i<width*height;i++){
            if(outputRedGreen32[i]<outputRedGreen32B[i])
                outputRedGreen32[i]=outputRedGreen32B[i];
        }*/
        ippiConvert_32f8u_C1R(outputRedGreen32,psb32,outputRedGreen,psb,srcsize,ippRndZero);
        //ippiFree(&outputRedGreen32[1 + psb32*1]);
        //ippiFree(outputRedGreen32B);
    }

    else if (CONVFILTER){
        
        int CONVFILTER_TH=5;
        //ippiFilterSobelHoriz_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
        //ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,inputRedGreen,psb,srcsize);
        //ippiFilter_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,dstsize,src1,src2Size,anchor);
        //ippiFilterGauss_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize5x5);
        //ippiFilterGauss_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize,ippMskSize5x5);
        //ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,srcsize);

        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen,psb,dstsize,&src0s[0],src2Size,anchor,CONVFILTER_TH);
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src1s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src2s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src3s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src4s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src5s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src6s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        ippiFilter_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,outputRedGreen2,psb,dstsize,&src7s[0],src2Size,anchor,CONVFILTER_TH);
        for(int i=0; i<width*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
        //ippiFree(outputRedGreen2);
        //ippiCopy_32f_C1R(inputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize);
    }

    else if (IPPISOBEL){
        
        ippiCopyConstBorder_8u_C1R(redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize,redGreenBorder,psb_border,bordersize,1,1,0);
        //ippiFilterSobelHoriz_8u_C1R(redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),outputRedGreen3,psb,srcsize);
        //ippiFilterSobelVert_8u_C1R(redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),outputRedGreen2,psb,srcsize);
        ippiFilterSobelHoriz_8u_C1R(redGreenBorder,psb_border,outputRedGreen3,psb,srcsize);
        ippiFilterSobelVert_8u_C1R(redGreenBorder,psb_border,outputRedGreen2,psb,srcsize);
        for(int i=0; i<psb*height;i++){
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
            outputRedGreen[i]=outputRedGreen3[i];
        }

        /*ippiAdd_8u_C1IRSfs(outputRedGreen2,psb,outputRedGreen3,psb,srcsize,0.5);
        ippiCopy_8u_C1R(outputRedGreen3,psb,outputRedGreen,psb,srcsize);*/

        /*IppiSize msksize={3,3};
        Ipp8u src[3*3]={1,1,1,1,1,1,1,1,1};
        ippiConvValid_8u_C1R(redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize,src,3,msksize,&outputRedGreen[1 + width*1],psb,5);*/

        
        //ippiFree(outputRedGreen2);
        //ippiFree(outputRedGreen3);
    }
    if (IPPICROSS){
        conv_8u_to_32f(redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),outputRedGreen32,psb32,srcsize);
        ippiFilterSobelCross_32f_C1R(outputRedGreen32,psb32,outputRedGreen32B,psb32,srcsize,ippMskSize3x3);
        conv_32f_to_8u(outputRedGreen32B,psb32,outputRedGreen2,psb,srcsize);
        for(int i=0; i<psb*height;i++){
            if(outputRedGreen[i]<outputRedGreen2[i])
                outputRedGreen[i]=outputRedGreen2[i];
        }
    }

    if (OPENCVSOBEL){
        
        IppiSize msksize={3,3};
        if(redGreen_flag){
            cvSobel(redGreen_yarp->getIplImage(),cvImage16,1,1,3);
            cvConvertScale(cvImage16,cvImage8);
        }
        else
            cvImage8=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
        ippiCopy_8u_C1R((unsigned char*)cvImage8->imageData,cvImage8->widthStep,outputRedGreen2,this->psb,srcsize);
        Ipp8u src[3*3]={1,maskSeed,1,
                        maskSeed,maskTop,maskSeed,
                        1,maskSeed,1};
        ippiConvValid_8u_C1R(outputRedGreen2,psb,srcsize,src,3,msksize,&outputRedGreen[1 + width*1],psb,1);
        //ippiFilterMedian_8u_C1R((unsigned char*)cvImage->imageData,width,outputRedGreen,psb,srcsize,msksize,anchor);
        //ippiFree(outputRedGreen2);
    }


    //ippiConvValid_8u_C1R(redGreen_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputBlueYellow[1 + width*1],psb,10);
    //ippiConvValid_8u_C1R(blueYellow_yarp->getPixelAddress(0,0),width,srcsize,src2,3,src2Size,&outputRedGreen[1 + width*1],psb,10);
    //ippiFilterSobelCross_8s16s_C1R(redGreen_ippi,width,src2,3*sizeof(Ipp16s),srcs,ippMskSize5x5);
    //printf("result of the computation [%d,%d,%d;%d,%d,%d;;%d,%d,%d]",src2[0],src2[1],src2[2],src2[3],src2[4],src2[5],src2[6],src2[7],src2[8]);
    
    //4. thresholding the images
    //redGreenEdges=new ImageOf<PixelMono>;
    //redGreenEdges->resize(width,height);
    //if((unsigned int)redGreenEdges==0xcdcdcdcd){
    //	redGreenEdges=new ImageOf<PixelMono>;
    //	redGreenEdges->resize(width,height);
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
    //blankBorder(outputRedGreen);
    ippiCopy_8u_C1R(outputRedGreen,psb,redGreenEdges->getRawImage(),redGreenEdges->getRowSize(),srcsize);
    //ippiCopy_8u_C1R(redGreen_yarp->getRawImage(),psb,redGreenEdges->getRawImage(),redGreenEdges->getRowSize(),srcsize);
    //ippiAdd_8u_C1IRSfs(redGreenEdges->getPixelAddress(0,0),width,redGreenEdges->getPixelAddress(0,0),width,srcsize,0.5);

    
    //ippiFree(inputRedGreen32);
    //ippiFree(outputRedGreen);
    //ippiFree(outputRedGreen2);
    //ippiFree(outputRedGreen3);
    

    //return redGreenEdges;
}

/**
*blank maxk numbers of rows from the top and the bottom of the image
* @param image input image
* @param maxk number of lines removed both from the top and the bottm
*/
void ImageProcessor::blankBorder(Ipp8u* image, int maxk){
   // int height=320;
   // int width=240;

    
    int y0=0,ylast=height-1;
    
    for(int x=0;x<width;x++)
     for(int k=0;k<maxk;k++){
        image[x+(y0+k)*width]=(Ipp8u)0;
        //image[x+(ylast-k)*width]=(Ipp8u)0;
    }
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
        ippiSubC_8u_C1RSfs(src->getPixelAddress(0,0),width,*minValue_ippi,out_ippi,psb,srcsize,1);
        //ippiSubC_8u_C1RSfs(src->getPixelAddress(0,0),width,*minValue_ippi,out_ippi,psb,srcsize,0.5);
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

/**
* function that processes the input image accordingly to the processing options selected for this processor
* @param src source image of the processing
*/

ImageOf<PixelRgb>* ImageProcessor::process (ImageOf<PixelRgb> *src){
    //ImageOf<PixelMono> *image_tmp=new ImageOf<PixelMono>;
    //ImageOf<PixelRgb> *image_out=new ImageOf<PixelRgb>;
    this->width=src->width();
    this->height=src->height();
    //image_tmp->resize(width,height);
    //image_out->resize(width,height);
    IppiSize srcsize;
    srcsize.height=src->height();
    srcsize.width=src->width();
    if(image_tmp==NULL){
        printf("process:image_tmp NULL");
        //image_tmp=new ImageOf<PixelMono>;
        image_tmp->resize(width,height);
    }
    if(image_out==NULL){
        printf("process:image_out NULL");
        //image_out=new ImageOf<PixelRgb>;
        image_out->resize(width,height);
    }
    
    //int image_tmp_flag=0;

    if(canProcess_flag==0){
        return NULL;
    }

    
        if(this->redPlane_flag){
            if(this->colourOpponency_flag){
                printf("runs colourOpponency");
                //this->colourOpponency(src);
                if(this->findEdges_flag){
                    printf("runs findsEdges");
                    if(this->combineMax_flag){
                        //image_tmp=this->combineMax();
                    }
                    else{
                       // image_tmp=this->findEdgesRedOpponency();
                    }
                }
                else
                    image_tmp=this->redGreen_yarp;
            }
            else{
                //this->colourOpponency(src);
                if(this->findEdges_flag){
                    //image_tmp=this->findEdgesRedOpponency();
                }
                else{
                    //image_tmp=this->getRedPlane(src);
                    image_tmp=redPlane;
                }
                    
            }
        }
        else if(bluePlane_flag)
            if(this->colourOpponency_flag){
                //this->colourOpponency(src);
                
                if(this->findEdges_flag){
                    if(this->combineMax_flag){
                        //image_tmp=this->combineMax();
                    }
                    else{
                        //image_tmp=this->findEdgesBlueOpponency();
                    }
                }
                else
                    image_tmp=this->blueYellow_yarp;
            }
            else{
                //this->colourOpponency(src);
                if(this->findEdges_flag){
                    //image_tmp=this->findEdgesBlueOpponency();
                }
                else{
                    //image_tmp=this->getBluePlane(src);
                    ippiCopy_8u_C1R(bluePlane->getRawImage(),256,image_tmp->getRawImage(),256,srcsize);
                }
            }
        else if(greenPlane_flag)
            if(this->colourOpponency_flag){
                //this->colourOpponency(src);
                if(this->findEdges_flag){
                    if(this->combineMax_flag){
                        //image_tmp=this->combineMax();
                    }
                    else{
                        //image_tmp=this->findEdgesGreenOpponency();
                    }
                }
                else
                    image_tmp=this->greenRed_yarp;

            }
            else{
                //this->colourOpponency(src);
                if(this->findEdges_flag){
                    //image_tmp=this->findEdgesGreenOpponency();
                }
                else{
                    //image_tmp=this->getGreenPlane(src);
                    image_tmp=greenPlane;
                }
            }
        else if(this->yellowPlane_flag){
                //this->colourOpponency(src);
                printf("yellow image size %d \n",yellowPlane->getRowSize() );
                ippiCopy_8u_C1R(yellowPlane->getRawImage(),yellowPlane->getRowSize(),image_tmp->getRawImage(),yellowPlane->getRowSize(),srcsize);
        }

    

    /*if(this->normalize_flag){
        image_tmp=this->normalize(image_tmp);
    }*/
    
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
        printf("runs the code for multiplexing 1 channel to 3 \n");
        im_out = ippiMalloc_8u_C1(width,height,&psb);
        ippiCopy_8u_C1R(image_tmp->getRawImage(), image_tmp->getRowSize(),im_out,psb,srcsize);
        Ipp8u* im_tmp[3]={im_out,im_out,im_out};
        //im_tmp[0]=ippiMalloc_8u_C1(width,height,&psb);
        //im_tmp[1]=ippiMalloc_8u_C1(width,height,&psb);
        //im_tmp[2]=ippiMalloc_8u_C1(width,height,&psb);
        
        //ippiCopy_8u_C1R(im_out,psb,im_tmp[0],psb,srcsize);
        //ippiCopy_8u_C1R(im_out,psb,im_tmp[1],psb,srcsize); 
        //ippiCopy_8u_C1R(im_out,psb,im_tmp[2],psb,srcsize);
        //im_tmp[0]=im_out;
        //im_tmp[1]=im_out;
        //im_tmp[2]=im_out;
        //the second transforms the 3-channel image into colorImage for yarp
        ippiCopy_8u_P3C3R(im_tmp,psb,image_out->getPixelAddress(0,0),image_out->getRowSize(),srcsize); 
        //ippiCopy_8u_C3R(src->getPixelAddress(0,0),width*3,image_out->getPixelAddress(0,0),width*3,srcsize);
        printf("copied to the portImage \n");
        this->portImage=image_out;
        ippiFree(im_out);
        //ippiFree(im_tmp);
        //ippiFree(im_tmp[0]);
        //ippiFree(im_tmp[1]);
        //ippiFree(im_tmp[2]);
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
    //int x, y, z;
    //int Offset;
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
    ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
    ippiCopy_8u_C1R(shift[0],psb,tmp->getRawImage(),psb,srcsize);
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
    //int x, y, z;
    //int Offset;
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
    ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
    ippiCopy_8u_C1R(shift[1],psb,tmp->getRawImage(),psb,srcsize);
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
    //int x, y, z;
    //int Offset;
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
    //ippiCopy_8u_C3P3R(inputImage->getPixelAddress(0,0),width*3,shift,psb,srcsize);
    ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
    ippiCopy_8u_C1R(shift[2],psb,tmp->getRawImage(),psb,srcsize);
    ippiFree(shift[0]);
    ippiFree(shift[1]);
    ippiFree(shift[2]);
    return outputImage;
}


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


 void ImageProcessor::combineMax(ImageOf<PixelMono>* edgesOutput){

    int psb;
    Ipp8u* edgesOutput_ippi=ippiMalloc_8u_C1(width,height,&psb);

    IppiSize srcsize ={width,height};
    
    Ipp8u* edgesBlue_ippi=ippiMalloc_8u_C1(width,height,&psb); 
    Ipp8u* edgesGreen_ippi=ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* edgesRed_ippi=ippiMalloc_8u_C1(width,height,&psb);
    
    
    //Ipp8u* edgesOutput2_ippi=ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* edgesMask_ippi=ippiMalloc_8u_C1(width,height,&psb);
    

 
    //edgesBlue=this->findEdgesBlueOpponency();
    //if(edgesBlue==NULL)
    //    return NULL;
    ippiCopy_8u_C1R(blueYellowEdges_yarp->getRawImage(),blueYellowEdges_yarp->getRowSize(),edgesBlue_ippi,psb,srcsize);

    //edgesGreen=this->findEdgesGreenOpponency();
    //if(edgesGreen==NULL)
    //    return NULL;
    ippiCopy_8u_C1R(greenRedEdges_yarp->getRawImage(),greenRedEdges_yarp->getRowSize(),edgesGreen_ippi,psb,srcsize);

    //edgesRed=this->findEdgesRedOpponency();
    //if(edgesRed==NULL)
    //    return NULL;
    ippiCopy_8u_C1R(redGreenEdges_yarp->getRawImage(),redGreenEdges_yarp->getRowSize(),edgesRed_ippi,psb,srcsize);


    int i=0,r=0,c=0;
    
    
    /*edgesOutput=new ImageOf<PixelMono>;
    edgesOutput->resize(width,height);*/
   
    
    
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


    for(int i=0; i<redGreenEdges_yarp->getRowSize()*height;i++){
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

    ippiThreshold_8u_C1R(edgesOutput_ippi,psb,edgesMask_ippi,psb,srcsize,10,ippCmpLess);
    for (int i=0;i<redGreenEdges_yarp->getRowSize()*height;i++){
        if(edgesMask_ippi[i]==10)
            edgesMask_ippi[i]=0;
        edgesOutput_ippi[i]=edgesMask_ippi[i];
    }

    /*ippiAdd_8u_C1IRSfs(edgesGreen_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize,1);
    ippiAdd_8u_C1IRSfs(edgesRed_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize,1);
    ippiAdd_8u_C1IRSfs(edgesBlue_ippi,psb,edgesOutput->getPixelAddress(0,0),width,srcsize,1);*/
    blankBorder(edgesOutput_ippi,4);
    ippiCopy_8u_C1R(edgesOutput_ippi,psb,edgesOutput->getRawImage(),edgesOutput->getRowSize(),srcsize);
    //edgesOutput->setExternal((unsigned char*)pointerRed,height,width);

    //free the memory
    ippiFree(edgesBlue_ippi);
    ippiFree(edgesRed_ippi);
    ippiFree(edgesGreen_ippi);

    ippiFree(edgesOutput_ippi);

    /*ippiFree(edgesOutput2_ippi);
    ippiFree(edgesMask_ippi);*/
    //return edgesOutput;
    //return redGreenEdges_yarp;
}

ImageOf<PixelMono> ImageProcessor::lineMax(ImageOf<PixelMono> &src)
{
	/*ImageOf<PixelMono> dst;
    dst.resize(width,height);
    int fovHeight=(int)height/3;
    int x,y;
    for (int y=0; y<fovHeight; y++)
		for (int x=0; x<width; x++)
            dst.getPixelAddress(x,y)=src.getPixelAddress(x,y)*2.5; //con soglia 4
			//dst(x,y)=src(x,y)*3.5; //con soglia 6

	for (y=fovHeight; y<height; y++)
		for (int x=0; x<width; x++)
            dst.getPixelAddress(x,y)=src.getPixelAddress(x,y)*(3.51773*exp(-0.00832991*y)); //con soglia 4
			//dst(x,y)=src(x,y)*(5.58286*exp(-0.011389*y)); //con soglia 6
    return dst;*/
    return src;
}

ImageOf<PixelMono>* ImageProcessor::subtract(ImageOf<PixelMono> *src1, ImageOf<PixelMono> *src2){
    int width=src1->width();
    int height=src1->height();
    ImageOf<PixelMono>* outputImage=new ImageOf<PixelMono>;
    outputImage->resize(width,height);
    IppiSize srcsize ={width,height};
    //memory allocation	
    int psb;
//	int psb4;
    Ipp8u* gray_in1 = ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* gray_in2 = ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u* gray_out = ippiMalloc_8u_C1(width,height,&psb);
    ippiCopy_8u_C1R(src1->getPixelAddress(0,0),width,gray_in1,psb,srcsize);
    ippiCopy_8u_C1R(src2->getPixelAddress(0,0),width,gray_in2,psb,srcsize);
    //ippiSub_8u_C1RSfs(gray_in1,psb,gray_in2,psb,gray_out,psb,srcsize,0.3);
    ippiSub_8u_C1RSfs(gray_in1,psb,gray_in2,psb,gray_out,psb,srcsize,1);
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


//----- end-of-file --- ( next line intentionally left blank ) ------------------

