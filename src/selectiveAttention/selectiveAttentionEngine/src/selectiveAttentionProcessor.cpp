// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/selectiveAttentionProcessor.h>



#include <ipp.h>
#include <ipps.h>
//#include <qimage.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>




// available methods for edges detection
//#define CONVMAX
//#define CONVFILTER
//#define IPPISOBEL
//#define OPENCVSOBEL
//#define CONVSEQ

selectiveAttentionProcessor::selectiveAttentionProcessor():RateThread(THREAD_RATE){
    this->inImage=NULL;
    

    maskSeed=4;
    maskTop=20;

    //flags
    inputImage_flag=0;
    idle=true;
    resized_flag=false;
    
    k1=0.5;
    k2=0.0;
    k3=0.0;
    k4=0.0;
    k5=0;
    k6=0;

    // images
    edges_yarp=new ImageOf<PixelMono>;
    tmp=new ImageOf<PixelMono>;
    
    map1_yarp=new ImageOf<PixelMono>;
    map2_yarp=new ImageOf<PixelMono>;
    map3_yarp=new ImageOf<PixelMono>;
    map4_yarp=new ImageOf<PixelMono>;
    map5_yarp=new ImageOf<PixelMono>;
    map6_yarp=new ImageOf<PixelMono>;
    

    map1_ippi = 0;
    map2_ippi = 0;
    map3_ippi = 0;
    map4_ippi = 0;
    map5_ippi = 0;
    map6_ippi = 0;


    tmp=new ImageOf<PixelMono>;
    
    image_out=new ImageOf<PixelRgb>;
    image_tmp=new ImageOf<PixelMono>;
    outputImage=new ImageOf<PixelMono>;
    linearCombinationImage=new ImageOf<PixelMono>;
        
}

selectiveAttentionProcessor::~selectiveAttentionProcessor(){
    printf("Destructor /n");
    delete inImage;
   // delete portImage;
    delete edges_yarp;

    delete map1_yarp;
    delete map2_yarp;
    delete map3_yarp;	
    delete map4_yarp;
    delete map5_yarp;
    delete map6_yarp;
    
    ippiFree(map1_ippi );
    ippiFree(map2_ippi );
    ippiFree(map3_ippi );
    ippiFree(map4_ippi );
    ippiFree(map5_ippi );
    ippiFree(map6_ippi );

    delete tmp;
    delete image_out;
    delete image_tmp;
    delete outputImage;
    
}

selectiveAttentionProcessor::selectiveAttentionProcessor(ImageOf<PixelRgb>* inputImage):RateThread(THREAD_RATE){
    this->inImage=inputImage;
//    this->portImage=portImage;
 


    //IppiSize srcsize={320,240};

    
    
    //edgesOutput=new ImageOf<PixelMono>;
    //portImage=new ImageOf<PixelRgb>;
    
    tmp=new ImageOf<PixelMono>;
    
    
    
    
}
/**
* 
*/
void selectiveAttentionProcessor::resizeImages(int width,int height){
    this->height=height;
    this->width=width;

    
    IppiSize srcsize={width,height};
    tmp->resize(width,height);
    //portImage->resize(width,height);
    map1_yarp->resize(width,height);
    map2_yarp->resize(width,height);
    map3_yarp->resize(width,height);
    map4_yarp->resize(width,height);
    map5_yarp->resize(width,height);
    map6_yarp->resize(width,height);

    

    if(map1_ippi ==0){
        map2_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map3_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map4_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map5_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map6_ippi  = ippiMalloc_8u_C1(width,height,&psb);
    }

    tmp->resize(width,height);
    image_out->resize(width,height);
    image_tmp->resize(width,height);
    outputImage->resize(width,height);
    linearCombinationImage->resize(width,height);

    cvImage16= cvCreateImage(cvSize(width,height),IPL_DEPTH_16S,1);
    cvImage8= cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
    
    resized_flag=true;
}

/**
*	initialization of the thread 
*/
bool selectiveAttentionProcessor::threadInit(){
    printf("Thread initialization .... \n");
    return true;
}
/**
* active loop of the thread
*/
void selectiveAttentionProcessor::run(){
    IppiSize srcsize;
    srcsize.height=this->height;
    srcsize.width=this->width;
    int rowSize=outputImage->getRowSize();
    unsigned char maxValue=0;
    if(!idle){
        for(int y=0;y<height;y++){
            for(int x=0;x<width;x++){
                unsigned char value=0;
                if(map1_yarp!=0)
                    value+=(unsigned char)map1_yarp->getRawImage()[x+y*rowSize]*k1;
                if(map2_yarp!=0)
                    value+=(unsigned char)map2_yarp->getRawImage()[x+y*rowSize]*k2;
                if(map3_yarp!=0)
                    value+=(unsigned char)map3_yarp->getRawImage()[x+y*rowSize]*k3;
                if(map4_yarp!=0)
                    value+=(unsigned char)map4_yarp->getRawImage()[x+y*rowSize]*k4;
                if(map5_yarp!=0)
                    value+=(unsigned char)map5_yarp->getRawImage()[x+y*rowSize]*k5;
                if(map6_yarp!=0)
                    value+=(unsigned char)map6_yarp->getRawImage()[x+y*rowSize]*k6;
                if((map1_yarp==0)&&(map2_yarp==0)&&(map3_yarp==0)&&(map4_yarp==0)&&(map5_yarp==0)&&(map6_yarp==0))
                    value=0;
                else
                    linearCombinationImage->getRawImage()[x+y*rowSize]=value;
                if(maxValue<value)
                    maxValue=value;
                linearCombinationImage->getRawImage()[x+y*rowSize]=value;
            }
        }
                
        for(int y=0;y<height;y++){
            for(int x=0;x<width;x++){
                if(maxValue==0)
                    outputImage->getRawImage()[x+y*rowSize]=0;
                if(linearCombinationImage->getRawImage()[x+y*rowSize]==maxValue)
                    outputImage->getRawImage()[x+y*rowSize]=255;
                else    
                    outputImage->getRawImage()[x+y*rowSize]=0;
            }   
        }
        
        extractContour(outputImage,outputImage,centroid_x,centroid_y);
        //ippiCopy_8u_C1R((const Ipp8u *)dst->imageData,dst->widthStep, outputImage->getRawImage(), outputImage->getRowSize(),srcsize);

    }
    
}

void selectiveAttentionProcessor::extractContour(ImageOf<PixelMono>* inputImage,ImageOf<PixelMono>* outputImage,int& x,int& y){
    IppiSize srcsize;
    srcsize.height=inputImage->height();
    srcsize.width=inputImage->width();
    CvMemStorage* stor=cvCreateMemStorage(0);
    CvBox2D box;
    CvSeq* cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
    cvFindContours( inputImage->getIplImage(), stor, &cont, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    IplImage* dst = cvCreateImage( cvGetSize(outputImage->getIplImage()), 8, 1 );
    cvZero(dst);   
    double numObj = 0;
    float line[4];
    CvPoint center;
    CvPoint pt1;
    CvPoint pt2;

    for(;cont;cont = cont->h_next){
        numObj ++;
        //  int count = cont->total; // This is number point in contour
        box = cvMinAreaRect2(cont, stor);
        center.x = cvRound(box.center.x);
        center.y = cvRound(box.center.y);
        x=center.x;
        y=center.y;
        float v = box.size.width;
        float v1 = box.size.height;
           
        cvDrawContours(dst,cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1,8);
        cvCircle (dst, center, 3, CV_RGB(255,255,255));
           
        cvFitLine(cont, CV_DIST_L2, 1, 0.01, 0.01, line);
        float t = (v + v1)/2;
        pt1.x = cvRound(line[2] - line[0] *t );
        pt1.y = cvRound(line[3] - line[1] *t );
        pt2.x = cvRound(line[2] + line[0] *t );
        pt2.y = cvRound(line[3] + line[1] *t );

        cvCircle(dst, pt1, 3, CV_RGB(255,0,255));
        cvCircle(dst, pt2, 3, CV_RGB(255,0,255));
       
        cvLine( dst, pt1, pt2, CV_RGB(255,0,0), 2, CV_AA, 0);
        double theta = 0;
        // double theta = 180 / M_PI * atan2( (pt2.y - pt1.y) , (pt2.x - pt1.x) );

        //sprintf(Angles, "%d%d",x,y);
        //cvPutText ( dst , Angles, cvPoint( 10, 10), &font, cvScalar(255,0,0) );

    }
    //ippiCopy_8u_C1R((const Ipp8u *)dst->imageData,dst->widthStep, outputImage->getRawImage(), outputImage->getRowSize(),srcsize);
    cvReleaseMemStorage(&stor);
    cvReleaseImage(&dst);
    //cvReleaseMemStorage(&storage);

        
}

/**
*	releases the thread
*/
void selectiveAttentionProcessor::threadRelease(){
    printf("Thread realeasing .... \n");
    
}

void selectiveAttentionProcessor::setIdle(bool value){
    mutex.wait();
    idle=value;
    mutex.post();
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

