#include <iCub/rgbProcessorThread.h>
#include <iCub/convert_bitdepth.h>
#include <ipps.h>
#include <iostream>
#include <string>
#include <cassert>
#include <cmath>


using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

rgbProcessorThread::rgbProcessorThread()//:RateThread(THREAD_RATE)
{
    reinit_flag=false;
    interrupted_flag=false;
    
    redPlane=0;    
    greenPlane=0;
    bluePlane=0;

    redGreen_yarp=0;
    greenRed_yarp=0;
    blueYellow_yarp=0;

    img=0;

    string name("");
}

rgbProcessorThread::~rgbProcessorThread()
{
    delete redPlane;
    delete greenPlane;
    delete bluePlane;    
    delete greenRed_yarp;
    delete redGreen_yarp;
    delete blueYellow_yarp;
}

/*processorThread::processorThread(Property &op):processorThread(){
        
        
}*/

void rgbProcessorThread::reinitialise(int width, int height){
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

   
    yPlane=new ImageOf<PixelMono>;
    yPlane->resize(width,height);
    uPlane=new ImageOf<PixelMono>;
    uPlane->resize(width,height);
    vPlane=new ImageOf<PixelMono>;
    vPlane->resize(width,height);
    uvPlane=new ImageOf<PixelMono>;
    uvPlane->resize(width,height);


    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(width, height);



    //1. ------------------ Initialisation ---------------------------
    /*redPlane_ippi= ippiMalloc_8u_C1(width,height,&psb);
    greenPlane_ippi= ippiMalloc_8u_C1(width,height,&psb);
    bluePlane_ippi= ippiMalloc_8u_C1(width,height,&psb);*/ 
    yellowPlane_ippi = ippiMalloc_8u_C1(width,height,&psb);

    redGreen_ippi= ippiMalloc_8u_C1(width,height,&psb);
    greenRed_ippi= ippiMalloc_8u_C1(width,height,&psb);
    blueYellow_ippi= ippiMalloc_8u_C1(width,height,&psb);

    
	//2.2 convert the shifted planes to the IPP Image (unsigned 8bit)
	bluePlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	redPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	yellowPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
	greenPlane_ippi_f = ippiMalloc_8u_C1(width,height,&psb);
    
	
	//3. convolve with a gaussian in order to filter image planes red and green
	
	/*ippiFilterGauss_8u_C1R(bluePlane_ippi,psb,bluePlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(redPlane_ippi,psb,redPlane_ippi_f,psb,srcsize,ippMskSize5x5);
	ippiFilterGauss_8u_C1R(greenPlane_ippi,psb,greenPlane_ippi_f,psb,srcsize,ippMskSize5x5);*/

	
    //int psb2;
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

    redPlane_ippi8u_f = ippiMalloc_8u_C1(width,height,&psb);
	bluePlane_ippi8u_f = ippiMalloc_8u_C1(width,height,&psb);
	yellowPlane_ippi8u_f = ippiMalloc_8u_C1(width,height,&psb);
	greenPlane_ippi8u_f = ippiMalloc_8u_C1(width,height,&psb);

    	
    int pBufferSize;
	ippiFilterGaussGetBufferSize_32f_C1R(srcsize, 3, &pBufferSize);
	
	pBufferBlue = ippsMalloc_8u(pBufferSize);
	pBufferRed = ippsMalloc_8u(pBufferSize);
	pBufferGreen = ippsMalloc_8u(pBufferSize);
    
    
}

void rgbProcessorThread::setName(string str){
   this->name=str;
   printf("name: %s", name.c_str());
}


std::string rgbProcessorThread::getName(const char* p){
    string str(name);
    str.append(p);
    //printf("name: %s", name.c_str());
    return str;
}

bool rgbProcessorThread::threadInit(){
    printf("Thread initialisation.. \n"); 
    inputPort.open(getName("/image:i").c_str());
        
    redPort.open(getName("/red:o").c_str());
    greenPort.open(getName("/green:o").c_str());
    bluePort.open(getName("/blue:o").c_str());

    rgPort.open(getName("/rg:o").c_str());
    grPort.open(getName("/gr:o").c_str());
    byPort.open(getName("/by:o").c_str());

    yPort.open(getName("/ychannel:o").c_str());
    uPort.open(getName("/uchannel:o").c_str());
    vPort.open(getName("/vchannel:o").c_str());
    uvPort.open(getName("/uvchannel:o").c_str());
    return true;
}


void rgbProcessorThread::run(){
    while (!isStopping()) {
        if(!interrupted_flag){
            img=this->inputPort.read(true);
            //printf("out of the waiting.... \n");

            if(0!=img){
                if(!reinit_flag){
                    this->width=img->width();
                    this->height=img->height();
                    srcsize.height=img->height();
	                srcsize.width=img->width();
                    reinitialise(img->width(), img->height());
                    reinit_flag=true;
                }
                
                
                //copy the inputImg into a buffer
                //ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),inputImg->getRawImage(), inputImg->getRowSize(),srcsize);
                
                extractPlanes(img);
                //colourOpponency();
                
                //extractYUV();
                //addUVPlanes();

                outPorts();   
            }//if
        }//if
    }//while
    
}

void rgbProcessorThread::interrupted(){
    printf("input port closing .... \n");
    inputPort.interrupt();
    
    interrupted_flag=true;
    
}

/*
 * extracts the y channel (intensity) from the input image
 */
void rgbProcessorThread::getYPlane(ImageOf<PixelMono>* tmp){
 

     py=tmp->getPixelAddress(0,0);
     pr=redPlane->getPixelAddress(0,0);
     pg=greenPlane->getPixelAddress(0,0);
     pb=bluePlane->getPixelAddress(0,0);

     int rowsize=redPlane->getRowSize();
     int padding=rowsize-width;

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            (*py)=(unsigned char)ceil(yr*(*pr)+yg*(*pg)+yb*(*pb));
            py++;
            pr++;pg++;pb++;
        }
        for (int i=0;i<padding;i++){
            py++;
            pr++;pg++;pb++;
        }
    }
}

/*
 * extracts the U channel from the input image
 */
void rgbProcessorThread::getUPlane(ImageOf<PixelMono>* tmp){
     pu=tmp->getPixelAddress(0,0);
     pr=redPlane->getPixelAddress(0,0);
     pg=greenPlane->getPixelAddress(0,0);
     pb=bluePlane->getPixelAddress(0,0);

     int rowsize=redPlane->getRowSize();
     int padding=rowsize-width;

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            (*pu)=(unsigned char)ceil(ur*(*pr)+ug*(*pg)+ub*(*pb)+128);
            assert((*pu)>=0);
            assert((*pu)<=255);
            pu++;
            pr++;pg++;pb++;
        }
        for (int i=0;i<padding;i++){
            pu++;
            pr++;pg++;pb++;
        }
    }

    
}

/*
 * extracts the V channel from the input image
 */
void rgbProcessorThread::getVPlane(ImageOf<PixelMono>* tmp){
	 pv=tmp->getPixelAddress(0,0);
     pr=redPlane->getPixelAddress(0,0);
     pg=greenPlane->getPixelAddress(0,0);
     pb=bluePlane->getPixelAddress(0,0);

     int rowsize=redPlane->getRowSize();
     int padding=rowsize-width;

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            (*pv)=(unsigned char)ceil(vr*(*pr)+vg*(*pg)+vb*(*pb)+128);
            assert((*pv)>=0);
            assert((*pv)<=255);
            pv++;
            pr++;pg++;pb++;
        }
        for (int i=0;i<padding;i++){
            pv++;
            pr++;pg++;pb++;
        }
    }
}

/*
 * extracts the UV channel adding Uand V planes
 */
void rgbProcessorThread::addUVPlanes(){
	pv=vPlane->getPixelAddress(0,0);
    pu=uPlane->getPixelAddress(0,0);
    puv=uvPlane->getPixelAddress(0,0);

    int rowsize=redPlane->getRowSize();
    int padding=rowsize-width;

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            double value=(double)((*pu)+(*pv));
            (*puv)=(unsigned char)((value/510)*255);
            //(*puv)= (*pv);
            assert((*pv)>=0);
            assert((*pv)<=255);
            puv++;
            pv++;pu++;
        }
        for (int i=0;i<padding;i++){
            puv++;
            pv++;pu++;
        }
    }
}

void rgbProcessorThread::extractYUV(){
    IppiSize srcsize ={width,height};

	/*tmp=new ImageOf<PixelMono>;
	tmp->resize(width,height);*/

    //1.get the red,blue and green planes
	getYPlane(yPlane);	
	//ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),yPlane->getRawImage(),yPlane->getRowSize(),srcsize);
	
	getUPlane(uPlane);
	//ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),uPlane->getRawImage(),uPlane->getRowSize(),srcsize);

	getVPlane(vPlane);
	//ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),vPlane->getRawImage(),vPlane->getRowSize(),srcsize);
}
   


void rgbProcessorThread::threadRelease(){
    //----------- freeing memory ----------
    //6. free memory space
    //ippiFree(bluePlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	//ippiFree(redPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);
	//ippiFree (greenPlane_ippi); // ippiMalloc_8u_C1(width,height,&psb);

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

    ippiFree(redPlane_ippi8u_f);// = ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(bluePlane_ippi8u_f);// = ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(yellowPlane_ippi8u_f);// = ippiMalloc_8u_C1(width,height,&psb);
	ippiFree(greenPlane_ippi8u_f);//= ippiMalloc_8u_C1(width,height,&psb);

	ippiFree(redGreen_ippi32);
	ippiFree(blueYellow_ippi32);
	ippiFree(greenRed_ippi32);
    
	
	ippiFree(redPlane_ippi32);
	ippiFree(bluePlane_ippi32);
	ippiFree(greenPlane_ippi32);
	
	ippsFree(pBufferBlue);
	ippsFree(pBufferRed);
	ippsFree(pBufferGreen);
    
    
    printf("Thread releasing.. \n");
    printf("input port closing .... \n");
    inputPort.close();
    
    printf("red channel port closing .... \n");
    redPort.close();
    printf("green channel port closing .... \n");
    greenPort.close();
    printf("blue channel port closing .... \n");
    bluePort.close();

    printf("R+G- colourOpponency port closing .... \n");
    rgPort.close();
    printf("G+R- colourOpponency port closing .... \n");
    grPort.close();
    printf("B+Y- colourOpponency port closing .... \n");
    byPort.close();

    yPort.close();
    printf("chrominance channel port closing .... \n");
    uPort.close();
    vPort.close();
    uvPort.close();
}



void rgbProcessorThread::setInputImage(ImageOf<PixelRgb>* inputImage){
    this->img=inputImage;
    this->width=inputImage->width();
    this->height=inputImage->height();
    reinitialise(width,height);
    reinit_flag=true;
}

void rgbProcessorThread::outPorts(){
    //port2.prepare() = *img;	
    //rgb section
    if((redPlane!=0)&&(redPort.getOutputCount())){
        redPort.prepare() = *(redPlane);		
        redPort.write();
    }
    if((bluePlane!=0)&&(bluePort.getOutputCount())){
        bluePort.prepare() = *(bluePlane);		
        bluePort.write();
    }
    if((greenPlane!=0)&&(greenPort.getOutputCount())){
        greenPort.prepare() = *(greenPlane);		
        greenPort.write();
    }

    //colour opponency section
    if((redGreen_yarp!=0)&&(rgPort.getOutputCount())){
        rgPort.prepare()=*(redGreen_yarp);
        rgPort.write();
    }
    if((greenRed_yarp!=0)&&(grPort.getOutputCount())){
        grPort.prepare()=*(greenRed_yarp);
        grPort.write();
    }
    if((blueYellow_yarp!=0)&&(byPort.getOutputCount())){
        byPort.prepare()=*(blueYellow_yarp);
        byPort.write();
    }

     if((yPlane!=0)&&(yPort.getOutputCount())){
        yPort.prepare() = *(yPlane);		
        yPort.write();
    }
    if((uPlane!=0)&&(uPort.getOutputCount())){
        uPort.prepare() = *(uPlane);		
        uPort.write();
    }
    if((vPlane!=0)&&(vPort.getOutputCount())){
        vPort.prepare() = *(vPlane);		
        vPort.write();
    }
    if((uvPlane!=0)&&(uvPort.getOutputCount())){
        uvPort.prepare() = *(uvPlane);		
        uvPort.write();
    }

}
/*
 * extracts the plane red for the input image
 */
void rgbProcessorThread::getRedPlane(ImageOf<PixelRgb>* inputImage,ImageOf<PixelMono>* tmp){
    
	
	//int width=inputImage->width();
	//int height=inputImage->height();
   
	/*IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();*/

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
    
	
	
	/*IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();*/

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
   		
	/*IppiSize srcsize;
	srcsize.height=inputImage->height();
	srcsize.width=inputImage->width();*/

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
    
	
    /*ippiConvert_8u32f_C1R(greenPlane->getRawImage(),greenPlane->getRowSize(),greenPlane_ippi32,psb2,srcsize);
    ippiConvert_8u32f_C1R(bluePlane->getRawImage(),bluePlane->getRowSize(),bluePlane_ippi32,psb2,srcsize);
    ippiConvert_8u32f_C1R(redPlane->getRawImage(),redPlane->getRowSize(),redPlane_ippi32,psb2,srcsize);*/

    
    //ippiCopy_8u_C1R(bluePlane->getRawImage(),bluePlane->getRowSize(),bluePlane_ippi,psb,srcsize);
	//ippiCopy_8u_C1R(redPlane->getRawImage(),redPlane->getRowSize(),redPlane_ippi,psb,srcsize);
    //ippiCopy_8u_C1R(greenPlane->getRawImage(),greenPlane->getRowSize(),greenPlane_ippi,psb,srcsize);

    
    //Conversion planes in signed char
    ippiRShiftC_8u_C1IR(1,redPlane->getRawImage(),redPlane->getRowSize(),srcsize);
    ippiRShiftC_8u_C1IR(1,greenPlane->getRawImage(),greenPlane->getRowSize(),srcsize);
    ippiRShiftC_8u_C1IR(1,bluePlane->getRawImage(),bluePlane->getRowSize(),srcsize);
    //conversion to 32f in order to apply the gauss filter
	ippiConvert_8u32f_C1R(greenPlane->getRawImage(),greenPlane->getRowSize(),greenPlane_ippi32,psb2,srcsize);
	ippiConvert_8u32f_C1R(bluePlane->getRawImage(),bluePlane->getRowSize(),bluePlane_ippi32,psb2,srcsize);
	ippiConvert_8u32f_C1R(redPlane->getRawImage(),redPlane->getRowSize(),redPlane_ippi32,psb2,srcsize);
    
  

	//ippiFilter possible variables for border ippBorderConst,ippBorderRepl,ippBorderWrap,ippBorderMirror,ippBorderMirrorR
	Ipp32f value=0;
	Ipp32f sigma=1;
	int kernelSize=3;
    ippiFilterGaussBorder_32f_C1R(greenPlane_ippi32,psb2,greenPlane_ippi32_f,psb2,srcsize,kernelSize,sigma,ippBorderConst,value,pBufferGreen);
	ippiFilterGaussBorder_32f_C1R(bluePlane_ippi32,psb2,bluePlane_ippi32_f,psb2,srcsize,kernelSize,sigma,ippBorderConst,value,pBufferBlue);
	ippiFilterGaussBorder_32f_C1R(redPlane_ippi32,psb2,redPlane_ippi32_f,psb2,srcsize,kernelSize,sigma,ippBorderConst,value,pBufferRed);
	
    ippiConvert_32f8u_C1R(redPlane_ippi32_f,psb2,redPlane_ippi8u_f,psb,srcsize,ippRndNear);
    ippiConvert_32f8u_C1R(greenPlane_ippi32_f,psb2,greenPlane_ippi8u_f,psb,srcsize,ippRndNear);
    ippiConvert_32f8u_C1R(bluePlane_ippi32_f,psb2,bluePlane_ippi8u_f,psb,srcsize,ippRndNear);
    


    /*ippiSub_8u_C1RSfs(redPlane_ippi8u,psb,greenPlane_ippi8u_f,psb,redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize,0);
    ippiSub_8u_C1RSfs(greenPlane_ippi8u,psb,redPlane_ippi8u_f,psb,greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),srcsize,0);
    ippiRShiftC_8u_C1IR(1,redPlane_ippi8u_f,psb,srcsize);
    ippiRShiftC_8u_C1IR(1,greenPlane_ippi8u_f,psb,srcsize);
    ippiAdd_8u_C1RSfs(redPlane_ippi8u_f,psb,greenPlane_ippi8u_f,psb,yellowPlane_ippi8u_f,psb,srcsize,0);
    ippiSub_8u_C1RSfs(bluePlane_ippi8u,psb,yellowPlane_ippi8u_f,psb,blueYellow_yarp->getRawImage(),blueYellow_yarp->getRowSize(),srcsize,0);*/

    
    /*
	//4.1 subtract the red from the gree    n to obtain R+G-
	ippiSub_32f_C1R(redPlane_ippi32,psb2,greenPlane_ippi32_f,psb2,greenRed_ippi32,psb2,srcsize);
	//ippiSub_8u_C1RSfs(redGreen_ippi,psb,greenPlane_ippi_f,psb,redGreen_ippi,psb,srcsize,-1);
	//4.2 subtract the green from the red to obtain G+R-
	ippiSub_32f_C1R(greenPlane_ippi32,psb2,redPlane_ippi32_f,psb2,redGreen_ippi32,psb2,srcsize);
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
	*/
    


    
    //4.1 subtract the red from the green to obtain R+G-
    ippiSub_32f_C1R(greenPlane_ippi32_f,psb2,redPlane_ippi32,psb2,redGreen_ippi32,psb2,srcsize);
	//ippiSub_8u_C1RSfs(redGreen_ippi,psb,greenPlane_ippi_f,psb,redGreen_ippi,psb,srcsize,-1);
	//4.2 subtract the green from the red to obtain G+R-
	ippiSub_32f_C1R(redPlane_ippi32_f,psb2,greenPlane_ippi32,psb2,greenRed_ippi32,psb2,srcsize);
	//ippiSub_8u_C1RSfs(greenRed_ippi,psb,redPlane_ippi_f,psb,greenRed_ippi,psb,srcsize,-1);
	//4.3 risht shift;
	//ippiRShiftC_32s_C1IR(1,greenPlane_ippi32_f,psb2,srcsize);
	//ippiRShiftC_32s_C1IR(1,redPlane_ippi32_f,psb2,srcsize);
	//4.4 sum the filtered red and filtered green to get the filtered yellow
	ippiAdd_32f_C1R(redPlane_ippi32_f,psb2,greenPlane_ippi32_f,psb2,yellowPlane_ippi32_f,psb2,srcsize);
	//ippiAdd_8u_C1RSfs(redPlane_ippi_f,psb,greenPlane_ippi_f,psb,yellowPlane_ippi_f,psb,srcsize,-1);
	//4.5. subtract the yellow from the blue in order to get B+Y-
	ippiSub_32f_C1R(yellowPlane_ippi32_f,psb2,bluePlane_ippi32,psb2,blueYellow_ippi32,psb2,srcsize);

	//ippiSub_8u_C1RSfs(yellowPlane_ippi_f,psb,blueYellow_ippi,psb,blueYellow_ippi,psb,srcsize,-1);

    


    /*for(int i=0;i< width*height; i++){
        //searchRG=((targetRED-targetGREEN+255)/510)*255;
        //searchGR=((targetGREEN-targetRED+255)/510)*255;
        //PixelMono addRG=((targetRED+targetGREEN)/510)*255;
        //searchBY=((targetBLUE-addRG+255)/510)*255;
        redGreen_ippi32[i]=(((redPlane_ippi32_f[i]-greenPlane_ippi32_f[i])+255)/510)*255;
        greenRed_ippi32[i]=(((greenPlane_ippi32_f[i]-redPlane_ippi32_f[i])+255)/510)*255;
        yellowPlane_ippi32_f[i]= ((redPlane_ippi32_f[i]+greenPlane_ippi32_f[i])/510)*255;
        blueYellow_ippi32[i]=((bluePlane_ippi32_f[i]-yellowPlane_ippi32_f[i])/510)*255;
    }*/


	//5. convesion back to 8u images 
	
    conv_32f_to_8u(blueYellow_ippi32,psb2,blueYellow_yarp->getRawImage(),blueYellow_yarp->getRowSize(),srcsize);
    conv_32f_to_8u(redGreen_ippi32,psb2,redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize);
    conv_32f_to_8u(greenRed_ippi32,psb2,greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),srcsize);

    

    //printf("RG:%d ", redGreen_yarp->getRawImage()[100]);
    //printf("GR:%d ", greenRed_yarp->getRawImage()[100]);
    //printf("BY:%d ", blueYellow_ippi[100]);

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

    
    
    
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------
