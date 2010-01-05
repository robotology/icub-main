#include <iCub/yuvProcessorThread.h>
#include <cassert>


yuvProcessorThread::yuvProcessorThread():RateThread(THREAD_RATE_YUV)
{
    reinit_flag=false;
    
    redPlane=0;    
    greenPlane=0;
    bluePlane=0;

    yPlane=0;
    uPlane=0;
    vPlane=0;
    uvPlane=0;
}

yuvProcessorThread::~yuvProcessorThread()
{
    delete redPlane;
    delete greenPlane;
    delete bluePlane;    
    delete yPlane;
    delete uPlane;
    delete vPlane;
    delete uvPlane;
}


void yuvProcessorThread::reinitialise(){
    
    shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
	shift[1]=ippiMalloc_8u_C1(width,height,&psb);
	shift[2]=ippiMalloc_8u_C1(width,height,&psb);
   
    yPlane=new ImageOf<PixelMono>;
    yPlane->resize(width,height);
    uPlane=new ImageOf<PixelMono>;
    uPlane->resize(width,height);
    vPlane=new ImageOf<PixelMono>;
    vPlane->resize(width,height);
    uvPlane=new ImageOf<PixelMono>;
    uvPlane->resize(width,height);
}

bool yuvProcessorThread::threadInit(){
    printf("Thread initialisation.. \n");
    return true;
}


void yuvProcessorThread::run(){
    if((0==redPlane)||(0==greenPlane)||(0==bluePlane)){
        return;	
    }
    extractYUV();
    addUVPlanes();

}
   


void yuvProcessorThread::threadRelease(){
    printf("Thread releasing.. \n");
    delete redPlane;
    delete greenPlane;
    delete bluePlane;    
    delete yPlane;
    delete uPlane;
    delete vPlane;
    delete uvPlane;
}



void yuvProcessorThread::setInputImage(ImageOf<PixelMono>* redImage,ImageOf<PixelMono>* greenImage,ImageOf<PixelMono>* blueImage){
    
    this->redPlane=redImage;
    this->greenPlane=greenImage;
    this->bluePlane=blueImage;
    width=redPlane->width();
    height=redPlane->height();
    srcsize.height=height;
    srcsize.width=width;
    reinitialise();
}

/*
 * extracts the y channel (intensity) from the input image
 */
void yuvProcessorThread::getYPlane(ImageOf<PixelMono>* tmp){
 

     py=tmp->getPixelAddress(0,0);
     pr=redPlane->getPixelAddress(0,0);
     pg=greenPlane->getPixelAddress(0,0);
     pb=bluePlane->getPixelAddress(0,0);

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            double value=yr*(double)(*pr);
            (*py)=yr*(*pr)+yg*(*pg)+yb*(*pb);
            py++;
            pr++;pg++;pb++;
        }
    }
}

/*
 * extracts the U channel from the input image
 */
void yuvProcessorThread::getUPlane(ImageOf<PixelMono>* tmp){
     pu=tmp->getPixelAddress(0,0);
     pr=redPlane->getPixelAddress(0,0);
     pg=greenPlane->getPixelAddress(0,0);
     pb=bluePlane->getPixelAddress(0,0);

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            double value=ur*(double)(*pr);
            (*pu)=ur*(*pr)+ug*(*pg)+ub*(*pb)+128;
            assert((*pu)>=0);
            assert((*pu)<=255);
            pu++;
            pr++;pg++;pb++;
        }
    }
}

/*
 * extracts the V channel from the input image
 */
void yuvProcessorThread::getVPlane(ImageOf<PixelMono>* tmp){
	 pv=tmp->getPixelAddress(0,0);
     pr=redPlane->getPixelAddress(0,0);
     pg=greenPlane->getPixelAddress(0,0);
     pb=bluePlane->getPixelAddress(0,0);

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            double value=vr*(double)(*pr);
            (*pv)=vr*(*pr)+vg*(*pg)+vb*(*pb)+128;
            assert((*pv)>=0);
            assert((*pv)<=255);
            pv++;
            pr++;pg++;pb++;
        }
    }
}

/*
 * extracts the UV channel adding Uand V planes
 */
void yuvProcessorThread::addUVPlanes(){
	pv=vPlane->getPixelAddress(0,0);
    pu=uPlane->getPixelAddress(0,0);
    puv=uvPlane->getPixelAddress(0,0);

    for (int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            double value=(double)((*pu)+(*pv));
            (*puv)=((double)((*pu)+(*pv))/510)*255;
            assert((*pv)>=0);
            assert((*pv)<=255);
            puv++;
            pv++;pu++;
        }
    }
}

void yuvProcessorThread::extractYUV(){
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
