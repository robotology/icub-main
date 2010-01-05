#include <iCub/blobFinderThread.h>

#include <iostream>
using namespace std;

#define _inputImg (*(ptr_inputImg))
#define _inputImgRed (*(ptr_inputImgRed))
#define _inputImgGreen (*(ptr_inputImgGreen))
#define _inputImgBlue (*(ptr_inputImgBlue))
#define _inputImgRG (*(ptr_inputImgRG))
#define _inputImgGR (*(ptr_inputImgGR))
#define _inputImgBY (*(ptr_inputImgBY))
#define _tagged (*(ptr_tagged))

blobFinderThread::blobFinderThread(){
    reinit_flag=false;
}


void blobFinderThread::reinitialise(int width, int height){
    img=new ImageOf<PixelRgb>;
    img->resize(width,height);
    this->width=width;
    this->height=height;
    resizeImages(width,height);
}

void blobFinderThread::resizeImages(int width, int height){
    this->width=width;
    this->height=height;

    int widthstep=256;

    ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
    ptr_tagged->resize(widthstep,height);
    
    this->wOperator=new WatershedOperator(true,width,height,widthstep,10);
    //_wOperator=this->wOperator;
    this->salience=new SalienceOperator(width,height);
    //_salience=this->salience;

    //maxSalienceBlob_img->resize(width,height);
    //outMeanColourLP->resize(width,height);
    //outContrastLP->resize(width,height);

    //image_out->resize(width,height);
    _outputImage->resize(width,height);
    //_outputImage3->resize(width,height);

    //ptr_inputRed->resize(width,height);
    //ptr_inputGreen->resize(width,height);
    //ptr_inputBlue->resize(width,height);
    //ptr_inputRG->resize(width,height);
    //ptr_inputGR->resize(width,height);
    //ptr_inputBY->resize(width,height);

    _inputImgRGS->resize(width,height);
    _inputImgGRS->resize(width,height);
    _inputImgBYS->resize(width,height);
    //blobFov->resize(width,height);

    //blobList = new char [width*height+1];

    _inputImg.resize(width,height);
    _inputImgRed.resize(width,height);
    _inputImgGreen.resize(width,height);
    _inputImgBlue.resize(width,height);
    _inputImgRG.resize(width,height);
    _inputImgGR.resize(width,height);
    _inputImgBY.resize(width,height);

    resized_flag=true;
}



/**
*	initialization of the thread 
*/
bool blobFinderThread::threadInit(){
    return true;
}

/**
* active loop of the thread
*/
void blobFinderThread::run(){
    rain();
}
/**
*	releases the thread
*/
void blobFinderThread::threadRelease(){

}

/**
* applies the watershed (rain falling) algorithm
*/
void blobFinderThread::rain(){
    max_tag=wOperator->apply(*_outputImage,_tagged);
    //printf("MAX_TAG=%d",wModule->max_tag);
    /*bool ret=getPlanes();
    if(ret==false){
        //printf("No Planes! \n");
        //return;
    }
    ret=getOpponencies();
    if(ret==false){
        //printf("No Opponency! \n");
        //return;
    }*/
    int psb32s;
    IppiSize srcsize={this->width,this->height};
    Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(this->width,this->height,&psb32s);
    Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(this->width,this->height,&psb32s);
    Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(this->width,this->height,&psb32s);
    /*ImageOf<PixelMono> *_inputImgRGS=new ImageOf<PixelMono>;
    ImageOf<PixelMono> *_inputImgGRS=new ImageOf<PixelMono>;
    ImageOf<PixelMono> *_inputImgBYS=new ImageOf<PixelMono>;
    _inputImgRGS->resize(320,240);
    _inputImgGRS->resize(320,240);
    _inputImgBYS->resize(320,240);*/
    if(ptr_inputImgRG!=0){
        ippiScale_8u32s_C1R(_inputImgRG.getRawImage(),_inputImgRG.getRowSize(),_inputImgRGS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgRGS32,psb32s,(Ipp8s*)_inputImgRGS->getRawImage(),_inputImgRGS->getRowSize(),srcsize);
        //_inputImgRGS->copy(_inputImgRG,320,240);
    }
    else
        return;
    if(ptr_inputImgGR!=0){
        ippiScale_8u32s_C1R(_inputImgGR.getRawImage(),_inputImgGR.getRowSize(),_inputImgGRS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgGRS32,psb32s,(Ipp8s*)_inputImgGRS->getRawImage(),_inputImgGRS->getRowSize(),srcsize);
        //_inputImgGRS->copy(_inputImgGR,320,240);
    }
    else
        return;
    if(ptr_inputImgBY!=0){
        ippiScale_8u32s_C1R(_inputImgBY.getRawImage(),_inputImgBY.getRowSize(),_inputImgBYS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgBYS32,psb32s,(Ipp8s*)_inputImgBYS->getRawImage(),_inputImgBYS->getRowSize(),srcsize);
        //_inputImgBYS->copy(_inputImgBY,320,240);
    }
    else
        return;
    salience->blobCatalog(_tagged, *_inputImgRGS, *_inputImgGRS, *_inputImgBYS,
        _inputImgBlue, _inputImgGreen, _inputImgRed, max_tag);
    blobCataloged_flag=true;
    //istruction to set the ptr_tagged in the Watershed Module with the static variable _tagged
    tagged=ptr_tagged; //ptr_tagged is the pointer to _tagged
    ippiFree(_inputImgRGS32); //Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
    ippiFree(_inputImgGRS32); //Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
    ippiFree(_inputImgBYS32); //Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
}

