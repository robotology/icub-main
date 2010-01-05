#include <iCub/blobFinderThread.h>

#include <iostream>
using namespace std;

#define _inputImgRed (*(ptr_inputImgRed))
#define _inputImgGreen (*(ptr_inputImgGreen))
#define _inputImgBlue (*(ptr_inputImgBlue))
#define _inputImgRG (*(ptr_inputImgRG))
#define _inputImgGR (*(ptr_inputImgGR))
#define _inputImgBY (*(ptr_inputImgBY))

blobFinderThread::blobFinderThread(){
    reinit_flag=false;
}


void blobFinderThread::reinitialise(int width, int height){
    img=new ImageOf<PixelRgb>;
    img->resize(width,height);
    this->width=width;
    this->height=height;
}


/**
*	initialization of the thread 
*/
bool blobFinderThread::threadInit(){

}

/**
* active loop of the thread
*/
void blobFinderThread::run(){
    
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
        ippiConvert_32s8s_C1R(_inputImgRGS32,psb32s,(Ipp8s*)wModule->_inputImgRGS->getRawImage(),wModule->_inputImgRGS->getRowSize(),srcsize);
        //_inputImgRGS->copy(_inputImgRG,320,240);
    }
    else
        return;
    if(ptr_inputImgGR!=0){
        ippiScale_8u32s_C1R(_inputImgGR.getRawImage(),_inputImgGR.getRowSize(),_inputImgGRS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgGRS32,psb32s,(Ipp8s*)wModule->_inputImgGRS->getRawImage(),wModule->_inputImgGRS->getRowSize(),srcsize);
        //_inputImgGRS->copy(_inputImgGR,320,240);
    }
    else
        return;
    if(ptr_inputImgBY!=0){
        ippiScale_8u32s_C1R(_inputImgBY.getRawImage(),_inputImgBY.getRowSize(),_inputImgBYS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgBYS32,psb32s,(Ipp8s*)wModule->_inputImgBYS->getRawImage(),wModule->_inputImgBYS->getRowSize(),srcsize);
        //_inputImgBYS->copy(_inputImgBY,320,240);
    }
    else
        return;
    salience->blobCatalog(_tagged, *wModule->_inputImgRGS, *wModule->_inputImgGRS, *wModule->_inputImgBYS,
        _inputImgBlue, _inputImgGreen, _inputImgRed, wModule->max_tag);
    blobCataloged_flag=true;
    //istruction to set the ptr_tagged in the Watershed Module with the static variable _tagged
    tagged=ptr_tagged; //ptr_tagged is the pointer to _tagged
    ippiFree(_inputImgRGS32); //Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
    ippiFree(_inputImgGRS32); //Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
    ippiFree(_inputImgBYS32); //Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
}

