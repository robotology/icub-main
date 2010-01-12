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


const int THREAD_RATE=30;

blobFinderThread::blobFinderThread():RateThread(THREAD_RATE){
    reinit_flag=false;

    ct=0;

    //inputImage_flag=false;
    freetorun=false;
    meanColour_flag=true;
    contrastLP_flag=false;
    blobCataloged_flag=true;
    foveaBlob_flag=false;
    colorVQ_flag=false;
    blobList_flag=false;
    maxSaliencyBlob_flag=false;
    tagged_flag=false;
    watershed_flag=false;
    //bluePlane_flag=false;
    //redPlane_flag=false;
    //greenPlane_flag=false;
    //RG_flag=false;
    //GR_flag=false;
    //BY_flag=false;
    //noOpponencies_flag=true;
    //noPlanes_flag=true;
    resized_flag=false;
    //----
    //maxSalienceBlob_img=new ImageOf<PixelMono>;
    outContrastLP=new ImageOf<PixelMono>;
    outMeanColourLP=new ImageOf<PixelBgr>;
    
    //wModule=this;

    max_boxes = new YARPBox[3];
    //initializing the image plotted out int the drawing area
    image_out=new ImageOf<PixelRgb>;
    _procImage=new ImageOf<PixelRgb>;
    _outputImage3=new ImageOf<PixelRgb>;
    _outputImage=new ImageOf<PixelMono>;

    ptr_inputImg=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image
    ptr_inputRed=new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the red plane
    ptr_inputGreen= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the green plane
    ptr_inputBlue= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the blue plane
    ptr_inputImgRed=new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the red plane
    ptr_inputImgGreen= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the green plane
    ptr_inputImgBlue= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the blue plane
    ptr_inputRG= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the R+G- colour opponency
    ptr_inputGR= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the G+R- colour opponency
    ptr_inputBY= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the B+Y- colour opponency
    ptr_inputImgRG= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the R+G- colour opponency
    ptr_inputImgGR= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the G+R- colour opponency
    ptr_inputImgBY= new ImageOf<yarp::sig::PixelMono>; //pointer to the input image of the B+Y- colour opponency
    
    _inputImgRGS=new ImageOf<PixelMono>;
    _inputImgGRS=new ImageOf<PixelMono>;
    _inputImgBYS=new ImageOf<PixelMono>;
    
    blobFov=new ImageOf<PixelMono>;
    

    salienceBU=10;
    salienceTD=10;
    maxBLOB=4096;
    minBLOB=100;

    targetRED=1;
    targetGREEN=1;
    targetBLUE=1;
    searchRG=0;
    searchGR=0;
    searchBY=0;
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
    srcsize.height=height;
    srcsize.width=width;

    //int widthstep=256;
    double divider=ceil(width/32.0);
    int widthstep=divider*32;

    ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
    ptr_tagged->resize(widthstep,height);
    
    this->wOperator=new WatershedOperator(true,width,height,widthstep,10);
    //_wOperator=this->wOperator;
    this->salience=new SalienceOperator(width,height);
    //_salience=this->salience;

    //maxSalienceBlob_img->resize(width,height);
    outMeanColourLP->resize(width,height);
    outContrastLP->resize(width,height);

    image_out->resize(width,height);
    _procImage->resize(width,height);
    _outputImage->resize(width,height);
    _outputImage3->resize(width,height);

    ptr_inputRed->resize(width,height);
    ptr_inputGreen->resize(width,height);
    ptr_inputBlue->resize(width,height);
    ptr_inputRG->resize(width,height);
    ptr_inputGR->resize(width,height);
    ptr_inputBY->resize(width,height);

    _inputImgRGS->resize(width,height);
    _inputImgGRS->resize(width,height);
    _inputImgBYS->resize(width,height);
    blobFov->resize(width,height);

    blobList = new char [width*height+1];

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
	contrastLP_flag=true;
	meanColour_flag=false;
	blobCataloged_flag=false;
	foveaBlob_flag=false;
	colorVQ_flag=false;
	maxSaliencyBlob_flag=false;
	blobList_flag=false;
    tagged_flag=false;
	watershed_flag=false;

    return true;
}

void blobFinderThread::resetFlags(){
    contrastLP_flag=false;
	meanColour_flag=false;
	blobCataloged_flag=false;
	foveaBlob_flag=false;
	colorVQ_flag=false;
	maxSaliencyBlob_flag=false;
	blobList_flag=false;
    tagged_flag=false;
	watershed_flag=false;
}

/**
* active loop of the thread
*/
void blobFinderThread::run(){


    if(!freetorun)
        return;

    bool conversion=true;
   _outputImage=wOperator->getPlane(&_inputImg);
    rain();
    if(this->foveaBlob_flag){
        this->salience->drawFoveaBlob(*this->salience->foveaBlob,*this->tagged);
        ippiCopy_8u_C1R(this->salience->foveaBlob->getRawImage(),this->salience->foveaBlob->getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
        conversion=true;
    }
    else if(this->watershed_flag){
        ippiCopy_8u_C1R(this->wOperator->tSrc.getRawImage(),this->wOperator->tSrc.getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
        conversion=true;
    }
    else if(this->tagged_flag){
        //printf("dimension of the tagged image %d,%d \n", this->tagged->width(), this->tagged->height());
        for(int y=0; y<this->tagged->height(); y++){
            for (int x=0; x<this->tagged->width(); x++){
                _outputImage->pixel(x,y)=(int)((this->max_tag/255)*this->tagged->pixel(x,y));
            }
        }
        //cvCvtColor(this->tagged->getIplImage(),_outputImage->getIplImage(),CV_GRAY2RGB);
        //cvCopy(this->tagged->getIplImage(),_outputImage->getIplImage());
        //ippiCopy_8u_C1R(this->tagged->getRawImage(),320,_outputImage->getRawImage(),320,srcsize);
        conversion=true;
    }
    else if(this->blobList_flag){
        this->drawAllBlobs(false);
    }
    else if(this->maxSaliencyBlob_flag){
        this->drawAllBlobs(false);
        this->salience->DrawMaxSaliencyBlob(*this->salience->maxSalienceBlob_img,this->max_tag,*this->tagged);
        ippiCopy_8u_C1R(salience->maxSalienceBlob_img->getRawImage(),salience->maxSalienceBlob_img->getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
        conversion=true;
    }
    else if(this->contrastLP_flag){
        this->drawAllBlobs(false);
        //ippiCopy_8u_C3R(this->outMeanColourLP->getRawImage(),320*3,_outputImage3->getRawImage(),320*3,srcsize);	
        ippiCopy_8u_C1R(this->outContrastLP->getRawImage(),this->outContrastLP->getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
        conversion=true;
    }
    else if(this->colorVQ_flag){
        this->salience->DrawVQColor(*this->salience->colorVQ_img,*this->tagged);
        ippiCopy_8u_C3R(this->salience->colorVQ_img->getRawImage(),this->salience->colorVQ_img->getRowSize(),_outputImage3->getRawImage(),_outputImage3->getRowSize(),srcsize);
        conversion=false;
    }
    else if(this->blobCataloged_flag){
        if(this->contrastLP_flag){
            this->drawAllBlobs(false);
            //ippiCopy_8u_C3R(this->outMeanColourLP->getRawImage(),320*3,_outputImage3->getRawImage(),320*3,srcsize);	
            ippiCopy_8u_C1R(this->outContrastLP->getRawImage(),this->outContrastLP->getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
            conversion=true;
        }
        else if(this->meanColour_flag){
            //_outputImage=_wOperator->getPlane(&_inputImg); 
            //rain();
            this->drawAllBlobs(false);
            ippiCopy_8u_C3R(this->outMeanColourLP->getRawImage(),this->outMeanColourLP->getRowSize(),_outputImage3->getRawImage(),_outputImage3->getRowSize(),srcsize);	
            conversion=false;
        }
        else{
            _outputImage=wOperator->getPlane(&_inputImg); //the input is a RGB image, whereas the watershed is working with a mono image
            conversion=true;
        }
    }
    else{
        _outputImage=wOperator->getPlane(&_inputImg); //the input is a RGB image, whereas the watershed is working with a mono image
        conversion=true;
    }

    //-------
    
    if(conversion){
        int psb;
        int width=this->width;
        int height=this->height;
        
        Ipp8u* im_out = ippiMalloc_8u_C1(width,height,&psb);
        //Ipp8u* im_tmp0 = ippiMalloc_8u_C1(width,height,&psb);
        //Ipp8u* im_tmp1= ippiMalloc_8u_C1(width,height,&psb);
        //Ipp8u* im_tmp2 = ippiMalloc_8u_C1(width,height,&psb);
        //two copies in order to have 2 conversions
        //the first transform the yarp mono into a 4-channel image
        ippiCopy_8u_C1R(_outputImage->getRawImage(),_outputImage->getRowSize(),im_out,psb,srcsize);

        //ippiCopy_8u_C1R(im_out, width,im_tmp0,psb,srcsize);
        //ippiCopy_8u_C1R(im_out, width,im_tmp1,psb,srcsize);
        //ippiCopy_8u_C1R(im_out, width,im_tmp2,psb,srcsize);

        //im_tmp0=im_out;
        //im_tmp1=im_out;
        //im_tmp2=im_out;

        //Ipp8u* im_tmp[3]={im_tmp0,im_tmp1,im_tmp2};

        Ipp8u* im_tmp[3]={im_out,im_out,im_out};
        //Ipp8u* im_tmp[3]={_outputImage->getRawImage(),_outputImage->getRawImage(),_outputImage->getRawImage()};
        //the second transforms the 4-channel image into colorImage for yarp
        ippiCopy_8u_P3C3R(im_tmp,psb,image_out->getRawImage(),image_out->getRowSize(),srcsize);
        ippiFree(im_out);
        //printf("freeing im_tmp0  \n");	
        //ippiFree(im_tmp0);
        //printf("freeing im_tmp1 \n");	
        //ippiFree(im_tmp1);
        //printf("freeing im_tmp2  \n");	
        //ippiFree(im_tmp2);
        //printf("freeing ended  \n");	
    }
    else
        ippiCopy_8u_C3R(_outputImage3->getRawImage(),_outputImage3->getRowSize(),this->image_out->getRawImage(),this->image_out->getRowSize(),srcsize);
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

void blobFinderThread::drawAllBlobs(bool stable)
{
    salience->ComputeSalienceAll(this->max_tag,this->max_tag);
    //extracts the PixelBgr color of a particular blob identified by the id (last parameter)
    PixelBgr varFoveaBlob = salience->varBlob(*tagged, *ptr_inputRG, *ptr_inputGR, *ptr_inputBY, 1);

    salience->drawFoveaBlob(*blobFov, *tagged);
    //__OLD//salience.drawBlobList(blobFov, tagged, blobList, max_tag, 127);
    
    //list of boolean whether is present or not a blob
    
    memset(blobList, 0, sizeof(char)*(max_tag+1));
    // - faster
    // - it considers also "lateral" pixels
    // - it doesn't add pixels iteratively
    wOperator->findNeighborhood(*tagged, 0, 0, blobList);
    salience->fuseFoveaBlob3(*tagged, blobList, varFoveaBlob,max_tag);

    // alternative method
    //__OLD//rain.fuseFoveaBlob(tagged, blobList, max_tag);
    
    //__OLD//blobList[1]=2; // so the fovea blob is eliminated by the removeBlobList
    //__OLD//salience.statBlobList(tagged, blobList, max_tag, fovBox);
    YARPBox fovBox;
    fovBox=salience->getBlobNum(1);
    //__OLD//salience.removeBlobList(blobList, max_tag);
    salience->removeFoveaBlob(*tagged);
    //__OLD//salience.updateFoveaBlob(tagged, blobList, max_tag);

    if (stable) {
        for (int i=0; i<2; i++) {
            memset(blobList, 0, sizeof(char)*(max_tag+1));
            wOperator->findNeighborhood(*tagged, 0, 0, blobList);
            const int minBoundingArea=15*15;
            int count=salience->countSmallBlobs(*tagged, blobList, max_tag, minBoundingArea);
            printf("Count of small blobs: %d \n",count);
            blobList[1]=0;
            salience->mergeBlobs(*tagged, blobList, max_tag, 1);
        }

        /*__OLD//while (num!=0) {
            blobList[1]=0;
            salience.mergeBlobs(tagged, blobList, max_tag, 1);
            memset(blobList, 0, sizeof(char)*max_tag);
            rain.findNeighborhood(tagged, 0, 0, blobList);
            num = salience.checkSmallBlobs(tagged, blobList, max_tag, minBoundingArea);
        }*/
    }
        
    //__OLD//salience.drawFoveaBlob(blobFov, tagged);
    //__OLD//salience.drawBlobList(blobFov, tagged, blobList, max_tag, 127);
    
    // Comment the following line to disable the elimination of non valid blob
    //salience->RemoveNonValidNoRange(max_tag, BLOB_MAXSIZE, BLOB_MINSIZE);
    salience->RemoveNonValidNoRange(max_tag,maxBLOB,minBLOB);
    
    //__OLD//salience.DrawContrastLP(rg, gr, by, tmp1, tagged, max_tag, 0, 1, 30, 42, 45); // somma coeff pos=3 somma coeff neg=-3
    //__OLD//salience.checkIOR(tagged, IORBoxes, num_IORBoxes);
    //__OLD//salience.doIOR(tagged, IORBoxes, num_IORBoxes);
    //float salienceBU=1.0,salienceTD=0.0;
    IppiSize srcsize={this->width,this->height};
    PixelMono searchTD=0;
    searchRG=((targetRED-targetGREEN+255)/510)*255;
    searchGR=((targetGREEN-targetRED+255)/510)*255;
    PixelMono addRG=((targetRED+targetGREEN)/510)*255;
    searchBY=((targetBLUE-addRG+255)/510)*255;
    int psb32s;
    //int psb8u;
    Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(this->width,this->height,&psb32s);
    Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(this->width,this->height,&psb32s);
    Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(this->width,this->height,&psb32s);
    //Ipp8s* _inputImgRGS8s=ippiMalloc_8u_C1(width,height,&psb8u);
    //_inputImgGR
    if(ptr_inputImgRG!=NULL){
        //_inputImgRGS->copy(*ptr_inputImgRG,320,240);
        ippiScale_8u32s_C1R(_inputImgRG.getRawImage(),_inputImgRG.getRowSize(),_inputImgRGS32,psb32s,srcsize);
        //_inputImgRGS8s=(Ipp8s*)_inputImgRGS->getRawImage();
        ippiConvert_32s8s_C1R(_inputImgRGS32,psb32s,(Ipp8s*)_inputImgRGS->getRawImage(),_inputImgRGS->getRowSize(),srcsize);
        //ippiCopy_8u_C1R(_inputImgRG.getPixelAddress(0,0),320,_inputImgRGS->getPixelAddress(0,0),320,srcsize);
    }
    else
        return;
    //_inputImgGR
    if(ptr_inputImgGR!=NULL){
        //_inputImgGRS->copy(*ptr_inputImgGR,320,240);
        ippiScale_8u32s_C1R(_inputImgGR.getRawImage(),_inputImgGR.getRowSize(),_inputImgGRS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgGRS32,psb32s,(Ipp8s*)_inputImgGRS->getRawImage(),_inputImgGRS->getRowSize(),srcsize);
        //ippiCopy_8u_C1R(_inputImgGR.getPixelAddress(0,0),320,_inputImgGRS->getPixelAddress(0,0),320,srcsize);
    }
    else
        return;
    //_inputImgBY
    if(ptr_inputImgBY!=NULL){
        //_inputImgBYS->copy(*ptr_inputImgBY,320,240);
        ippiScale_8u32s_C1R(_inputImgBY.getRawImage(),_inputImgBY.getRowSize(),_inputImgBYS32,psb32s,srcsize);
        ippiConvert_32s8s_C1R(_inputImgBYS32,psb32s,(Ipp8s*)_inputImgBYS->getRawImage(),_inputImgBYS->getRowSize(),srcsize);
        //ippiCopy_8u_C1R(_inputImgBY.getPixelAddress(0,0),320,_inputImgBYS->getPixelAddress(0,0),320,srcsize);
    }
    else
        return;
    int nBlobs=salience->DrawContrastLP2(*_inputImgGRS, *_inputImgGRS, *_inputImgBYS,
        *outContrastLP, *tagged, max_tag,
        salienceBU, salienceTD,
        searchRG, searchGR, searchBY, 255); // somma coeff pos=3 somma coeff neg=-3
    //printf("The number of blobs: %d",nBlobs);
    salience->ComputeMeanColors(max_tag); //compute for every box the mean Red,Green and Blue Color.
    salience->DrawMeanColorsLP(*outMeanColourLP,*tagged);
    
    //__OLD//meanOppCol.Zero();
    //__OLD//salience.DrawMeanOpponentColorsLP(meanOppCol, tagged);

    /*__OLD//blobFinder.DrawGrayLP(tmp1, tagged, 200);
    //__OLD//ACE_OS::sprintf(savename, "./rain.ppm");
    //__OLD//YARPImageFile::Write(savename, tmp1);*/

    

    //__OLD//rain.tags2Watershed(tagged, oldWshed);

    //delete(blobList);
    ippiFree(_inputImgRGS32); //Ipp32s* _inputImgRGS32=ippiMalloc_32s_C1(320,240,&psb32s);
    ippiFree(_inputImgGRS32); //Ipp32s* _inputImgGRS32=ippiMalloc_32s_C1(320,240,&psb32s);
    ippiFree(_inputImgBYS32); //Ipp32s* _inputImgBYS32=ippiMalloc_32s_C1(320,240,&psb32s);
    //ippiFree(_inputImgRGS8s);
}