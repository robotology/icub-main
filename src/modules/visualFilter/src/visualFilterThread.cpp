#include <iCub/visualFilterThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define maxKernelSize 5

visualFilterThread::visualFilterThread() {
    redPlane=new ImageOf<PixelMono>;
    greenPlane=new ImageOf<PixelMono>;
    bluePlane=new ImageOf<PixelMono>;
    yellowPlane=new ImageOf<PixelMono>;
    inputExtImage=new ImageOf<PixelRgb>;
}


bool visualFilterThread::threadInit() {
    /* open ports  */ 

    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    /* initialize variables and create data-structures if needed */
    return true;
}

void visualFilterThread::setName(string str){
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string visualFilterThread::getName(const char* p){
    string str(name);
    str.append(p);
    //printf("name: %s", name.c_str());
    return str;
}

void visualFilterThread::run() {

   /* 
    * do some work ....
    * for example, convert the input image to a binary image using the threshold provided 
    */ 
   
   unsigned char value;

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      
        
        inputImage = imagePortIn.read(true);

        if(inputImage!=NULL){
            resize(inputImage->width(),inputImage->height());      
          
            //extending logpolar input image
            extending();
            //extracting RGB and Y planes
            extractPlanes();
            //gaussing filtering of the of RGBY
            filtering();
            //colourOpponency map construction
            colourOpponency();
            //applying sobel operators on the colourOpponency maps and combining via maximisation of the 3 edges
            edgesExtract();
            //sending the edge image on the outport
                 
            
            if((redPlane!=0)&&(imagePortOut.getOutputCount())){
                imagePortOut.prepare() = *(redPlane);		
                imagePortOut.write();
            }
        }
   }//while 
}

void visualFilterThread::resize(int width_orig,int height_orig) {
    this->width_orig=width_orig;
    this->height_orig=height_orig;
    this->width=width_orig+2*maxKernelSize;
    this->height=height_orig+maxKernelSize;
    //resizing the ROI
    srcsize.width=width;
    srcsize.height=height;
    //resizing plane images
    inputExtImage->resize(width,height);
    redPlane->resize(width,height);
    greenPlane->resize(width,height);
    bluePlane->resize(width,height);
    yellowPlane->resize(width,height);
}


void visualFilterThread::extending() {
    extender(inputImage,5);
}

ImageOf<PixelRgb>* visualFilterThread::extender(ImageOf<PixelRgb>* inputOrigImage,int maxSize) {
    IppiSize originalSrcsize;
    originalSrcsize.height=inputOrigImage->height();
    originalSrcsize.width=inputOrigImage->width();
        
    //copy of the image 
    ippiCopy_8u_C3R(inputOrigImage->getRawImage(),inputOrigImage->getRowSize(),inputExtImage->getPixelAddress(maxSize,maxSize),inputExtImage->getRowSize(),originalSrcsize);    
    //memcpy of the orizontal fovea lines (rows) 
    int sizeBlock=width_orig/2;
    for( int i=0;i<maxSize;i++) {
        memcpy(inputExtImage->getPixelAddress(sizeBlock+maxSize,maxSize-1-i),inputExtImage->getPixelAddress(maxSize,maxSize+i),sizeBlock*sizeof(PixelRgb));
        memcpy(inputExtImage->getPixelAddress(maxSize,maxSize-1-i),inputExtImage->getPixelAddress(sizeBlock,maxSize+i),sizeBlock*sizeof(PixelRgb));
    }
    //copy of the block adiacent angular positions (columns)
    unsigned char* ptrDestRight;
    unsigned char* ptrOrigRight;
    unsigned char* ptrDestLeft;
    unsigned char* ptrOrigLeft;
    for(int row=0;row<height;row++) {
        ptrDestRight=inputExtImage->getPixelAddress(width-maxSize,row);
        ptrOrigRight=inputExtImage->getPixelAddress(maxSize,row);
        ptrDestLeft=inputExtImage->getPixelAddress(0,row);
        ptrOrigLeft=inputExtImage->getPixelAddress(width-maxSize,row);
        for(int i=0;i<maxSize;i++){
            //right block
            *ptrDestRight=*ptrOrigRight;
            ptrDestRight++;ptrOrigRight++;
            *ptrDestRight=*ptrOrigRight;
            ptrDestRight++;ptrOrigRight++;
            *ptrDestRight=*ptrOrigRight;
            ptrDestRight++;ptrOrigRight++;
            //left block
            *ptrDestLeft=*ptrOrigLeft;
            ptrDestLeft++;ptrOrigLeft++;
            *ptrDestLeft=*ptrOrigLeft;
            ptrDestLeft++;ptrOrigLeft++;
            *ptrDestLeft=*ptrOrigLeft;
            ptrDestLeft++;ptrOrigLeft++;
        }
    }
    return inputExtImage;
}
   
    
void visualFilterThread::extractPlanes() {
    Ipp8u* shift[3];
    int psb;
	shift[0]=redPlane->getRawImage(); 
	shift[1]=greenPlane->getRawImage();
	shift[2]=bluePlane->getRawImage();
    ippiCopy_8u_C3P3R(inputExtImage->getRawImage(),inputExtImage->getRowSize(),shift,redPlane->getRowSize(),srcsize);
    ippiAdd_8u_C1RSfs(redPlane->getRawImage(),redPlane->getRowSize(),greenPlane->getRawImage(),greenPlane->getRowSize(),yellowPlane->getRawImage(),yellowPlane->getRowSize(),srcsize,1);
}

    
void visualFilterThread::filtering() {
}

void visualFilterThread::colourOpponency() {
    //ippiAdd_8u_C1RSfs(

}

void visualFilterThread::edgesExtract() {

}

void visualFilterThread::threadRelease() {
    /* for example, delete dynamically created data-structures */
    delete redPlane;
    delete greenPlane;
    delete bluePlane;
    delete yellowPlane;
    delete inputExtImage;
}

void visualFilterThread::onStop(){
   imagePortIn.interrupt();
   imagePortOut.interrupt();
   imagePortIn.close();
   imagePortOut.close();
}


