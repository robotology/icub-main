#include <iCub/visualFilterThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define maxKernelSize 5

visualFilterThread::visualFilterThread() {
    redPlane=new ImageOf<PixelMono>;
    redPlane2=new ImageOf<PixelMono>;
    redPlane3=new ImageOf<PixelMono>;
    greenPlane=new ImageOf<PixelMono>;
    greenPlane2=new ImageOf<PixelMono>;
    greenPlane3=new ImageOf<PixelMono>;
    bluePlane=new ImageOf<PixelMono>;
    bluePlane2=new ImageOf<PixelMono>;
    bluePlane3=new ImageOf<PixelMono>;
    yellowPlane=new ImageOf<PixelMono>;
    yellowPlane2=new ImageOf<PixelMono>;
    inputExtImage=new ImageOf<PixelRgb>;
    inputImageFiltered=new ImageOf<PixelRgb>;

    redPlus=new ImageOf<PixelMono>;
    redMinus=new ImageOf<PixelMono>;
    greenPlus=new ImageOf<PixelMono>;
    greenMinus=new ImageOf<PixelMono>;
    bluePlus=new ImageOf<PixelMono>;
    yellowMinus=new ImageOf<PixelMono>;

    redGreen=new ImageOf<PixelMono>;
    greenRed=new ImageOf<PixelMono>;
    blueYellow=new ImageOf<PixelMono>;
    redGreenAbs=new ImageOf<PixelMono>;
    greenRedAbs=new ImageOf<PixelMono>;
    blueYellowAbs=new ImageOf<PixelMono>;

    edges=new ImageOf<PixelMono>;

    buffer=0;
    redGreenH16s=0;
    greenRedH16s=0;
    blueYellowH16s=0;
    redGreenV16s=0;
    greenRedV16s=0;
    blueYellowV16s=0;
    lambda=0.5;

    resized=false;
}

bool visualFilterThread::threadInit() {
    ippSetNumThreads(1);

    /* open ports  */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortExt.open(getName("/imageExt:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!rgPort.open(getName("/rg:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!grPort.open(getName("/gr:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!byPort.open(getName("/by:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    return true;
}

void visualFilterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string visualFilterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void visualFilterThread::run() {
    while (isStopping() != true) {
        inputImage = imagePortIn.read(true);

        if (inputImage != NULL) {
            if (!resized) {
                resize(inputImage->width(), inputImage->height());
                resized = true;
            }
            else {
                filterInputImage();
            }
          
            // extend logpolar input image
            extender(inputImage, maxKernelSize);
            // extract RGB and Y planes
            extractPlanes();
            // gaussian filtering of the of RGB and Y
            filtering();
            // colourOpponency map construction
            colourOpponency();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges
            edgesExtract();
            // sending the edge image on the outport
                 
            // it sends output to the port even if nothing is connected to it, including a copy of the image into the buffer :(
            if((edges!=0)&&(imagePortOut.getOutputCount())) {
                imagePortOut.prepare() = *(edges);
                imagePortOut.write();
            }
            if((redGreen!=0)&&(rgPort.getOutputCount())) {
                rgPort.prepare() = *(redGreen);
                rgPort.write();
            }
            if((greenRed!=0)&&(grPort.getOutputCount())) {
                grPort.prepare() = *(greenRed);
                grPort.write();
            }
            if((blueYellow!=0)&&(byPort.getOutputCount())) {
                byPort.prepare() = *(blueYellow);
                byPort.write();
            }
            if((inputExtImage!=0)&&(imagePortExt.getOutputCount())) {
                imagePortExt.prepare() = *(inputExtImage);
                imagePortExt.write();
            }
        }
   }
}

void visualFilterThread::resize(int width_orig,int height_orig) {
    this->width_orig=width_orig;
    this->height_orig=height_orig;
    this->width=width_orig+2*maxKernelSize;
    this->height=height_orig+maxKernelSize;

    // resizing the ROI
    originalSrcsize.height=height_orig;
    originalSrcsize.width=width_orig;
    srcsize.width=width;
    srcsize.height=height;

    // resizing plane images
    edges->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
 
    inputExtImage->resize(width,height);
    redPlane->resize(width,height);
    redPlane2->resize(width,height);
    redPlane3->resize(width,height);
    greenPlane->resize(width,height);
    greenPlane2->resize(width,height);
    greenPlane3->resize(width,height);
    bluePlane->resize(width,height);
    bluePlane2->resize(width,height);
    bluePlane3->resize(width,height);
    yellowPlane->resize(width,height);
    yellowPlane2->resize(width,height);

    redPlus->resize(width,height);
    redMinus->resize(width,height);
    greenPlus->resize(width,height);
    greenMinus->resize(width,height);
    bluePlus->resize(width,height);
    yellowMinus->resize(width,height);

    redGreen->resize(width, height);
    greenRed->resize(width, height);
    blueYellow->resize(width, height);
    redGreenAbs->resize(width, height);
    greenRedAbs->resize(width, height);
    blueYellowAbs->resize(width, height);

    ippiFilterSobelHorizGetBufferSize_8u16s_C1R(srcsize, ippMskSize3x3, &size1);
    buffer = ippsMalloc_8u(size1);
    redGreenH16s= ippiMalloc_16s_C1(width,height,&psb16s);
    greenRedH16s= ippiMalloc_16s_C1(width,height,&psb16s);
    blueYellowH16s= ippiMalloc_16s_C1(width,height,&psb16s);
    redGreenV16s= ippiMalloc_16s_C1(width,height,&psb16s);
    greenRedV16s= ippiMalloc_16s_C1(width,height,&psb16s);
    blueYellowV16s= ippiMalloc_16s_C1(width,height,&psb16s);
}

void visualFilterThread::filterInputImage() {
    int i;
    const int sz = inputImage->getRawImageSize();
    unsigned char * pFiltered = inputImageFiltered->getRawImage();
    unsigned char * pCurr = inputImageFiltered->getRawImage();
    const float ul = 1.0f - lambda;
    for (i = 0; i < sz; i++) {
        *pFiltered = (unsigned char)(lambda * *pCurr++ + ul * *pFiltered + .5f);
        pFiltered ++;
    }
}

ImageOf<PixelRgb>* visualFilterThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    // copy of the image 
    ippiCopy_8u_C3R(inputOrigImage->getRawImage(),inputOrigImage->getRowSize(),inputExtImage->getPixelAddress(maxSize,maxSize),inputExtImage->getRowSize(),originalSrcsize);    

    // memcpy of the horizontal fovea lines (rows) 
    int sizeBlock = width_orig / 2;
    for(int i = 0; i < maxSize; i++) {
        memcpy(inputExtImage->getPixelAddress(sizeBlock+maxSize,maxSize-1-i),inputExtImage->getPixelAddress(maxSize,maxSize+i),sizeBlock*sizeof(PixelRgb));
        memcpy(inputExtImage->getPixelAddress(maxSize,maxSize-1-i),inputExtImage->getPixelAddress(sizeBlock,maxSize+i),sizeBlock*sizeof(PixelRgb));
    }

    // copy of the block adjacent angular positions (columns)
    unsigned char* ptrDestRight;
    unsigned char* ptrOrigRight;
    unsigned char* ptrDestLeft;
    unsigned char* ptrOrigLeft;
    for (int row = 0; row < height; row++) {
        ptrDestRight=inputExtImage->getPixelAddress(width-maxSize,row);
        ptrOrigRight=inputExtImage->getPixelAddress(maxSize,row);
        ptrDestLeft=inputExtImage->getPixelAddress(0,row);
        ptrOrigLeft=inputExtImage->getPixelAddress(width-maxSize-maxSize,row);
        
        for (int i = 0; i < maxSize; i++) {
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
    Ipp8u* yellowP;
    shift[0]=redPlane->getRawImage();
    shift[1]=greenPlane->getRawImage();
    shift[2]=bluePlane->getRawImage();
    yellowP=yellowPlane->getRawImage();
    Ipp8u* inputPointer=inputExtImage->getRawImage();
    int paddingMono=redPlane->getRowSize()-redPlane->width();
    int padding3C=inputExtImage->getRowSize()-inputExtImage->width()*3;

    for(int r=0;r<inputExtImage->height();r++) {
        for(int c=0;c<inputExtImage->width();c++) {
            *shift[0]=*inputPointer;
            inputPointer++;
            *shift[1]=*inputPointer;
            inputPointer++;
            *shift[2]=*inputPointer;
            inputPointer++;
            *yellowP=(unsigned char)ceil((double)*shift[0]/2+(double)*shift[1]/2);
            shift[0]++;
            shift[1]++;
            shift[2]++;
            yellowP++;
        }
        inputPointer+=padding3C;
        shift[0]+=paddingMono;
        shift[1]+=paddingMono;
        shift[2]+=paddingMono;
        yellowP+=paddingMono;
    }
}

void visualFilterThread::filtering() {
    IppiSize srcPlusSize = { 5, 5 }; //variance=1
    IppiSize srcMinusSize = { 7, 7 }; //variance=3 which is 3 times the variance 1
    Ipp32f srcMinus[7*7] = {
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0186f, 0.0246f, 0.0290f, 0.0307f, 0.0290f, 0.0246f, 0.0186f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f
    };
    int divisor = 1;
    IppiPoint anchor = {4,4};
    
    ippiFilter32f_8u_C1R(redPlane->getRawImage(),redPlane->getRowSize(),redMinus->getRawImage(),redMinus->getRowSize(),srcsize,srcMinus,srcMinusSize,anchor);
    ippiFilter32f_8u_C1R(yellowPlane->getRawImage(),yellowPlane->getRowSize(),yellowMinus->getRawImage(),yellowMinus->getRowSize(),srcsize,srcMinus,srcMinusSize,anchor);
    ippiFilter32f_8u_C1R(greenPlane->getRawImage(),greenPlane->getRowSize(),greenMinus->getRawImage(),greenMinus->getRowSize(),srcsize,srcMinus,srcMinusSize,anchor);

    ippiFilterGauss_8u_C1R(bluePlane->getRawImage(), bluePlane->getRowSize(),bluePlus->getRawImage(),bluePlus->getRowSize(),srcsize,ippMskSize5x5);
    ippiFilterGauss_8u_C1R(redPlane->getRawImage(), redPlane->getRowSize(),redPlus->getRawImage(),redPlus->getRowSize(),srcsize,ippMskSize5x5);
    ippiFilterGauss_8u_C1R(greenPlane->getRawImage(), greenPlane->getRowSize(),greenPlus->getRawImage(),greenPlus->getRowSize(),srcsize,ippMskSize5x5);
}

void visualFilterThread::colourOpponency() {
    ippiRShiftC_8u_C1R(redPlus->getRawImage(),redPlane->getRowSize(),1,redPlane2->getRawImage(),redPlane2->getRowSize(),srcsize);
    ippiAddC_8u_C1RSfs(redPlane2->getRawImage(),redPlane2->getRowSize(),128,redPlane3->getRawImage(),redPlane3->getRowSize(),srcsize,0);
    ippiRShiftC_8u_C1R(redMinus->getRawImage(),redMinus->getRowSize(),1,redPlane2->getRawImage(),redPlane2->getRowSize(),srcsize);
    ippiRShiftC_8u_C1R(greenPlus->getRawImage(),greenPlus->getRowSize(),1,greenPlane2->getRawImage(),greenPlane2->getRowSize(),srcsize);
    ippiAddC_8u_C1RSfs(greenPlane2->getRawImage(),greenPlane2->getRowSize(),128,greenPlane3->getRawImage(),greenPlane3->getRowSize(),srcsize,0);
    ippiRShiftC_8u_C1R(greenMinus->getRawImage(),greenMinus->getRowSize(),1,greenPlane2->getRawImage(),greenPlane2->getRowSize(),srcsize);
    ippiRShiftC_8u_C1R(bluePlus->getRawImage(),bluePlus->getRowSize(),1,bluePlane2->getRawImage(),bluePlane2->getRowSize(),srcsize);
    ippiAddC_8u_C1RSfs(bluePlane2->getRawImage(),bluePlane2->getRowSize(),128,bluePlane3->getRawImage(),bluePlane3->getRowSize(),srcsize,0);
    ippiRShiftC_8u_C1R(yellowMinus->getRawImage(),yellowMinus->getRowSize(),1,yellowPlane2->getRawImage(),yellowPlane2->getRowSize(),srcsize);

    ippiSub_8u_C1RSfs(greenPlane2->getRawImage(),greenPlane2->getRowSize(),redPlane3->getRawImage(),redPlane3->getRowSize(),redGreen->getRawImage(),redGreen->getRowSize(),srcsize,0);
    ippiSub_8u_C1RSfs(redPlane2->getRawImage(),redPlane2->getRowSize(),greenPlane3->getRawImage(),greenPlane3->getRowSize(),greenRed->getRawImage(),greenRed->getRowSize(),srcsize,0);
    ippiSub_8u_C1RSfs(yellowPlane2->getRawImage(),yellowPlane2->getRowSize(),bluePlane3->getRawImage(),bluePlane3->getRowSize(),blueYellow->getRawImage(),blueYellow->getRowSize(),srcsize,0);
}

float max(float a,float b,float c) {
    if(a>b)
        if(a>c)
            return a;
        else
            return c;
    else
        if(b>c)
            return b;
        else
            return c;
}

void visualFilterThread::edgesExtract() {
    unsigned char* prg=redGreen->getRawImage();
    unsigned char* pgr=greenRed->getRawImage();
    unsigned char* pby=blueYellow->getRawImage();
    unsigned char* prga=redGreenAbs->getRawImage();
    unsigned char* pgra=greenRedAbs->getRawImage();
    unsigned char* pbya=blueYellowAbs->getRawImage();
    int padding=redGreen->getRowSize()-width;

    for(int row=0;row<height;row++) {
        for(int col=0;col<width;col++) {
            if(*prg<128) {
                *prga=128+(127-*prg);
            }
            else
                *prga=*prg;
            if(*pgr<128) {
                *pgra=128+(127-*pgr);
            }
            else
                *pgra=*pgr;
            if(*pby<128) {
                *pbya=128+(127-*pby);
            }
            else
                *pbya=*pby;
            
            prg++;pgr++;pby++;
            prga++;pgra++;pbya++;
        }
        prg+=padding;
        pgr+=padding;
        pby+=padding;
        prga+=padding;
        pgra+=padding;
        pbya+=padding;
    }

    //sobel operations
    ippiFilterSobelHorizBorder_8u16s_C1R(redGreenAbs->getRawImage(),redGreenAbs->getRowSize(), redGreenH16s, psb16s, srcsize,ippMskSize3x3, ippBorderRepl, 0,buffer);
    ippiFilterSobelHorizBorder_8u16s_C1R(greenRedAbs->getRawImage(),greenRedAbs->getRowSize(), greenRedH16s, psb16s, srcsize,ippMskSize3x3, ippBorderRepl, 0,buffer);
    ippiFilterSobelHorizBorder_8u16s_C1R(blueYellowAbs->getRawImage(),blueYellowAbs->getRowSize(), blueYellowH16s, psb16s, srcsize,ippMskSize3x3, ippBorderRepl, 0,buffer);
    ippiFilterSobelVertBorder_8u16s_C1R(redGreenAbs->getRawImage(),redGreenAbs->getRowSize(), redGreenV16s, psb16s, srcsize,ippMskSize3x3, ippBorderRepl, 0,buffer);
    ippiFilterSobelVertBorder_8u16s_C1R(greenRedAbs->getRawImage(),greenRedAbs->getRowSize(), greenRedV16s, psb16s, srcsize,ippMskSize3x3, ippBorderRepl, 0,buffer);
    ippiFilterSobelVertBorder_8u16s_C1R(blueYellowAbs->getRawImage(),blueYellowAbs->getRowSize(), blueYellowV16s, psb16s, srcsize,ippMskSize3x3, ippBorderRepl, 0,buffer);    

    unsigned char* pedges=edges->getRawImage();
    int rowsize=edges->getRowSize();
    int rowsize2=psb16s;
    double rgvmax=0,rghmax=0,maxa=0,maxb=0,maxc=0,rghminusmax=0,rghminusmin=255, maxValuemax=0;
    int j=maxKernelSize*(rowsize2/sizeof(signed short))+maxKernelSize;

    // edges extraction
    for(int row=0;row<height_orig;row++) {
        for(int col=0;col<width_orig;col++) {
            float rghd=(float)redGreenH16s[j];float rgvd=(float)redGreenV16s[j];
            float grhd=(float)greenRedH16s[j];float grvd=(float)greenRedV16s[j];
            float byhd=(float)blueYellowH16s[j];float byvd=(float)blueYellowV16s[j];

            // module of the vector
            float a=sqrt(pow(rghd,2)+pow(rgvd,2));
            float b=sqrt(pow(grhd,2)+pow(grvd,2));
            float c=sqrt(pow(byhd,2)+pow(byvd,2));

            // normalisation max module=179.60/181.01
            if(maxa<a)
                maxa=a;
            if(maxb<b)
                maxb=b;
            if(maxc<c)
                maxc=c;
            
            float rgnorm=(255.0/300.0)*a;
            float grnorm=(255.0/300.0)*b;
            float bynorm=(255.0/300.0)*c;
            
            if (row<height_orig-2) {
                float maxValue=floor(max(rgnorm,grnorm,bynorm));
                if(maxValuemax<maxValue)
                    maxValuemax=maxValue;
                unsigned char maxChar=(unsigned char) maxValue;
                *pedges=maxChar;
            }
            else {
                *pedges=0;
            }
                        
            pedges++;
            j++;
        }

        pedges+=rowsize-width_orig;
        for(int i=0;i<(rowsize2/sizeof(signed short))-width_orig-maxKernelSize+maxKernelSize;i++) {
            j++;
        }
        
    }
}

void visualFilterThread::threadRelease() {
    /* for example, delete dynamically created data-structures */
    delete redPlane;
    delete redPlane2;
    delete redPlane3;
    delete greenPlane;
    delete greenPlane2;
    delete greenPlane3;
    delete bluePlane;
    delete bluePlane2;
    delete bluePlane3;
    delete yellowPlane;
    delete yellowPlane2;
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;

    delete redPlus;
    delete redMinus;
    delete greenPlus;
    delete greenMinus;
    delete bluePlus;
    delete yellowMinus;

    delete redGreen;
    delete greenRed;
    delete blueYellow;
    delete redGreenAbs;
    delete greenRedAbs;
    delete blueYellowAbs;
    delete edges;

    if(buffer!=0)
        ippsFree(buffer);
    if(redGreenH16s!=0)
        ippiFree(redGreenH16s);
    if(greenRedH16s!=0)
        ippiFree(greenRedH16s);
    if(blueYellowH16s!=0)
        ippiFree(blueYellowH16s);
    if(redGreenV16s!=0)
        ippiFree(redGreenV16s);
    if(greenRedV16s!=0)
        ippiFree(greenRedV16s);
    if(blueYellowV16s!=0)
        ippiFree(blueYellowV16s);
}

void visualFilterThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    imagePortExt.interrupt();
    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();
    
    imagePortOut.close();
    imagePortExt.close();
    rgPort.close();
    grPort.close();
    byPort.close();
    imagePortIn.close();
}

