// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/WatershedOperator.h>

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


WatershedOperator::WatershedOperator(const bool lp, const int width1, 
                                     const int height1, const int wstep, const PixelMono th){
    neighborhood8=true;
    watershedColor=0;
    basinColor=255;
    neigh=NULL;
    logpolar=lp;

    outputImage=new ImageOf<PixelMono>;
    outputImage->resize(width1,height1);
    

    resize(width1, height1, wstep, th);
}


void WatershedOperator::resize(const int width1, const int height1, const int wstep, const PixelMono th){
    width=width1;
    height=height1;
    imageSize=width*height;
    padding=wstep-width1;
    //padding=0;
    widthStep=wstep;
    //widthStep=width1;
    threshold=th;
    createNeighborhood(wstep, neighborhood8);
    //createNeighborhood(width1, neighborhood8);

    downPos.resize(widthStep,height);
    downPos2.resize(widthStep,height);
    
    tempRegion = new int[imageSize];
}


WatershedOperator::~WatershedOperator()
{
    if (tempRegion!=NULL) delete [] tempRegion;
    if (neigh!=NULL) delete [] neigh;
    if (neighL!=NULL) delete [] neighL;
    if (neighR!=NULL) delete [] neighR;
}


void WatershedOperator::createNeighborhood(const int widthStep, const bool neigh8)
{
    if (!neigh8)
        neighSize=4;
    else
        neighSize=8;
    if (neigh!=NULL)
        delete [] neigh;
    neigh = new int [neighSize];
    neighL = new int [neighSize];
    neighR = new int [neighSize];

    if (neigh8) {
        //	0 1 2
        //	3   4
        //	5 6 7

        neigh[4]= +1;			//right
        neigh[1]= -widthStep;	//up
        neigh[3]= -1;			//left
        neigh[6]= +widthStep;	//down
        neigh[2]= neigh[1]+neigh[4];	//up, right
        neigh[0]= neigh[1]+neigh[3];	//up, left
        neigh[5]= neigh[6]+neigh[3];	//down, left
        neigh[7]= neigh[6]+neigh[4];	//down, right

        neighL[4]= +1;			//right
        neighL[1]= -widthStep;	//up
        neighL[3]= +(width-1);	//left
        neighL[6]= +widthStep;	//down
        neighL[2]= neighL[1]+neighL[4];	//up, right
        neighL[0]= neighL[1]+neighL[3];	//up, left
        neighL[5]= neighL[6]+neighL[3];	//down, left
        neighL[7]= neighL[6]+neighL[4];	//down, right

        neighR[4]= -(width-1);	//right
        neighR[1]= -widthStep;	//up
        neighR[3]= -1;			//left
        neighR[6]= +widthStep;	//down
        neighR[2]= neighR[1]+neighR[4];	//up, right
        neighR[0]= neighR[1]+neighR[3];	//up, left
        neighR[5]= neighR[6]+neighR[3];	//down, left
        neighR[7]= neighR[6]+neighR[4];	//down, right
    } else {
        //	  0
        //	1   2
        //	  3

        neigh[1]= -widthStep;	//up
        neigh[2]= -1;			//left
        neigh[0]= +1;			//right
        neigh[3]= +widthStep;	//down

        neighL[0]= +1;			//right
        neighL[1]= -widthStep;	//up
        neighL[2]= +(width-1);	//left
        neighL[3]= +widthStep;	//down

        neighR[0]= -(width-1);	//right
        neighR[1]= -widthStep;	//up
        neighR[2]= -1;			//left
        neighR[3]= +widthStep;	//down
    }
}


void WatershedOperator::tags2Watershed(const ImageOf<PixelInt>& src, ImageOf<PixelMono>& dest)
{
    //TODO: translate the following single instruction in a series of istr.
    PixelInt *p_src=(PixelInt *)src.getRawImage();
    PixelMono *p_dst;
    int i,j,n,pos,p;

    const PixelMono val = 255;

    //__OLD//dest.Resize(width,height);
    p_dst=(PixelMono *)dest.getRawImage();

    dest.zero();

    // first row
    // first pixel
    p=0;
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighL[n];
        if (p_src[p]!=p_src[pos]) {
            p_dst[p] = val;
            break;
        }
    }
    p++;
    // first row, 1->150
    // Note that the pixel of the first row are all the same, so it is
    // useless to check the "far" neighborhoods
    for(i=1; i<width-1; i++) {
        for(n=neighSize/2-1; n<neighSize; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_src[p]!=p_src[pos]) {
                p_dst[p] = val;
                break;
            }
        }
        p++;
    }
    // last pixel of the first row
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighR[n];
        if (p_src[p]!=p_src[pos]) {
            p_dst[p] = val;
            break;
        }
    }
    p++;
    p+=padding;

    // inner part
    for(j=1; j<height-1; j++) {
        // first pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighL[n];
            if (p_src[p]!=p_src[pos]) {
                p_dst[p] = val;
                break;
            }
        }
        p++;
        
        for(i=1; i<width-1; i++) {
            for(n=0; n<neighSize; n++) {
                pos = p + neigh[n];
                if (p_src[p]!=p_src[pos]) {
                    p_dst[p] = val;
                    break;
                }
            }
            p++;
        }
        
        //last pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighR[n];
            if (p_src[p]!=p_src[pos]) {
                p_dst[p] = val;
                break;
            }
        }
        p++;
        
        p+=padding;
    }

    // last row
    // first pixel
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighL[n];
        if (p_src[p]!=p_src[pos]) {
            p_dst[p] = val;
            break;
        }
    }
    p++;
    // last row, 1->150
    for(i=1; i<width-1; i++) {
        for(n=0; n<neighSize/2+1; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_src[p]!=p_src[pos]) {
                p_dst[p] = val;
                break;
            }
        }
        p++;
    }
    // last pixel of the last row
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighR[n];
        if (p_src[p]!=p_src[pos]) {
            p_dst[p] = val;
            break;
        }
    }
}


/*__OLD//void WatershedOperator::tags2Watershed(const ImageOf<PixelInt>& src, ImageOf<PixelMono>& dest)
{
    const int WSHED =  0; // watersheds have value 0
    
    int *p_src=(int *)src.getRawImage();
    unsigned char *p_dest;

    // 4-neighborhood is sufficient here, so the watersheds won't get too thick
    // only for visualisation (calculation by user-parameter)
    createNeighborhood(((IplImage *)src)->widthStep, false);

    dest.Resize(width, height);
    p_dest=(unsigned char *)dest.getRawImage();

    int currentPoint=0;
    for (int r=0; r<height; r++) {
        for (int c=0; c<width; c++)	{
            int currentValue = p_src[currentPoint];

            if (currentValue == WSHED) // watershed
                p_dest[currentPoint] = watershedColor;
            else {                     // point is labeled
                // assume point is in basin
                p_dest[currentPoint] = basinColor;
                // check for adjacent basins
                int n;
                for(n=0;n<neighSize;n++) {
                    int currentNeighbor = currentPoint + neigh[n];
                    // skip invalid neighbors (pixel outside of image)
                    if (invalidNeighbor(currentPoint,currentNeighbor)) continue;
                    if (currentValue != p_src[currentNeighbor]) {
                        // different basin bordering => declare point as watershed
                        p_dest[currentPoint] = watershedColor;
                        break; // next currentPoint
                    }
                }
            }
            currentPoint++;
        }
        currentPoint+=padding;
    }
}*/


void WatershedOperator::connectivityGraph(const ImageOf<PixelInt>& src, bool *matr, int max_tag)
{
    PixelInt *p_src=(PixelInt *)src.getRawImage();
    int i,j,n,pos,p;

    memset(matr, false, sizeof(bool)*max_tag*max_tag);

    // first row
    // first pixel
    /*__OLD//p=0;
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighL[n];
        if (p_src[p]!=p_src[pos]) {
            p_dst[p] = val;
            break;
        }
    }*/
    p=1;

    // first row, 1->150
    // Note that the pixel of the first row are all the same, so it is
    // useless to check the "far" neighborhoods...
    for(i=1; i<width-1; i++) {
        for(n=neighSize/2-1; n<neighSize; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_src[p]!=p_src[pos]) {
                matr[max_tag*p_src[p]+p_src[pos]]=true;
                matr[max_tag*p_src[pos]+p_src[p]]=true;
            }
        }
        p++;
    }
    // last pixel of the first row
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighR[n];
        if (p_src[p]!=p_src[pos]) {
            matr[max_tag*p_src[p]+p_src[pos]]=true;
            matr[max_tag*p_src[pos]+p_src[p]]=true;
        }
    }
    p++;
    p+=padding;

    // inner part
    for(j=1; j<height-1; j++) {
        // first pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighL[n];
            if (p_src[p]!=p_src[pos]) {
                matr[max_tag*p_src[p]+p_src[pos]]=true;
                matr[max_tag*p_src[pos]+p_src[p]]=true;
            }
        }
        p++;
        
        for(i=1; i<width-1; i++) {
            for(n=0; n<neighSize; n++) {
                pos = p + neigh[n];
                if (p_src[p]!=p_src[pos]) {
                    matr[max_tag*p_src[p]+p_src[pos]]=true;
                    matr[max_tag*p_src[pos]+p_src[p]]=true;
                }
            }
            p++;
        }
        
        //last pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighR[n];
            if (p_src[p]!=p_src[pos]) {
                matr[max_tag*p_src[p]+p_src[pos]]=true;
                matr[max_tag*p_src[pos]+p_src[p]]=true;
            }
        }
        p++;
        
        p+=padding;
    }

    // last row
    // first pixel
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighL[n];
        if (p_src[p]!=p_src[pos]) {
            matr[max_tag*p_src[p]+p_src[pos]]=true;
            matr[max_tag*p_src[pos]+p_src[p]]=true;
        }
    }
    p++;
    // last row, 1->150
    for(i=1; i<width-1; i++) {
        n = neighSize/2;
        pos = p + neigh[n];
        if (p_src[p]!=p_src[pos]) {
            matr[max_tag*p_src[p]+p_src[pos]]=true;
            matr[max_tag*p_src[pos]+p_src[p]]=true;
        }
        p++;
    }
    // last pixel of the last row
    /*__OLD//for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighR[n];
        if (p_src[p]!=p_src[pos]) {
            matr[max_tag*p_src[p]+p_src[pos]]=true;
            matr[max_tag*p_src[pos]+p_src[p]]=true;
        }
    }*/
}


/** 
*creates regions (numbers by "counter") which are lokal minima(s)
*inputs: downPos2 and tSrc 
* @param result reference to the output result image
*/
int WatershedOperator::markMinimas(ImageOf<PixelInt>& result)
{
    // smelt points of same level and give it a number(counter)
    const int MASK = -2;
    int countF, countB ;
    int counter = 1; // number of region
    int tempi,tempiNeigh;
    int n,i;
    int *p_result=(int *)result.getRawImage();
    int *p_downPos=(int *)downPos2.getRawImage();
    unsigned char *p_src=(unsigned char *)tSrc.getRawImage();

    int maxNeigh;
    int minNeigh;
    int *fixNeigh;

    i=0;
    //for every pixel in the p_result and p_downPos
    for (int r=0; r<height; r++) {
        for (int c=0; c<width; c++)	{
            if (p_result[i]==-1 && p_downPos[i]<0) { //unused & minima
                // i is point of a new found region
                countF = countB = 0;
                tempRegion[countB++] = i; // put point in queue
                int tempLevel = p_src[i];
                // find all points which have the same tempLevel like point i
                // and mark them with counter
                while(countF<countB) {
                    tempi = tempRegion[countF++]; //get point from queue
                    p_result[tempi] = counter; // label point
                    int rT=tempi/widthStep;
                    int cT=tempi%widthStep;
                    if (rT==0) {
                        minNeigh=neighSize/2-1;
                        maxNeigh=neighSize;
                    } else if (rT==height-1) {
                        minNeigh=0;
                        maxNeigh=neighSize/2+1;
                    } else {
                        minNeigh=0;
                        maxNeigh=neighSize;
                    }

                    if (cT==0)
                        fixNeigh=neighL;
                    else if (cT==width-1)
                        fixNeigh=neighR;
                    else
                        fixNeigh=neigh;
                    for(n=minNeigh; n<maxNeigh; n++) {
                        tempiNeigh = tempi + fixNeigh[n];
                        //tempiNeigh a valid image point
                        if( p_result[tempiNeigh]==-1 && // unused
                            p_src[tempiNeigh]==tempLevel ) //same level
                        {
                            //each point only once in queue
                            tempRegion[countB++] = tempiNeigh; // put point in queue
                            p_result[tempiNeigh] = MASK; // prevent double in queue
                        }
                    }
                }
                counter++;
            } // end of if
            i++;
        } // end of for width
        i+=padding;
    }
    
    return (counter-1);
}
  
/** 
*function that applies the watershed (rainfalling algorithm)
*		input: downPos2 previously developed
*		output: result image of PixelInt result of the watershed algorithm
*/
void WatershedOperator::letsRain(ImageOf<PixelInt>& result)
{
    int i,cc,tempi;
    int *p_result=(int *)result.getRawImage();
    int *p_downPos=(int *)downPos2.getRawImage();

    int regionC;
    i=0;
    for (int r=0; r<height; r++) {
        for (int c=0; c<width; c++)	{
            regionC = 0;
            tempi = i;
            while(p_result[tempi] == -1) { // unassigned pixel
                tempRegion[regionC++] = tempi;
                tempi = p_downPos[tempi];
            }
            // a way found down to a lokalMin(lake/point)
            // set all points belong to the way down := tempi,
            // which is the counterNumber of the lokalMin
            int numOfLokalMin = p_result[tempi];
            for(cc=0; cc<regionC; cc++)
                p_result[tempRegion[cc]] = numOfLokalMin;
            i++;
        }
        i+=padding;
    }
}

/**
* finds the lower neighbour for every pixel in the src file
*/
void WatershedOperator::findLowerNeigh(const ImageOf<PixelMono>& src)
{
    static const int lokalMin = -1;
    static const int saddle = -2;
    unsigned char *p_src=(unsigned char *)src.getRawImage();
    
    //__OLD//unsigned char *p_tSrc;
    int i,j,n,max,pos,diff,p;
    PixelInt *p_downPos;
    //__OLD//downPos.Resize(widthStep,height);
    p_downPos=(PixelInt *)downPos.getRawImage();

    p=0;
    for(j=0; j<height; j++) {
        for(i=0; i<width; i++) {
            p_downPos[p]=lokalMin;
            p++;
        }
        p+=padding;
    }
    
    // first row
    // first pixel
    p=0;
    max = -1;
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighL[n];
        diff = p_src[p]-p_src[pos];
        if(diff > max) { // neigh with lower level
            max = diff;
            p_downPos[p] = pos;
        }
    }
    if( max == 0 ) { // all neighours are on the same level = saddlePoint
        p_downPos[p] = saddle;
    }
    p++;
    // first row, 1->150
    // Note that the pixel of the first row are all the same, so it is
    // useless to check the "far" neighborhoods
    for(i=1; i<width-1; i++) {
        max = -1;
        for(n=neighSize/2-1; n<neighSize; n++) { // no top Neighbor
            pos = p + neigh[n];
            diff = p_src[p]-p_src[pos];
            if(diff > max) { // neigh with lower level
                max = diff;
                p_downPos[p] = pos;
            }
        }
        if( max == 0 ) { // all neighours are on the same level = saddlePoint
            p_downPos[p] = saddle;
        }
        p++;
    }
    // last pixel of the first row
    max = -1;
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighR[n];
        diff = p_src[p]-p_src[pos];
        if(diff > max) { // neigh with lower level
            max = diff;
            p_downPos[p] = pos;
        }
    }
    if( max == 0 ) { // all neighours are on the same level = saddlePoint
        p_downPos[p] = saddle;
    }
    p++;
    p+=padding;

    // inner part
    for(j=1; j<height-1; j++) {
        // first pixel of the inner part
        max = -1;
        for(n=0; n<neighSize; n++) {
            pos = p + neighL[n];
            diff = p_src[p]-p_src[pos];
            if(diff > max) { // neigh with lower level
                max = diff;
                p_downPos[p] = pos;
            }
        }
        if( max == 0 ) { // all neighours are on the same level = saddlePoint
            p_downPos[p] = saddle;
        }
        p++;
        
        for(i=1; i<width-1; i++) {
            max = -1;
            for(n=0; n<neighSize; n++) {
                pos = p + neigh[n];
                diff = p_src[p]-p_src[pos];
                if(diff > max) { // neigh with lower level
                    max = diff;
                    p_downPos[p] = pos;
                }
            }
            if( max == 0 ) { // all neighours are on the same level = saddlePoint
                p_downPos[p] = saddle;
            }
            p++;
        }
        
        //last pixel of the inner part
        max = -1;
        for(n=0; n<neighSize; n++) {
            pos = p + neighR[n];
            diff = p_src[p]-p_src[pos];
            if(diff > max) { // neigh with lower level
                max = diff;
                p_downPos[p] = pos;
            }
        }
        if( max == 0 ) { // all neighours are on the same level = saddlePoint
            p_downPos[p] = saddle;
        }
        p++;
        
        p+=padding;
    }

    // last row
    // first pixel
    max = -1;
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighL[n];
        diff = p_src[p]-p_src[pos];
        if(diff > max) { // neigh with lower level
            max = diff;
            p_downPos[p] = pos;
        }
    }
    if( max == 0 ) { // all neighours are on the same level = saddlePoint
        p_downPos[p] = saddle;
    }
    p++;
    // last row, 1->150
    for(i=1; i<width-1; i++) {
        max = -1;
        for(n=0; n<neighSize/2+1; n++) { // no top Neighbor
            pos = p + neigh[n];
            diff = p_src[p]-p_src[pos];
            if(diff > max) { // neigh with lower level
                max = diff;
                p_downPos[p] = pos;
            }
        }
        if( max == 0 ) { // all neighours are on the same level = saddlePoint
            p_downPos[p] = saddle;
        }
        p++;
    }
    // last pixel of the last row
    max = -1;
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighR[n];
        diff = p_src[p]-p_src[pos];
        if(diff > max) { // neigh with lower level
            max = diff;
            p_downPos[p] = pos;
        }
    }
    if( max == 0 ) { // all neighours are on the same level = saddlePoint
        p_downPos[p] = saddle;
    }
    
    // try if a saddlePoint have lower "neigh"
    bool change = true;
    while (change) {
        change = false;
        
        // first row
        // first pixel
        p=0;
        if(p_downPos[p] == saddle) {
            for(n=neighSize/2-1; n<neighSize; n++) {
                pos = p + neighL[n];
                if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                    // no more saddle, no lokalMin
                    p_downPos[p] = p_downPos[pos];
                    change = true;
                    break; // next i
                }
            }
        }
        p++;
        // first row, 1->150
        for(i=1; i<width-1; i++) {
            if(p_downPos[p] == saddle) {
                for(n=neighSize/2-1; n<neighSize; n++) {
                    pos = p + neigh[n];
                    if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                        // no more saddle, no lokalMin
                        p_downPos[p] = p_downPos[pos];
                        change = true;
                        break; // next i
                    }
                }
            }
            p++;
        }
        // last pixel of the first row
        if(p_downPos[p] == saddle) {
            for(n=neighSize/2-1; n<neighSize; n++) {
                pos = p + neighR[n];
                if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                    // no more saddle, no lokalMin
                    p_downPos[p] = p_downPos[pos];
                    change = true;
                    break; // next i
                }
            }
        }
        p++;
        p+=padding;

        // inner part
        for(j=1; j<height-1; j++) {
            // first pixel inner part
            if(p_downPos[p] == saddle) {
                for(n=0; n<neighSize; n++) {
                    pos = p + neighL[n];
                    if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                        // no more saddle, no lokalMin
                        p_downPos[p] = p_downPos[pos];
                        change = true;
                        break; // next i
                    }
                }
            }
            p++;

            // 1->150
            for(i=1; i<width-1; i++) {
                if(p_downPos[p] == saddle) {
                    for(n=0; n<neighSize; n++) {
                        pos = p + neigh[n];
                        if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                            // no more saddle, no lokalMin
                            p_downPos[p] = p_downPos[pos];
                            change = true;
                            break; // next i
                        }
                    }
                }
                p++;
            }

            // last pixel inner part
            if(p_downPos[p] == saddle) {
                for(n=0; n<neighSize; n++) {
                    pos = p + neighR[n];
                    if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                        // no more saddle, no lokalMin
                        p_downPos[p] = p_downPos[pos];
                        change = true;
                        break; // next i
                    }
                }
            }
            p++;

            p+=padding;
        }
        

        // last row
        // first pixel
        if(p_downPos[p] == saddle) {
            for(n=0; n<neighSize/2+1; n++) {
                pos = p + neighL[n];
                if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                    // no more saddle, no lokalMin
                    p_downPos[p] = p_downPos[pos];
                    change = true;
                    break; // next i
                }
            }
        }
        p++;
        // 1->150
        for(i=1; i<width-1; i++) {
            if(p_downPos[p] == saddle) {
                for(n=0; n<neighSize/2+1; n++) {
                    pos = p + neigh[n];
                    if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                        // no more saddle, no lokalMin
                        p_downPos[p] = p_downPos[pos];
                        change = true;
                        break; // next i
                    }
                }
            }
            p++;
        }

        // last pixel
        if(p_downPos[p] == saddle) {
            for(n=0; n<neighSize/2+1; n++) {
                pos = p + neighR[n];
                if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                    // no more saddle, no lokalMin
                    p_downPos[p] = p_downPos[pos];
                    change = true;
                    break; // next i
                }
            }
        }

        
        /*__OLD//p=0;
        for(j=0; j<height; j++) {
            for(i=0; i<width; i++) {
                if(p_downPos[p] == saddle) {
                    for(n=0; n<neighSize; n++) {
                        pos = p + neigh[n];
                        //if (invalidNeighbor(p,pos)) continue;
                        if(p_src[p]==p_src[pos] && p_downPos[pos] >= 0) { 
                            // no more saddle, no lokalMin
                            p_downPos[p] = p_downPos[pos];
                            change = true;
                            break; // next i
                        }
                    }
                }
                p++;
            }
            p+=padding;
        }*/
    }

}

/**
*   @fn createTmpImage
*   @brief function that creates a temporary image, remaining saddle points must be lokalMins
*
*	@param src Mono Image src which is copied into tSrc
*	@return pointer to PixelInt p_downPos2
*   @warning none
*/
void WatershedOperator::createTmpImage(const ImageOf<PixelMono>& src)
{
    static const int lokalMin = -1;
    static const int saddle = -2;
    unsigned char *p_src=(unsigned char *)src.getRawImage();
    PixelInt *p_downPos;
    PixelInt *p_downPos2;
    unsigned char *p_tSrc;
    int i,j,p;

    //__OLD//downPos2=downPos;
    //__OLD//downPos2.Resize(widthStep,height);

    p_downPos=(PixelInt *)downPos.getRawImage();
    p_downPos2=(PixelInt *)downPos2.getRawImage();
    
    //remaining saddle points must be lokalMins
    // and all points<threshold are lokalMins
    tSrc=src;
    p_tSrc=(unsigned char *)tSrc.getRawImage();
    p=0;
    for(j = 0; j < height ; j++) {
        for(i = 0; i < width; i++) {
            if (p_tSrc[p] < threshold) {
                p_downPos2[p] = lokalMin;
                p_tSrc[p] = threshold;
            }
            else if(p_downPos[p] == saddle)
                p_downPos2[p] = lokalMin;
            else
                p_downPos2[p]=p_downPos[p];
            p++;
        }
        for(i=0;i<padding;i++){
            p_downPos2[p] = lokalMin;
            p++;
        }
    }
}
/*void WatershedOperator::createTmpImage(const ImageOf<PixelMono>& src)
{
    static const int lokalMin = -1;
    static const int saddle = -2;
    unsigned char *p_src=(unsigned char *)src.getRawImage();
    PixelInt *p_downPos;
    PixelInt *p_downPos2;
    unsigned char *p_tSrc;
    int i,j,p;

    //__OLD//downPos2=downPos;
    //__OLD//downPos2.Resize(widthStep,height);

    p_downPos=(PixelInt *)downPos.getRawImage();
    p_downPos2=(PixelInt *)downPos2.getRawImage();
    
    //remaining saddle points must be lokalMins
    // and all points<threshold are lokalMins
    tSrc=src;
    p_tSrc=(unsigned char *)tSrc.getRawImage();
    p=0;
    for(j = 0; j < height ; j++) {
        for(i = 0; i < width; i++) {
            if (p_tSrc[p] < threshold) {
                p_downPos2[p] = lokalMin;
                p_tSrc[p] = threshold;
            }
            else if(p_downPos[p] == saddle)
                p_downPos2[p] = lokalMin;
            else
                p_downPos2[p]=p_downPos[p];
            p++;
        }
        p+=padding;
    }
}*/


// On place apply
/*__OLD//bool WatershedOperator::apply(ImageOf<PixelMono>& srcdest)
{
    return apply(srcdest, srcdest);
}*/


/*__OLD//bool WatershedOperator::apply(const ImageOf<PixelMono>& src, ImageOf<PixelMono>& dest)
{
    //matrix<int> result;
    ImageOf<PixelInt> result;

    if (apply(src, result)) {
      tags2Watershed(result, dest);
      return true;
    }

    return false;
}*/

/**
* applies the rain watershed from the edge picture, and returns the tagged image of integers
*/
int WatershedOperator::apply(const ImageOf<PixelMono> &src, ImageOf<PixelInt> &tagged)
{
    int num_tags;
    int i;
    int *p_tagged;
    //result.resize(width,height);
    //__OLD//result.Resize(widthStep, height);
    p_tagged=(int *)tagged.getRawImage();
    
    this->height=src.height();
    this->width=src.width();
    i=0;
    for (int r=0; r<height; r++) {
        for (int c=0; c<width; c++)	{
            p_tagged[i]=-1;
            i++;
        }
        i+=padding;
    }

    /*
    * according to idea of the rainfallingWatersheds from
    * P. De Smet and Rui Luis V.P.M.Pires
    * http://telin.rug.ac.be/ipi/watershed
    */

    //findLowerNeigh(src);
    //creates tmp images and copy the src input file into the tSrc file
    createTmpImage(src);
    num_tags=markMinimas(tagged);
    letsRain(tagged);

    return num_tags;
}


/*
 * load the blue plane from ImageOf<PixelRgb>
 */
ImageOf<PixelMono>* WatershedOperator::getPlane(ImageOf<PixelRgb>* inputImage){
    unsigned char c = 0;
    int x, y, z;
    int Offset;
    int psb;
    int width=inputImage->width();
    int height=inputImage->height();
    
    IppiSize srcsize;
    srcsize.height=inputImage->height();
    srcsize.width=inputImage->width();
    Ipp8u* shift[3];
    shift[0]=ippiMalloc_8u_C1(width,height,&psb); 
    shift[1]=ippiMalloc_8u_C1(width,height,&psb);
    shift[2]=ippiMalloc_8u_C1(width,height,&psb);
    ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,psb,srcsize);
    ippiCopy_8u_C1R(shift[0],psb,outputImage->getRawImage(),outputImage->getRowSize(),srcsize);
    ippiFree(shift[0]);
    ippiFree(shift[1]);
    ippiFree(shift[2]);
    return outputImage;
}

void WatershedOperator::findNeighborhood(ImageOf<PixelInt>& tagged, int x, int y, char *blobList)
{
    int i,j,n,pos,p;
    PixelInt seed=tagged(x,y);
    PixelInt *p_tagged=(PixelInt *)tagged.getRawImage();

    // first row
    // first pixel
    p=0;
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighL[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
    p++;
    // first row, 1->150
    // Note that the pixel of the first row are all the same, so it is
    // useless to check the "far" neighborhoods
    for(i=1; i<width-1; i++) {
        for(n=neighSize/2-1; n<neighSize; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
    }
    // last pixel of the first row
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighR[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
    p++;
    p+=padding;

    // inner part
    for(j=1; j<height-1; j++) {
        // first pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighL[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
        
        for(i=1; i<width-1; i++) {
            for(n=0; n<neighSize; n++) {
                pos = p + neigh[n];
                if (p_tagged[pos]==seed) {
                    blobList[p_tagged[p]]=1;
                    break;
                }
            }
            p++;
        }
        
        //last pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighR[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
        
        p+=padding;
    }

    // last row
    // first pixel
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighL[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
    p++;
    // last row, 1->150
    for(i=1; i<width-1; i++) {
        for(n=0; n<neighSize/2+1; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
    }
    // last pixel of the last row
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighR[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
}


int WatershedOperator::applyOnOld(const ImageOf<PixelMono> &src, ImageOf<PixelInt> &result)
{
    int num_tags;
    int i;
    int *p_result;

    //__OLD//result.Resize(widthStep, height);
    p_result=(int *)result.getRawImage();
    
    i=0;
    for (int r=0; r<height; r++) {
        for (int c=0; c<width; c++)	{
            p_result[i]=-1;
            i++;
        }
        i+=padding;
    }

    createTmpImage(src);
    num_tags=markMinimas(result);
    letsRain(result);

    return num_tags;
}


/*void WatershedOperator::findNeighborhood(ImageOf<PixelInt>& tagged, int x, int y, char *blobList)
{
    int i,j,n,pos,p;
    PixelInt seed=tagged(x,y);
    PixelInt *p_tagged=(PixelInt *)tagged.getRawImage();

    // first row
    // first pixel
    p=0;
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighL[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
    p++;
    // first row, 1->150
    // Note that the pixel of the first row are all the same, so it is
    // useless to check the "far" neighborhoods
    for(i=1; i<width-1; i++) {
        for(n=neighSize/2-1; n<neighSize; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
    }
    // last pixel of the first row
    for(n=neighSize/2-1; n<neighSize; n++) {
        pos = p + neighR[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
    p++;
    p+=padding;

    // inner part
    for(j=1; j<height-1; j++) {
        // first pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighL[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
        
        for(i=1; i<width-1; i++) {
            for(n=0; n<neighSize; n++) {
                pos = p + neigh[n];
                if (p_tagged[pos]==seed) {
                    blobList[p_tagged[p]]=1;
                    break;
                }
            }
            p++;
        }
        
        //last pixel of the inner part
        for(n=0; n<neighSize; n++) {
            pos = p + neighR[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
        
        p+=padding;
    }

    // last row
    // first pixel
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighL[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
    p++;
    // last row, 1->150
    for(i=1; i<width-1; i++) {
        for(n=0; n<neighSize/2+1; n++) { // no top Neighbor
            pos = p + neigh[n];
            if (p_tagged[pos]==seed) {
                blobList[p_tagged[p]]=1;
                break;
            }
        }
        p++;
    }
    // last pixel of the last row
    for(n=0; n<neighSize/2+1; n++) {
        pos = p + neighR[n];
        if (p_tagged[pos]==seed) {
            blobList[p_tagged[p]]=1;
            break;
        }
    }
}*/
