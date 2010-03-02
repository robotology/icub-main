// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/math/Math.h>
#include <iCub/SalienceOperator.h>
#include <string>
#include <sstream>

SalienceOperator::SalienceOperator(const int width1, const int height1)//:_gaze( YMatrix(_dh_nrf, 5, DH_left[0]), YMatrix(_dh_nrf, 5, DH_right[0]), YMatrix(4, 4, TBaseline[0]) )
{
    resize(width1, height1);
    integralRG=new YARPIntegralImage(width1,height1);
    integralGR=new YARPIntegralImage(width1,height1);
    integralBY=new YARPIntegralImage(width1,height1);

    foveaBlob=new ImageOf<PixelMono>;
    foveaBlob->resize(width1,height1);
    colorVQ_img=new ImageOf<PixelBgr>;
    colorVQ_img->resize(width1,height1);
    maxSalienceBlob_img=new ImageOf<PixelMono>;
    maxSalienceBlob_img->resize(width1,height1);

    

    colorVQ=new ColorVQ(width1,height1,10);
    //reset the angular shifts
    //_angShiftMap = (double *) malloc (152 * sizeof(double));
    _angShiftMap = (double *) malloc (height1 * sizeof(double));
    for(int i=0;i<height1;i++){
        _angShiftMap[i]=0.0;
    }

    centroid_x=0;
    centroid_y=0;
    
    //Image_Data YARPLogpolar::_img;
    using namespace _logpolarParams;

    /*_img.Size_X_Orig=256;
    _img.Size_Y_Orig=256;
    _img.Size_Rho=1090;
    _img.Size_Theta=20;
    _img.Size_Fovea=256.0/1090.0;*/

    /*_img = Set_Param(
                _xsize, _ysize,
                320, 240, //256,256
                320, 240, _sfovea,
                1090,
                CUST,
                256.0/1090.0);*/

    //parameter for the giotto 2
    /*image.Size_Rho = 152;
        image.Size_Theta = 252;
        image.Size_Fovea = 42;
        image.Resolution = 1090;*/

    _img = Set_Param(
                _xsize, _ysize,
                _xsize, _ysize, //256,256
                252, 152, 42,
                1090,
                Giotto2,
                252.0/1090.0);

    _img.padding =1; // YarpImageAlign;
    _img.Pix_Numb = 2;
    _img.Fovea_Type = 0;


}

SalienceOperator::~SalienceOperator(){
    /*
    std::map<const char*,int>::iterator iterMap;
    iterMap=spikeMap.begin();
    
    int previousValue=iterMap->second;
    std::string finalKey("");
    //iterMap=spikeMap.begin();
    for(;iterMap==spikeMap.end();iterMap++){
        if(iterMap->second>previousValue){
            sprintf((char*)finalKey.c_str(),"%s",iterMap->first);
        }
    }
    */
    
}

void SalienceOperator::DrawVQColor(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged)
{
    for (int r=0; r<height; r++)
        for (int c=0; c<width; c++)
            colorVQ->DominantQuantization(m_boxes[tagged(c, r)].meanColors, id(c,r), 0.3*255);
}

/**
* Defines Valid and not valid boxes
*/
void SalienceOperator::ComputeSalienceAll(int num_blob, int last_tag){	
    int i=1;
    for (; i <= last_tag; i++) {
    //__OLD//while (i<=last_tag && num_blob>0) {
        if (m_boxes[i].areaLP) {

            m_boxes[i].areaCart=TotalArea(m_boxes[i]);
            m_boxes[i].centroid_y = (double)m_boxes[i].ysum / m_boxes[i].areaLP;
            m_boxes[i].centroid_x = (double)m_boxes[i].xsum / m_boxes[i].areaLP;

            m_boxes[i].meanRG = m_boxes[i].rgSum / m_boxes[i].areaLP;
            m_boxes[i].meanGR = m_boxes[i].grSum / m_boxes[i].areaLP;
            m_boxes[i].meanBY = m_boxes[i].bySum / m_boxes[i].areaLP;
            
            
            //__OLD//m_boxes[box_num].id = max_tag;
        } else
            m_boxes[i].valid=false;
        //__OLD//i++;
    }
}

/**
*function that returns the PixelBgr mean of the selectioned blob
*	@param tagged tagged image
*	@param rg the R+G- image
*	@param gr the G+R- image
*	@param by the B+Y- image
*	@param tag the index of the blob
*/
PixelBgr SalienceOperator::varBlob(ImageOf<PixelInt>& tagged, ImageOf<PixelMono> &rg, ImageOf<PixelMono> &gr, ImageOf<PixelMono> &by, int tag)
{
    int tmpr, tmpg, tmpb;
    PixelBgr tmp;

    tmpr=0;
    tmpg=0;
    tmpb=0;

    for (int r=m_boxes[tag].rmin; r<=m_boxes[tag].rmax; r++)
        for (int c=m_boxes[tag].cmin; c<=m_boxes[tag].cmax; c++)
            if (tagged(c, r)==tag) {
                tmpr+=rg(c, r)*rg(c, r);
                tmpg+=gr(c, r)*gr(c, r);
                tmpb+=by(c, r)*by(c, r);
            }
    
    tmpr = tmpr / m_boxes[tag].areaLP;
    tmpg = tmpg / m_boxes[tag].areaLP;
    tmpb = tmpb / m_boxes[tag].areaLP;

    tmp.r = floor(sqrt((double)tmpr - m_boxes[tag].meanRG*m_boxes[tag].meanRG));
    tmp.g = floor(sqrt((double)tmpg - m_boxes[tag].meanGR*m_boxes[tag].meanGR));
    tmp.b = floor(sqrt((double)tmpb - m_boxes[tag].meanBY*m_boxes[tag].meanBY));

    if (tmp.r==0) tmp.r=1;
    if (tmp.g==0) tmp.g=1;
    if (tmp.b==0) tmp.b=1;
    
    return tmp;
}

int SalienceOperator::countSmallBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int min_size)
{
    int num=0;
    
    for (int i=2; i<=max_tag; i++)
        if (blobList[i] && m_boxes[i].valid && m_boxes[i].areaCart < min_size)
            num++;
        else
            blobList[i]=0;

    return num;
}

void SalienceOperator::mergeBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int numBlob)
{
    for (int r=0; r<height; r++)
        for (int c=0; c<width; c++)
            if (blobList[tagged(c, r)]==1) {
                tagged(c, r)=numBlob;
                m_boxes[tagged(c, r)].valid=false;
            }
}

void SalienceOperator::RemoveNonValidNoRange(int last_tag, const int max_size, const int min_size)
{
    for (int i = 1; i <= last_tag;i++) {
        if (m_boxes[i].valid) {
            //__OLD//int area = TotalArea(m_boxes[i]);
            int area = (int)floor(m_boxes[i].areaCart);
            // eliminare i blob troppo piccoli nn è molto utile se
            // i blob vengono comunque ordinati x grandezza.
            if ( area < min_size ||	area > max_size ) {
                m_boxes[i].valid=false;
                //printf("a valid Blob with non valid Dimension \n");
            }
        }
    }
}


void SalienceOperator::ComputeMeanColors(int last_tag)
{	
    // create the subset of attentional boxes.
    for (int i = 1; i <= last_tag; i++)
        if (m_boxes[i].areaLP!=0) {
            // note that I should cast to double and add 0.5 to the result to round
            // but the presence of noise in the image has the same effect
            // and in this way it is faster!
            double value=m_boxes[i].rSum / m_boxes[i].areaLP + 0.5;
            m_boxes[i].meanColors.r = (unsigned char)value;
            value=m_boxes[i].gSum / m_boxes[i].areaLP + 0.5;
            m_boxes[i].meanColors.g = (unsigned char)value;
            value=m_boxes[i].bSum / m_boxes[i].areaLP + 0.5;
            m_boxes[i].meanColors.b = (unsigned char)value;
        }
}

void SalienceOperator::DrawMeanColorsLP(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged)
{
    /*__OLD
    //if (last_tag<numBlob || numBlob==-1) numBlob=last_tag;
    
    for (int i = 0; i < numBlob; i++) {
    //for (int i = 0; i < MaxBoxes; i++) {
        for (int r=m_boxes[i].rmin; r<=m_boxes[i].rmax; r++)
            for (int c=m_boxes[i].cmin; c<=m_boxes[i].cmax; c++)
                if (tagged(c, r)==m_boxes[i].id) {
                    id(c ,r)=m_boxes[i].meanColors;
                }
    }
    return numBlob;
    */

    for (int r=0; r<height; r++)
        for (int c=0; c<width; c++){
          id(c,r)=m_boxes[tagged(c,r)].meanColors;
        }
}

void SalienceOperator::DrawMaxSaliencyBlob(ImageOf<PixelMono>& id,int max_tag,ImageOf<PixelInt>& tagged)
{
    id.zero();
    PixelMono pixelColour=255;
    YARPBox box;
    maxSalienceBlob(tagged,max_tag,box);
    //printf("id: %d \n",box.id);
    for (int r=0; r<height; r++)
        for (int c=0; c<width; c++){
            //printf("%d ",tagged(c,r));
            if (tagged(c,r)==box.id)
                id(c,r)=pixelColour;
        }
}

void SalienceOperator::DrawStrongestSaliencyBlob(ImageOf<PixelMono>& id,int max_tag,ImageOf<PixelInt>& tagged){
    id.zero();
    PixelMono pixelColour=255;
    YARPBox box;
    std::map<std::string,int>::iterator iterMap;
    int previousValue=0;
    /*if(iterMap==NULL)
        return;*/
    //int previousValue=iterMap->second;
    std::string finalKey("");
    std::string maxKey("");
    int maxValue=0;
    //iterMap=spikeMap.begin();
    for(iterMap=spikeMap.begin();iterMap!=spikeMap.end();iterMap++){
        printf("%s->%d \n",iterMap->first.c_str(),iterMap->second);
        if(iterMap->second>maxValue){
            maxKey=iterMap->first;
            maxValue=iterMap->second;
        }
    }

    printf("strongest %s with %d \n",maxKey.c_str(),maxValue);
    std::string xStr;
    std::string yStr;
    size_t found=maxKey.find(",");
    xStr=maxKey.substr(0,found);
    yStr=maxKey.substr(found+1,maxKey.length()-found+1);
    target_x=(int)atoi(xStr.c_str());
    target_y=(int)atoi(yStr.c_str());

    for (int r=0; r<height; r++){
        for (int c=0; c<width; c++){
            //printf("%d ",tagged(c,r));
            /*if (tagged(c,r)==maxKey)
                id(c,r)=pixelColour;*/
        }
    }
    //printf("erasing the map \n");
    spikeMap.clear();
}


void SalienceOperator::countSpikes(ImageOf<PixelInt>& tagged, int max_tag, YARPBox &box){
    int max=1;
    int xcart=0;
    int ycart=0;
    
    
    for (int m = 1; m < max_tag; m++){
        //printf("blob number %d:",m);
        if (m_boxes[m].valid){
            //printf(" valid %d ---> %f \n",m,m_boxes[m].salienceTotal);
            if (m_boxes[m].salienceTotal>m_boxes[max].salienceTotal)
                max=m;
        }
        else{
            //printf(" non valid");
        }
    }

    box=m_boxes[max];
    if(max==1){
        printf("max saliency correspond to the fovea blob \n");
        maxr=0;
        maxc=0;
        centroid_x=width/2;
        centroid_y=height/2;
    }
    else{
        centerOfMassAndMass(tagged, box.id, &xcart, &ycart, &box.areaCart);
        box.centroid_x=xcart;
        box.centroid_y=ycart;
        centroid_x=xcart;
        centroid_y=ycart;
        maxc=(box.cmax+box.cmin)/2;
        maxr=(box.rmax+box.rmin)/2;
    }
    //sprintf(ke,"%d,%d",(int)(centroid_x),(int)(centroid_y));
    std::string maxKeyStr("");
    std::stringstream streamStr;
    std::stringstream streamStr2;

    streamStr<<centroid_y;
    std::string yStr(streamStr.str());
    
    streamStr2<<centroid_x;
    std::string xStr=streamStr2.str();
    
    maxKeyStr.append(xStr);
    maxKeyStr.append(",");
    maxKeyStr.append(yStr);
    

    /*
    if(centroid_x>=100)
        if(centroid_y>=100)
            sprintf((char*)maxKeyStr.c_str(),"%d,%d",(int)centroid_x,(int)centroid_y);
        else
            sprintf(maxKeyStr,"%d,0%d",(int)centroid_x,(int)centroid_y);
    else
        if(centroid_y>=100)
            sprintf(maxKeyStr,"0%d,%d",(int)centroid_x,(int)centroid_y);
        else
            sprintf(maxKeyStr,"0%d,0%d",(int)centroid_x,(int)centroid_y);
        */
    
      int previousValue=spikeMap[maxKeyStr];
      spikeMap[maxKeyStr]=previousValue+1;
    
}


void SalienceOperator::drawFoveaBlob(ImageOf<PixelMono>& id,ImageOf<PixelInt>& tagged){
    //id.Zero();
    id.zero();
    PixelMono pixelColour=255;
    //__OLD//for (int r=m_boxes[1].rmin; r<=m_boxes[1].rmax; r++)
    for (int r=0; r<height; r++)
        for (int c=0; c<width; c++)
            if (tagged(c, r)==1)
                id(c ,r)=pixelColour;
}

int SalienceOperator::DrawContrastLP(ImageOf<PixelMono>& rg, ImageOf<PixelMono>& gr,
                                     ImageOf<PixelMono>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
                                     int numBlob, float pBU, float pTD,
                                     PixelMono prg, PixelMono pgr, PixelMono pby)
{
    //__OLDIplROI zdi;
    /*int salienceBU, salienceTD;
    int borderRG;
    int borderGR;
    int borderBY;

    int a1,b1,a2,b2;

    int minSalienceBU=INT_MAX;
    int maxSalienceBU=INT_MIN;

    int minSalienceTD=INT_MAX;
    int maxSalienceTD=INT_MIN;

    int pixel0=0;
    int pixel1=1;
    
    tmp.Resize(width, height);
    tmp.Zero();
    
    /*__OLDzdi.coi=0;
    zdi.width=1;
    zdi.height=1;*/
    /*

    //__OLD((IplImage *)tmp)->roi=&zdi;
    ((IplImage *)tmp)->maskROI=tmpMsk;

    borderRG=((IplImage *)rg)->BorderMode[IPL_SIDE_BOTTOM_INDEX];
    iplSetBorderMode((IplImage *)rg, IPL_BORDER_REPLICATE, IPL_SIDE_BOTTOM, 0);
    borderGR=((IplImage *)gr)->BorderMode[IPL_SIDE_BOTTOM_INDEX];
    iplSetBorderMode((IplImage *)gr, IPL_BORDER_REPLICATE, IPL_SIDE_BOTTOM, 0);
    borderBY=((IplImage *)by)->BorderMode[IPL_SIDE_BOTTOM_INDEX];
    iplSetBorderMode((IplImage *)by, IPL_BORDER_REPLICATE, IPL_SIDE_BOTTOM, 0);

    dst.Zero();
    
    if (numBlob>imageSize) numBlob=imageSize;
    
    for (int i = 1; i < numBlob; i++) {
        if (m_boxes[i].valid) {
            int rdim=(m_boxes[i].rmax-m_boxes[i].rmin+1);
            int cdim=(m_boxes[i].cmax-m_boxes[i].cmin+1);

            //__OLDzdi.xOffset=m_boxes[i].cmin;
            //__OLDzdi.yOffset=m_boxes[i].rmin;

            iplPutPixel((IplImage*) tmpMsk, m_boxes[i].cmin, m_boxes[i].rmin, &pixel1);

            iplBlur((IplImage*) rg, (IplImage*) tmp, 3*cdim, 3*rdim, cdim, rdim);
            m_boxes[i].cRG=abs(tmp(m_boxes[i].cmin, m_boxes[i].rmin)-m_boxes[i].meanRG);
            //__OLDm_boxes[i].cRG=min(tmp(m_boxes[i].cmin, m_boxes[i].rmin), m_boxes[i].meanRG)*prg;

            iplBlur((IplImage*) gr, (IplImage*) tmp, 3*cdim, 3*rdim, cdim, rdim);
            m_boxes[i].cGR=abs(tmp(m_boxes[i].cmin, m_boxes[i].rmin)-m_boxes[i].meanGR);
            //__OLDm_boxes[i].cGR=min(tmp(m_boxes[i].cmin, m_boxes[i].rmin), m_boxes[i].meanGR)*pgr;
            
            iplBlur((IplImage*) by, (IplImage*) tmp, 3*cdim, 3*rdim, cdim, rdim);
            m_boxes[i].cBY=abs(tmp(m_boxes[i].cmin, m_boxes[i].rmin)-m_boxes[i].meanBY);
            //__OLDm_boxes[i].cBY=min(tmp(m_boxes[i].cmin, m_boxes[i].rmin), m_boxes[i].meanBY)*pby;

            salienceBU=sqrt(m_boxes[i].cRG*m_boxes[i].cRG+
                            m_boxes[i].cGR*m_boxes[i].cGR+
                            m_boxes[i].cBY*m_boxes[i].cBY);
            salienceBU=salienceBU/sqrt(3);

            /*__OLDsalienceBU=m_boxes[i].cRG;

            if (salienceBU<m_boxes[i].cGR)
                salienceBU=m_boxes[i].cGR;

            if (salienceBU<m_boxes[i].cBY)
                salienceBU=m_boxes[i].cBY;*/

            //__OLDsalienceBU=m_boxes[i].cRG+m_boxes[i].cGR+m_boxes[i].cBY;


            /*__OLDsalienceTD=abs(m_boxes[i].meanRG-prg);
                            
            if (salienceTD<abs(m_boxes[i].meanGR-pgr))
                salienceTD=abs(m_boxes[i].meanGR-pgr);
                
            if (salienceTD<abs(m_boxes[i].meanBY-pby))
                salienceTD=abs(m_boxes[i].meanBY-pby);

            salienceTD=255-salienceTD;*/
    /*

            salienceTD=sqrt((m_boxes[i].meanRG-prg)*(m_boxes[i].meanRG-prg)+
                            (m_boxes[i].meanGR-pgr)*(m_boxes[i].meanGR-pgr)+
                            (m_boxes[i].meanBY-pby)*(m_boxes[i].meanBY-pby));
            salienceTD=255-salienceTD/sqrt(3);

            //__OLD if the color is too different it isn't in the scene!
            if (salienceTD<200) salienceTD=0;

            //__OLDif (salienceTD<0) salienceTD=0;
            
            m_boxes[i].salienceBU=salienceBU;
            m_boxes[i].salienceTD=salienceTD;

            if (salienceBU<minSalienceBU)
                minSalienceBU=salienceBU;
            else if (salienceBU>maxSalienceBU)
                maxSalienceBU=salienceBU;

            if (salienceTD<minSalienceTD)
                minSalienceTD=salienceTD;
            else if (salienceTD>maxSalienceTD)
                maxSalienceTD=salienceTD;

            iplPutPixel((IplImage*) tmpMsk, m_boxes[i].cmin, m_boxes[i].rmin, &pixel0);
        }
    }

    const int maxDest=200;
    
    if (maxSalienceBU!=minSalienceBU) {
        a1=255*(maxDest-1)/(maxSalienceBU-minSalienceBU);
        b1=1-a1*minSalienceBU/255;
    } else {
        a1=0;
        b1=0;
    }

    if (maxSalienceTD!=minSalienceTD) {
        a2=255*(maxDest-1)/(maxSalienceTD-minSalienceTD);
        b2=1-a2*minSalienceTD/255;
        //__OLDa2=255*254/(minSalienceTD-maxSalienceTD);
        //__OLDb2=1-a2*maxSalienceTD/255;
    } else {
        a2=0;
        b2=0;
    }

    for (i = 0; i < numBlob; i++) {
        if (m_boxes[i].valid) {
            m_boxes[i].salienceTotal=pBU*(a1*m_boxes[i].salienceBU/255+b1)+pTD*(a2*m_boxes[i].salienceTD/255+b2);
            for (int r=m_boxes[i].rmin; r<=m_boxes[i].rmax; r++)
                for (int c=m_boxes[i].cmin; c<=m_boxes[i].cmax; c++)
                    if (tagged(c, r)==m_boxes[i].id) {
                        /*__OLDunsigned char sal=(a1*m_boxes[i].salienceBU/255+b1+a2*m_boxes[i].salienceTD/255+b2)/2;
                        if (sal>th) dst(c ,r)=sal;
                        else dst(c ,r)=0;*/
    /*
                        dst(c ,r)=m_boxes[i].salienceTotal;
                        //__OLDdst(c ,r)=a1*m_boxes[i].salienceBU/255+b1;
                        /*__OLDdst(c ,r)=a2*m_boxes[i].salienceTD/255+b2;
                        if (dst(c ,r)<240) dst(c ,r)=1;
                        else dst(c ,r)=dst(c ,r)-240;*/
    /*
                    }
        }
    }
    
    ((IplImage *)rg)->BorderMode[IPL_SIDE_BOTTOM_INDEX]=borderRG;
    ((IplImage *)gr)->BorderMode[IPL_SIDE_BOTTOM_INDEX]=borderGR;
    ((IplImage *)by)->BorderMode[IPL_SIDE_BOTTOM_INDEX]=borderBY;

    return numBlob;*/
    return 0;
}

int SalienceOperator::DrawContrastLP2(ImageOf<PixelMono>& rg, ImageOf<PixelMono>& gr,
                                  ImageOf<PixelMono>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
                                  int numBlob, float pBU, float pTD,
                                  PixelMono prg, PixelMono pgr, PixelMono pby, PixelMono maxDest)
{
    int salienceBU, salienceTD;
    //coefficients
    double a1,b1,a2,b2,a3,b3;

    int minSalienceBU=INT_MAX;
    int maxSalienceBU=INT_MIN;

    int minSalienceTD=INT_MAX;
    int maxSalienceTD=INT_MIN;

    int minSalienceTot=INT_MAX;
    int maxSalienceTot=INT_MIN;
    //set to zero the outContrastLP
    dst.zero();
    
    if (numBlob>imageSize) 
        numBlob=imageSize;

    integralRG->computeCartesian(rg);
    integralGR->computeCartesian(gr);
    integralBY->computeCartesian(by);

    for (int i = 1; i < numBlob; i++) {
        // The salience should not change if a blob is eliminated because it is too small
        // or too big
        // I could not change the scale of the salience and change only the color of the
        // most salient blob
        if (m_boxes[i].valid) {
            int tmp,t,t_abs;
            double mlp;
            
            //__OLD//int rdim=(double)(m_boxes[i].rmax-m_boxes[i].rmin+1)*.75;
            //__OLD//int cdim=(double)(m_boxes[i].cmax-m_boxes[i].cmin+1)*.75;
            int rdim=(double)(m_boxes[i].rmax-m_boxes[i].rmin+1)*1;
            int cdim=(double)(m_boxes[i].cmax-m_boxes[i].cmin+1)*1;


            int a,b,c,d;
            c = m_boxes[i].rmax+rdim;
            d = m_boxes[i].rmin-rdim;
            if (cdim<width/3) {
                a = m_boxes[i].cmin+cdim;
                b = m_boxes[i].cmin-cdim;
            } else {
                a = 0;
                b = 251;
            }

            /*__OLD//tmp=255*height*width*integralRG.getSaliencyLp(m_boxes[i].cmax+cdim,
                m_boxes[i].cmin-cdim,
                m_boxes[i].rmax+rdim,
                m_boxes[i].rmin-rdim)/(rdim*cdim);
            m_boxes[i].cRG=abs(tmp-m_boxes[i].meanRG);
            tmp=255*height*width*integralGR.getSaliencyLp(m_boxes[i].cmax+cdim,
                m_boxes[i].cmin-cdim,
                m_boxes[i].rmax+rdim,
                m_boxes[i].rmin-rdim)/(rdim*cdim);
            m_boxes[i].cGR=abs(tmp-m_boxes[i].meanGR);
            tmp=255*height*width*integralBY.getSaliencyLp(m_boxes[i].cmax+cdim,
                m_boxes[i].cmin-cdim,
                m_boxes[i].rmax+rdim,
                m_boxes[i].rmin-rdim)/(rdim*cdim);
            m_boxes[i].cBY=abs(tmp-m_boxes[i].meanBY);*/

            //CALCULATES BOTTOM-UP SALIENCY
            // as the absolute distance between the colour of the blob and
            // the colour of surrondings(rectangular window)

            mlp=integralRG->getMeanLp(a,b,c,d);
            tmp=255*height*width*(int)floor(mlp);
            t=tmp-m_boxes[i].meanRG;
            t_abs=abs(t);
            m_boxes[i].cRG=t_abs;

            mlp=integralGR->getMeanLp(a,b,c,d);
            tmp=255*height*width*mlp;
            t=tmp-m_boxes[i].meanGR;
            t_abs=abs(t);
            m_boxes[i].cGR=t_abs;
            
            mlp=integralBY->getMeanLp(a,b,c,d);
            tmp=255*height*width*(int)floor(integralBY->getMeanLp(a,b,c,d));
            t=tmp-m_boxes[i].meanBY;
            t_abs=abs(t);
            m_boxes[i].cBY=t_abs;
            

            /*__OLD//salienceBU=sqrt(m_boxes[i].cRG*m_boxes[i].cRG+
                            m_boxes[i].cGR*m_boxes[i].cGR+
                            m_boxes[i].cBY*m_boxes[i].cBY);
            salienceBU=salienceBU/sqrt(3);*/

            /*__OLD//salienceBU=m_boxes[i].cRG;
            if (salienceBU<m_boxes[i].cGR)
                salienceBU=m_boxes[i].cGR;
            if (salienceBU<m_boxes[i].cBY)
                salienceBU=m_boxes[i].cBY;*/

            //__OLD//sum of abs of contrast differences
            // BU(bottom up) is the sum of abs of contrast divergence
            salienceBU=m_boxes[i].cRG+m_boxes[i].cGR+m_boxes[i].cBY;
            
            // CALCULATE THE TOP-DOWN SALIENCY 
            // as the euclidian distance between the colour of the blob and the colour of the target
            //prg mono plane of RG,pgr mono plane of GR, pby mono plane of BY
            //TD is the square root of the sum of square errors between planes and means
            double RGdistance=m_boxes[i].meanRG-prg;
            double GRdistance=m_boxes[i].meanGR-pgr;
            double BYdistance=m_boxes[i].meanBY-pby;
           
            salienceTD=sqrt((double)RGdistance*RGdistance+
                            GRdistance*GRdistance+
                            BYdistance*BYdistance);
            //printf("%d:   %d,%d,%d \n",i,m_boxes[i].meanRG,m_boxes[i].meanGR,m_boxes[i].meanBY);
            salienceTD=255-salienceTD/sqrt(3.0);
           
            
            /*__OLD//salienceTD=abs(m_boxes[i].meanRG-prg);
                            
            if (salienceTD<abs(m_boxes[i].meanGR-pgr))
                salienceTD=abs(m_boxes[i].meanGR-pgr);
                
            if (salienceTD<abs(m_boxes[i].meanBY-pby))
                salienceTD=abs(m_boxes[i].meanBY-pby);

            salienceTD=255-salienceTD;*/

            //__OLD//if the color is too different it isn't in the scene!
            //__OLD//if (salienceTD<0) salienceTD=0;
            /*if (salienceTD<200)
                salienceTD=200;*/
            
            
            m_boxes[i].salienceBU=salienceBU;
            m_boxes[i].salienceTD=salienceTD;

            if (salienceBU<minSalienceBU)
                minSalienceBU=salienceBU;
            else if (salienceBU>maxSalienceBU)
                maxSalienceBU=salienceBU;

            if (salienceTD<minSalienceTD)
                minSalienceTD=salienceTD;
            else if (salienceTD>maxSalienceTD)
                maxSalienceTD=salienceTD;
        }
    }

    //coefficients for normalisation of BU (a1,b1)
    if (maxSalienceBU!=minSalienceBU) {
        //__OLD//a1=255.*(maxDest-1)/(maxSalienceBU-minSalienceBU);
        a1=254./(maxSalienceBU-minSalienceBU);
        b1=1.-a1*minSalienceBU;
    } else {
        a1=0;
        b1=0;
    }
    //coefficient for normalisation of TD (a2,b2)
    if (maxSalienceTD!=minSalienceTD) {
        //__OLD//a2=255.*(maxDest-1)/(maxSalienceTD-minSalienceTD);
        a2=254./(maxSalienceTD-minSalienceTD);
        b2=1.-a2*minSalienceTD;
    } else {
        a2=0;
        b2=0;
    }
    //for every blobs calculates the salienceTotal
    for (int i = 1; i < numBlob; i++) {
            if (m_boxes[i].valid) {
            //calculates the salience total
            m_boxes[i].salienceTotal=pBU*(a1*m_boxes[i].salienceBU+b1)+pTD*(a2*m_boxes[i].salienceTD+b2);
            //correction between salience interval
            if (m_boxes[i].salienceTotal<minSalienceTot)
                minSalienceTot=m_boxes[i].salienceTotal;
            else if (m_boxes[i].salienceTotal>maxSalienceTot)
                maxSalienceTot=m_boxes[i].salienceTotal;
        }
    }
    //normalise the value into the range of grayscale image (a3,b3)
    if (maxSalienceTot!=minSalienceTot) {
        a3=((double)(maxDest-1))/(maxSalienceTot-minSalienceTot);
        b3=1.-a3*minSalienceTot;
    } else {
        a3=0;
        b3=0;
    }
    //creates the image composed of the value of saliency of every blob
    for (int i = 1; i < numBlob; i++) {
        if (m_boxes[i].valid) {
            //__OLD//if ((m_boxes[i].salienceBU==maxSalienceBU && pBU==1) || (m_boxes[i].salienceTD==maxSalienceTD && pTD==1))
            //if ((m_boxes[i].salienceBU==maxSalienceBU && pBU==1)||(m_boxes[i].salienceTD==maxSalienceTD && pTD==1))
            if (m_boxes[i].salienceTotal==maxSalienceTot)
                m_boxes[i].salienceTotal=255;
            else 
                m_boxes[i].salienceTotal=a3*m_boxes[i].salienceTotal+b3;
            
            //m_boxes[i].salienceTotal=m_boxes[i].salienceTotal;

            //for the whole blob in this loop
            for (int r=m_boxes[i].rmin; r<=m_boxes[i].rmax; r++)
                for (int c=m_boxes[i].cmin; c<=m_boxes[i].cmax; c++){
                    int tag=tagged(c,r);
                    if (tag==m_boxes[i].id) {
                        //__OLD//if (sal>th) dst(c ,r)=sal;
                        //__OLD//else dst(c ,r)=0;
                        //set the image outContrastLP with the value salienceTotal
                        dst(c ,r)=(PixelMono)m_boxes[i].salienceTotal;
                    }
                }
        }
    }
    
    return numBlob;
}



// Variance method
void SalienceOperator::fuseFoveaBlob3(ImageOf<PixelInt>& tagged, char *blobList, PixelBgr var, int max_tag)
{
    const PixelMono rg0=m_boxes[1].meanRG;
    const PixelMono gr0=m_boxes[1].meanGR;
    const PixelMono by0=m_boxes[1].meanBY;
    
    for (int i=2; i<=max_tag; i++) {
        if (blobList[i])
            if (abs(m_boxes[i].meanRG-rg0)/var.r+abs(m_boxes[i].meanGR-gr0)/var.g+abs(m_boxes[i].meanBY-by0)/var.b<=2) {
                blobList[i]=2;
                m_boxes[1].areaLP+=m_boxes[i].areaLP;
                //__OLD//m_boxes[1].areaCart+=TotalArea(m_boxes[tag]);
                m_boxes[1].areaCart+=m_boxes[i].areaCart;
                m_boxes[1].rgSum+=m_boxes[i].rgSum;
                m_boxes[1].grSum+=m_boxes[i].grSum;
                m_boxes[1].bySum+=m_boxes[i].bySum;
                m_boxes[1].rSum+=m_boxes[i].rSum;
                m_boxes[1].gSum+=m_boxes[i].gSum;
                m_boxes[1].bSum+=m_boxes[i].bSum;
                m_boxes[1].xsum+=m_boxes[i].xsum;
                m_boxes[1].ysum+=m_boxes[i].ysum;

                if (m_boxes[i].xmin<m_boxes[1].xmin) m_boxes[1].xmin=m_boxes[i].xmin;
                if (m_boxes[i].ymin<m_boxes[1].ymin) m_boxes[1].ymin=m_boxes[i].ymin;
                if (m_boxes[i].xmax>m_boxes[1].xmax) m_boxes[1].xmax=m_boxes[i].xmax;
                if (m_boxes[i].xmax>m_boxes[1].xmax) m_boxes[1].xmax=m_boxes[i].xmax;

                if (m_boxes[i].cmin<m_boxes[1].cmin) m_boxes[1].cmin=m_boxes[i].cmin;
                if (m_boxes[i].rmin<m_boxes[1].rmin) m_boxes[1].rmin=m_boxes[i].rmin;
                if (m_boxes[i].cmax>m_boxes[1].cmax) m_boxes[1].cmax=m_boxes[i].cmax;
                if (m_boxes[i].rmax>m_boxes[1].rmax) m_boxes[1].rmax=m_boxes[i].rmax;

                m_boxes[i].valid=false;
            }
    }
        
    m_boxes[1].centroid_y = (double)m_boxes[1].ysum / m_boxes[1].areaLP;
    m_boxes[1].centroid_x = (double)m_boxes[1].xsum / m_boxes[1].areaLP;

    m_boxes[1].meanRG = m_boxes[1].rgSum / m_boxes[1].areaLP;
    m_boxes[1].meanGR = m_boxes[1].grSum / m_boxes[1].areaLP;
    m_boxes[1].meanBY = m_boxes[1].bySum / m_boxes[1].areaLP;

    for (int r=0; r<height; r++)
        for (int c=0; c<width; c++)
            if (blobList[tagged(c,r)]==2) 
                tagged(c,r)=1;
}
/**
* calculates the center of mass of a blob with a particular tag
*/
void SalienceOperator::centerOfMassAndMass(ImageOf<PixelInt> &in, PixelInt tag, int *x, int *y, double *mass){
    int r,t;
    double J=0.5;
    double areaT = 0.0;
    double sumX = 0.0;
    double sumY = 0.0;
    PixelInt *src;
    for(r = 0; r < height; r++) {
        int sumTmpX = 0;
        int sumTmpY = 0;
        int areaTmp = 0;

        src = (PixelInt *)in.getRowArray()[r];
        for(t = 0; t < width; t++) {
            if (*src==tag) {
                int tmpX;
                int tmpY;

                this->logPolar2Cartesian(r, t, tmpX, tmpY);

                sumTmpX += tmpX;
                sumTmpY += tmpY;
                areaTmp++;
            }
            src++;
        }
        
        sumX+=sumTmpX*J;
        sumY+=sumTmpY*J;
        areaT+=areaTmp*J;
    }
    
    *mass=areaT;
    
    if (areaT != 0)	{
        *x = (int)(sumX/areaT + .5);
        *y = (int)(sumY/areaT + .5);
    } else {
        *x = 0;
        *y = 0;
    }
}

/**
* selected the blob with maximum saliency
*/
void SalienceOperator::maxSalienceBlob(ImageOf<PixelInt>& tagged, int max_tag, YARPBox &box)
{
    int max=1;
    int xcart=0;
    int ycart=0;
    
    for (int m = 1; m < max_tag; m++){
        //printf("blob number %d:",m);
        if (m_boxes[m].valid){
            //printf(" valid %d ---> %f \n",m,m_boxes[m].salienceTotal);
            if (m_boxes[m].salienceTotal>m_boxes[max].salienceTotal)
                max=m;
        }
        else{
            //printf(" non valid");
        }
    }

    box=m_boxes[max];
    if(max==1){
        printf("max saliency correspond to the fovea blob \n");
        maxr=0;
        maxc=0;
        centroid_x=width/2;
        centroid_y=height/2;
    }
    else{
        centerOfMassAndMass(tagged, box.id, &xcart, &ycart, &box.areaCart);
        box.centroid_x=xcart;
        box.centroid_y=ycart;
        this->centroid_x=xcart;
        this->centroid_y=ycart;
        this->maxc=(box.cmax+box.cmin)/2;
        this->maxr=(box.rmax+box.rmin)/2;
    }
    //printf("rho:%f,theta:%f \n",maxc,maxr);
    //printf("centroidx:%f,centroid_y:%f \n",centroid_x,centroid_y);
    //_gaze.computeRay(YARPBabybotHeadKin::KIN_LEFT_PERI, box.elev, box.az, (int)box.centroid_x, (int)box.centroid_y);
}


void SalienceOperator::resize(const int width1, const int height1)
{
    this->width=width1;
    this->height=height1;


    imageSize=width*height;

    m_boxes = new YARPBox[imageSize];

    _checkCutted = new bool[imageSize];

    /*tmpMsk = iplCreateImageHeader(
        1,
        0,
        IPL_DEPTH_1U,
        "GRAY",
        "G",
        IPL_DATA_ORDER_PLANE,
        IPL_ORIGIN_TL,
        IPL_ALIGN_QWORD,
        width1,
        height1,
        NULL,
        NULL,
        NULL,
        NULL);*/

    //iplAllocateImage(tmpMsk,1,0);

    //integralRG.resize(width1, height1);
    //integralGR.resize(width1, height1);
    //integralBY.resize(width1, height1);
}

/**
*  creates a catalog of blobs from the saved present in tagged
* @param rg R+G-
* @param gr G+R-
* @param by B+Y-
* @param red the Red plane
* @param blue the blue plane
* @param green green planes
* @param tagged tagged image 
*/
void SalienceOperator::blobCatalog(ImageOf<PixelInt>& tagged,
                               ImageOf<PixelMono> &rg,
                               ImageOf<PixelMono> &gr,
                               ImageOf<PixelMono> &by,
                               ImageOf<PixelMono> &r1,
                               ImageOf<PixelMono> &g1,
                               ImageOf<PixelMono> &b1,
                               int last_tag){
    //initialisation of all the blobs
    for (int i = 0; i <= last_tag; i++) {
        m_boxes[i].cmax = m_boxes[i].rmax = 0;
        m_boxes[i].cmin = width;
        m_boxes[i].rmin = height;

        m_boxes[i].xmax = m_boxes[i].ymax = 0;
        m_boxes[i].xmin = m_boxes[i].ymin = 255;

        //m_boxes[i].xmin = m_lp.GetCWidth();
        //m_boxes[i].ymin = m_lp.GetCHeight();

        //__OLD//m_boxes[i].total_sal = 0;
        m_boxes[i].areaLP = 0;
        //__OLD//m_boxes[i].areaCart = 0;
        m_boxes[i].xsum = m_boxes[i].ysum = 0;
        m_boxes[i].valid = false;
        
        m_boxes[i].cutted = false;
        
        m_boxes[i].id = i;
        
        //__OLD//m_boxes[i].edge = false;

        m_boxes[i].rgSum=0;
        m_boxes[i].grSum=0;
        m_boxes[i].bySum=0;

        m_boxes[i].rSum=0;
        m_boxes[i].gSum=0;
        m_boxes[i].bSum=0;
    }
  
    // ------   special case for the null tag (0) which is not fovea ----------
    // null tag is not used!!!
    m_boxes[0].rmax = m_boxes[0].rmin = height/2;
    m_boxes[0].cmax = m_boxes[0].cmin = width/2;
    //m_boxes[0].xmax = m_boxes[0].xmin = m_lp.GetCWidth() / 2;
    //m_boxes[0].ymax = m_boxes[0].ymin = m_lp.GetCHeight() / 2;
    m_boxes[0].salienceTotal=0;
    m_boxes[0].valid = false;

    //---- check cutted blobs -------
    memset(_checkCutted, 0, sizeof(bool)*width*height);
    for (int r = 0; r < height; r++)
        if (tagged(0, r)==tagged(width-1, r)) {
            m_boxes[tagged(0, r)].cutted=true;
            m_boxes[tagged(0, r)].indexCutted=r;
        }
    

    //------ avereging of pixels, getting the dimensions ----------
    // pixels are logpolar, averaging is done in cartesian
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            long int tag_index = tagged(c, r);
            //printf("%d,", tag_index);
            m_boxes[tag_index].areaLP++;
            //m_boxes[tag_index].areaCart+=pixelSize[r];

            m_boxes[tag_index].valid = true;

            int x, y;
            //get the x,y in order to have the averaging
            //m_lp.Logpolar2Cartesian(r, c, x, y);
            logPolar2Cartesian(r,c,x,y);
            //printf("x:%d,y:%d  ",x,y);


            if (m_boxes[tag_index].ymax < y) 
                m_boxes[tag_index].ymax = y;
            if (m_boxes[tag_index].ymin > y) 
                m_boxes[tag_index].ymin = y;
            if (m_boxes[tag_index].xmax < x) 
                m_boxes[tag_index].xmax = x;
            if (m_boxes[tag_index].xmin > x) 
                m_boxes[tag_index].xmin = x;

            m_boxes[tag_index].ysum += y;
            m_boxes[tag_index].xsum += x;

            if (m_boxes[tag_index].rmax < r) 
                m_boxes[tag_index].rmax = r;
            if (m_boxes[tag_index].rmin > r) 
                m_boxes[tag_index].rmin = r;
            if (!m_boxes[tag_index].cutted) {
                if (m_boxes[tag_index].cmax < c) m_boxes[tag_index].cmax = c;
                if (m_boxes[tag_index].cmin > c) m_boxes[tag_index].cmin = c;
            } else
                _checkCutted[m_boxes[tag_index].indexCutted*height+c]=true;
                

            m_boxes[tag_index].rgSum += rg(c, r);
            m_boxes[tag_index].grSum += gr(c, r);
            m_boxes[tag_index].bySum += by(c, r);

            //unsigned char value=r1(c, r);
            m_boxes[tag_index].rSum += r1(c, r);
            m_boxes[tag_index].gSum += g1(c, r);
            m_boxes[tag_index].bSum += b1(c, r);
        }
    }
    
    //----- processing of cutted blobs -------------------
    for (int i = 1; i <= last_tag; i++) {
        if (m_boxes[i].cutted) {
            int index = m_boxes[i].indexCutted;
            int c = 1;
            while (c<width && _checkCutted[index*height+c]==true)
                c++;
            if (c<width) {
                m_boxes[i].cmax=c-1;
                c++;
                while (_checkCutted[index*height+c]==false)
                    c++;
                m_boxes[i].cmin=c-width;
            } else {
                m_boxes[i].cmin=0;
                m_boxes[i].cmax=width;
            }
        }
    }
}	

/**
* set paramenters of the saliency operator
* @param SXO size along x axis of the original image
* @param SYO size of the original image along y axis
* @param SXR size of the reconstruted image along x axis
* @param SYR size of the reconstructed image along y axis
* @param rho rho of logPolar conversion
* @param theta theta of the logPolar conversion
* @param fovea fovea size
* @param resolution resolution of the image
* @param LPMode logPolar modality (GIOTTO1, GIOTTO2, CUST)
* @param ZoomLevel level of the zoom
*/
Image_Data SalienceOperator::Set_Param(
                     int SXO,  //
                     int SYO,   //
                     int SXR, //
                     int SYR, //
                     int rho, //
                     int theta, //
                     int fovea, //
                     int resolution, //
                     int LPMode,  //
                     double ZoomLevel  //level of the zoom
                     )
{
    int Color = 3;
    bool Landscape = true;
    Image_Data image;
    image.padding = 1; //No Padding

    switch (LPMode)
    {
    case Giotto1:

        image.Size_Rho = 76;
        image.Size_Theta = 128;
        image.Size_Fovea = 20;
        image.Resolution = 600;
        break;

    case Giotto2:

        image.Size_Rho = 152;
        image.Size_Theta = 252;
        image.Size_Fovea = 42;
        image.Resolution = 1090;
        break;

    case CUST:

        image.Size_Rho = rho;
        image.Size_Theta = theta;
        image.Size_Fovea = fovea;
        image.Resolution = resolution;
        image.Size_X_Remap = SXR;
        image.Size_Y_Remap = SYR;
        break;
    }
    
    image.Size_LP = image.Size_Rho * image.Size_Theta;
    image.Size_X_Orig = SXO;
    image.Size_Y_Orig = SYO;
    image.Size_X_Remap= SXR;
    image.Size_Y_Remap= SYR;
    image.Size_Img_Orig = image.Size_X_Orig*image.Size_Y_Orig;
    image.Size_Img_Remap = image.Size_X_Remap * image.Size_Y_Remap;
    image.LP_Planes = Color;
    image.Orig_Planes = Color;
    image.Remap_Planes = Color;
    image.Valid_Log_Index = false;
    image.Log_Index = 1.0;

    if (ZoomLevel == FITIN){
        image.Zoom_Level = (double)(image.Size_Y_Remap);
        image.Zoom_Level /= (double)(image.Resolution);
    }
    else if (ZoomLevel == FITOUT){
        image.Zoom_Level = (double)(image.Size_Y_Remap*image.Size_Y_Remap);
        image.Zoom_Level += (double)(image.Size_X_Remap*image.Size_X_Remap);
        image.Zoom_Level = (double)sqrt(image.Zoom_Level);
        image.Zoom_Level /= (double)(image.Resolution);
    }
    else image.Zoom_Level = ZoomLevel;
    image.Orig_LandScape  = Landscape;
    image.Remap_LandScape = Landscape;
    image.Pix_Numb = 4;
    image.Fovea_Type = 0;
    image.Ratio = 1.00;
    image.dres = (double) image.Resolution;
    image.Fovea_Display_Mode = 0;

    return image;
}


/**
* trasforms a logPolar coordinate (rho,theta) into a cartesian coordinate (x,y)
*/
void SalienceOperator::logPolar2Cartesian(int irho, int itheta, int& ox, int& oy){
    double xx = 0;
    double yy = 0;
    //double _xsize=2;
    //double _ysize=2;
    

    Get_XY_Center(&xx, &yy, irho, itheta, &_img, _angShiftMap);

    using namespace _logpolarParams;
    ox = int(xx + .5) + _xsize/2;
    oy = _ysize/2 - int(yy + .5);

    //return true;
}

double SalienceOperator::Compute_Index(double Resolution, int Fovea, int SizeRho)
{
    double DValue,Value, Tempt, Dx, ADx;
    double x1,x2;
    double Tolerance = 0.0001;

    int exp = SizeRho - Fovea;
    int j;

    x1 = 1.0;
    x2 = 3.0;
    Dx = 100;
    ADx = 100;

    j=0;
    Tempt = (double)(x1+x2)/2;
    while  (ADx>Tolerance) 
    {
        if (Dx>=0)
            ADx = Dx;
        else
            ADx = -Dx;

        Value = pow(Tempt,exp+1)-((Resolution/2)-Fovea+0.5)*(Tempt-1)-Tempt;
        Value = ((Tempt*(pow(Tempt,exp)-1))/(Tempt-1)) -(Resolution/2)+Fovea-0.5;
        DValue = (exp+1)*pow(Tempt,exp)-((Resolution/2)-Fovea+0.5)-1;
        DValue = ((exp)*pow(Tempt,exp+1)-(exp+1)*pow(Tempt,exp)+1)/((Tempt-1)*(Tempt-1));
        Dx = Value/DValue;
        Tempt -= Dx;
        j++;
    }

    return Tempt;
}

int SalienceOperator::Get_XY_Center(double *xx, double *yy, int rho, int theta, Image_Data *par, double *Ang_Shift)
{
    double scalefactor;
    int Temp_Size_Theta;
    double A,B;
    double mod;
    double rd, td;

    if (rho != 0)
    {
        rd = rho+0.5;
        td = theta+0.5;
    }
    else
    {
        rd = rho;
        td = theta;
    }

    if (!par->Valid_Log_Index){
        par->Log_Index = Compute_Index(par->Resolution,par->Size_Fovea,par->Size_Rho);
        par->Valid_Log_Index = true;
    }

    scalefactor = par->Zoom_Level;
    B = par->Log_Index/(par->Log_Index-1);
    A = par->Size_Fovea - B - 0.5;

    if (rho<par->Size_Fovea)
    {
        Temp_Size_Theta = par->Size_Theta;
        mod = rd-0.5;
        if (rho==0)
        {
            Temp_Size_Theta = 1;
            mod = 0;
        }
        else if (par->Fovea_Display_Mode < 2)
            Temp_Size_Theta = (par->Size_Theta/par->Size_Fovea) * rho;
    }
    else
    {
        Temp_Size_Theta = par->Size_Theta;
        mod = A+B*pow(par->Log_Index,rd-par->Size_Fovea);
    }
        if (Temp_Size_Theta>par->Size_Theta)
            Temp_Size_Theta = par->Size_Theta;

    *xx = mod * cos(Ang_Shift[rho]+td*PI/(Temp_Size_Theta/2.0)) * scalefactor;
    *yy = mod * sin(Ang_Shift[rho]+td*PI/(Temp_Size_Theta/2.0)) * scalefactor;

    return 0;
}

void SalienceOperator::get_XY_Center(int *xx, int *yy, int irho, int itheta, yarp::sig::ImageOf<PixelRgb> *_img){
    int width=_img->width();
    int height=_img->height();
    *xx=width/2;
    *yy=height/2;
}