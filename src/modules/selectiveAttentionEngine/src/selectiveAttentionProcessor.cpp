// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/selectiveAttentionProcessor.h>


#include <ipp.h>
#include <ipps.h>
//#include <qimage.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::dev;
using namespace std;

#define Giotto1 0
#define Giotto2 1
#define CUST 20
#define FITIN   99
#define FITOUT 101
#define CODELENGHT 7

namespace _logpolarParams {
    const int _xsize = 320;     // const int _xsize = 256;
    const int _ysize = 240;     // const int _ysize = 256;
    const int _srho = 152;
    const int _stheta = 240;    // const int _stheta = 252;
    const int _sfovea = 42;
    
    const int _xsizefovea = 128;
    const int _ysizefovea = 128;
    // this is the ratio between the full size cartesian image and the actual one
    const double _ratio = 0.25;		// 1/4
    
    // parameter of the transformation
    const double _q = _stheta/(2*3.1415926535897932384626433832795);
    const double _lambda = 1.02314422608633;
    const double _logLambda = log(_lambda);
    const double _k1 = (_sfovea - 0.5)+(_lambda)/(1-_lambda);
    const double _k2 = _lambda/(pow(_lambda,_sfovea)*(_lambda-1));
};

struct Image_Data {
    // Logarithm Index
    double Log_Index;
    bool Valid_Log_Index;

    // Zoom Level of the Remapped Image
    double Zoom_Level;

    // Ratio between the diameter of the image and the size of the smallest pixel
    int Resolution;
    double dres;

//	int Fovea_Display_Mode; //0 Sawtooth (Raw); 1 Triangular; 2 Complete

    // Log Polar Metrics
    int Size_Rho;
    int Size_Theta;
    int Size_Fovea;
    int Size_LP;
    int Fovea_Type; //0->3 Giotto 2.0; 4->7 Giotto 2.1 //0;4 Sawtooth (Raw); 1;5 Triangular; 2;6 Complete
    int Pix_Numb;
    int Fovea_Display_Mode;

    // Remapped Cartesian Metrics
    int Size_X_Remap;
    int Size_Y_Remap;
    int Size_Img_Remap;

    // Original Cartesian Metrics
    int Size_X_Orig;
    int Size_Y_Orig;
    int Size_Img_Orig;

    // Color Depth of the Images
    int Orig_Planes;
    int Remap_Planes;
    int LP_Planes;

    // Orientation of the Cartesian Image
    bool Orig_LandScape;
    bool Remap_LandScape;

    int padding;

    float Ratio;  //Used just for naming purpose
};

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
Image_Data Set_Param(
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
        image.Size_Fovea = 64;
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

double Compute_Index(double Resolution, int Fovea, int SizeRho)
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

int Get_XY_Center(double *xx, double *yy, int rho, int theta, Image_Data *par, double *Ang_Shift) {
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

    const double PI = 3.1415926535897932384626433832795;
    *xx = mod * cos(Ang_Shift[rho]+td*PI/(Temp_Size_Theta/2.0)) * scalefactor;
    *yy = mod * sin(Ang_Shift[rho]+td*PI/(Temp_Size_Theta/2.0)) * scalefactor;

    return 0;
}


selectiveAttentionProcessor::selectiveAttentionProcessor(int rateThread):RateThread(rateThread)
{
    this->inImage=new ImageOf<PixelRgb>;
    reinit_flag=false;
    inputImage_flag=0;
    idle=true;
    interrupted=false;
    gazePerform=false;

    cLoop=0;
    
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
    outputImage2=new ImageOf<PixelMono>;
    linearCombinationImage=new ImageOf<PixelMono>;

}

selectiveAttentionProcessor::~selectiveAttentionProcessor(){
    printf("Destructor \n");
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
    delete outputImage2;
    
}

selectiveAttentionProcessor::selectiveAttentionProcessor(ImageOf<PixelRgb>* inputImage):RateThread(THREAD_RATE) {
    this->inImage=inputImage;
    //this->portImage=portImage;

    //edgesOutput=new ImageOf<PixelMono>;
    //portImage=new ImageOf<PixelRgb>;
    tmp=new ImageOf<PixelMono>;
}

void selectiveAttentionProcessor::reinitialise(int width, int height){
    this->srcsize.width=width;
    this->srcsize.height=height;
    this->width=width;
    this->height=height;

    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(width,height);

    linearCombinationImage=new ImageOf<PixelMono>;
    linearCombinationImage->resize(width,height);

    outputImage=new ImageOf<PixelMono>;
    outputImage->resize(width,height);
    map1_yarp=new ImageOf<PixelMono>;
    map1_yarp->resize(width,height);
    map2_yarp=new ImageOf<PixelMono>;
    map2_yarp->resize(width,height);
    map3_yarp=new ImageOf<PixelMono>;
    map3_yarp->resize(width,height);
    map4_yarp=new ImageOf<PixelMono>;
    map4_yarp->resize(width,height);
    map5_yarp=new ImageOf<PixelMono>;
    map5_yarp->resize(width,height);
    map6_yarp=new ImageOf<PixelMono>;
    map6_yarp->resize(width,height);
    
}

void selectiveAttentionProcessor::resizeImages(int width,int height) {

    tmp->resize(width,height);
    //portImage->resize(width,height);
    /*map1_yarp->resize(width,height);
    map2_yarp->resize(width,height);
    map3_yarp->resize(width,height);
    map4_yarp->resize(width,height);
    map5_yarp->resize(width,height);
    map6_yarp->resize(width,height);*/

    inImage->resize(width,height);

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
    outputImage2->resize(width,height);
    linearCombinationImage->resize(width,height);

    cvImage16= cvCreateImage(cvSize(width,height),IPL_DEPTH_16S,1);
    cvImage8= cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
}

/**
*	initialization of the thread 
*/
bool selectiveAttentionProcessor::threadInit(){
    printf("Thread initialization .... \n");
    //input ports 
    inImagePort.open(getName("/image:i").c_str());
    map1Port.open(getName("/map1:i").c_str());
    map2Port.open(getName("/map2:i").c_str());
    map3Port.open(getName("/map3:i").c_str());
    map4Port.open(getName("/map4:i").c_str());
    map5Port.open(getName("/map5:i").c_str());
    map6Port.open(getName("/map6:i").c_str());

    selectedAttentionPort.open(getName("/attention:o").c_str());
    linearCombinationPort.open(getName("/combination2:o").c_str());
    centroidPort.open(getName("/centroid:o").c_str());
    feedbackPort.open(getName("/feedback:o").c_str());

    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/gaze");

    clientGazeCtrl=new PolyDriver(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    return true;
}

void selectiveAttentionProcessor::setName(std::string str){
    this->name=str; 
}


std::string selectiveAttentionProcessor::getName(const char* p){
    string str(name);
    str.append(p);
    //printf("name: %s", name.c_str());
    return str;
}

/**
* active loop of the thread
*/
void selectiveAttentionProcessor::run(){
    cLoop++;
    //synchronisation with the input image occuring
    if(!interrupted){
        
        //--------read value from the preattentive level
        if(feedbackPort.getOutputCount()){
            /*
            Bottle in,out;
            out.clear();
            out.addString("get");
            out.addString("ktd");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            salienceTD=in.pop().asDouble();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("kbu");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            salienceBU=in.pop().asDouble();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("rin");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            targetRED=in.pop().asInt();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("gin");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            targetGREEN=in.pop().asInt();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("bin");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            targetBLUE=in.pop().asDouble();
            out.clear();
            in.clear();
            */
        }

        //

        //-------------read input maps
        //if(map1Port.getInputCount()){
        //    tmp=map1Port.read(false);
        //}
        //if(inImagePort.getInputCount()){
        tmp2=inImagePort.read(false);
        
        if(tmp2==0){
            return;
        }
        
        if(!reinit_flag){
            //srcsize.height=img->height();
            //srcsize.width=img->width();
            reinitialise(tmp2->width(), tmp2->height());
            reinit_flag=true;
            //currentProcessor=new selectiveAttentionProcessor();
            //passes the temporary variable for the mode
            //currentProcessor->resizeImages(tmp2->width(),tmp2->height());
            //startselectiveAttentionProcessor();
            //currentProcessor->setIdle(false);
        }
        
        //currentProcessor->inImage=tmp2;
        
        if(map1Port.getInputCount()) {
            tmp=map1Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map1_yarp->getRawImage(),map1_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
            
        }
        if(map2Port.getInputCount()) {
            tmp=map2Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map2_yarp->getRawImage(),map2_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(map3Port.getInputCount()) {
            tmp=map3Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map3_yarp->getRawImage(),map3_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(map4Port.getInputCount()) {
            tmp=map4Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map4_yarp->getRawImage(),map4_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        
        if(map5Port.getInputCount()) {
            tmp=map5Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map5_yarp->getRawImage(),map5_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(map6Port.getInputCount()) {
            tmp=map6Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map6_yarp->getRawImage(),map6_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }

        //2. processing of the input images
        unsigned char* pmap1= map1_yarp->getRawImage();
        unsigned char* pmap2= map2_yarp->getRawImage();
        unsigned char* pmap3= map3_yarp->getRawImage();
        unsigned char* pmap4= map4_yarp->getRawImage();
        unsigned char* pmap5= map5_yarp->getRawImage();
        unsigned char* pmap6= map6_yarp->getRawImage();
        unsigned char* plinear=linearCombinationImage->getRawImage();
        int padding=map1_yarp->getPadding();
        int rowSize=map1_yarp->getRowSize();
        unsigned char maxValue=0;
        double sumK=k1+k2+k3+k4+k5+k6;
        if(!idle){
            for(int y=0;y<height;y++){
                for(int x=0;x<width;x++){
                    unsigned char value=0;
                    if(map1_yarp!=0)
                        value+=(unsigned char)ceil((double)(*pmap1 * (k1/sumK)));
                    if(map2_yarp!=0)
                        value+=(unsigned char)ceil((double)(*pmap2 * (k2/sumK)));
                    if(map3_yarp!=0)
                        value+=(unsigned char)ceil((double)(*pmap3 * (k3/sumK)));
                    if(map4_yarp!=0)
                        value+=(unsigned char)ceil((double)(*pmap4 * (k4/sumK)));
                    if(map5_yarp!=0)
                        value+=(unsigned char)ceil((double)(*pmap5 * (k5/sumK)));
                    if(map6_yarp!=0)
                        value+=(unsigned char)ceil((double)(*pmap6 * (k6/sumK)));
                    pmap1++;pmap2++;pmap3++;
                    pmap4++;pmap5++;pmap6++;
                    if((map1_yarp==0)&&(map2_yarp==0)&&(map3_yarp==0)&&(map4_yarp==0)&&(map5_yarp==0)&&(map6_yarp==0))
                        value=0;
                    if(maxValue<value)
                        maxValue=value;
                    *plinear=value;
                    plinear++;
                }
                pmap1+=padding;
                pmap2+=padding;
                pmap3+=padding;
                pmap4+=padding;
                pmap5+=padding;
                pmap6+=padding;
                plinear+=padding;
            }

            if(maxValue==0) {
                outputImage->zero();
            }
            else {
                unsigned char* pout=outputImage->getRawImage();
                unsigned char* plinear=linearCombinationImage->getRawImage();
                xm=0, ym=0;
                int count=0;
                float d=0;
                for(int y=0;y<height;y++) {
                    for(int x=0;x<width;x++) {
                        if(*plinear==maxValue){
                            *pout=255;
                            count++;
                            xm+=x;
                            ym+=y;
                        }
                        else {
                            *pout=0;
                        }
                        plinear++;
                        pout++;
                    }
                    pout+=padding;
                    plinear+=padding;
                }
                xm=xm/count;
                ym=ym/count;
            }
            //specify the pixel where to look
            Vector px(2);
            //convert the logpolar coordinates to cartesian coordinates
                
            double xx = 0;
            double yy = 0;
            //double _xsize=2;
            //double _ysize=2;

            /**
            * pointers to angleShift
            */
            double *_angShiftMap;
            _angShiftMap = (double *) malloc (252 * sizeof(double));
            for(int i=0;i<252;i++){
                _angShiftMap[i]=0.0;
            }
            /**
            * feature of the input image
            */
            int _xsize=320;
            int _ysize=240;
            Image_Data _img;
            _img = Set_Param(
            _xsize, _ysize,
            240,240, //256,256
            252, 152, 64,
            1090,
            CUST,
            252.0/1090.0);

            Get_XY_Center(&xx, &yy, xm, ym, &_img, _angShiftMap);

            using namespace _logpolarParams;
            px[0] = int(xx + .5) + 240/2;
            px[1] = 240/2 - int(yy + .5);

            printf("******************************** \n");
            printf("cartesian:%f,%f \n",px[0],px[1]);
            if(gazePerform){
                
                if(cLoop>=100) {
                    
                    //we still have one degree of freedom given by
                    //the distance of the object from the image plane
                    //if you do not have it, try to guess :)
                    double z=1.0;   // distance [m]
                    igaze->lookAtMonoPixel(camSel,px,z); 
                    cLoop=0;
                }
            }
            printf("logpolar:%f,%f \n",xm,ym);
            outPorts();
        }
    }
}



void selectiveAttentionProcessor::setCamSelection(int value) {
    camSel=value;
}


bool selectiveAttentionProcessor::outPorts(){
    bool ret = false;
    if((0!=linearCombinationImage)&&(linearCombinationPort.getOutputCount())){
        linearCombinationPort.prepare() = *(linearCombinationImage);
        linearCombinationPort.write();
    }
    
    if((0!=outputImage)&&(selectedAttentionPort.getOutputCount())){
        selectedAttentionPort.prepare() = *(outputImage);
        selectedAttentionPort.write();
    }

    if(centroidPort.getOutputCount()){  
        Bottle& commandBottle=centroidPort.prepare();
        commandBottle.clear();
        commandBottle.addString("sac");
        commandBottle.addString("img");
        //commandBottle.addInt(centroid_x);
        //commandBottle.addInt(centroid_y);
        centroidPort.write();
    }

    if(feedbackPort.getOutputCount()){  
        //Bottle& commandBottle=feedbackPort.prepare();
        Bottle in,commandBottle;
        commandBottle.clear();
        
        
        time (&end2);
        double dif = difftime (end2,start2);
        if(dif>30+2){
                //restart the time interval
                 time(&start2);
        }
        else if((dif>2)&&(dif<30+2)){
            //setting coefficients
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','t','d'));
            salienceTD=salienceTD+0.1;
        
            //if(salienceTD>0.99)
                salienceTD=1.0;
            printf("salienceTD \n");
            commandBottle.addDouble((double)salienceTD);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','b','u'));
            salienceBU=salienceBU-0.1;
            
            //if(salienceBU<=0)
                salienceBU=0;
            commandBottle.addDouble((double)salienceBU);
            feedbackPort.write(commandBottle,in);    
            printf("read: %f,%f,%f \n",(double)targetRED,(double)targetGREEN,(double)targetBLUE);
            
        }
        else{
            printf("salienceBU \n");
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','t','d'));
            salienceTD=0.0;
            commandBottle.addDouble((double)salienceTD);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','b','u'));
            salienceBU=1.0;
            commandBottle.addDouble((double)salienceBU);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('r','i','n'));
            commandBottle.addDouble((double)targetRed);
            //commandBottle.addDouble(255.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('g','i','n'));
            commandBottle.addDouble((double)targetGreen);
            //commandBottle.addDouble(0.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('b','i','n'));
            commandBottle.addDouble((double)targetBlue);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            //commandBottle.addDouble(0.0);
            printf("%f,%f,%f \n",(double)targetRed,(double)targetGreen,(double)targetBlue);
        }
    }
    return true;
}


void selectiveAttentionProcessor::extractContour(ImageOf<PixelMono>* inputImage,ImageOf<PixelRgb>* inputColourImage,int& x,int& y){
    
    CvMemStorage* stor=cvCreateMemStorage(0);
    CvBox2D box;
    CvSeq* cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
    cvFindContours(inputImage->getIplImage(), stor, &cont, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    IplImage* dst = cvCreateImage( cvGetSize(outputImage->getIplImage()), 8, 3 );
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

        /*
        //unsigned char targetRed=0, targetGreen=0, targetBlue=0;
        unsigned char tmpRed=0, tmpGreen=0, tmpBlue=0;
        getPixelColour(inImage, x,y,targetRed,targetGreen,targetBlue);
        getPixelColour(inImage, x+1,y,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        getPixelColour(inImage, x-1,y,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        getPixelColour(inImage, x,y+1,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        getPixelColour(inImage, x,y-1,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        */
        
           
        cvDrawContours(dst,cont,CV_RGB(targetBlue,targetGreen,targetRed),CV_RGB(0,0,0),0,1,8);
        cvCircle (dst, center, 1, CV_RGB(targetBlue,targetGreen,targetRed));
           
        cvFitLine(cont, CV_DIST_L2, 1, 0.01, 0.01, line);
        float t = (v + v1)/2;
        pt1.x = cvRound(line[2] - line[0] *t );
        pt1.y = cvRound(line[3] - line[1] *t );
        pt2.x = cvRound(line[2] + line[0] *t );
        pt2.y = cvRound(line[3] + line[1] *t );

        //cvCircle(dst, pt1, 1, CV_RGB(targetRed,targetGreen,targetBlue));
        //cvCircle(dst, pt2, 1, CV_RGB(targetBlue,targetGreen,targetRed));
       
        /*
        //cvLine(dst, pt1, pt2, CV_RGB(targetRed,targetGreen,targetBlue), 3, CV_AA, 0);
        double theta = 0;
        // double theta = 180 / M_PI * atan2( (pt2.y - pt1.y) , (pt2.x - pt1.x) );
        CvFont font;
        double hScale=0.3;
        double vScale=0.3;
        int    lineWidth=1;

        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
        char* Angles=new char;

        sprintf(Angles, "%d,%d,%d",(int)targetRed,(int)targetGreen,(int)targetBlue);
        cvPutText ( dst , Angles, cvPoint( 10, 10), &font, cvScalar(255,0,0) );
        */

    }
    //cvCopy(dst,outputImage->getIplImage());
    //ippiCopy_8u_C1R((const Ipp8u *)dst->imageData,dst->widthStep, outputImage->getRawImage(), outputImage->getRowSize(),srcsize);
    cvReleaseMemStorage(&stor);
    cvReleaseImage(&dst);
    //cvReleaseMemStorage(&storage);
}


void selectiveAttentionProcessor::getPixelColour(ImageOf<PixelRgb>* inputColourImage,int x ,int y, unsigned char &targetRed, unsigned char &targetGreen, unsigned char &targetBlue){
    //printf("max image dim:%d with rowsize %d \n",inImage->getRawImageSize(),inImage->getRowSize());
    unsigned char pColour=inImage->getRawImage()[(x*3)+y*inImage->getRowSize()];
    targetRed=pColour;
    //pColour++;
    pColour=inImage->getRawImage()[(x*3)+1+y*inImage->getRowSize()];
    targetGreen=pColour;
    //pColour++;
    pColour=inImage->getRawImage()[(x*3)+2+y*inImage->getRowSize()];
    targetBlue=pColour;
    //printf("colour found: %d %d %d \n",(int) targetBlue,(int)targetGreen,(int)targetRed);
}

/**
* function called when the module is poked with an interrupt command
*/
void selectiveAttentionProcessor::interrupt(){
    interrupted=true;
    printf("interrupting the module.. \n");
    map1Port.interrupt();
    map2Port.interrupt();
    map3Port.interrupt();
    
    map4Port.interrupt();
    map5Port.interrupt();
    map6Port.interrupt();
    
    selectedAttentionPort.interrupt();
    linearCombinationPort.interrupt();
    centroidPort.interrupt();
    feedbackPort.interrupt();
    
    inImagePort.interrupt();
    
}

/**
*	releases the thread
*/
void selectiveAttentionProcessor::threadRelease(){
    printf("Thread realeasing .... \n");
    printf("Closing all the ports.. \n");
    //closing input ports
    inImagePort.close();
    map1Port.close();
    map2Port.close();
    map3Port.close();
    map4Port.close();
    map5Port.close();
    map6Port.close();

    selectedAttentionPort.close();
    linearCombinationPort.close();
    centroidPort.close();
    feedbackPort.close();
    
    clientGazeCtrl->close();
}

void selectiveAttentionProcessor::setIdle(bool value){
    mutex.wait();
    idle=value;
    mutex.post();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

