#include <iCub/graphicThread.h>
#include <iCub/BMLInterface.h>

#include <ace/OS.h>
#include <iostream>
using namespace std;



#define BLOB_MAXSIZE 4096
#define BLOB_MINSIZE 600



static GtkWidget* saveSingleDialog;
static GtkWidget* saveSetDialog;
static GtkWidget* loadDialog;
static GtkWidget *menubar;
static GtkWidget *fileMenu, *imageMenu, *helpMenu;
static GtkWidget *fileItem, *imageItem, *helpItem;
static GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
static GtkWidget *mainWindow = NULL;
//status bar of the windows
static GtkWidget *statusbar;
//drawing area
static GtkWidget *da;
// Current frame
static GdkPixbuf *frame = NULL;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;



// Image to Display
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;



static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_middleImg;
static yarp::sig::ImageOf<yarp::sig::PixelInt> *ptr_tagged;
static yarp::sig::ImageOf<yarp::sig::PixelMono>* _outputImage;
static yarp::sig::ImageOf<yarp::sig::PixelRgb>* _outputImage3;

static ImageOf<PixelMonoSigned> rgs;
static ImageOf<PixelMonoSigned> grs;
static ImageOf<PixelMonoSigned> bys;
static ImageOf<PixelMono> r2;
static ImageOf<PixelMono> g2;
static ImageOf<PixelMono> b2;


// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
// Timeout ID
static guint timeout_ID;

static BMLInterface *wModule;


#define _inputImg (*(ptr_inputImg))

#define _imgRecv (*(ptr_imgRecv))
static YARPImgRecv *ptr_imgRecvLayer0;
static YARPImgRecv *ptr_imgRecvLayer1;
static YARPImgRecv *ptr_imgRecvLayer2;
static YARPImgRecv *ptr_imgRecvLayer3;
static YARPImgRecv *ptr_imgRecvLayer4;
static YARPImgRecv *ptr_imgRecvLayer5;
static YARPImgRecv *ptr_imgRecvLayer6;
static YARPImgRecv *ptr_imgRecvLayer7;
static YARPImgRecv *ptr_imgRecvLayer8;
#define _imgRecvLayer0 (*(ptr_imgRecvLayer0))
#define _imgRecvLayer1 (*(ptr_imgRecvLayer1))
#define _imgRecvLayer2 (*(ptr_imgRecvLayer2))
#define _imgRecvLayer3 (*(ptr_imgRecvLayer3))
#define _imgRecvLayer4 (*(ptr_imgRecvLayer4))
#define _imgRecvLayer5 (*(ptr_imgRecvLayer5))
#define _imgRecvLayer6 (*(ptr_imgRecvLayer6))
#define _imgRecvLayer7 (*(ptr_imgRecvLayer7))
#define _imgRecvLayer8 (*(ptr_imgRecvLayer8))


static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer0;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer1;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer2;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer3;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer4;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer5;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer6;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer7;
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImgLayer8;
#define _inputImgLayer0 (*(ptr_inputImgLayer0))
#define _inputImgLayer1 (*(ptr_inputImgLayer1))
#define _inputImgLayer2 (*(ptr_inputImgLayer2))
#define _inputImgLayer3 (*(ptr_inputImgLayer3))
#define _inputImgLayer4 (*(ptr_inputImgLayer4))
#define _inputImgLayer5 (*(ptr_inputImgLayer5))
#define _inputImgLayer6 (*(ptr_inputImgLayer6))
#define _inputImgLayer7 (*(ptr_inputImgLayer7))
#define _inputImgLayer8 (*(ptr_inputImgLayer8))


#define _middleImg (*(ptr_middleImg))
#define _tagged (*(ptr_tagged))
#define _semaphore (*(ptr_semaphore))

#define THREADRATE 30

/**
* default constructor
*/
graphicThread::graphicThread():RateThread(THREADRATE){
    inLayer0_flag=true;
    inLayer1_flag=false;
    inLayer2_flag=false;
    inLayer3_flag=false;
    inLayer4_flag=false;
    inLayer5_flag=false;
    inLayer6_flag=false;
    inLayer7_flag=false;
    inLayer8_flag=false;
    SelectLayer0_flag=true;
    SelectLayer1_flag=false;
    SelectLayer2_flag=false;
    SelectLayer3_flag=false;
    SelectLayer4_flag=false;
    SelectLayer5_flag=false;
    SelectLayer6_flag=false;
    SelectLayer7_flag=false;
    SelectLayer8_flag=false;
    //----
    
    //outContrastLP=new ImageOf<PixelMono>;
    //outContrastLP->resize(320,240);
    //outMeanColourLP=new ImageOf<PixelBgr>;
    //outMeanColourLP->resize(320,240);
    

    //max_boxes = new YARPBox[3];
    //initializing the image plotted out int the drawing area
    /*image_out=new ImageOf<PixelRgb>;
    image_out->resize(320,240);*/
    _outputImage3=new ImageOf<PixelRgb>;
    _outputImage3->resize(320,240);
    _outputImage=new ImageOf<PixelMono>;
    _outputImage->resize(320,240);

    ptr_inputImage=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer0
    ptr_inputImage->resize(320,240);
    ptr_inputLayer0=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer0
    ptr_inputLayer0->resize(320,240);
    ptr_inputLayer1=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer1
    ptr_inputLayer1->resize(320,240);
    ptr_inputLayer2=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer2
    ptr_inputLayer2->resize(320,240);
    ptr_inputLayer3=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer3
    ptr_inputLayer3->resize(320,240);
    ptr_inputLayer4=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer4
    ptr_inputLayer4->resize(320,240);
    ptr_inputLayer5=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer5
    ptr_inputLayer5->resize(320,240);
    ptr_inputLayer6=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer6
    ptr_inputLayer6->resize(320,240);
    ptr_inputLayer7=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer7
    ptr_inputLayer7->resize(320,240);
    ptr_inputLayer8=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer8
    ptr_inputLayer8->resize(320,240);
    /*_inputImgRGS=new ImageOf<PixelMonoSigned>;
    _inputImgGRS=new ImageOf<PixelMonoSigned>;
    _inputImgBYS=new ImageOf<PixelMonoSigned>;
    _inputImgRGS->resize(320,240);
    _inputImgGRS->resize(320,240);
    _inputImgBYS->resize(320,240);*/
    /*blobFov=new ImageOf<PixelMono>;
    blobFov->resize(320,240);*/

    this->colDim=10;
    this->rowDim=10;

    //inputImageReady_flag=false;
}

/**
* destructor
*/
graphicThread::~graphicThread(){
}






/**
* active loop of the thread
*/
void graphicThread::run(){
     // Shows all widgets in main Window
    //gtk_widget_show_all (mainWindow);
    //gtk_window_move(GTK_WINDOW(mainWindow), 10,10);
    // All GTK applications must have a gtk_main(). Control ends here
    // and waits for an event to occur (like a key press or
    // mouse event).

    gtk_main ();
    wModule->close();
    gtk_widget_destroy(mainWindow);
    this->close();
    yarp::os::Network::fini();
    
    ACE_OS::exit(0);
}
/**
*	releases the thread
*/
void graphicThread::threadRelease(){
    //delete outputImage3; //new ImageOf<PixelRgb>;
    delete _outputImage; // =new ImageOf<PixelMono>;

    delete ptr_inputImage;// =new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer0
    delete ptr_inputLayer0;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer0
    delete ptr_inputLayer1;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer1
    delete ptr_inputLayer2;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer2
    delete ptr_inputLayer3;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer3
    delete ptr_inputLayer4;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer4
    delete ptr_inputLayer5;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer5
    delete ptr_inputLayer6;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer6
    delete ptr_inputLayer7;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer7
    delete ptr_inputLayer8;//=new ImageOf<yarp::sig::PixelRgb>; //pointer to the input image of Layer8
}

void graphicThread::close(){
    closePorts();
}

void graphicThread::setModule(void *){

}

bool graphicThread::openPorts(){
    
    bool ret = false;
    //int res = 0;
    // Registering Port(s)
    //reduce verbosity --paulfitz
    printf("Registering ports %s on network %s...\n",(char *) wModule->getName().c_str(),"default");
    ret = _imgRecv.Connect((char *)wModule->getName("/image:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    //--------
    ret = _imgRecvLayer0.Connect((char *)wModule->getName("/layer0:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer1.Connect((char *)wModule->getName("/layer1:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer2.Connect((char *)wModule->getName("/layer2:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    //--------
    ret = _imgRecvLayer3.Connect((char *)wModule->getName("/layer3:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer4.Connect((char *)wModule->getName("/layer4:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer5.Connect((char *)wModule->getName("/layer5:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer6.Connect((char *)wModule->getName("/layer6:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer7.Connect((char *)wModule->getName("/layer7:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    ret = _imgRecvLayer8.Connect((char *)wModule->getName("/layer8:i").c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
        }
    //-------------
    
    
    return true;
    
}

bool graphicThread::closePorts(){
    
    bool ret = false;
    //int res = 0;
    // Registering Port(s)
    //reduce verbosity --paulfitz
    printf("Closing port %s on network %s...\n", "/rea/BMLInterface/in","default");
    _imgRecv.Disconnect();//("/rea/BMLInterface/image:i","default");
    //--------
    ret = _imgRecvLayer0.Disconnect();//("/rea/BMLInterface/inLayer0","default");
    ret = _imgRecvLayer1.Disconnect();//("/rea/BMLInterface/inLayer1","default");
    ret = _imgRecvLayer2.Disconnect();//("/rea/BMLInterface/inLayer2","default");
    //--------
    ret = _imgRecvLayer3.Disconnect();//("/rea/BMLInterface/inLayer3","default");
    ret = _imgRecvLayer4.Disconnect();//("/rea/BMLInterface/inLayer4","default");
    ret = _imgRecvLayer5.Disconnect();//("/rea/BMLInterface/inLayer5","default");
    ret = _imgRecvLayer6.Disconnect();//("/rea/BMLInterface/inLayer6","default");
    ret = _imgRecvLayer7.Disconnect();//("/rea/BMLInterface/inLayer7","default");
    ret = _imgRecvLayer8.Disconnect();//("/rea/BMLInterface/inLayer8","default");
    //-------------
    if (false)
        {		
            //_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
            printf("Closing port %s on network %s...\n", "/rea/BMLInterface/out","dafult");
            _pOutPort->close();//open("/rea/BMLInterface/out");
            //_pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;
            printf("Closing port %s on network %s...\n", "/rea/BMLInterface/out","dafult");
            _pOutPort2->close();//open("/rea/BMLInterface/outBlobs");
        }
        
    return true;
}


bool graphicThread::getLayers(){
    bool ret = true;
    
    ret = _imgRecvLayer0.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer0.GetLastImage(&_inputImgLayer0);
        ptr_inputImgLayer0=&_inputImgLayer0;
        _semaphore.post();
    }

    ret = _imgRecvLayer1.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer1.GetLastImage(&_inputImgLayer1);
         ptr_inputLayer1=&_inputImgLayer1;
        _semaphore.post();
    }
    ret = _imgRecvLayer2.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer2.GetLastImage(&_inputImgLayer2);
         ptr_inputLayer2=&_inputImgLayer2;
        _semaphore.post();
    }
    ret = _imgRecvLayer3.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer3.GetLastImage(&_inputImgLayer3);
         ptr_inputLayer3=&_inputImgLayer3;
        _semaphore.post();
    }
    ret = _imgRecvLayer4.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer4.GetLastImage(&_inputImgLayer4);
         ptr_inputLayer4=&_inputImgLayer4;
        _semaphore.post();
    }
    ret = _imgRecvLayer5.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer5.GetLastImage(&_inputImgLayer5);
         ptr_inputLayer5=&_inputImgLayer5;
        _semaphore.post();
    }
    ret = _imgRecvLayer6.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer6.GetLastImage(&_inputImgLayer6);
         ptr_inputLayer6=&_inputImgLayer6;
        _semaphore.post();
    }
    ret = _imgRecvLayer7.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer7.GetLastImage(&_inputImgLayer7);
         ptr_inputLayer7=&_inputImgLayer7;
        _semaphore.post();
    }
    ret = _imgRecvLayer8.Update();
    if (ret != false){
        _semaphore.wait();
        ret = _imgRecvLayer8.GetLastImage(&_inputImgLayer8);
         ptr_inputLayer8=&_inputImgLayer8;
        _semaphore.post();
    }	

    
    
    
    //printf("GetImage: out of the semaphore \n");

    
    ret=true;
    return ret;
}

bool getOpponencies(){
    bool ret = false;
    /*ret = _imgRecvRG.Update();
    ret = _imgRecvGR.Update();
    ret = _imgRecvBY.Update();*/

    if (ret == false){
        return false;
    }

    /*_semaphore.wait();
    ret = _imgRecvGR.GetLastImage(&_inputImgGR);
    wModule->ptr_inputGR=&_inputImgGR;
    _semaphore.post();
    _semaphore.wait();
    ret = _imgRecvRG.GetLastImage(&_inputImgRG);
    wModule->ptr_inputRG=&_inputImgRG;
    _semaphore.post();
    _semaphore.wait();
    ret = _imgRecvBY.GetLastImage(&_inputImgBY);
    wModule->ptr_inputBY=&_inputImgBY;
    _semaphore.post();
    //printf("GetImage: out of the semaphore \n");*/
    ret=true;
    return ret;
}


//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------

/* usual callback function */
static void callback( GtkWidget *widget,gpointer   data ){
    printf ("Hello again - %s was pressed \n", (char *) data);
    
    if(!strcmp((char *)data,"Execute")){
        printf("Execute");
        string _command;
        if(wModule->runFreely_flag)
            _command.assign("ExecuteFreely");
        else if(wModule->runClamped_flag)
            _command.assign("ExecuteClamped");
        else if(wModule->stopEvolution_flag)
            _command.assign("ExecuteStop");
        wModule->command->assign(_command); 
    }
    else if(!strcmp((char *)data,"Learn")){
        printf("Learn");
        string _command("Learn");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"outHeadBehaviour")){
        printf("outHeadBehaviour");
        string _command("outHeadBehaviour");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"outEyesBehaviour")){
        printf("outEyesBehaviour");
        string _command("outEyesBehaviour");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"EvolveFreely")){
        printf("EvolveFreely");
        string _command("EvolveFreely");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"EvolveClamped")){
        printf("EvolveClamped");
        string _command("EvolveClamped");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"Stop")){
        printf("Stop");
        string _command("Stop");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"Load")){
        printf("Load");
        string _command("Load");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"AddLayer")){
        printf("AddLayer request \n");
        string _command("AddLayer");
        yarp::os::Bottle tmp;
        tmp.addString("row");
        tmp.addInt(wModule->rowDim);
        wModule->bOptions.addList()=tmp;
        tmp.clear();
        tmp.addString("col");
        tmp.addInt(wModule->colDim);
        wModule->bOptions.addList()=tmp;
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"ConnectLayer")){
        printf("ConnectLayer request \n");
        string _command("ConnectLayer");
        string Aname("");
        string Bname("");
        /*if(wModule->inLayer0_flag){
            printf("LayerA: layer0 \n");
            Aname.append("layer0");
        }
        else if(wModule->inLayer1_flag){
            printf("LayerA: layer1 \n");
            Aname.append("layer1");
        }
        if(wModule->SelectLayer0_flag){
            printf("LayerB: layer0 \n");
            Bname.append("layer0");
        }
        else if(wModule->SelectLayer1_flag){
            printf("LayerB: layer1 \n");
            Bname.append("layer1");
        }*/
        Bottle tmp;
        tmp.addString("LayerA");
        tmp.addString(Aname.c_str());
        wModule->bOptions.addList()=tmp;
        tmp.clear();
        tmp.addString("LayerB");
        tmp.addString(Bname.c_str());
        wModule->bOptions.addList()=tmp;
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"ClampPattern")){
        printf("ClampPattern \n");
        string _command("ClampPattern");
        Bottle tmp;
        tmp.addString("pattern");
        tmp.addString("1,0,0,0");
        wModule->bOptions.addList()=tmp;
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"ClampLayer")){
        printf("ClampLayer \n");
        string _command("ClampLayer");
        Bottle tmp;
        tmp.addString("layer");
        /*if(wModule->inLayer0_flag)
            tmp.addString("layer0");
        else if(wModule->inLayer1_flag)
            tmp.addString("layer1");*/
        wModule->bOptions.addList()=tmp;
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"setProbabilityFreely")){
        printf("setProbabilityFreely \n");
        string _command("setProbabilityFreely");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"setProbabilityNull")){
        printf("setProbabilityNull \n");
        string _command("setProbabilityNull");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"setProbabilityClamped")){
        printf("setProbabilityClamped \n");
        string _command("setProbabilityClamped");
        wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"drawFoveaBlob3")){
        printf("drawFoveaBlob3");
        
    }
    else if(!strcmp((char *)data,"drawVQColor1")){
        printf("drawColorVQ1 function");
    }
    else if(!strcmp((char *)data,"drawVQColor2")){
        printf("drawFoveaBlob2");
        
    }
    else if(!strcmp((char *)data,"drawVQColor3")){
        printf("drawFoveaBlob3");
        
    }
    else if(!strcmp((char *)data,"maxSalienceBlob1")){
        printf("drawColorVQ1 function");
    }
    else if(!strcmp((char *)data,"maxSalienceBlob2")){
        printf("drawFoveaBlob2");
    }
    else if(!strcmp((char *)data,"maxSalienceBlob3")){
        printf("drawFoveaBlob3");
    }
    
}

static gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
    //printf("entering expose_CB \n");
    if(frame){
        //printf("frame not null");
        if ( mainWindow){
                //printf("frame and mainWindow present \n");
                guchar *pixels;
                unsigned int rowstride;
                unsigned int imageWidth,imageHeight,areaWidth, areaHeight;
                //IppiSize srcsize={320,240};

                bool ret=wModule->gui->getLayers();
                if((ret==false)&&(!wModule->inputImageReady_flag)){
                    printf("No Layers and NO Image! \n");
                    return true;
                }
            
                //=new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
                //_outputImage->resize(320,240);
                bool conversion=true;
                if(wModule->gui->inputImage_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputImage->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputImage->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer0_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer0->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer0->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer1_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer1->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer1->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer2_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer2->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer2->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer3_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer3->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer3->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer4_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer4->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer4->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer5_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer5->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer5->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer6_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer6->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer6->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer7_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer7->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer7->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                else if(wModule->gui->inLayer8_flag){
                    //ippiCopy_8u_C3R(wModule->ptr_inputLayer8->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(wModule->gui->ptr_inputLayer8->getIplImage(),_outputImage3->getIplImage());
                    conversion=false;
                }
                if(false){
                    //ippiCopy_8u_C3R(wModule->salience->colorVQ_img->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);
                    conversion=false;
                }
                else if(false){
                    if(false){
                        //ippiCopy_8u_C1R(wModule->outContrastLP->getPixelAddress(0,0),320,_outputImage->getPixelAddress(0,0),320,srcsize);
                        conversion=true;
                    }
                    else if(false){
                        //ippiCopy_8u_C3R(wModule->outMeanColourLP->getPixelAddress(0,0),320*3,_outputImage3->getPixelAddress(0,0),320*3,srcsize);	
                        conversion=false;
                    }
                    else _outputImage->zero(); //the input is a RGB image, whereas the watershed is working with a mono image
                }
                else _outputImage->zero(); //the input is a RGB image, whereas the watershed is working with a mono image

                //-------
                
                if(conversion){
                    //int psb,width=320,height=240;
                    //Ipp8u* im_out = ippiMalloc_8u_C1(width,height,&psb);
                    //Ipp8u* im_tmp[3];
                    //two copies in order to have 2 conversions
                    //the first transform the yarp mono into a 4-channel image
                    //ippiCopy_8u_C1R(_outputImage->getPixelAddress(0,0), width,im_out,psb,srcsize);
                    cvCvtColor(_outputImage->getIplImage(),wModule->image_out->getIplImage(),CV_GRAY2RGB);
                    //im_tmp[0]=im_out;
                    //im_tmp[1]=im_out;
                    //im_tmp[2]=im_out;
                    //the second transforms the 4-channel image into colorImage for yarp
                    //ippiCopy_8u_P3C3R(im_tmp,psb,wModule->image_out->getPixelAddress(0,0),width*3,srcsize);
                    //ippiFree(im_out);
                    //ippiFree(im_tmp); //throws exception for heap corruption if done for every position of the vector
                    //ippiFree(im_tmp[1]);
                    //ippiFree(im_tmp[2]);
                }
                else{
                    //ippiCopy_8u_C3R(_outputImage3->getPixelAddress(0,0),320*3,wModule->image_out->getPixelAddress(0,0),320*3,srcsize);
                    cvCopyImage(_outputImage3->getIplImage(),wModule->image_out->getIplImage());
                }
                
                //----------
                _semaphore.wait();
                bool result=yarpImage2Pixbuf(wModule->image_out, frame);
                //bool result=yarpImage2Pixbuf(&_inputImg, frame);
                imageWidth = wModule->image_out->width();
                imageHeight = wModule->image_out->height();
                _semaphore.post();
                
                
                if (imageWidth==0||imageHeight==0) {
                    printf("exit for dimension nil \n");
                    return TRUE;
                }
     
                areaWidth = event->area.width;
                areaHeight = event->area.height;

                unsigned int pixbufWidth=gdk_pixbuf_get_width(frame);
                unsigned int pixbufHeight=gdk_pixbuf_get_height(frame);

                if ((imageWidth!=pixbufWidth) || (imageHeight!=pixbufHeight))
                    {
                        g_object_unref(frame);
                        //printf("unreferencing frame \n");
                        frame=gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, imageWidth, imageHeight);
                    }

                if ( (areaWidth != imageWidth) || (areaHeight != imageHeight) )
                    {
                        GdkPixbuf *scaledFrame;
                        //printf("scaling image... \n");
                        scaledFrame = gdk_pixbuf_scale_simple(	frame,
                                                                areaWidth,
                                                                areaHeight,
                                                                GDK_INTERP_BILINEAR); // Best quality
                        //GDK_INTERP_NEAREST); // Best speed

                        pixels = gdk_pixbuf_get_pixels (scaledFrame);
                        rowstride = gdk_pixbuf_get_rowstride(scaledFrame);
                        gdk_draw_rgb_image (widget->window,
                                            widget->style->black_gc,
                                            event->area.x, event->area.y,
                                            event->area.width, event->area.height,
                                            GDK_RGB_DITHER_NORMAL,
                                            pixels,
                                            rowstride);
                        g_object_unref(scaledFrame);
                
                    }
                else
                    {
                        pixels = gdk_pixbuf_get_pixels (frame);
                        rowstride = gdk_pixbuf_get_rowstride(frame);
                        //printf("drawing image... \n");
                        gdk_draw_rgb_image (widget->window,
                                            widget->style->black_gc,
                                            event->area.x, event->area.y,
                                            event->area.width, event->area.height,
                                            GDK_RGB_DITHER_NORMAL,
                                            pixels,
                                            rowstride);
                    }
        }
        else{
            //printf("mainWindow results nil");
        }
    
    }
    return TRUE;
}

static void cb_draw_value( GtkToggleButton *button )
{
    /* Turn the value display on the scale widgets off or on depending
     *  on the state of the checkbutton */
    printf("callbacks from draw value %s \n",button->button.label_text);
    string _command;
    if(!strcmp(button->button.label_text,"inputImage")){
        printf("inputImage request \n");
        /*if(button->active){
            wModule->inputImage_flag=true;
        }
        else
            wModule->inputImage_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"SelectLayer0-->")){
        printf("Select Layer0 \n");
        /*if(button->active){
            wModule->SelectLayer0_flag=true;
            wModule->SelectLayer1_flag=false;
            wModule->SelectLayer2_flag=false;
            wModule->SelectLayer3_flag=false;
            wModule->SelectLayer4_flag=false;
            wModule->SelectLayer5_flag=false;
            wModule->SelectLayer6_flag=false;
            wModule->SelectLayer7_flag=false;
            wModule->SelectLayer8_flag=false;
        }
        else
            wModule->SelectLayer0_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"SelectLayer1-->")){
        printf("Select B: Layer1 \n");
        /*if(button->active){
            wModule->SelectLayer0_flag=false;
            wModule->SelectLayer1_flag=true;
            wModule->SelectLayer2_flag=false;
            wModule->SelectLayer3_flag=false;
            wModule->SelectLayer4_flag=false;
            wModule->SelectLayer5_flag=false;
            wModule->SelectLayer6_flag=false;
            wModule->SelectLayer7_flag=false;
            wModule->SelectLayer8_flag=false;
        }
        else
            wModule->SelectLayer1_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"SelectLayer2-->")){
        printf("Select B: Layer2 \n");
        /*if(button->active){
            wModule->SelectLayer0_flag=false;
            wModule->SelectLayer1_flag=false;
            wModule->SelectLayer2_flag=true;
            wModule->SelectLayer3_flag=false;
            wModule->SelectLayer4_flag=false;
            wModule->SelectLayer5_flag=false;
            wModule->SelectLayer6_flag=false;
            wModule->SelectLayer7_flag=false;
            wModule->SelectLayer8_flag=false;
        }
        else
            wModule->SelectLayer2_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"SelectLayer3-->")){
        printf("Select B Layer3 \n");
        /*if(button->active){
            wModule->SelectLayer0_flag=false;
            wModule->SelectLayer1_flag=false;
            wModule->SelectLayer2_flag=false;
            wModule->SelectLayer3_flag=true;
            wModule->SelectLayer4_flag=false;
            wModule->SelectLayer5_flag=false;
            wModule->SelectLayer6_flag=false;
            wModule->SelectLayer7_flag=false;
            wModule->SelectLayer8_flag=false;
        }
        else
            wModule->SelectLayer3_flag=true;*/
    }
    else if(!strcmp(button->button.label_text,"SelectLayer4-->")){
        printf("Select B: Layer4 \n");
        /*if(button->active){
            wModule->SelectLayer0_flag=false;
            wModule->SelectLayer1_flag=false;
            wModule->SelectLayer2_flag=false;
            wModule->SelectLayer3_flag=false;
            wModule->SelectLayer4_flag=true;
            wModule->SelectLayer5_flag=false;
            wModule->SelectLayer6_flag=false;
            wModule->SelectLayer7_flag=false;
            wModule->SelectLayer8_flag=false;
        }
        else
            wModule->SelectLayer4_flag=false;*/

    }
    else if(!strcmp(button->button.label_text,"SelectLayer5-->")){
        printf("Select B: Layer5 \n");
        /*if(button->active){
            wModule->SelectLayer0_flag=false;
            wModule->SelectLayer1_flag=false;
            wModule->SelectLayer2_flag=false;
            wModule->SelectLayer3_flag=false;
            wModule->SelectLayer4_flag=false;
            wModule->SelectLayer5_flag=true;
            wModule->SelectLayer6_flag=false;
            wModule->SelectLayer7_flag=false;
            wModule->SelectLayer8_flag=false;
        }
        else
            wModule->SelectLayer5_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer0")){
        printf("Layer0 request \n");
        if(button->active){
            /*wModule->inLayer0_flag=true;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;*/
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(0);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        /*else
            wModule->inLayer0_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer1")){
        printf("Layer1 request \n");
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=true;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(1);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer1_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer2")){
        printf("Layer2 request \n");
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=true;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(2);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer2_flag=false;*/
    }
    if(!strcmp(button->button.label_text,"Layer3")){
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=true;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(3);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer3_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer4")){
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=true;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(4);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer4_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer5")){
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=true;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(5);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer5_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer6")){
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=true;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(6);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer6_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer7")){
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=true;
            wModule->inLayer8_flag=false;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(7);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer7_flag=false;*/
    }
    else if(!strcmp(button->button.label_text,"Layer8")){
        /*if(button->active){
            wModule->inLayer0_flag=false;
            wModule->inLayer1_flag=false;
            wModule->inLayer2_flag=false;
            wModule->inLayer3_flag=false;
            wModule->inLayer4_flag=false;
            wModule->inLayer5_flag=false;
            wModule->inLayer6_flag=false;
            wModule->inLayer7_flag=false;
            wModule->inLayer8_flag=true;
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(8);
            wModule->bOptions.addList()=tmp;
            tmp.clear();
            //tmp.addString("col");
            //tmp.addInt(wModule->colDim);
            //wModule->bOptions.addList()=tmp;
            wModule->command->assign(_command);
        }
        else
            wModule->inLayer8_flag=false;*/
    }
    if(!strcmp(button->button.label_text,"EvolveFreely-->")){
        if(button->active)
            wModule->runFreely_flag=true;
        else
            wModule->runFreely_flag=false;
    }
    else if(!strcmp(button->button.label_text,"EvolveClamped-->")){
        if(button->active)
            wModule->runClamped_flag=true;
        else
            wModule->runClamped_flag=false;
    }
    else if(!strcmp(button->button.label_text,"StopEvolution")){
        if(button->active)
            wModule->stopEvolution_flag=true;
        else
            wModule->stopEvolution_flag=false;
    }
    else if(!strcmp(button->button.label_text,"Blue2-->")){
        if(button->active){
            //imageProcessModule->processor2->redPlane_flag=0;
            //imageProcessModule->processor2->greenPlane_flag=0;
            //imageProcessModule->processor2->bluePlane_flag=1;
        }
    }
    if(!strcmp(button->button.label_text,"Red3-->")){
        if(button->active){
            //imageProcessModule->processor3->redPlane_flag=1;
            //imageProcessModule->processor3->greenPlane_flag=0;
            //imageProcessModule->processor3->bluePlane_flag=0;
        }
    }
    else if(!strcmp(button->button.label_text,"Green3-->")){
        if(button->active){
            //imageProcessModule->processor3->redPlane_flag=0;
            //imageProcessModule->processor3->greenPlane_flag=1;
            //imageProcessModule->processor3->bluePlane_flag=0;
        }

    }
    else if(!strcmp(button->button.label_text,"Blue3-->")){
        if(button->active){
            //imageProcessModule->processor3->redPlane_flag=0;
            //imageProcessModule->processor3->greenPlane_flag=0;
            //imageProcessModule->processor3->bluePlane_flag=1;
        }
    }
}

bool getImage(){
    /*
    bool ret = false;
    ret = _imgRecv.Update();

    if (ret == false){
        return false;
    }*/

    /*_semaphore.wait();
    ret = _imgRecv.GetLastImage(&_inputImg);
    wModule->ptr_inputImage=&_inputImg;
    _semaphore.post();*/
    
    //printf("GetImage: out of the semaphore \n");
    return true;
}

void cleanExit(){
    /*g_source_remove (timeout_ID);
    timeout_ID = 0;
    //closePorts();
    if (_options.saveOnExit != 0)
        saveOptFile(_options.fileName);
    if (frame)
        g_object_unref(frame);*/
    // Exit from application
    gtk_main_quit ();
    //deleteObjects();
}

static gint menuFileQuit_CB(GtkWidget *widget, gpointer data)
{
    cleanExit();
 
    return TRUE;
}

static gint menuLoadFile_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);
            gtk_widget_show_all (loadDialog);
        
        } 
    else 
        {
            gtk_widget_hide (loadDialog);	
        }

    return TRUE;
}

static gint menuFileSet_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
#if GTK_CHECK_VERSION(2,6,0)

    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSingleItem), FALSE);
                    
            gtk_widget_show_all (saveSetDialog);
        } 
    else 
        {
            gtk_widget_hide (saveSetDialog);
        }

#endif

    return TRUE;
}

static gint saveSingleDelete_CB (GtkWidget *widget, gpointer data)
{
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSingleItem), FALSE);

    return (TRUE);
}

static gint saveSetStartClicked_CB(GtkWidget *widget, gpointer data)
{
    //_savingSet = true;
        
    return (TRUE);
}

static gint saveSetDelete_CB (GtkWidget *widget, gpointer data)
{
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);

    return (TRUE);
}

static gint saveSetStopClicked_CB(GtkWidget *widget, gpointer data)
{
    //_savingSet = false;
        
    return (TRUE);
}

/* Get the selected filename and print it to the console */
static void file_ok_sel( GtkWidget        *w,
                         GtkFileSelection *fs ){
    printf ("%s\n", gtk_file_selection_get_filename (GTK_FILE_SELECTION (fs)));
}


static gint saveSingleClicked_CB(GtkWidget *widget, gpointer data)
{
    //saveCurrentFrame();
    
    return (TRUE);
}

static gint timeout_CB (gpointer data){
    /*gtk_widget_queue_draw (da);
    if (getImage()){
            wModule->inputImageReady_flag=true;
            //             int imageWidth, imageHeight, pixbufWidth, pixbufHeight;
            //             _semaphore.wait();
            //            imageWidth = _inputImg.width();
            //            imageHeight = _inputImg.height();
            //            _semaphore.post();
            //            pixbufWidth = gdk_pixbuf_get_width(frame);
            //            pixbufHeight = gdk_pixbuf_get_height(frame);
            //            if ( (imageWidth != pixbufWidth) || (imageHeight != pixbufHeight) )
            //                            {
            //                    g_object_unref(frame);
            //                    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, imageWidth, imageHeight);
            //            }
            //            _frameN++;
            gtk_widget_queue_draw (da);
            //            if (_savingSet)
            //                saveCurrentFrame();
    }

    wModule->outPorts();*/
    return TRUE;
}




//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
static void updateStatusbar (GtkStatusbar  *statusbar)
{
    gchar *msg;
    float fps;
    fps = 1000 / float(50);
 
    gtk_statusbar_pop (statusbar, 0); // clear any previous message, underflow is allowed 
                    
    msg = g_strdup_printf ("%s - %.1f fps","/rea/imageProcess", fps);

    gtk_statusbar_push (statusbar, 0, msg);

    g_free (msg);
}

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog(void)
{

    GtkWidget *dialog = NULL;
    GtkWidget *button;
    GtkWidget *hbox;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Snapshot");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    //gtk_window_resize(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    button = gtk_button_new_from_stock(GTK_STOCK_SAVE);
    gtk_widget_set_size_request (GTK_WIDGET(button), 150,50);
    gtk_box_pack_start (GTK_BOX (hbox), button, TRUE, TRUE, 16); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (saveSingleClicked_CB), NULL);
    gtk_signal_connect (GTK_OBJECT (dialog), "delete_event", GTK_SIGNAL_FUNC (saveSingleDelete_CB), NULL);
    
    //gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    
    return dialog;
}

GtkWidget* createLoadDialog(void)
{

    /* Create a new file selection widget */
    GtkWidget* filew = gtk_file_selection_new ("File selection");
    
    g_signal_connect (G_OBJECT (filew), "destroy",
                  G_CALLBACK (gtk_main_quit), NULL);
    /* Connect the ok_button to file_ok_sel function */
    g_signal_connect (G_OBJECT (GTK_FILE_SELECTION (filew)->ok_button),
              "clicked", G_CALLBACK (file_ok_sel), (gpointer) filew);
    
    /* Connect the cancel_button to destroy the widget */
    g_signal_connect_swapped (G_OBJECT (GTK_FILE_SELECTION (filew)->cancel_button),
                          "clicked", G_CALLBACK (gtk_widget_destroy),
                  G_OBJECT (filew));
    
    /* Lets set the filename, as if this were a save dialog, and we are giving
     a default filename */
    gtk_file_selection_set_filename (GTK_FILE_SELECTION(filew), 
                     "penguin.png");

    
    return filew;
}

GtkWidget* createSaveSetDialog(void)
{
    GtkWidget *dialog = NULL;
    GtkWidget *saveButton;
    GtkWidget *stopButton;
    GtkWidget *hbox;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Image Set");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 190, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
#if GTK_CHECK_VERSION(2,6,0)
    saveButton = gtk_button_new_from_stock(GTK_STOCK_MEDIA_RECORD);
    stopButton = gtk_button_new_from_stock(GTK_STOCK_MEDIA_STOP);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif
    gtk_widget_set_size_request (GTK_WIDGET(saveButton), 80,50);
    gtk_widget_set_size_request (GTK_WIDGET(stopButton), 80,50);

    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_box_pack_start (GTK_BOX (hbox), saveButton, TRUE, TRUE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (hbox), stopButton, TRUE, TRUE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_signal_connect (GTK_OBJECT (saveButton), "clicked", GTK_SIGNAL_FUNC (saveSetStartClicked_CB), NULL);
    gtk_signal_connect (GTK_OBJECT (stopButton), "clicked", GTK_SIGNAL_FUNC (saveSetStopClicked_CB), NULL);
    gtk_signal_connect (GTK_OBJECT (dialog), "delete_event", GTK_SIGNAL_FUNC (saveSetDelete_CB), NULL);

    return dialog;
}



//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* graphicThread::createMenubar(void)
{
    GtkWidget *menubar;

    menubar =  gtk_menu_bar_new ();
    GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    imageItem = gtk_menu_item_new_with_label ("Image");
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    fileSingleItem = gtk_check_menu_item_new_with_label ("Load Configuration");
    gtk_menu_append( GTK_MENU(fileMenu), fileSingleItem);
    gtk_signal_connect( GTK_OBJECT(fileSingleItem), "toggled", GTK_SIGNAL_FUNC(menuLoadFile_CB), mainWindow);
    fileSetItem = gtk_check_menu_item_new_with_label ("Load Configuration");
    gtk_menu_append( GTK_MENU(fileMenu), fileSetItem);
//    gtk_signal_connect( GTK_OBJECT(fileSetItem), "toggled", GTK_SIGNAL_FUNC(menuSaveFile_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    
    // Submenu: Image  
    /*imageMenu = gtk_menu_new();
    imageSizeItem = gtk_menu_item_new_with_label ("Original size");
    gtk_menu_append( GTK_MENU(imageMenu), imageSizeItem);
    gtk_signal_connect( GTK_OBJECT(imageSizeItem), "activate", GTK_SIGNAL_FUNC(menuImageSize_CB), mainWindow);
    imageRatioItem = gtk_menu_item_new_with_label ("Original aspect ratio");
    gtk_menu_append( GTK_MENU(imageMenu), imageRatioItem);
    gtk_signal_connect( GTK_OBJECT(imageRatioItem), "activate", GTK_SIGNAL_FUNC(menuImageRatio_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    imageFreezeItem = gtk_check_menu_item_new_with_label ("Freeze");
    gtk_menu_append( GTK_MENU(imageMenu), imageFreezeItem);
    gtk_signal_connect( GTK_OBJECT(imageFreezeItem), "toggled", GTK_SIGNAL_FUNC(menuImageFreeze_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    imageFramerateItem = gtk_menu_item_new_with_label ("Change refresh interval..");
    gtk_menu_append( GTK_MENU(imageMenu), imageFramerateItem);
    gtk_signal_connect( GTK_OBJECT(imageFramerateItem), "activate", GTK_SIGNAL_FUNC(menuImageFramerate_CB), mainWindow);
    imageIntervalItem = gtk_menu_item_new_with_label ("Show Interval..");
    gtk_menu_append( GTK_MENU(imageMenu), imageIntervalItem);
    gtk_signal_connect( GTK_OBJECT(imageIntervalItem), "activate", GTK_SIGNAL_FUNC(menuImageInterval_CB), mainWindow);*/
    // Submenu: Help
    /*helpMenu = gtk_menu_new();	
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);*/

    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(imageItem), imageMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), imageItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
    return menubar;
}

//-------------------------------------------------
// Service Fuctions
//-------------------------------------------------

void graphicThread::createObjects() {
    ptr_imgRecv = new YARPImgRecv;
    ptr_imgRecvLayer0 = new YARPImgRecv;
    ptr_imgRecvLayer1 = new YARPImgRecv;
    ptr_imgRecvLayer2 = new YARPImgRecv;
    ptr_imgRecvLayer3 = new YARPImgRecv;
    ptr_imgRecvLayer4 = new YARPImgRecv;
    ptr_imgRecvLayer5 = new YARPImgRecv;
    ptr_imgRecvLayer6 = new YARPImgRecv;
    ptr_imgRecvLayer7 = new YARPImgRecv;
    ptr_imgRecvLayer8 = new YARPImgRecv;

    ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
    ptr_tagged->resize(320,240);

    ptr_inputImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer0 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer1 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer2 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer3 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer4 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer5 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer6 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer7 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_inputImgLayer8 = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    
    ptr_semaphore = new yarp::os::Semaphore;
}

void graphicThread::setUp()
{
    if (true)
        _imgRecv.SetLogopolar(false);
    else
        _imgRecv.SetLogopolar(true);
    
    if (true)
        _imgRecv.SetFovea(false);
    else
        _imgRecv.SetFovea(true);
    
    if (openPorts() == false)
    {
        printf("ERROR: not opened ports \n");
        //ACE_OS::exit(1);
    }
    
    //_inputImg.resize(320,240);
    _inputImgLayer0.resize(320,240);
    _inputImgLayer1.resize(320,240);
    _inputImgLayer2.resize(320,240);
    _inputImgLayer3.resize(320,240);
    _inputImgLayer4.resize(320,240);
    _inputImgLayer5.resize(320,240);
    _inputImgLayer6.resize(320,240);
    _inputImgLayer7.resize(320,240);
    _inputImgLayer8.resize(320,240);


}

static void scale_set_default_values( GtkScale *scale )
{
    gtk_range_set_update_policy (GTK_RANGE (scale),
                                 GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 0);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}

static void cb_digits_scale2( GtkAdjustment *adj )
{
    wModule->setRowDim((int)adj->value);
    printf("RowDimension: %f",(double) adj->value);
}

static void cb_digits_scale1( GtkAdjustment *adj )
{
    wModule->setColDim((int)adj->value);
    printf("ColumnDimension: %f",(double) adj->value);
}


static GtkWidget *xpm_label_box( gchar     *xpm_filename,
                                 gchar     *label_text )
{
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    /* Create box for image and label */
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    /* Now on to the image stuff */
    if(xpm_filename!=NULL)
        image = gtk_image_new_from_file (xpm_filename);

    /* Create a label for the button */
    label = gtk_label_new (label_text);

    /* Pack the image and label into the box */
    if(xpm_filename!=NULL)
        gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
        gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* graphicThread::createMainWindow(void)
{
    //Module=this; //it is necessary to synchronise the static function with this class
    
    GtkRequisition actualSize;
    GtkWidget* window;
    GtkWidget *label;
    GtkWidget *separator;
    
    //gtk_init (&argc, &argv);
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "Boltmann Machine Graphical Interface");
    gtk_window_set_default_size(GTK_WINDOW (window), 320, 700); 
    gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (gtk_main_quit),
                      NULL);
    
    // When the window is given the "delete_event" signal (this is given
    // by the window manager, usually by the "close" option, or on the
    // titlebar), we ask it to call the delete_event () function
    // as defined above. The data passed to the callback
    // function is NULL and is ignored in the callback function.
    //g_signal_connect (G_OBJECT (window), "delete_event", G_CALLBACK (delete_event), NULL);

    // Box for main window in a ordered list
    GtkWidget *box,*box2,*boxA,*box3,*box4,*box5;
    box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
    // MenuBar for main window
    menubar = createMenubar();
    gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    //gtk_widget_size_request(menubar, &actualSize);
    // Drawing Area : here the image will be drawed
    da = gtk_drawing_area_new ();
    
    g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
    /*if (_options.outputEnabled == 1)
        {
            g_signal_connect (da, "button_press_event", G_CALLBACK (clickDA_CB), NULL);
            // Ask to receive events the drawing area doesn't normally subscribe to
            gtk_widget_set_events (da, gtk_widget_get_events (da) | GDK_BUTTON_PRESS_MASK);
        }*/
    gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);
    //Toolbox area
    //creates the area as collection of port processes sequence
    box2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box2);
    GtkWidget *button,*button2,*buttonCheck;
    GtkWidget *boxButton,*boxButton2;
    GtkWidget *boxButtons;
    GtkWidget *boxSliders;
    boxButtons = gtk_hbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
    boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
     /* Create a new button */
    button = gtk_button_new ();
    button2 = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "output1");
    g_signal_connect (G_OBJECT (button2), "clicked",G_CALLBACK (callback), (gpointer) "output2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "output1");
    boxButton2= xpm_label_box (NULL, "output2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_widget_show (boxButton2);
    gtk_container_add (GTK_CONTAINER (button2), boxButton2);
    gtk_widget_show (button2);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button2);

    label = gtk_label_new ("LayerA Selection: ");
    gtk_box_pack_start (GTK_BOX (boxButtons), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheckGreen = gtk_check_button_new_with_label("inputImage");
    char* valueii="inputImage";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheckGreen), FALSE);
    g_signal_connect (G_OBJECT (buttonCheckGreen), "toggled",G_CALLBACK (cb_draw_value), valueii);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheckGreen, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheckGreen);
    
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheckGreen = gtk_check_button_new_with_label("Layer0");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheckGreen), FALSE);
    char* valueL0="Layer0";
    g_signal_connect (G_OBJECT (buttonCheckGreen), "toggled",G_CALLBACK (cb_draw_value), valueL0);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheckGreen, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheckGreen);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer1");
    char* valueL1="Layer1";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL1);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer2");
    char* valueL2="Layer2";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL2);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer3");
    char* valueL3="Layer3";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL3);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer4");
    char* valueL4="Layer4";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL4);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer5");
    char* valueL5="Layer5";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL5);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer6");
    char* valueL6="Layer6";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL6);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer7");
    char* valueL7="Layer7";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL7);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Layer8");
    char* valueL8="Layer8";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueL8);
    gtk_box_pack_start (GTK_BOX (boxButtons), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    
    gtk_container_add (GTK_CONTAINER (box2), boxButtons);
    
    //---- vSeparator
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);

    
    //-----main section
    GtkWidget *scrollbar;
    
    
    GtkWidget *scale;
    GtkObject *adj1, *adj2;
    GtkWidget *hscale, *vscale;


    adj1 = gtk_adjustment_new (0.0, 0.0, 101.0, 0.1, 1.0, 1.0);
    vscale = gtk_vscale_new (GTK_ADJUSTMENT (adj1));
    scale_set_default_values (GTK_SCALE (vscale));
    gtk_box_pack_start (GTK_BOX (boxSliders), vscale, TRUE, TRUE, 0);
    gtk_widget_show (vscale);

    /*separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    gtk_widget_show (separator);*/

    

    //----------BOXA SECTION:1
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    
    //---- vSeparator
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxA), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section A

    adj1 = gtk_adjustment_new (10.0, 0.0, 100.0, 1.0, 1.0, 1.0);
    adj2 = gtk_adjustment_new (10.0, 0.0, 100.0, 1.0, 1.0, 1.0);
    box3 = gtk_hbox_new (FALSE, 0);
    
    box5 = gtk_vbox_new (FALSE, 0);

    label = gtk_label_new ("Column Layer Dimension:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 100, 100);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj1), "value_changed",
                      G_CALLBACK (cb_digits_scale1), NULL);

    label = gtk_label_new ("Row Layer Dimension:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj2));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 100, 100);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj2), "value_changed",
                      G_CALLBACK (cb_digits_scale2), NULL);

    gtk_box_pack_start (GTK_BOX (box3), box5, TRUE, TRUE, 0);
    gtk_widget_show (box5);

    //_______

    //label = gtk_label_new ("BM Evolution Control:");
    //gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    //gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    // Notice how this causes the scales to always be updated
    // continuously when the scrollbar is moved 
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    box4 = gtk_hbox_new (FALSE, 0);
    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "EvolveFreely");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "EvolveFreely");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "EvolveClamped");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "EvolveClamped");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Stop");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Stop");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    
    box5 = gtk_vbox_new (FALSE, 0);
    
    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "setProbabilityFreely");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "setProbabilityFreely");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "setProbabilityClamped");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "SetProbabilityClamped");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);
    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "setProbabilityNull");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "setProbabilityNull");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_box_pack_start (GTK_BOX (box3), box5, TRUE, TRUE, 0);
    gtk_widget_show (box5);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Learn");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Learn");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section B
    //box3 = gtk_hbox_new (FALSE, 0);
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    /*label = gtk_label_new ("Configuration Manager:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);*/


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
        gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);

    /*
    // A checkbutton to control whether the value is displayed or not
    buttonCheck = gtk_check_button_new_with_label("AddLayer");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "AddLayer");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    
    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("RemoveLayer");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "RemoveLayer");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("ConnectLayer");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "ConnectLayer");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("ShowLayer");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "ShowLayer");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "MeanColoursLP1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("FoveaBlob1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "FoveaBlob1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("ColorVQ1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "ColorVQ1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);*/


    
    
    
    //---box 4
    //-------run button
    
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "outEyesBehaviour");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "outEyesBehaviour");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "outHeadBehaviour");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "outHeadBehaviour");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Learn");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Learn");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Load");
    boxButton = xpm_label_box (NULL, "Load");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    /*button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor1");
    boxButton = xpm_label_box (NULL, "drawVQColor1");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "maxSalienceBlob1");
    boxButton = xpm_label_box (NULL, "maxSalienceBlob1");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);*/

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //----------BOXA SECTION:2
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    
    
    //--box3 section A
    box3 = gtk_hbox_new (FALSE, 0);
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    GtkWidget* box33 = gtk_vbox_new (FALSE, 1);
    label = gtk_label_new ("Considering operation between LayerA and LayerB");
    gtk_box_pack_start (GTK_BOX (box33), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    label = gtk_label_new ("                                              ");
    gtk_box_pack_start (GTK_BOX (box33), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    label = gtk_label_new ("LayerA in LayerA Box;             LayerB--->");
    gtk_box_pack_start (GTK_BOX (box33), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    gtk_box_pack_start (GTK_BOX (box3), box33, TRUE, TRUE, 0);
    gtk_widget_show (box33);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("SelectLayer0-->");
    char* valueSL0="SelectLayer0-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL0);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("SelectLayer1-->");
    char* valueSL1="SelectLayer1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("SelectLayer2-->");
    char* valueSL2="SelectLayer2-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL2);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("SelectLayer3-->");
    char* valueSL3="SelectLayer3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("SelectLayer4-->");
    char* valueSL4="SelectLayer4-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL4);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("SelectLayer5-->");
    char* valueSL5="SelectLayer5-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL5);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);


    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    char* valueN1="Normalize1";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueN1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4


    

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    char* valueCL="ConnectLayer";
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) valueCL);
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ConnectLayer(A,B)");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    char* valueAL="AddLayer";
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) valueAL);
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "AddLayer()");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "ClampLayer");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ClampLayer(A)");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section B
    //box3 = gtk_hbox_new (FALSE, 0);
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    label = gtk_label_new ("Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    
    /*-------------
    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    ---------------*/

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern1-->");
    char* valueP1="Pattern1";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern2-->");
    char* valueP2="Pattern2";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP2);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern3-->");
    char* valueP3="Pattern3";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "ClampPattern");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ClampPattern");	
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4

    /*button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs2");
    boxButton = xpm_label_box (NULL, "DrawAllBlobs2");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob2");
    boxButton = xpm_label_box (NULL, "drawFoveaBlob2");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor2");
    boxButton = xpm_label_box (NULL, "drawVQColor2");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);*/

    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //----------BOXA SECTION:3
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    box3 = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);
    
    //--box3 section A
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    label = gtk_label_new ("Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    char* valueG1="Green1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueG1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    char* valueR1="Red1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),valueR1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    char* valueB1="Blue1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueB1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern1-->");
    char* valueP1b="Pattern1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP1b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern2-->");
    char* valueP2b="Pattern2-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP2b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern3-->");
    char* valueP3b="Pattern3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP3b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "ClampPattern");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ClampPattern");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);
    //---box 4

    

    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    //gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    //gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    //gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    //gtk_widget_show (separator);
    
    //--box3 section B
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    label = gtk_label_new ("CheckList:");
    //gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    //gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);

    /*--------------------
    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    -----*/


    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    buttonCheck = gtk_check_button_new_with_label("ContrastLP3-->");
    char* valueCLP3="ContrastLP3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueCLP3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP3-->");
    char* valueMCLP3="MeanColoursLP3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueMCLP3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    char* valueN1b="Normalize1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueN1b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);
    
    //---box 4

    //-------run button
    /*button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs3");
    boxButton = xpm_label_box (NULL, "DrawAllBlobs3");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob3");
    boxButton = xpm_label_box (NULL, "drawFoveaBlob3");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor3");
    boxButton = xpm_label_box (NULL, "drawVQColor3");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);*/

    //gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    //gtk_widget_show (button);

    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //gtk_container_add (GTK_CONTAINER (box2), boxSliders);
    gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
    // StatusBar for main window
    statusbar = gtk_statusbar_new ();
    updateStatusbar(GTK_STATUSBAR (statusbar));
    gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
    gtk_widget_size_request(statusbar, &actualSize);
    //_occupiedHeight += 2*(actualSize.height);

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
    // TimeOut used to refresh the screen
    timeout_ID = gtk_timeout_add (100, timeout_CB, NULL);

    mainWindow=window;

    return window;
}


/**
*	initialization of the thread 
*/
bool graphicThread::threadInit(){


    // create a new window
    this->createObjects();
    this->setUp();
    mainWindow = this->createMainWindow();
    // Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
    gtk_window_move(GTK_WINDOW(mainWindow), 10,10);

        // Non Modal Dialogs
/*#if GTK_CHECK_VERSION(2,6,0)
    loadDialog= createLoadDialog();
    saveSingleDialog = createSaveSingleDialog();
    saveSetDialog = createSaveSetDialog();
#else
    printf("Functionality omitted for older GTK version\n");
#endif*/

    //bool ret = _imgRecv.Connect((char*)imageProcessModule->getName("/in").c_str(),"default");
    
    return true;
}

void graphicThread::setImageProcessModule(void *module){
    wModule=(BMLInterface*) module;
}