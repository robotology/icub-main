// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/selectiveAttentionModule.h>
#include <iCub/graphicThread.h>
#include <cstring>
#include <cstdlib>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace std;

// We need a macro for efficient switching.
// Use as, for example, VOCAB('s','e','t')
#define VOCAB(a,b,c,d) ((((int)(d))<<24)+(((int)(c))<<16)+(((int)(b))<<8)+((int)(a)))
#define VOCAB4(a,b,c,d) VOCAB((a),(b),(c),(d))
#define VOCAB3(a,b,c) VOCAB((a),(b),(c),(0))
#define VOCAB2(a,b) VOCAB((a),(b),(0),(0))
#define VOCAB1(a) VOCAB((a),(0),(0),(0))


/**
* generic constructor
*/
selectiveAttentionModule::selectiveAttentionModule(){
    command=new string("");
}
/**
* destructor
*/
selectiveAttentionModule::~selectiveAttentionModule(){
    delete command;
}

bool selectiveAttentionModule::open(Searchable& config) {
    ct = 0;

    inputImage_flag=false;
    
    //port.open(getName());
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    //port2.open(getName("edges"));
    //port_plane.open(getName("blue"));
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
    commandOutput.open(getName("command:o"));
    //starting the graphical thread (GUI)
    this->gui=new graphicThread();
    gui->setselectiveAttentionModule(this); //it is necessary to synchronise the static function with this class
    gui->setName(this->getName());
    gui->start();

    return true;
}

// try to interrupt any communications or resource usage
bool selectiveAttentionModule::interruptModule() {
    //port.interrupt();
    //port2.interrupt();
    //port_plane.interrupt();
    cmdPort.interrupt();
    commandOutput.interrupt();
    return true;
}

bool selectiveAttentionModule::close() {
    
    //yarp::os::Network::fini();
    printf("closing all the ports of the module \n");
    //port.close();
    //port2.close();
    //port_plane.close();
    cmdPort.close();
    commandOutput.close();
    this->closePorts();   
    gui->close();
    
    
    //currentProcessor->~ImageProcessor();
    //delete processor1;
    //delete processor2;
    //delete processor3;
    return true;
    }

void selectiveAttentionModule::setOptions(yarp::os::Property opt){
    options	=opt;
}

bool selectiveAttentionModule::updateModule() {
    // output the images
    //port.prepare() = *img;
    //port.write();
    //port2.prepare() = *yarpReturnImagePointer;
    //port2.write();
    //port_plane.prepare()= *blue_plane;
    //port_plane.write();
    
    //delete yarpReturnImagePointer;
    //-----------------------------------
    return true;
}

/*_DEPRECATED bool selectiveAttentionModule::updateModule() {
    ImageOf<PixelRgb> *img = port.read();
    if (img==NULL) return false;;
    int width=img->width();
    int height=img->height();
    int psb,psb4;
    
    Ipp8u *colour1=ippiMalloc_8u_C4(width,height,&psb4);
    Ipp8u *y1=ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u *u1=ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u *v1=ippiMalloc_8u_C1(width,height,&psb);
    Ipp8u *out=ippiMalloc_8u_C1(width,height,&psb);
    
    IppiSize srcsize;
    srcsize.width=width;
    srcsize.height=height;
    
    IplImage *cvImage = cvCreateImage(cvSize(img->width(),  img->height()), 	IPL_DEPTH_8U, 1 );
    // add a blue circle
    PixelRgb blue(0,0,255);
    addCircle(*img,blue,ct,50,10);
    ct = (ct+5)%img->width();
    ImageProcessor* imageProcessor=new ImageProcessor(img);
    ImageOf<PixelRgb> *yarpReturnImagePointer=imageProcessor->findEdges(img,1,1);   
    ImageOf<PixelMono> *blue_plane=imageProcessor->getBluePlane(img);
    cvCvtColor((IplImage*)img->getIplImage(), cvImage, CV_RGB2GRAY);
    //delete img;
    printf("Showing OpenCV/IPL image\n");
    
    //--------conversion to logPolar----------------
    CvScalar s;
    //----------conversion to YARP COLOUR------------------
    yarpReturnImagePointer=imageProcessor->getOutputImage();
    // output the images
    port.prepare() = *img;
    port.write();
    port2.prepare() = *yarpReturnImagePointer;
    port2.write();
    port_plane.prepare()= *blue_plane;
    port_plane.write();
    
    //delete yarpReturnImagePointer;

    //-----------------------------------
    this->createMainWindow();

    return true;
}*/







bool selectiveAttentionModule::openPorts(){
    bool ret = false;
    //int res = 0;
    // Registering Port(s)
    //reduce verbosity --paulfitz
    /*printf("trying to open ports \n");
    printf("Registering port %s on network %s...\n", "/in","default");
    printf("%s",getName("/in"));*/
    
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
    

    return true;
}

bool selectiveAttentionModule::outPorts(){
    bool ret = false;
    //if((processor1->canProcess_flag)&&(processor2->canProcess_flag)&&(processor3->canProcess_flag))
    /*if(false)
    {
        //printf("Entered in outPorts \n");
        this->_pOutPort->prepare()=*(this->processor1->portImage);
        this->_pOutPort2->prepare()=*(this->processor2->portImage);
        this->_pOutPort3->prepare()=*(this->processor3->portImage);
        //printf("After prepares \n");
        this->_pOutPort->write();
        this->_pOutPort2->write();
        this->_pOutPort3->write();
        //printf("Entered in outPorts \n");
    }*/
    //if((currentProcessor->blueYellow_flag)&&(currentProcessor->redGreen_flag)&&(currentProcessor->greenRed_flag)){
    /*
    if(false){
        //if(currentProcessor->redGreen_yarp!=0xcdcdcdcd)
        this->portRg->prepare()=*(this->currentProcessor->redGreen_yarp);
        //if((unsigned int)currentProcessor->greenRed_yarp!=0xcdcdcdcd)
        this->portGr->prepare()=*(this->currentProcessor->greenRed_yarp);
        //if((unsigned int)currentProcessor->blueYellow_yarp!=0xcdcdcdcd)
        this->portBy->prepare()=*(this->currentProcessor->blueYellow_yarp);
        //printf("After prepares \n");
        this->portRg->write();
        this->portGr->write();
        this->portBy->write();
        //printf("Entered in outPorts \n");
        //if((unsigned int)currentProcessor->redPlane!=0xcdcdcdcd)
        this->portRedPlane->prepare()=*(this->currentProcessor->redPlane);
        //if((unsigned int)currentProcessor->greenPlane!=0xcdcdcdcd)
        this->portGreenPlane->prepare()=*(this->currentProcessor->greenPlane);
        //if((unsigned int)currentProcessor->bluePlane!=0xcdcdcdcd)
        this->portBluePlane->prepare()=*(this->currentProcessor->bluePlane);
        //printf("After prepares \n");
        this->portRedPlane->write();
        this->portGreenPlane->write();
        this->portBluePlane->write();
    }*/
    
    //command->assign("help");
    if(strcmp(command->c_str(),"")&&(commandOutput.getOutputCount())){
    
        Bottle& outBot1=commandOutput.prepare();
        outBot1.clear();
        //bOptions.addString("to");
        //bOptions.addString("Layer0");
        //outBot1.fromString(command->c_str());
        if(command->length()>5){
            outBot1.addVocab(VOCAB3(command->at(0),command->at(1),command->at(2)));
            outBot1.addVocab(VOCAB2(command->at(4),command->at(5)));
            std::string sub=command->substr(7,command->length()-6);
            double value=atof(sub.c_str());
            outBot1.addDouble(value);
        }
        //outBot1.addList()=bOptions;
        this->commandOutput.writeStrict();
        command->clear();
        //bOptions.clear();
    }

    return ret;
}

bool selectiveAttentionModule::closePorts(){
    bool ret = false;
    //int res = 0;
    // Closing Port(s)
    //reduce verbosity --paulfitz
    
    if (false)
        {
        
            /*_pOutPort;
            _pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb>>;
            _pOutPort3 = new yarp::os::BufferedPort<ImageOf<PixelRgb>>;*/
            
            
            
            printf("Closing port %s on network %s...\n", getName("/out"),"dafult");
            printf("Closing port %s on network %s...\n", getName("/out2"),"dafult");
            printf("Closing port %s on network %s...\n", getName("/out3"),"dafult");
            _pOutPort->close(); //->open(getName("/out");
            _pOutPort2->close(); //open(getName("/out2");
            _pOutPort3->close(); //open(getName("/out3");
            printf("Closing port %s on network %s...\n", getName("/outRG"),"dafult");
            printf("Closing port %s on network %s...\n", getName("/outGR"),"dafult");
            printf("Closing port %s on network %s...\n", getName("/outBY"),"dafult");
            portRg->close(); //open(getName("/outRG");
            portGr->close(); //open(getName("/outGR");
            portBy->close(); //open(getName("/outBY");
            printf("Closing port %s on network %s...\n", getName("/outRed"),"dafult");
            printf("Closing port %s on network %s...\n", getName("/outGreen"),"dafult");
            printf("Closing port %s on network %s...\n", getName("/outBlue"),"dafult");
            portRedPlane->close(); //open(getName("/outRed");
            portGreenPlane->close(); //open(getName("/outGreen");
            portBluePlane->close(); //open(getName("/outBlue");*/

            if(true)
                printf("All ports closed succeed!\n");
            else 
                {
                    printf("ERROR: Ports closing failed.\nQuitting, sorry.\n");
                    return false;
                }

        }

    return true;
}






