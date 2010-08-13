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

void selectiveAttentionModule::interrupt() {
    cmdPort.interrupt();
    commandOutput.interrupt();
}

void selectiveAttentionModule::setOptions(yarp::os::Property opt){
    options	=opt;
}

bool selectiveAttentionModule::updateModule() {
    return true;
}

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
            
            
            
            printf("Closing port %s on network %s...\n", getName("/out").c_str(),"default");
            printf("Closing port %s on network %s...\n", getName("/out2").c_str(),"default");
            printf("Closing port %s on network %s...\n", getName("/out3").c_str(),"default");
            _pOutPort->close(); //->open(getName("/out");
            _pOutPort2->close(); //open(getName("/out2");
            _pOutPort3->close(); //open(getName("/out3");
            printf("Closing port %s on network %s...\n", getName("/outRG").c_str(),"default");
            printf("Closing port %s on network %s...\n", getName("/outGR").c_str(),"default");
            printf("Closing port %s on network %s...\n", getName("/outBY").c_str(),"default");
            portRg->close(); //open(getName("/outRG");
            portGr->close(); //open(getName("/outGR");
            portBy->close(); //open(getName("/outBY");
            printf("Closing port %s on network %s...\n", getName("/outRed").c_str(),"default");
            printf("Closing port %s on network %s...\n", getName("/outGreen").c_str(),"default");
            printf("Closing port %s on network %s...\n", getName("/outBlue").c_str(),"default");
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
