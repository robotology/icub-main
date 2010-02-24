#include <iCub/blobFinderModule.h>
#include <iCub/WatershedModule.h>


// We need a macro for efficient switching.
// Use as, for example, VOCAB('s','e','t')
#define VOCAB(a,b,c,d) ((((int)(d))<<24)+(((int)(c))<<16)+(((int)(b))<<8)+((int)(a)))
#define VOCAB4(a,b,c,d) VOCAB((a),(b),(c),(d))
#define VOCAB3(a,b,c) VOCAB((a),(b),(c),(0))
#define VOCAB2(a,b) VOCAB((a),(b),(0),(0))
#define VOCAB1(a) VOCAB((a),(0),(0),(0))

blobFinderModule::blobFinderModule(){
    gui=new WatershedModule();
}

blobFinderModule::~blobFinderModule(){
    delete gui;
}


bool blobFinderModule::open(Searchable& config) {
    printf("Opening the command port at %s \n",getName());
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
    gui->setName(getName());
    gui->setModule(this);
    gui->start();
    return true;
}

bool blobFinderModule::close(){
    cmdPort.close();
    gui->close();
    return true;
}

bool blobFinderModule::interruptModule(){
    cmdPort.interrupt();
    return true;
}

void blobFinderModule::setOptions(yarp::os::Property opt){
    options	=opt;
}

bool blobFinderModule::updateModule(){
    return true;
}
