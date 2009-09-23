#include "moduleYarpNeck.hpp"

#include <unistd.h>
#include <sstream>
#include <iomanip>

#include <Common/Helper/iqrUtils.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarpIF;

MAKE_MODULE_DLL_INTERFACE(iqrcommon::moduleYarpNeck,"IQR Yarp Neck Module")

iqrcommon::moduleYarpNeck::moduleYarpNeck() : ClsThreadModule() {
    tid = 0;
  
    headEncs = addOutputToGroup("Neck Encoders", "Neck Encoders");
    headCmds = addInputFromGroup("Neck Motors Commands", "Neck Motors Commands");	
}

iqrcommon::moduleYarpNeck::~moduleYarpNeck(){ 
}

void iqrcommon::moduleYarpNeck::init(){
    openYarpNetwork();

    Property options("(limb head) (local /zenon/head)");

    head=new iCubLimbIF();

    head->configure(options);

    nAxes=head->getAxes();

    Vector vels(nAxes);
    vels=30.0;
    head->setRefSpeeds(vels);
}

void iqrcommon::moduleYarpNeck::update(){
    Vector v(nAxes);
 
    qmutexThread->lock();
    head->readEncs(v);

    //debug.beg
    cout<<"got: "<<v.toString()<<endl;
    //debug.end

    StateArray &stateArrayIn = headEncs->getStateArray();
    for (int i=0; i<nAxes; i++)   
    	stateArrayIn[0][i]=v[i];
    qmutexThread->unlock();

    qmutexThread->lock();
    StateArray &stateArrayOut = headCmds->getTarget()->getStateArray();
    for (int i=0; i<nAxes; i++)
    	v[i]=stateArrayOut[0][i]; 
    head->writePosCmds(v);

    //debug.beg
    cout<<"sent: "<<v.toString()<<endl;
    //debug.end
    qmutexThread->unlock();

    usleep(1e6);

    //debug.beg
    cout<<endl<<endl;
    //debug.end    
}

void iqrcommon::moduleYarpNeck::cleanup(){
    delete head;

    closeYarpNetwork();
}
