#include <stdio.h>
#include <iCub/demoAff.h>

#include <iCub/processobjdata.h>

#define USE_LEFT    0
#define USE_RIGHT   1

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace vislab::math;

 
enum stateenum {
  FIRSTINIT,
  IDLE,
  INIT,
  GETOBJDESC,
  WAITINGDEMO,
  ONDEMO,
  OBJSELEC,
  DECISION,
  REACHING,
  TAPPING,
  GRASPING,
  TOUCHING,
  UNGRASP,
  UNTAP,
  UNTOUCH,
  DETECTMARKS,
  NUMSTATES
};
char statename[NUMSTATES][20]={"firstinit","idle","init","getobjdesc","waitingdemo","ondemo","objselec","decision","reaching","tapping","grasping","touching","ungrasp","untap","untouch","detectmarks"};

enum processdatastates {
  GRASP,
  TAP,
   TOUCH
} ;

enum objselectsubstates {
  LOOK,
  MOVING,
  GETPROP
} ;

//added here due to version problem
#define VOCAB_FAILED VOCAB4('f','a','i','l')
#define VOCAB_OK VOCAB2('o','k')
#define VOCAB_OBJS VOCAB1('y')
#define VOCAB_INCH VOCAB1('i')
#define VOCAB_DECH VOCAB1('o') 
#define VOCAB_QUIT VOCAB4('q','u','i','t')
#define VOCAB_GR VOCAB1('1')
#define VOCAB_TR VOCAB1('2')
#define VOCAB_GL VOCAB1('3')
#define VOCAB_TL VOCAB1('4')
#define VOCAB_Q VOCAB1('q')

static const char colors[16][16]={"red","red","orange","orange","yellow","yellow","green","green","green","blue","blue","blue","indigo","indigo","violet","violet"};
static const char shapes[2][16]={"ball","box"};
static const char actions[3][16]={"grasp","tap","touch"};


int trrest;
double imgdemopos[2];
double imgobjpos[2][2];
int shape1, shape2;
double prevwo, prevho;
double objposreach[2];



DemoAff::DemoAff(){
  state=IDLE;
  substate=0;

  printf("After constructor\n");
  trrest=0;
  OK_MSG=0;


  numObjs = 0;

  objDescTable = new BlobInfo[maxObjects];
  trackDescTable = new TrackInfo[maxObjects]; 

  table_az=0;
  table_el=0;
  InitAffPrimitives();
}

bool DemoAff::InitAffPrimitives() {
  // Action related initialization
  
  graspOrienL.resize(4);    graspOrienR.resize(4);
  graspDispL.resize(4);     graspDispR.resize(3);
  dOffsL.resize(3);         dOffsR.resize(3);
  dLiftL.resize(3);         dLiftR.resize(3);
  home_xL.resize(3);        home_xR.resize(3);
  home_oL.resize(4);        home_oR.resize(4);
  
  // default values for arm-dependent quantities
  graspOrienL[0]=-0.171542; graspOrienR[0]=-0.0191;
  graspOrienL[1]= 0.124396; graspOrienR[1]=-0.983248;
  graspOrienL[2]=-0.977292; graspOrienR[2]= 0.181269;
  graspOrienL[3]= 3.058211; graspOrienR[3]= 3.093746;
  
  graspDispL[0]= 0.0;       graspDispR[0]= 0.0; 
  graspDispL[1]= 0.0;       graspDispR[1]= 0.0; 
  graspDispL[2]= 0.05;      graspDispR[2]= 0.05;
  
  dOffsL[0]=-0.03;          dOffsR[0]=-0.03;
  dOffsL[1]=-0.07;          dOffsR[1]=-0.07;
  dOffsL[2]=-0.02;          dOffsR[2]=-0.02;
  
  dLiftL[0]= 0.0;           dLiftR[0]= 0.0;  
  dLiftL[1]= 0.0;           dLiftR[1]= 0.0;  
  dLiftL[2]= 0.15;          dLiftR[2]= 0.15; 
  
  home_xL[0]=-0.29;         home_xR[0]=-0.29;
  home_xL[1]=-0.21;         home_xR[1]= 0.24;
  home_xL[2]= 0.11;         home_xR[2]= 0.07;
  home_oL[0]=-0.029976;     home_oR[0]=-0.193426;
  home_oL[1]= 0.763076;     home_oR[1]=-0.63989;
  home_oL[2]=-0.645613;     home_oR[2]= 0.743725;
  home_oL[3]= 2.884471;     home_oR[3]= 2.995693;
  
  action=actionL=actionR=NULL;
  graspOrien=NULL;
  graspDisp=NULL;
  dOffs=NULL;
  dLift=NULL;
  home_x=NULL;
  home_o=NULL;
  
  openPorts=false;

}

DemoAff::~DemoAff(){ 

}


bool DemoAff::respond(const Bottle &command,Bottle &reply){
    bool ok = false; // command executed successfully

  switch (command.get(0).asVocab()) {
  case VOCAB_OBJS:
    printf("VOCAB OBJS\n ");
    reply.addVocab(VOCAB_OK);
    ok = true;
    OK_MSG = 1;
    break;
  case VOCAB_QUIT:
    reply.addVocab(VOCAB_OK);
    ok = true;
    break;
    
  default:
    printf("VOCAB default\n ");
    reply.addVocab(VOCAB_FAILED);
  }

    ok = RFModule::respond(command,reply); // will add message 'not recognized' if not recognized

    return ok;
}

bool DemoAff::InitAff(Searchable& config){
  
  int  cpdSize;
  int i,j;
  char str[10];

  float *_CPD;


  printf("Reading conf file\n");

  _numNodes = config.check("NUMNODES", yarp::os::Value(0),"number of nodes of the BN").asInt();
  if (_numNodes==0){
    printf("No nodes for the BN\n");
    return false;
  }
    
  printf("Num nodes: %d\n",_numNodes);

  Bottle& S = config.findGroup("SIZES", "Nodes sizes");

  if( !S.isNull() )
    {
      if( S.size() != _numNodes+1 ){
	printf("SIZES: wrong number of components\n");
	return false;
      }
      for (i=0; i<_numNodes; i++) {
	_sizes[i]=S.get(i+1).asInt();
	if (_sizes[i]<1){
	  printf("SIZES: negative size\n");
	  return false;
	}
      }
    }
  else {
    printf("SIZES not found\n");
    return false;
  }

  printf("SIZES: ");for (i=0; i<_numNodes; i++) printf("%d ",_sizes[i]); printf("\n");
  
  
  Bottle& M = config.findGroup("STRUCTURE", "Net structure matrix");

  if( !M.isNull() )
    {
      if( M.size() != _numNodes*_numNodes+1){
	printf("STRUCTURE: wrong number of components\n");
	return false;
      }
      for (i=0; i<_numNodes*_numNodes; i++){
	_mat[i]=M.get(i+1).asInt();
	cout << _mat[i] << " \n";
	if (_mat[i]!=0 && _mat[i]!=1 ){
	  printf("STRUCTURE: elements should be binary \n");
	  return false;
	}
      }
      cout << "\n";
    }
  else {
    printf("STRUCTURE not found\n");
    return false;
  }


  cout << "creating BN and engine num nodes" <<_numNodes <<"...";
  affBN.InitializeNetStructure(_numNodes, _sizes, _mat);

  cout << "done\n";

  for (i=0; i<_numNodes; i++){
    sprintf(str,"CPD%d",i+1);
    M = config.findGroup(str, "Conditional Probability Distribution");
    
    // compute table size and book space
    cpdSize=_sizes[i];
    for (j=0; j<_numNodes; j++)
      if (i!=j && _mat[i*_numNodes+j]==1)
	cpdSize*=_sizes[j];
    _CPD=(float *)malloc(cpdSize*sizeof(float));
    
    // read the CPD
    cout << "size for node "<< i << " is " << cpdSize << "\n";

    if( !M.isNull() )
      {
	if( M.size() != cpdSize +1 ){
	  printf("CPD: wrong number of components\n");
	  return false;
	}
	for (j=0; j<cpdSize; j++) {
	  _CPD[j]=(float)M.get(j+1).asDouble();
	  cout << _CPD[j] << " ";
	  if (_CPD[j]<0 || _CPD[j]>1 ){
	    printf("STRUCTURE: elements should be between 1 and 0 \n");
	    return false;
	  }
	}
	cout << "\n";
      }
    else {
      printf("STRUCTURE not found\n");
      return false;
    }

    cout<< "Initialize CPD "<<i<< "\n";
    affBN.InitializeParameters(i, _CPD);
    
    free(_CPD);
  }
  affBN.CreateEngine();

  printf("BN engine created!!\n");
  return true;
}


bool DemoAff::configure(ResourceFinder &rf){

  string name=rf.find("name").asString().c_str();
  setName(name.c_str());

  bool ok;

  Property prop; prop.fromConfigFile(rf.findFile("affordance_database").c_str());
  ok = InitAff(prop);

  Property config; config.fromConfigFile(rf.findFile("aff_action_primitives").c_str());
  

  configureAffPrimitives(config,rf.findFile("hand_sequences_file").c_str(), name);

  configureEye2World(rf.findFile("camera_calib_file").c_str());

  // open camshiftplus ports: data and sync
  string fwslash="/";
  ok &= port_eff.open( (fwslash+name+"/effect").c_str());
  ok &= port_sync.open( (fwslash+name+"/synccamshift").c_str());
  ok &= port_descriptor.open( (fwslash+name+"/objsdesc").c_str());
  ok &= port_track_info.open( (fwslash+name+"/trackdesc").c_str());
  ok &= port_primitives.open( (fwslash+name+"/motioncmd").c_str());
  ok &= port_gaze_out.open( (fwslash+name+"/gazecmd").c_str());
  ok &= port_behavior_in.open( (fwslash+name+"/behavior:i").c_str());
  ok &= port_behavior_out.open( (fwslash+name+"/behavior:o").c_str());
  ok &= port_output.open( (fwslash+name+"/out").c_str());
  ok &= port_emotions.open( (fwslash+name+"/emotion").c_str());
  ok &= inPort.open((fwslash+name+"/in").c_str());
  ok &= rpcPort.open((fwslash+name+"/rpc").c_str());
  attach(rpcPort);
  
  openPorts=true;

  attachTerminal();
  return ok;

  
}

bool DemoAff::configureEye2World(yarp::os::ConstString calibrationFilename) {

  Property config;
  if (!config.fromConfigFile(calibrationFilename)) {
    return false;
  }
  rightEyeCalibration.fromString(config.findGroup("CAMERA_CALIBRATION_RIGHT").toString());
  leftEyeCalibration.fromString(config.findGroup("CAMERA_CALIBRATION_LEFT").toString());
  
  cameras["right"] = &rightEyeCalibration;
  cameras["left"] = &leftEyeCalibration;
  
  Property tableConfig;
  if (!tableConfig.fromConfigFile(tableConfiguration)) {
    cerr << "Unable to read table configuration. Assuming 0 vector as 3D offset." << endl;
  }
  tabletopPosition.fromString(tableConfig.findGroup("POSITION").toString());

  Vector v(3);
  v[0] = table.find("x").asDouble();
  v[1] = table.find("y").asDouble();
  v[2] = table.find("z").asDouble();
  
  map<const string, Property*>::const_iterator itr;
  for (itr = cams.begin(); itr != cams.end(); ++itr) {
    
    const string key = itr->first;
    Property calib = *itr->second;
    
    projections[key] = new EyeTableProjection(key.c_str(), calib, &v);
  }

}

bool DemoAff::configureAffPrimitives(Searchable &config, 
				     yarp::os::ConstString handSeqFile, 
				     string name)
{
  
  partUsed=config.check("part",yarp::os::Value("both_arms")).asString().c_str();
  if (partUsed!="both_arms" && partUsed!="left_arm" && partUsed!="right_arm")
    {
      cout<<"Invalid part requested !"<<endl;
      return false;
    }        
  
  Bottle &bGeneral=config.findGroup("general");
  if (bGeneral.isNull())
    {
      cout<<"Error: group general is missing!"<<endl;
      return false;
    }
  
  // parsing general config options
  Property option;
  for (int i=1; i<bGeneral.size(); i++)
    {
      Bottle *pB=bGeneral.get(i).asList();
      if (pB->size()==2)
	option.put(pB->get(0).asString().c_str(),pB->get(1));
      else
	{
	  cout<<"Error: invalid option!"<<endl;
	  return false;
	}
    }
  
  option.put("local",name.c_str());
  option.put("hand_sequences_file",handSeqFile);
  
  Property optionL(option); optionL.put("part","left_arm");
  Property optionR(option); optionR.put("part","right_arm");

  // parsing left_arm config options
  Bottle &bLeft=config.findGroup("left_arm");
  if (bLeft.isNull())
    {
      cout<<"Error: group left_arm is missing!"<<endl;
      return false;
    }
  else 
    getArmDependentOptions(bLeft,graspOrienL,graspDispL,
			   dOffsL,dLiftL,home_xL,home_oL);
  
  // parsing right_arm config options
  Bottle &bRight=config.findGroup("right_arm");
  if (bRight.isNull())
    {
      cout<<"Error: group right_arm is missing!"<<endl;
      return false;
    }
  else
    getArmDependentOptions(bRight,graspOrienR,graspDispR,
			   dOffsR,dLiftR,home_xR,home_oR);
  
  if (partUsed=="both_arms" || partUsed=="left_arm")
    {    
      cout<<"***** Instantiating primitives for left_arm"<<endl;
      actionL=new affActionPrimitivesLayer1(optionL);
      
      if (!actionL->isValid())
	{
	  delete actionL;
	  return false;
	}
      else
	useArm(USE_LEFT);
    }
  
  if (partUsed=="both_arms" || partUsed=="right_arm")
    {    
      cout<<"***** Instantiating primitives for right_arm"<<endl;
      actionR=new affActionPrimitivesLayer1(optionR);
      
      if (!actionR->isValid())
	{
	  delete actionR;
	  
	  // remind to check to delete the left as well (if any)
	  if (actionL)
	    delete actionL;
	  
	  return false;
	}
      else
	useArm(USE_RIGHT);
    }
  
  deque<string> q=action->getHandSeqList();
  cout<<"***** List of available hand sequence keys:"<<endl;
  for (size_t i=0; i<q.size(); i++)
    cout<<q[i]<<endl;
  
  
  return true;
}

void DemoAff::getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x,
                                Vector &_home_o)
{
  if (Bottle *pB=b.find("grasp_orientation").asList())
    {
      int sz=pB->size();
      int len=_gOrien.length();
      int l=len<sz?len:sz;
      
      for (int i=0; i<l; i++)
	_gOrien[i]=pB->get(i).asDouble();
    }
  
  if (Bottle *pB=b.find("grasp_displacement").asList())
    {
      int sz=pB->size();
      int len=_gDisp.length();
      int l=len<sz?len:sz;
      
      for (int i=0; i<l; i++)
	_gDisp[i]=pB->get(i).asDouble();
    }
  
  if (Bottle *pB=b.find("systematic_error_displacement").asList())
    {
      int sz=pB->size();
      int len=_dOffs.length();
      int l=len<sz?len:sz;
      
      for (int i=0; i<l; i++)
	_dOffs[i]=pB->get(i).asDouble();
    }
  
  if (Bottle *pB=b.find("lifting_displacement").asList())
    {
      int sz=pB->size();
      int len=_dLift.length();
      int l=len<sz?len:sz;
      
      for (int i=0; i<l; i++)
	_dLift[i]=pB->get(i).asDouble();
    }
  
  if (Bottle *pB=b.find("home_position").asList())
    {
      int sz=pB->size();
      int len=_home_x.length();
      int l=len<sz?len:sz;
      
      for (int i=0; i<l; i++)
	_home_x[i]=pB->get(i).asDouble();
    }
  
  if (Bottle *pB=b.find("home_orientation").asList())
    {
      int sz=pB->size();
      int len=_home_o.length();
      int l=len<sz?len:sz;
      
      for (int i=0; i<l; i++)
	_home_o[i]=pB->get(i).asDouble();
    }
}

bool DemoAff::close(){

  port_eff.close();
  port_sync.close();
  port_descriptor.close();
  port_track_info.close();
  port_primitives.close();
  port_gaze_out.close();
  port_behavior_in.close();
  port_behavior_out.close();
  port_output.close();
  port_emotions.close();

  // AffActionPrimitives
  if (actionL!=NULL)
    delete actionL;
  
  if (actionR!=NULL)
    delete actionR;
  
  if (openPorts)
    {
      inPort.close();
      rpcPort.close();
    }
  
  return true;
}


void DemoAff::useArm(const int arm) {
  if (arm==USE_LEFT)
    {
      action=actionL;
      
      graspOrien=&graspOrienL;
      graspDisp=&graspDispL;
      dOffs=&dOffsL;
      dLift=&dLiftL;
      home_x=&home_xL;
      home_o=&home_oL;
    }
  else if (arm==USE_RIGHT)
    {
      action=actionR;
      
      graspOrien=&graspOrienR;
      graspDisp=&graspDispR;
      dOffs=&dOffsR;
      dLift=&dLiftR;
      home_x=&home_xR;
      home_o=&home_oR;
    }
}

bool DemoAff::interruptModule(){

  port_eff.interrupt();
  port_sync.interrupt();
  port_descriptor.interrupt();
  port_track_info.interrupt();
  port_primitives.interrupt();
  port_gaze_out.interrupt();
  port_behavior_in.interrupt();
  port_behavior_out.interrupt();
  port_output.interrupt();
  port_emotions.interrupt();

  return true; 
}


bool DemoAff::updateModule(){

  double data[256];
  int shape = 0;
  int color = 0;
  int retaddpoint;
  int arrived;
  
  //	printf("In the update\n");
  printf(".");fflush(stdout);
  
  
  
  char chin;
  cout << "state: " << state << statename[state] << endl;
  switch (state){

  case IDLE:      
    {
      Bottle *input_obj=port_behavior_in.read(false);
    
      if (input_obj!=NULL) {
	int cmd;
	
	cmd = input_obj->get(0).asInt();	
	if (cmd==1)
	  state=INIT;
	else {
	  state=IDLE;
	}
      }
    }    
    state=GRASPING;
    

    break;
  
  case INIT:
    {

      // Set expression
      Bottle& output = port_emotions.prepare();
      output.clear();
      output.addVocab(Vocab::encode("set all neutral"));
      cout << "sending expression: " << output.toString().c_str() << " " << output.size() << endl;
      port_emotions.write();

      getObjInfo();

      if (numObjs>0) {
	// Select object closer to the center
	float max_dist=10000;
	int selobj;

	for (int i=0; i<numObjs; i++) {
	  float dist = objDescTable[i].roi_x*objDescTable[i].roi_x + objDescTable[i].roi_y*objDescTable[i].roi_y;
	  if (dist<max_dist) {
	    selobj=i;
	    max_dist=dist;
	  }
	}	

	// Keep info about the selected object 
	democolor = processdata.classifycolor(objDescTable[selobj].hist);
	demoshape = processdata.classifyshape(objDescTable[selobj]);
	demosize = processdata.classifysize(objDescTable[selobj]);
	
	// Init tracker on fixated obj (if any)
	// We have to re implement the init tracker to send the histogram 
	// and the region provided by the segmentation
	restartTracker(trackDescTable[selobj]);
	
	state = WAITINGDEMO;

      }
      else {
	// No obj change to attention
	 Bottle& output = port_behavior_out.prepare();
	 output.clear();
	 output.addVocab(Vocab::encode("off"));
	 port_behavior_out.write();
	 
	 state=IDLE;
      }
    }
    break;
  

  case WAITINGDEMO: 
    {

      double data[5];
      
      yarp::os::Time::delay(0.1);                 
      Bottle *input_obj=port_eff.read(true);
      
      if (input_obj!=NULL) {
	for(int cnt = 0;cnt<input_obj->size();cnt++)
	  data[cnt] = input_obj->get(cnt).asDouble();
	
	retaddpoint = processdata.addDataPoint((double*)data);
	
	if (processdata.detectActionInit()==1) {	
	  processdata.restartDataAcquisition();
	  state=ONDEMO;
	  printf("ACTION STARTED ON A %s %s OBJECT\n",colors[color],shapes[demoshape]);
	}
      }
      else {
	cout << "No data from tracker" << endl;
      }
    }
    break;

  case ONDEMO:
    
    if(processdata.bufferfull()) {
      // classify effects
      double effects[32];
      unsigned char effectclassification[32];
      
      processdata.computeEffects((double*)effects);
      processdata.classifyeffects((double*)effects, (unsigned char*)effectclassification);
      printf("effects %g %d %g %d \n",effects[0],effectclassification[0],
	     effects[1],effectclassification[1]);
      
      demoeffects[0]=effectclassification[0];
      demoeffects[1]=effectclassification[1];
      // move to next state
      state = OBJSELEC;
      numObservedObj=0;
      substate=LOOK;     
    }
    else {
      Bottle *input_obj=port_eff.read(true);
      
      if (input_obj!=NULL) {
	for(int cnt = 0;cnt<input_obj->size();cnt++)
	  data[cnt] = input_obj->get(cnt).asDouble();
	
	retaddpoint = processdata.addDataPoint((double*)data);
	
	yarp::os::Time::delay(0.1);                 
      }
    }
    break;	    
    
  case OBJSELEC: 
    {
      // Look for objects
      Bottle& outputGaze = port_gaze_out.prepare();
      outputGaze.clear();
      outputGaze.addDouble(table_az);
      outputGaze.addDouble(table_el);
      port_behavior_out.write();
      
      
      yarp::os::Time::delay(0.1);                 
      
      bool done=false;
      Bottle *input_obj;
      while (!done) {
	// Deberia ser algo de motor port_gaze_in
	input_obj=port_descriptor.read(true);
	done = input_obj->get(0).asInt()==5;	
      }
      
      // Set expression
      Bottle& output = port_emotions.prepare();
      output.clear();
      output.addVocab(Vocab::encode("set all neutral"));
      cout << "sending expression: " << output.toString().c_str() << " " << output.size() << endl;
      port_emotions.write();


      getObjInfo();
      
      if (numObjs<=0){
	// Stop aff demo and notify behavior
	Bottle& output = port_behavior_out.prepare();
	output.clear();
	output.addVocab(Vocab::encode("off"));
	port_behavior_out.write();
	
	state=IDLE;
	
	// Put sad expression TBD move head a bit do sth damn it!
	
      }
      else {

	for (int i=0; i<numObjs; i++) {
	  colorObj[i] = processdata.classifycolor(objDescTable[i].hist);
	  shapeObj[i] = processdata.classifyshape(objDescTable[i]);
	  sizeObj[i] = processdata.classifysize(objDescTable[i]);	  
	}
	// Move to next state to select object according to effect	
	state=DECISION;
      }
    }
    break;

    
  case DECISION:
    {
      // head.thinkingexpression();
      
      //recover action from effects
      int nquery;
      int query[4];
      int ev[4];
      float actprob[3];
      
      ev[0]=-1; ev[1]=demoshape; ev[2]=demoeffects[0];ev[3]=demoeffects[1];
      nquery=1; query[0]=0;
      affBN.SolveQueryPerfectObs(ev, nquery, query, actprob);
      printf("------------------\nAction probabilities: %f %f %f\n",actprob[0],actprob[1],actprob[2]);
      
      // Test action, object combinations to find the best pair
      float jeffectprob[4];
      float bestprob=0.0;
      for (int i=0; i<numObjs;i++) {
	for (int j=0; j<3; j++) {
	  ev[0]=j; ev[1]= shapeObj[i]; ev[2]=-1;ev[3]=-1;
	  nquery=2; query[0]=2; query[1]=3;
	  affBN.SolveQueryPerfectObs(ev, nquery, query, jeffectprob);
	  
	  if (bestprob<jeffectprob[demoeffects[0]*2+demoeffects[1]]){
	    bestprob=jeffectprob[demoeffects[0]*2+demoeffects[1]];
	    selectedobj=i;
	    selectedaction=j;
	  }
	}
      }
      
      if (bestprob==0.0) {
	printf("No good option. Let's try again and this time, do not cheat on me!!\n");
	state=INIT;
      }
      else {
	// 	    head.look2object(selectedobj);
	
	switch (selectedaction){
	case GRASP: state=GRASPING; break;
	case TAP: state=TAPPING; break;
	case TOUCH: state=GRASPING; break;
	}
	
	// Select the appropriate arm
	
	// Select the object position
	objposreach[0] = imgobjpos[selectedobj][0];
	objposreach[1] = imgobjpos[selectedobj][1];
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printf("Selected a %s on the %s %s located at pos %d <%g %g>\n", 
	       actions[selectedaction], colors[colorObj[selectedobj]], 
	       shapes[shapeObj[selectedobj]], selectedobj,objposreach[0],
	       objposreach[1]);
      }
      while (OK_MSG==0) yarp::os::Time::delay(0.04);
      OK_MSG=0;
    }  
    break;
    
  case REACHING:
    {
      // We will have different reachings or there is an initial common reaching phase?
      switch (selectedaction){
      case GRASP: state=GRASPING; 
	  break;
      case TAP: state=TAPPING; 
	break;
      case TOUCH: state=TOUCHING; 
	break;
	}
    }
   
    break;
  case GRASPING:
    {
      Vector xd(3);
      bool f;

      xd[0]=0.1; 
      xd[1]=0.1; 
      xd[2]=0.1; 
      
      // switch only if it's allowed
      if (partUsed=="both_arms")
	{
	  if (xd[1]>0.0)
	    useArm(USE_RIGHT);
	  else
	    useArm(USE_LEFT);
	}

      cout << " after arm selection" << endl;
      // apply systematic offset
      // due to uncalibrated kinematic
      xd=xd+*dOffs;
      
      // safe thresholding
      xd[0]=xd[0]>-0.1?-0.1:xd[0];
      
       cout << " before first push" << endl;      

      // grasp it (wait until it's done)
      action->grasp(xd,*graspOrien,*graspDisp);
      cout << " push in wainting" << endl;      
      action->checkActionsDone(f,true);
      
      cout << " first push" << endl;      

      // lift the object (wait until it's done)
      action->pushAction(xd+*dLift,*graspOrien);
      action->checkActionsDone(f,true);
      
      cout << " second push" << endl;      

      // release the object (wait until it's done)
      action->pushAction("open_hand");
      action->checkActionsDone(f,true);
      
      cout << " after releasing" << endl;

      // go home :)
      action->pushAction(*home_x,*home_o);

      cout << "gooing back home" << endl;
    }		
    
    break;

  case TAPPING:
    {
      printf("eliminar cuanto antes");
      
      
    }
    break;

  case TOUCHING:
    {
      printf("eliminar cuanto antes");
    }
    break;
    
  }
  
  
  return true;
}

bool DemoAff::restartTracker(TrackInfo obj) {

  Stamp writestamp;
  Bottle& bot = port_sync.prepare();

  bot.clear();
  bot.addInt(obj.roi_x);
  bot.addInt(obj.roi_y);
  bot.addInt(obj.roi_width);
  bot.addInt(obj.roi_height);
  for (int i=0; i<16; i++)
    bot.addInt(obj.hist[i]);
  bot.addInt(obj.v_min);
  bot.addInt(obj.v_max);
  bot.addInt(obj.s_min);
  port_sync.setEnvelope(writestamp);
  port_sync.write();
  
  cout << "writing " << bot.toString().c_str() << " " << bot.size() << endl;

  
  // flag to signal tracker has been restarted
  trrest=1;
  prevwo=10000; prevho=10000;
}




 bool DemoAff::getObjDesc(Bottle * input_obj) {

   /*
   if (input_obj->size()<2)
     numObjs=0;
   else {
     numObjs=input_obj->get(1).asInt();
     nelems=input_obj-<get(2).asInt();

     double data[nelems];

     for(int nobj=0; nobj<numobj; nobj++){

       for (cnt=0; cnt<nelems; cnt++)
	 data[cnt] = input_obj->get(nobj*nelems+cnt).asDouble();
       
       double *hist;
       hist = processdata.getcolorhist((double*)data);
       color[nobj] = processdata.classifycolor( hist );
       objpos[nobj][0]=data[0]; objpos[nobj][1]=data[1];       

       hist = processdata.getshapehist(data);
       shape[nobj] = processdata.classifyshape( hist );
       
       size[nobj]= data[2]*data[3];
       printf("Obj %d at %g %g %d %d\n",objpos[nobj][0],objpos[nobj][1],color[nobj],shape[nobj]);
     }
   }
   */
 }


bool DemoAff::readEffect() { 
  /*
  // Read data from CamShiftPlus
  Bottle *input_obj=port_eff.read(false);
  
  if (input_obj!=NULL)
    {
      double *hist;
      
      //printf(".\n");
      for(int cnt = 0;cnt<input_obj->size();cnt++)
	data[cnt] = input_obj->get(cnt).asDouble();
      
      hist = processdata.getcolorhist((double*)data);
      color = processdata.classifycolor( hist );
      //processdata.printhist( hist );
      // ler a posicao do objecto
      objpos[0]=data[0]; objpos[1]=data[1];
      printf("Obj at %g %g %d %d\n",objpos[0],objpos[1],color,shape);
      
      hist = processdata.getshapehist(data);
      shape = processdata.classifyshape( hist );
      //processdata.printhist( hist );
      
      processdata.setlastshape(shape);
      processdata.setlastcolor(color);
      
	    printf("data do tracker e: %g %g\n", data[2], data[3]);
	    if (trrest) {
	      bool stable;
	      stable=((prevwo-data[2])*(prevwo-data[2])<4) && ((prevho-data[3])*(prevho-data[3])<4);
	      trrest++;
	      if (stable) {
		retaddpoint = processdata.addDataPoint((double*)data);
		printf("Tracker stable after %d frames\n",trrest);
		trrest=0;
	      }
	      prevwo=data[2]; prevho=data[3];
	    }
	    else {
	      retaddpoint = processdata.addDataPoint((double*)data);
	    }
    }
  else
    {
      printf("No data\n");
      yarp::os::Time::delay(0.005);
    }
  */  
}

 bool DemoAff::readDetectorInput(Bottle *Input, string name, double *cx, double *cy)
 {
   string tmp;
   int siz = Input->size();
   
   
   for (int cnt = 0; cnt <(siz/22);cnt++)
     {
       tmp=Input->get(cnt*22).asString();
       
       if (tmp==name) {
	 *cx = Input->get(cnt*22+20).asDouble();
	 *cy = Input->get(cnt*22+21).asDouble();
	 
	 cout << "name" << Input->get(cnt*22).asString() << endl;
	 cout << "unnorm coord: " <<Input->get(cnt*22+15).asDouble() << Input->get(cnt*22+16).asDouble() << endl;
	 cout << "norm coord: " <<Input->get(cnt*22+20).asDouble() << Input->get(cnt*22+21).asDouble() << endl;  
	 return true;
       }
     }
   cout << "not detected\n";
   return false;
   
   
 }
 

bool DemoAff::getBlobInfo(const Bottle *msg) {
  /* This functino reads from a port written by BlobDescriptorModule.h
     Check there for details
  */

  // Read number of objects
  numBlobs=msg->get(0).asInt();

  for (int i=0; i<numObjs; i++) {
    // center coordinates
    //objDescTable[i].center.x = msg.get(i*29+1).asDouble();
    //objDescTable[i].center.y = msg.get(i*29+2).asDouble();
    // width and height
    objDescTable[i].roi_x = msg->get(i*29+1).asDouble();
    objDescTable[i].roi_y = msg->get(i*29+2).asDouble();
    objDescTable[i].roi_width = msg->get(i*29+3).asDouble();
    objDescTable[i].roi_height = msg->get(i*29+4).asDouble();
    objDescTable[i].angle = msg->get(i*29+5).asDouble();

    for (int j=0; j<16; j++) {
      objDescTable[i].hist[j]= msg->get(i*29+8+j).asDouble();
    }

    objDescTable[i].area = msg->get(i*29+24).asDouble();
    objDescTable[i].convexity = msg->get(i*29+25).asDouble();
    objDescTable[i].eccentricity = msg->get(i*29+26).asDouble();    
    objDescTable[i].compactness = msg->get(i*29+27).asDouble();
    objDescTable[i].circleness = msg->get(i*29+28).asDouble();
    objDescTable[i].squareness = msg->get(i*29+29).asDouble();    
  }
  

}


bool DemoAff::getTrackerInfo(const Bottle *msg) {
  /* This functino reads from a port written by BlobDescriptorModule.h
     Check there for details
  */

  int cnt=0;
  // Read number of objects
  numTracks=msg->get(cnt).asInt(); cnt++;

  for (int i=0; i<numObjs; i++) {
    // x, y, width and height
    trackDescTable[i].roi_x = msg->get(cnt).asInt();
    cnt++;
    trackDescTable[i].roi_y = msg->get(cnt).asInt();
    cnt++;
    trackDescTable[i].roi_width = msg->get(i*23+3).asInt();
    cnt++;
    trackDescTable[i].roi_height = msg->get(i*23+4).asInt();
    cnt++;

    // 16 bin histogram
    for (int j=0; j<16; j++) {
      trackDescTable[i].hist[j]= msg->get(cnt).asInt();
      cnt++;
    }

    trackDescTable[i].v_min = msg->get(cnt).asInt();
    cnt++;
    trackDescTable[i].v_max = msg->get(cnt).asInt();
    cnt++;
    trackDescTable[i].s_min = msg->get(cnt).asInt();
    cnt++;    
  }
  
}


bool DemoAff::getObjInfo() {
  
  Stamp objstamp, trackstamp; 
  Bottle *objbottle = port_descriptor.read(true);
  Bottle *trackbottle = port_track_info.read(true);
 
  
  /* check that both ports have timestamps */
  if( !port_descriptor.getEnvelope(objstamp) || !port_track_info.getEnvelope(trackstamp) )    {
    cout << getName() << ": this module requires ports with valid timestamp data. Stamps are missing. Exiting..." << endl;
    return false;
  }
  /* synchronize the obj and track descriptors, if one of them is delayed, so that they correspond */
  while( objstamp.getCount() < trackstamp.getCount() )	{
    objbottle = port_descriptor.read(true);
    port_descriptor.getEnvelope(objstamp);
  }
  while( objstamp.getCount() > trackstamp.getCount() )	{
    trackbottle = port_track_info.read(true);
    port_track_info.getEnvelope(trackstamp);
  }
  
  getBlobInfo(objbottle);
  getTrackerInfo(trackbottle);
  
  if (numBlobs!=numTracks)	{
    cout << getName() << "Perception non synchronized: #blobs not equal to #tracks" << endl;
    numObjs=0;
  }
  else numObjs=numBlobs;

  return true;
}
