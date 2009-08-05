#include <stdio.h>
#include <iCub/demoAff.h>

#include <iCub/processobjdata.h>

enum stateenum {
  FIRSTINIT,
  INIT,
  MOVE2DEMO,
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
char statename[NUMSTATES][20]={"firstinit","init","move2demo","waitingdemo","ondemo","objselec","decision","reaching","tapping","grasping","touching","ungrasp","untap","untouch","detectmarks"};

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

// preprogrammed positions (this should be in the config file??)
// Head
#ifdef BALTAZAR
double posDemo[4]={0.0, 0.0, -50.0, 10.0};
double initHeadPos[4]={0.0, 0.0, -50.0, 10.0};
#else
double posDemo[6]={-30.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double initHeadPos[6]={-30.0, 0.0, 0.0, 0.0, 0.0, 0.0};
#endif
double posObj[2][4]={0.0, 0.0, -10.0, -30.0,     0.0, 0.0, 10.0, 30.0};
int numObj2Observe=2;

// Arm
#ifdef BALTAZAR
double initArmPos[6]={0.0, -1.2, 0.0, -1.3, 0.0, 0.0};
double initArmPos2[6]={-0.48, -0.38, -0.36, -1.36, -0.92, -0.04};
#else
double initArmPos[16]={-50.0, 56.74, 8.43, 48.99, -0.01, 0.04, 0.09, 3.00, 2.26, -0.02, 0.07, 0.07, 0.22, 0.23, -0.02, 0.48};
double initArmPos2[16]={  -54.51, 56.74, 8.43, 48.99, -0.01, 0.04, 0.09, 3.00, 2.26, -0.02, 0.07, 0.07, 0.22, 0.23, -0.02, 0.48};

double initLArmPos[16]={-50.0, 56.74, 8.43, 48.99, -0.01, 0.04, 0.09, 3.00, 2.26, -0.02, 0.07, 0.07, 0.22, 0.23, -0.02, 0.48};




int numGraspVecArm;

double graspVecTimeArm[7]={3.0, 3.0, 7.0, 7.0, 5.0, 5.0, 5.0};

double graspVecArm[7][16]={
  -54.51, 56.74, 8.43, 48.99, -0.01, 0.04, 0.09, 3.00, 2.26, -0.02, 0.07, 0.07, 0.22, 0.23, -0.02, 0.48,
  -51.25, 38.63, 48.07, 73.69, 37.00, 10.00, 4.03, -1.00, 3.77, -0.09, -0.00, -0.15, 0.18, 0.32, -0.16, 0.45,
  -51.16, 36.96, 48.12, 68.33, 41.02, 7.01, -3.01, -6.00, 81.19, -0.02, -0.15, -0.25, 0.13, 0.38, -0.16, 1.25,
  -51.08, 37.04, 47.92, 66.04, 36.84, 10.03, 3.10, -3.00, 78.16, 52.45, 0.13, 90.25, 0.25, 40.46, 45.80, 49.85,
  -51.08, 37.04, 48.07, 66.04, 36.83, 10.03, 3.12, -3.00, 74.64, 52.86, 8.69, 80.75, 43.05, 20.55, 44.07, 54.32,
  -72.44, 64.91, 33.78, 11.19, 36.96, 9.99, 3.00, -2.00, 71.41, 52.75, 7.85, 81.27, 46.99, 20.00, 42.88, 53.59,
  -72.79, 65.26, 33.83, 10.22, 36.96, 9.98, 2.96, -4.00, -13.45, -0.28, 0.10, -0.17, 0.10, -0.25, -0.14, 0.58};
double graspVecVelArm[7][16]={
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 53.00, 75.00, 63.00, 46.00, 50.00, 50.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 36.00, 10.00, 53.00, 65.00, 63.00, 46.00, 50.00, 50.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 53.00, 65.00, 63.00, 46.00, 50.00, 50.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 50.00, 60.00, 53.00, 65.00, 63.00, 46.00, 50.00, 50.00};




int numTapVecArm;
/*double tapVecArm[3][16]={
  -54.51, 56.74, 8.43, 48.99, -0.01, 0.04,     0.09, 3.00, 2.26, -0.02, 0.07, 0.07, 0.22, 0.23, -0.02, 0.48,
  -34.46, 26.05, 19.95, 65.78, -17.04, -67.98, -19.98, 22.00, -13.86, -0.11, -0.00, -0.21, 0.19, 0.23, -0.16, -0.93,
  -35.34, 26.32, 19.90, 65.78, -14.04, 10.03, -20.00, 21.00, 4.47, -0.11, -0.00, -0.21, 0.19 ,0.23, -0.16, -0.32};

double tapVecVelArm[3][16]={
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 100.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 100.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00};
*/

double tapVecArm[4][16]={
  -54.51, 56.74, 8.43, 48.99, -0.01, 0.04, 0.09, 3.00, 2.26, -0.02, 0.07, 0.07, 0.22, 0.23, -0.02, 0.48,
  -25.11, 21.84 ,19.90, 72.46, -17.07, -68.00, -19.95, 23.00, -13.04, -0.26, 0.08, -0.12, 0.28, -0.01, -0.12, -0.65,
  -25.11, 21.92, 20.00 ,72.46, -17.00, 10.03, -20.01, 22.00, -13.21, -0.26, 0.08, -0.12, 0.28, -0.01, -0.12, -0.30,
  -24.93, 67.37, 20.05, 72.55, -17.01, 10.02, -20.01, 22.00, -13.13, -0.26, 0.08, -0.12, 0.28, -0.01, -0.12, 0.39};

double tapVecVelArm[4][16]={
  10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 100.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00 ,10.00,
  10.00, 10.00 ,10.00 ,10.00, 10.00, 100.00 ,10.00, 10.00 ,10.00 ,10.00 ,10.00, 10.00 ,10.00 ,10.00 ,10.00, 10.00,
  10.00, 10.00, 10.00, 10.00, 10.00, 100.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00, 10.00};

double tapVecTimeArm[4]={3.0, 5.0, 2.0, 5.0};


double encvec[16];
double encth[16]={0.1,0.1,0.1,0.1,0.1,0.1,0.1,2,1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
#endif

int trrest;
double imgdemopos[2];
double imgobjpos[2][2];
int shape1, shape2;
double prevwo, prevho;
double objposreach[2];

// performs servoing until closer than err, returns the final position in headpos, reentrant

float DemoAff::headServo(double errx, double erry, double* headvel, float gain)
{
  headvel[1] = -gain * errx;
  headvel[0] = gain * erry;

  if( (errx*errx+erry*erry) > 0.01)
    return 0;
  else
    return 1;
}


int DemoAff::saveLocation(double* pos, int locnumber)
{
  if(locnumber>MAXLOCATIONS)
   return -1;
  else
    {
      headlocations[locnumber][0] = pos[0];
      headlocations[locnumber][1] = pos[1];
      headlocations[locnumber][2] = pos[2];
      headlocations[locnumber][3] = pos[3];
    }

  return 0;
}

DemoAff::DemoAff(){
  state=FIRSTINIT;
  substate=0;
  detectedlocations=0;
  printf("After constructor\n");
  trrest=0;
  OK_MSG=0;
  detectedlocations=0;
  height=22;
  numGraspVecArm=7;
  numTapVecArm=4;

  for (int i=0; i<16; i++)
    encvec[i]=0;
}

DemoAff::~DemoAff(){ 
  port_obj.close();
  port_sync.close();


#ifdef BALTAZAR
  double vels[20];

  // move head to zero then set velocity to zero
  if (_numHeadAxes > 0){
      for (int i=0; i<_numHeadAxes; i++){
        ipos->positionMove( i, 0);
        vels[i]=0.0;
      }
      ivel->velocityMove( vels );
  }
#else
  double vels[20];

  if (_numHeadAxes > 0){
      for (int i=0; i<_numHeadAxes; i++){
        ipos->positionMove( i, 0);
        vels[i]=0.0;
      }
      ivel->velocityMove( vels );
  }

  if (_numRArmAxes > 0){
      for (int i=0; i<_numRArmAxes; i++){
        ar_ipos->positionMove( i, 0);
        vels[i]=0.0;
      }
      ar_ivel->velocityMove( vels );
  }
 if (_numLArmAxes > 0){
      for (int i=0; i<_numLArmAxes; i++){
        al_ipos->positionMove( i, 0);
        vels[i]=0.0;
      }
      al_ivel->velocityMove( vels );
  }
#endif

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
      case VOCAB_INCH:
	printf("VOCAB INCH\n ");
	reply.addVocab(VOCAB_OK);
	height++;
	printf("increase height: %g\n",height);
	ok= true;
      break;
      case VOCAB_DECH:
	printf("VOCAB DECH\n ");
	reply.addVocab(VOCAB_OK);
        height--;
        ok= true;
	break;
      case VOCAB_GR:
	selectedobj=0;
	selectedaction=0;	
	ok = true;
	OK_MSG = 1;	
	break;
      case VOCAB_GL:
	selectedobj=1;
	selectedaction=0;	
	ok = true;
	OK_MSG = 1;	
	break;
      case VOCAB_TR:
	selectedobj=0;
	selectedaction=1;	
	ok = true;
	OK_MSG = 1;	
	break;
      case VOCAB_TL:
	selectedobj=1;
	selectedaction=1;	
	ok = true;
	OK_MSG = 1;	
	break;
      case VOCAB_Q:
	ok = false;
	state = INIT;	
	break;
      default:
	printf("VOCAB default\n ");
	reply.addVocab(VOCAB_FAILED);
    }

    ok = Module::respond(command,reply); // will add message 'not recognized' if not recognized

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

bool DemoAff::open(Searchable& config){
   
  if (config.check("help","if present, display usage message")) {
    printf("Call with --name /module_prefix --file configFile.ini\n");
    return false;
  }

  bool ok;

  ConstString str = config.check("motorboard","/baltaHead","Name of the control board").asString();

  // used a reference here - otherwise check for null doesn't work
  // (cannot copy a null bottle)
  framerate = config.check("FrameRate", yarp::os::Value(20.0), "FrameRate").asDouble();


  ok = InitAff(config);

  Network::init();

  // head control ports
  Property propBoard;

#ifdef BALTAZAR
  cout << str << " oooooooiga" << endl;
  propBoard.put("device", "remote_controlboard");
  propBoard.put("remote" , str);
  cout << str << endl;
  propBoard.put("local", getName("controlboard"));
  dd.open(propBoard);
  if(!dd.view(ipos)){
    cout << "No position control... " << endl;
    return false;
  }
  if(!dd.view(ivel)){
    cout << "No velocity control... " << endl;
    return false;
  }
  if(!dd.view(ienc)){
    cout << "No encoders.. " << endl;
    return false;
  }

  if(!dd.view(ilim)){
    cout << "No limits.. " << endl;
    return false;
  }
  if(!dd.view(iamp)){
    cout << "No amplifier.. " << endl;
    return false;
  }

  if(!dd.view(ipid)){
    cout << "No pid.. " << endl;
    return false;
  }

  ipos->getAxes(&_numHeadAxes);
    
  if (_numHeadAxes == 0){
    cout << "*** Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value."
         << endl;
    // return false;
  }

  ipos->setRefSpeed( 0, 200); ipos->positionMove( 0, 10);

#else
  //iCub
  if (!InitHead() ) return false;
  if (!InitArm() ) return false;

#endif

  // open camshiftplus ports: data and sync
  ok &= port_obj.open("/demoAff/objectinfo");
  //Network::connect("/camshiftplus/all/o","/demoAff/objectinfo");                                                                                                       
  ok &= port_sync.open("/demoAff/synccamshift");
  //Network::connect("/demoAff/synccamshift","/camshiftplus/roi/i");      

  // open camshift ports: data and sync                                                                                                                     
  ok &= port_detector.open("/demoAff/marks");

  return ok;
}


bool DemoAff::close(){
//     _prtImgFloatSalienceIn.close();
//     _prtVctPosOut.close();
//     _configPort.close();

  port_detector.close();

  //port_hand_sync.close();
  //port_hand.close();
  port_sync.close();

  port_obj.close();

    return true;
}

bool DemoAff::interruptModule(){
//    _prtImgFloatSalienceIn.interrupt();
//     _prtVctPosOut.interrupt();
//     _configPort.interrupt();


  port_detector.interrupt();

  //port_hand_sync.interrupt();
  //port_hand.interrupt();
  port_sync.interrupt();

  port_obj.interrupt();

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


	// Read data from CamShiftPlus
	yarp::os::Time::delay(0.04);
	Bottle *input_obj=port_obj.read(false);
	
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
	

     char chin;
     cout << "state: " << state << statename[state] << endl;
     switch (state){
 	case FIRSTINIT:
	  {
	  // go to the init position
#ifdef BALTAZAR
      arm.MoveArm(initArmPos);
	  posmove(initHeadPos);
	  arrived=0;
      while (arm.MovingArm() || !arrived) {
	    arrived=mycheckmotion(0, initHeadPos[0]);
	    arrived*=mycheckmotion(1, initHeadPos[1]);
	    arrived*=mycheckmotion(2, initHeadPos[2]);
	    arrived*=mycheckmotion(3, initHeadPos[3]);
	  }
#else

      ipos->positionMove(initHeadPos);
      ar_ipos->positionMove(initArmPos);
      al_ipos->positionMove(initLArmPos);

      yarp::os::Time::delay(0.1);
      while (!allMotionDone(ipos, _numHeadAxes) || !encNoMotion(ar_ipos, _numRArmAxes,"right arm") || !encNoMotion(al_ipos, _numLArmAxes,"left arm"))
	{
	  printf("waiting on checkmotiondone\n");
          yarp::os::Time::delay(0.04);
	}




#endif	  	  


	  // Keep positions for demo
	  Bottle *input = port_detector.read(true);
	  
	  if(input != NULL)
	  {
	    double cx, cy;
	    while (!(input!=NULL &&  readDetectorInput(input, "pessoa", &cx, &cy)) )
	      input = port_detector.read(true);

	    imgdemopos[0] = cx;
	    imgdemopos[1] = cy;
	    printf("Demo location detected at %f %f\n ",imgdemopos[0],imgdemopos[1]);

	    double errx, erry;
	    double headvel[2];
	    errx=imgdemopos[0];
	    erry=imgdemopos[1];

	    while( !headServo( errx, erry, headvel, (float)30.0) )
	    {
	      printf("err: %f %f  vel: %f %f \n",errx, erry,headvel[0],headvel[1]);
#ifdef BALTAZAR
	      velmove(0.0,0.0,headvel[0],headvel[1]);
#else
          double vels[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          vels[0]=headvel[0];
          vels[2]=headvel[1];          
          ivel->velocityMove(vels);
#endif
	      input = port_detector.read(true);

	      double cx, cy;
	      readDetectorInput(input, "pessoa", &cx, &cy);
	      
	      errx = cx;
	      erry = cy; 
	    }

#ifdef BALTAZAR
	    velmove(0.0,0.0,0.0,0.0);
	    double pos[4];
	    ienc->getEncoder( 0, &posDemo[0]);
	    ienc->getEncoder( 1, &posDemo[1]);
	    ienc->getEncoder( 2, &posDemo[2]);
	    ienc->getEncoder( 3, &posDemo[3]);
#else
        ivel->stop();
        ienc->getEncoders(posDemo);
#endif
	    imgdemopos[0]=0.0;
	    imgdemopos[1]=0.0;
	  }
	  else printf("Using default demo location\n");

#ifdef BALTAZAR
	  posmove(initHeadPos);
          arrived=0;
          while (arm.MovingArm() || !arrived) {
            arrived=mycheckmotion(0, initHeadPos[0]);
            arrived*=mycheckmotion(1, initHeadPos[1]);
            arrived*=mycheckmotion(2, initHeadPos[2]);
            arrived*=mycheckmotion(3, initHeadPos[3]);
          }
#else
      ipos->positionMove(initHeadPos);

      yarp::os::Time::delay(0.5);
      while (!allMotionDone(ipos, _numHeadAxes) ) //|| !encNoMotion(a_ipos, _numArmAxes))
        {
          printf("waiting on checkmotiondone\n");
          yarp::os::Time::delay(0.04);
        }
#endif

	  input = port_detector.read(true);
	  if(input != NULL)
	  {
	      double cx, cy;
  	      while (input==NULL ||  !readDetectorInput(input, "hiro", &cx, &cy))
		input = port_detector.read(true);

	      
	      imgobjpos[0][0] = cx;
	      imgobjpos[0][1] = cy;
	      printf("Obj1 location detected at %f %f\n ",imgobjpos[0][0],imgobjpos[0][1]);
	  
	  }
          else printf("Using default obj1 location\n");
	  
	  input = port_detector.read(true);
	  if(input != NULL)
	    {
	      double cx, cy;
	      while (input==NULL ||  !readDetectorInput(input, "ponto", &cx, &cy))
		input = port_detector.read(true);
	      

	      imgobjpos[1][0] = cx;
	      imgobjpos[1][1] = cy;
	      printf("Obj2 location detected at %f %f\n ",imgobjpos[1][0],imgobjpos[1][1]);
	    }
          else printf("Using default obj2 location\n");
	
	  state = INIT;

		  
	  printf("Check demo and obj locations\n");
	  // Check calibration
	  while (OK_MSG==0) yarp::os::Time::delay(0.04);
	  OK_MSG=0;
	  printf("Move head to: %f %f %f %f\n", posDemo[0],posDemo[1],posDemo[2],posDemo[3]);

#ifdef BALTAZAR
	  posmove(posDemo);
	  arrived=0;
          while (!arrived) {
            arrived=mycheckmotion(0, posDemo[0]);
            arrived*=mycheckmotion(1, posDemo[1]);
            arrived*=mycheckmotion(2, posDemo[2]);
            arrived*=mycheckmotion(3, posDemo[3]);
          }
#else
      ipos->positionMove(posDemo);
     
      yarp::os::Time::delay(0.5);
      while (!allMotionDone(ipos, _numHeadAxes)) // || !encNoMotion(a_ipos, _numArmAxes))
        // LA || !encNoMotion(al_ienc, _numArmAxes)
	{
          printf("waiting on checkmotiondone\n");
          yarp::os::Time::delay(0.04);
        }
#endif

      yarp::os::Time::delay(1.5);

	  double zeropos[] = {0,0};
	  restartTracker( zeropos,.09);
	  printf("Demo object position\n");
	  
	  while (OK_MSG==0) yarp::os::Time::delay(0.04);
          OK_MSG=0;
#ifdef BALTAZAR
	  posmove(initHeadPos);
	  arrived=0;
      while (!arrived) {
          arrived=mycheckmotion(0, initHeadPos[0]);
          arrived*=mycheckmotion(1, initHeadPos[1]);
          arrived*=mycheckmotion(2, initHeadPos[2]);
          arrived*=mycheckmotion(3, initHeadPos[3]);
      }
#else
      ipos->positionMove(initHeadPos);
      
      yarp::os::Time::delay(0.5);
      while (!allMotionDone(ipos, _numHeadAxes)) // || !encNoMotion(a_ipos, _numArmAxes))
        {
          printf("waiting on checkmotiondone\n");
          yarp::os::Time::delay(0.04);
        }
#endif

      yarp::os::Time::delay(1.5);
	  printf("Obj 1 pos\n");
	  restartTracker(imgobjpos[0],0.09);

	  while (OK_MSG==0) yarp::os::Time::delay(0.04);
          OK_MSG=0;

          printf("Obj 2 pos\n");
          restartTracker(imgobjpos[1],0.09);

          while (OK_MSG==0) yarp::os::Time::delay(0.04);
          OK_MSG=0;
	  
	  
     }
      	break;

	   case INIT:
               state=MOVE2DEMO;
#ifdef BALTAZAR
	       posmove(posDemo);
	       arm.MoveArm(initArmPos2);
#else
      ipos->positionMove(posDemo);
      ar_ipos->positionMove(initArmPos2);
      al_ipos->positionMove(initArmPos2);

      yarp::os::Time::delay(0.5);
      while (!allMotionDone(ipos, _numHeadAxes) || !encNoMotion(ar_ipos, _numArmAxes,"right arm") || !encNoMotion(al_ipos, _numArmAxes,"left arm"))
        {
          printf("waiting on checkmotiondone\n");
          yarp::os::Time::delay(0.04);
        }
#endif               
               OK_MSG = 0;  
	       if(OK_MSG == 1)
		 {
		   state = INIT;
		   OK_MSG = 0;
		 }

	     break;
   // 
	   case MOVE2DEMO:
#ifdef BALTAZAR
	       arrived=mycheckmotion(0, posDemo[0]);
	       arrived*=mycheckmotion(1, posDemo[1]);
	       arrived*=mycheckmotion(2, posDemo[2]);
	       arrived*=mycheckmotion(3, posDemo[3]);
           arrived=(arrived && !arm.MovingArm());
#else
	   arrived=allMotionDone(ipos, _numHeadAxes) && encNoMotion(ar_ipos, _numArmAxes,"right arm") && encNoMotion(al_ipos, _numArmAxes,"left arm");
#endif

        if (arrived && OK_MSG){ //!moving( arm, NULL)) {
               printf("Arrived\n");
		 
               restartTracker(imgdemopos,0.09);

	       OK_MSG=0;
	       while (OK_MSG==0) {
		 yarp::os::Time::delay(0.04);
		 printf("On demo position. Is the object correct?\n");
	       }
	       OK_MSG=0;

               processdata.restartDataAcquisition();
               shape1=0; shape2=0;
               state=WAITINGDEMO;
               OK_MSG=0;
        }
	       
	   break;
   // 
	   case WAITINGDEMO:
		   if (processdata.detectActionInit()==1)
		   {
			   processdata.restartDataAcquisition();
			   state = ONDEMO;
			   demoshape=(shape1>shape2?0:1);
		      
			   printf("num shape 1: %d   num shape 2: %d \n",shape1,shape2);
			   printf("ACTION STARTED ON A %s %s OBJECT\n",colors[color],shapes[demoshape]);
		   }
		   else {		           
		     democolor=processdata.getlastcolor();
		     demoshape=processdata.getlastshape();
		     if (demoshape==0) shape1++;
		     else shape2++;
		     printf("Waiting on a  %s %s OBJECT\n",colors[democolor],shapes[demoshape]);
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

			   while (OK_MSG==0) yarp::os::Time::delay(0.04);
                           OK_MSG=0;

		   }
	   break;	    

	   case OBJSELEC:
	       switch (substate){
		   case LOOK:  
#ifdef BALTAZAR	
               posmove(initHeadPos);
#else
               ipos->positionMove(initHeadPos);
#endif
			   substate=MOVING;
		   break;
		   case MOVING:
#ifdef BALTAZAR
			   arrived=mycheckmotion(0, initHeadPos[0]);
			   arrived*=mycheckmotion(1, initHeadPos[1]);
			   arrived*=mycheckmotion(2, initHeadPos[2]);
			   arrived*=mycheckmotion(3, initHeadPos[3]);
#else
			   arrived=allMotionDone(ipos, _numHeadAxes);

#endif
			   printf("Moving head\n");
			   if (arrived) {
				substate=GETPROP;
				printf("Restart tracker\n");
				yarp::os::Time::delay(1.5);
				restartTracker(imgobjpos[numObservedObj],0.09);

				while (OK_MSG==0) yarp::os::Time::delay(0.04);
				OK_MSG=0;

				processdata.restartDataAcquisition();
			   }
		   break; 

		   case GETPROP:
		     if (OK_MSG!=0) {
		       OK_MSG=0;
		       printf("keep obj %d: %d %d\n",numObservedObj,
			      processdata.getlastcolor(), processdata.getlastshape());

		       colorobj[numObservedObj]=processdata.getlastcolor();
		       shapeobj[numObservedObj]=processdata.getlastshape();
			   //sizeobj[numObservedObj]=size;

			   numObservedObj++;
			
			   if (numObservedObj==numObj2Observe)
				   state=DECISION;
			   else 
				   substate=LOOK;
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
	     for (int i=0; i<numObservedObj;i++) {
	       for (int j=0; j<3; j++) {
		 ev[0]=j; ev[1]= shapeobj[i]; ev[2]=-1;ev[3]=-1;
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
#ifdef BALTAZAR
	       state=REACHING;
#else
	       switch (selectedaction){
	       case GRASP: state=GRASPING; nMoves=0; break;
	       case TAP: state=TAPPING; nMoves=0; break;
	       case TOUCH: state=GRASPING; break;
	       }

#endif
	       if (selectedobj==0)
		 selectArm('r');
	       else if ( selectedobj==1)
		 selectArm('l');

	       objposreach[0] = imgobjpos[selectedobj][0];
               objposreach[1] = imgobjpos[selectedobj][1];
	       printf("Selected a %s on the %s %s located at pos %d <%g %g>\n", actions[selectedaction],
		      colors[colorobj[selectedobj]], shapes[shapeobj[selectedobj]],
		      selectedobj,objposreach[0],objposreach[1]);
	     }
	     while (OK_MSG==0) yarp::os::Time::delay(0.04);
	     OK_MSG=0;
	     }  
	   break;

	   case REACHING:
	     {
	       int detHand;
	       double handpos[2];
	       Bottle *input = port_detector.read(true);

	       if(input != NULL)
		 {
		   double cx, cy;
		   if (readDetectorInput(input, "pessoa", &cx, &cy)) {
		     handpos[0] = cx; 
		     handpos[1] = cy; 
		     detHand = 1;
		     printf("Hand detected at %f %f\n ",handpos[0],handpos[1]);
		   }
		   else {
		     detHand = 0;
		     printf("Hand not detected\n");
		   }
		 }
	       else
		 {
		   detHand = 0;
		   printf("Hand not detected\n");
		 }


	    if (!detHand) { // We lost track of the hand
#ifdef BALTAZAR
	         arm.StopArm();
#else
             a_ivel->stop();
	     // LA al_ivel->stop();
#endif
		 state=INIT;
	    }
	    else {
	         // Compute vsError
	      printf("Height: %g\n", height);
#ifdef BALTAZAR
	      if (arm.VisualServoing((objposreach[0]-handpos[0]),-(objposreach[1]-handpos[1]),height))
#else
		if (1)
#endif
		       switch (selectedaction){
 		        case GRASP: state=GRASPING; break;
 		        case TAP: state=TAPPING; break;
 		        case TOUCH: state=TOUCHING; break;
		       }
		 
	      printf("I should be moving ... %g %g %g %g\n",objposreach[0],objposreach[1],handpos[0],handpos[1]);
	    }
	     }
 	break;
 	case GRASPING:
	  {

#ifdef BALTAZAR
	    if (arm.Grasp())
		      state=UNGRASP;
#else
	    if (encNoMotion(a_ipos, _numArmAxes,"not known")) {
	      if (nMoves<numGraspVecArm) {
		printf("Performing grasp. Go to step %d/%d\n", nMoves,numGraspVecArm);
		a_ipos->setRefSpeeds(graspVecVelArm[nMoves]);
		a_ipos->positionMove(graspVecArm[nMoves]);
		nMoves++;
	      }
	      else {
		state=INIT;
	      }
	    }
	    else {
	      printf("GGG: \n");
	      yarp::os::Time::delay(0.04);
	    }
#endif
	  }
 	break;
        case UNGRASP:
	  {
            printf("eliminar cuanto antes");
#ifdef BALTAZAR
	    if (arm.Ungrasp())
		      state=INIT;
#else
#endif
	  
	  }
        break;
 	case TAPPING:
	  {
            printf("eliminar cuanto antes");
#ifdef BALTAZAR
	    if (arm.Tap())
 			state=UNTAP;
#else
	    if (encNoMotion(a_ipos, _numArmAxes,"not known")) {
              if (nMoves<numTapVecArm) {
                printf("Performing grasp. Go to step %d/%d\n", nMoves,numTapVecArm);
		a_ipos->setRefSpeeds(tapVecVelArm[nMoves]);
                a_ipos->positionMove(tapVecArm[nMoves]);
		nMoves++;
	      }
              else {
                state=INIT;
              }
            }
            else {
	      yarp::os::Time::delay(0.04);
            }

#endif
	  }
 	break;
        case UNTAP:
	  {
            printf("eliminar cuanto antes");
#ifdef BALTAZAR
	    if (arm.FinalTap())
		        state=INIT;
#else
#endif
	   }
        break;
 	case TOUCHING:
	  {
            printf("eliminar cuanto antes");
#ifdef BALTAZAR
	    if (arm.Touch())
 			state=UNTOUCH;
#else
#endif
	  }
 	break;
        case UNTOUCH:
	  {
            printf("eliminar cuanto antes");
#ifdef BALTAZAR
	    if (arm.FinalTouch())
		        state=INIT;
#else
#endif
	    }
       break;
		case DETECTMARKS:
			{
				//read mark detection from port
				double errx, erry;
				double headvel[2];

#ifdef BALTAZAR
				if( headServo( errx, erry, headvel, (float)0.1) )
#else
#endif
				{
					//read headpos
					double pos[2];
					saveLocation( pos, detectedlocations);
					detectedlocations++;
				}	
				if(detectedlocations==MAXLOCATIONS)
					state = INIT;
			}
			break;

     }


    return true;
}

bool DemoAff::restartTracker(double *pos, double width) {
	Bottle& output = port_sync.prepare();
	output.clear();
	output.addDouble(pos[0]);
      	output.addDouble(-pos[1]);
	output.addDouble(width);
	output.addDouble(width);
	cout << "writing " << output.toString().c_str() << " " << output.size() << endl;
	port_sync.write();

	// flag to signal tracker has been restarted
	trrest=1;
	prevwo=10000; prevho=10000;
}

#ifdef BALTAZAR
int DemoAff::posmove(double *pos)
{
  bool ret = false;
  ipos->setRefSpeed( 3, 200);ipos->positionMove( 3, pos[3]);
  ipos->setRefSpeed( 0, 200); ipos->positionMove( 0, pos[0]);
  ipos->setRefSpeed( 1, 200); ipos->positionMove( 1, pos[1]); 
  ipos->setRefSpeed( 2, 200);ipos->positionMove( 2, pos[2]);

  //  while(!ret)
  //  {
  //    printf("checkmotion %f %f\n",pos[3],pos[4]);
  //    Time::delay(0.01);
  //    ret = mycheckmotion( 0, pos[0]);
  //    ret += mycheckmotion( 1, pos[1]);
  //    ret += mycheckmotion( 2, pos[2]);
  //    ret += mycheckmotion( 3, pos[3]);
  //  }
  printf("done\n");

  return ret;
}

int DemoAff::velmove(double v1, double v2, double v3, double v4)
{
  ivel->velocityMove(0, v1);
  ivel->velocityMove(1, v2);
  ivel->velocityMove(2, v3);
  ivel->velocityMove(3, v4);
}

int DemoAff::mycheckmotion( int axis, double target)
{
  double curr;
  double min, max;

  /*                                                                                                               
  // check if target over the limit                                                                                        ilim->getLimits(axis, &min, &max);                                                                                    if( (target>max) || (target<min) )                                                             
        return 1;                                                                                                     
  */

  ienc->getEncoder( axis, &curr);
  //printf("pos: %f ref: %f error: %f\n", curr, target, (curr-target)*(curr-target));
  if( ((curr-target)*(curr-target)) < 0.1 )
    return 1;
  else
    return 0;
}
#else
#endif


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
	

#ifdef BALTAZAR
#else
bool DemoAff::InitHead(){

	Property propBoard;

	propBoard.put("device", "remote_controlboard");
	propBoard.put("remote" , "/icub/head");
	//cout << strRemoteControlboard << endl;
	propBoard.put("local", "/demoAff/head");
	dd.open(propBoard);
	if(!dd.view(ipos)){
		cout << "No position control... " << endl;
		return false;
	}
	if(!dd.view(ivel)){
		cout << "No velocity control... " << endl;
		return false;
	}
	if(!dd.view(ienc)){
		cout << "No encoders.. " << endl;
		return false;
	}

	if(!dd.view(ilim)){
		cout << "No limits.. " << endl;
		return false;
	}
	if(!dd.view(iamp)){
		cout << "No amplifier.. " << endl;
		return false;
	}

	if(!dd.view(ipid)){
		cout << "No pid.. " << endl;
		return false;
	}

    ipos->getAxes(&_numHeadAxes);
    if (_numHeadAxes == 0){
        cout << "*** Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value." << endl;
        // return false;
    }

    for (int i=0; i<_numHeadAxes; i++){
        iamp->enableAmp(i);
        ipid->enablePid(i);
        //ipos->positionMove( i, 0);
        ivel->velocityMove( i, 0);
        ivel->setRefAcceleration(i, 100);
        ipos->setRefSpeed(i, 20);
        ipos->setRefAcceleration(i, 100);
    }

    return true;
}
bool DemoAff::InitArm(){

  Property propBoard, LpropBoard;

	propBoard.put("device", "remote_controlboard");
	propBoard.put("remote" , "/icub/right_arm");
	
	propBoard.put("local", "/demoAff/rarm");
	ar_dd.open(propBoard);
	if(!ar_dd.view(ar_ipos)){
		cout << "No position control... " << endl;
		return false;
	}
	if(!ar_dd.view(ar_ivel)){
		cout << "No velocity control... " << endl;
		return false;
	}
	if(!ar_dd.view(ar_ienc)){
		cout << "No encoders.. " << endl;
		return false;
	}

	if(!ar_dd.view(ar_ilim)){
		cout << "No limits.. " << endl;
		return false;
	}
	if(!ar_dd.view(ar_iamp)){
		cout << "No amplifier.. " << endl;
		return false;
	}

	if(!ar_dd.view(ar_ipid)){
		cout << "No pid.. " << endl;
		return false;
	}

    ar_ipos->getAxes(&_numRArmAxes);
    if (_numRArmAxes == 0){
        cout << "*** Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value." << endl;
        // return false;
    }

    for (int i=0; i<_numRArmAxes; i++){
        ar_iamp->enableAmp(i);
        ar_ipid->enablePid(i);
        //a_ipos->positionMove( i, 0);
        ar_ivel->velocityMove( i, 0);
        ar_ivel->setRefAcceleration(i, 100);
        ar_ipos->setRefSpeed(i, 20);
        ar_ipos->setRefAcceleration(i, 100);
    }


    printf("Right arm done!!!!!!");
  // Init left arm
	LpropBoard.put("device", "remote_controlboard");
	LpropBoard.put("remote" , "/icub/left_arm");
	
	LpropBoard.put("local", "/demoAff/larm");
	al_dd.open(LpropBoard);
	if(!al_dd.view(al_ipos)){
		cout << "No position control... " << endl;
		return false;
	}
	if(!al_dd.view(al_ivel)){
		cout << "No velocity control... " << endl;
		return false;
	}
	if(!al_dd.view(al_ienc)){
		cout << "No encoders.. " << endl;
		return false;
	}

	if(!al_dd.view(al_ilim)){
		cout << "No limits.. " << endl;
		return false;
	}
	if(!al_dd.view(al_iamp)){
		cout << "No amplifier.. " << endl;
		return false;
	}

	if(!al_dd.view(al_ipid)){
		cout << "No pid.. " << endl;
		return false;
	}

    al_ipos->getAxes(&_numLArmAxes);
    if (_numLArmAxes == 0){
        cout << "*** Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value." << endl;
        // return false;
    }

    for (int i=0; i<_numLArmAxes; i++){
        al_iamp->enableAmp(i);
        al_ipid->enablePid(i);
        //al_ipos->positionMove( i, 0);
        al_ivel->velocityMove( i, 0);
        al_ivel->setRefAcceleration(i, 100);
        al_ipos->setRefSpeed(i, 20);
        al_ipos->setRefAcceleration(i, 100);
    }

    return true;



}

bool DemoAff::allMotionDone(yarp::dev::IPositionControl *tmp_ipos, int naxes){

  bool ended, endvec[128];


  ended=true;
  printf("all motion done: ");
  for (int i=0; i<naxes; i++)
    {
      tmp_ipos->checkMotionDone(i,&endvec[i]);
      printf("%d ",endvec[i]);
      ended=ended && endvec[i];
    }

  printf("\n");
  return ended;
}

bool DemoAff::allSpeedZero(yarp::dev::IEncoders *tmp_enc, int naxes){
  bool ended;
  double  velvec[32];

  tmp_enc->getEncoderSpeeds(velvec);

  ended=true;
  for (int i=0; i<naxes; i++)
    {
      printf("%g ",velvec[i]);
      ended=ended && (fabs(velvec[i])<0.1);
    }

  printf("\n");
  return ended;
}

bool DemoAff::encNoMotion(yarp::dev::IPositionControl *tmp_pos, int njoint){
  bool done,tdone;
  tmp_pos->checkMotionDone(&done);
  if (!done)
    for (int i=0; i< njoint; i++) {
      tmp_pos->checkMotionDone(i, &tdone);
      printf("%d ",tdone);
    }
    
  return done;
  }


bool DemoAff::encNoMotion(yarp::dev::IPositionControl *tmp_pos, int njoint, char *msg){
  bool done,tdone;
  tmp_pos->checkMotionDone(&done);
  if (!done) {
    printf("%s: ",msg);
    done=true;
    for (int i=0; i< njoint; i++) {
      tmp_pos->checkMotionDone(i, &tdone);
      printf("%d ",tdone);
      if (i!=7)
	done=done&tdone;
    }
  }
    
  return done;
  }


bool DemoAff::encNoMotion(yarp::dev::IEncoders *tmp_enc, int naxes){
  bool ended;
  double  tmpvec[32];
  bool tmpres[32];

  tmp_enc->getEncoders(tmpvec);

  ended=true;
  for (int i=0; i<naxes; i++)
    {
      printf("%g ",tmpvec[i]);
      ended=ended && (fabs(encvec[i] - tmpvec[i])<encth[i]);
      tmpres[i]=fabs(encvec[i] - tmpvec[i])<encth[i];
      encvec[i]=tmpvec[i];
    }
  printf("\n");

  printf("Moving joints are: ");
  for (int i=0; i<naxes; i++)
    {
      if (!(tmpres[i]))
	printf("%d ",i);
    }

  printf("\n");
  if (ended)
    {
      for (int i=0; i<naxes; i++)
	encvec[i]=10000;
    }
  printf("encNoMotion return %d\n",ended);
  return ended;
}


bool DemoAff::selectArm(char sarm){

  if (sarm=='r') {
    a_ipos=ar_ipos;
    a_ivel=ar_ivel;
    a_ienc=ar_ienc;
    a_iamp=ar_iamp;
    a_ipid=ar_ipid;
    a_ilim=ar_ilim;
    _numArmAxes=_numRArmAxes;
  }
  else if (sarm=='l'){
    a_ipos=al_ipos;
    a_ivel=al_ivel;
    a_ienc=al_ienc;
    a_iamp=al_iamp;
    a_ipid=al_ipid;
    a_ilim=al_ilim;
    _numArmAxes=_numLArmAxes;
  }


}


#endif

