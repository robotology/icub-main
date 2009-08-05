#include <stdio.h>
#include <iCub/demoAff.h>

#include <iCub/processobjdata.h>

enum stateenum {
  FIRSTINIT,
  IDLE,
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
char statename[NUMSTATES][20]={"firstinit","idle","getobjdesc","waitingdemo","ondemo","objselec","decision","reaching","tapping","grasping","touching","ungrasp","untap","untouch","detectmarks"};

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
  detectedlocations=0;
  printf("After constructor\n");
  trrest=0;
  OK_MSG=0;
  detectedlocations=0;
}

DemoAff::~DemoAff(){ 
  port_obj.close();
  port_sync.close();


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

  // locate configuration file
  ResourceFinder rf;        
  if (config.check("context")){
    rf.setDefaultContext(config.find("context").asString());
  }
  if (config.check("from")){
    rf.setDefaultConfigFile(config.find("from").asString());
  }
  else if (config.check("file")){
    rf.setDefaultConfigFile(config.find("file").asString());
  }
  else{
    rf.setDefaultConfigFile("icubDemoAff.ini");
  }
  rf.configure("ICUB_ROOT",0,NULL);
  Property prop(rf.toString());
  prop.fromString(config.toString(), false);
  prop.setMonitor(config.getMonitor());
   

  bool ok;


  ok = InitAff(prop);

  // open camshiftplus ports: data and sync
  ok &= port_eff.open("/demoAffv2/effect");
  ok &= port_sync.open("/demoAffv2/synccamshift");
  ok &= port_descriptor.open("/demoAffv2/objsdesc");
  ok &= port_askobj.open("/demoAffv2/");
  ok &= port_primitives.open("/demoAffv2/motioncmd");
  ok &= port_gaze.open("/demoAffv2/gazecmd");
  ok &= port_behavior_in.open("/demoAffv2/behavior:i");
  ok &= port_behavior_out.open("/demoAffv2/behavior:o");
  ok &= port_output.open("/demoAffv2/out");
  ok &= port_emotions.open("/demoAffv2/out");
  return ok;
}


bool DemoAff::close(){

  port_eff.close();
  port_sync.close();
  port_descriptor.close();
  port_askobj.close();
  port_primitives.close();
  port_gaze.close();
  port_behavior_in.close();
  port_behavior_out.close();
  port_output.close();
  port_emotions.close();

  return true;
}

bool DemoAff::interruptModule(){

  port_eff.interrupt();
  port_sync.interrupt();
  port_descriptor.interrupt();
  port_askobj.interrupt();
  port_primitives.interrupt();
  port_gaze.interrupt();
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
      Bottle *input_obj=port_behavior_in.read(true);
    
      if (input_obj!=NULL) {
	int cmd;
	
	cmd = input_obj->get(0).asInt();	
	if (cmd==1)
	  state=INIT;
	else {
	  cout << "Got " << input_obj.toSting.c_str() << "from behavior. Ignoring." << endl;

	  state=IDLE;
	}
      }    
    break;
  
  case INIT:
    {
      // Ask for segmentation
      Bottle& output = port_askobj.prepare();
      output.clear();
      output.addVocab(Vocab::encode("do"));
      output.addVocab(Vocab::encode("seg"));
      cout << "sending to askobj: " << output.toString().c_str() << " " << output.size() << endl;
      port_askobj.write();
      
      // Set expression
      Bottle& output = port_emotions.prepare();
      output.clear();
      output.addVocab(Vocab::encode("set all neutral"));
      cout << "sending expression: " << output.toString().c_str() << " " << output.size() << endl;
      port_askobj.write();
      
      int trials=10;
      
      state=IDLE;
      while (trials) {
	yarp::os::Time::delay(0.5);      
	Bottle *input_obj=port_descriptor.read(false);
	
	if (input_obj!=NULL) {       
	  getObjDesc(input_obj);
	  trials=-1;
	  
	  if (num_obj<=0){
	    // Stop aff demo and notify behavior
	    Bottle& output = port_beharior_out.prepare();
	    output.clear();
	    output.addVocab(Vocab::encode("off"));
	    port_behavior_out.write();
	    
	    state=IDLE;
	  }
	  else {
	    // Select object closer to the center
	    float max_dist=10000;
	    for (int i=0; i<num_obj; i++) {
	      float dist = objpos[i][0]*objpos[i][0] + objpos[i][1]*objpos[i][1];
	      if (dist<max_dist) {
		selobj=i;
		max_dist=dist;
	      }
	    }	
	    
	  }
	}
	else trials--;
      }
      
      if (trials==-1 && num_obj>0) {
	// Init tracker on fixated obj (if any)
	// We have to re implement the init tracker to send the histogram 
	// and the region provided by the segmentation
	restartTracker(objpos[selobj],objsize[selobj]);
	demoshape=shape[selobj];
	demosize=size[selobj];
	democolor=color[selobj];
	
	
	while (processdata.detectActionInit()!=1) {
	  yarp::os::Time::delay(0.1);                 
	}
	processdata.restartDataAcquisition();
	
	printf("ACTION STARTED ON A %s %s OBJECT\n",colors[color],shapes[demoshape]);
	
	state = ONDEMO;
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
    else
      yarp::os::Time::delay(0.1);                 
    break;	    
    
  case OBJSELEC: 
    {
      Bottle& output = port_gaze_out.prepare();
      output.clear();
      output.addDouble(table_az);
      output.addDouble(table_el);
      port_behavior_out.write();
      
      
      yarp::os::Time::delay(0.1);                 
      
      bool done=false;
      Bottle *input_obj;
      while (!done) {
	input_obj=port_descriptor.read(true);
	done = input_obj->get(0).asInt()==5;	
      }
      
      // Ask for segmentation
      Bottle& output = port_askobj.prepare();
      output.clear();
      output.addVocab(Vocab::encode("do"));
      output.addVocab(Vocab::encode("seg"));
      cout << "sending to askobj: " << output.toString().c_str() << " " << output.size() << endl;
      port_askobj.write();
      
      // Set expression
      Bottle& output = port_emotions.prepare();
      output.clear();
      output.addVocab(Vocab::encode("set all neutral"));
      cout << "sending expression: " << output.toString().c_str() << " " << output.size() << endl;
      port_askobj.write();
      
      int trials=10;
      
      while (trials) {
	yarp::os::Time::delay(0.5);      
	Bottle *input_obj=port_descriptor.read(false);
	
	if (input_obj!=NULL) {       
	  getObjDesc(input_obj);
	  
	  if (num_obj<=0){
	    // Stop aff demo and notify behavior
	    Bottle& output = port_beharior_out.prepare();
	    output.clear();
	    output.addVocab(Vocab::encode("off"));
	    port_behavior_out.write();
	    
	    trials=-1;
	    state=IDLE;
	    
	    // Put sad expression TBD move head a bit do sth damn it!
	    
	  }
	  else {
	    // Select object according to effect
	    trials=-1;
	    state=DECISION;
	  }
	}
	else trials--;
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
	       printf("\n\n\n\n\n\n\n\n\n\n\n\n\n");
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




 bool DemoAff::getObjDesc(Bootle * input_obj) {


   if (input_obj->size()<2)
     numobj=0;
   else {
     numobj=input_obj->get(1).asInt();
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
       
       size[nobj}= data[2]*data[3];
       printf("Obj %d at %g %g %d %d\n",objpos[nobj][0],objpos[nobj][1],color[nobj],shape[nobj]);
     }
   }
 }


bool DemoAff::readEffect() { 

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
 

