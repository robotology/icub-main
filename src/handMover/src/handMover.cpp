
#include <stdio.h>
#include <iCub/handMover.h>

#include <iostream>
#include <fstream>


// Initialize robot
enum STATE
{
  OPEN,
  CLOSE,
  OPENING,
  CLOSING,
  IDLE
} handstate;

//added here due to version problem
#define VOCAB_FAILED VOCAB4('f','a','i','l')
#define VOCAB_QUIT VOCAB4('q','u','i','t')
#define VOCAB_OK VOCAB2('o','k')
#define VOCAB_OPEN VOCAB4('o','p','e','n')
#define VOCAB_CLOSE VOCAB4('c','l','o','s')

double openpos[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
double closepos[] = { 5, 5, 5, 5, 5, 5, 5, 5, 5};
char add[] = { 7, 8, 9, 10, 11, 12, 13, 14, 15};
int njtshand = 9;

HandMover::HandMover(){
  handstate = IDLE;
}

HandMover::~HandMover() { 
}


bool HandMover::respond(const Bottle &command,Bottle &reply){
	bool ok = false; // command executed successfully

	switch (command.get(0).asVocab()) {
	  case VOCAB_QUIT:
		  reply.addVocab(VOCAB_QUIT);
		  ok = true;
		  break;
	  case VOCAB_OPEN:
		  reply.addVocab(VOCAB_OK);
		  handstate = CLOSE;
		  ok = true;
		  break;
	  case VOCAB_CLOSE:
		  reply.addVocab(VOCAB_CLOSE);
		  handstate = OPEN;
		  ok = true;
		  break;
	  default:
		  reply.addVocab(VOCAB_FAILED);
	}

	ok = Module::respond(command,reply); // will add message 'not recognized' if not recognized

	return ok;
}


bool HandMover::open(Searchable& config){
  bool ok;

	if (config.check("help","if present, display usage message")) {
		printf("Call with --name /module_prefix --file configFile.ini\n");
		return false;
	}

	Network::init();

	//arm
	Property armpropBoard;
	ConstString strarm = config.check("motorboard","/RIGHTArm","Name of the ARM control board").asString();

	armpropBoard.put("device", "remote_controlboard");
	armpropBoard.put("remote" , strarm);

	armpropBoard.put("local", getName("controlboardarm"));
	armdd.open(armpropBoard);
	if(!armdd.view(armipos)){
		cout << "No position control... " << endl;
		return false;
	}
	if(!armdd.view(armivel)){
		cout << "No velocity control... " << endl;
		return false;
	}
	if(!armdd.view(armienc)){
		cout << "No encoders.. " << endl;
		return false;
	}
	if(!armdd.view(armilim)){
		cout << "No limits.. " << endl;
		return false;
	}
	if(!armdd.view(armiamp)){
		cout << "No amplifier.. " << endl;
		return false;
	}
	if(!armdd.view(armipid)){
		cout << "No pid.. " << endl;
		return false;
	}
	armipos->getAxes(&_numArmAxes);

	if (_numArmAxes == 0){
		cout << "Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value."
			<< endl;
		// return false;
	}
	for(int cnt=0;cnt<_numArmAxes;cnt++) {
		armivel->setRefAcceleration( cnt, 20);
		armipos->setRefAcceleration( cnt, 10);
		armipos->setRefSpeed( cnt,10);		
	}

	return ok;
}


bool HandMover::close(){


	return true;
}

bool HandMover::interruptModule(){


	return true;
}



bool HandMover::updateModule(){

  switch (handstate) 
    {
    case OPEN:
      handstate = OPENING;
      break;
    case OPENING:
      break;
    case CLOSE:
      handstate = CLOSING;
      break;
    case CLOSING:
      break;
    case IDLE:
      break;
    default:
      printf("\n\n \t WRONG STATE\n");
    }

  //armipos->positionMove(initArmPos);
	
	return true;
}


int HandMover::moveto(double *targetpos)
{
  for(int cnt = 0; cnt<njtshand;cnt++)
    armipos->positionMove( add[cnt], targetpos[cnt]);

  return 1;
}

int HandMover::hasarrived(double *targetpos)
{
  double encs[16];

  armienc->getEncoders( (double*)encs );

  for(int cnt = 0; cnt<njtshand;cnt++)
    {
      if( fabs(encs[add[cnt]]-targetpos[cnt]) > 1)
	return 0;
    }

  return 1;

}
