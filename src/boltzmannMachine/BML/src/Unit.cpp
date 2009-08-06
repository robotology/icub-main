// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -
#include <iCub/Unit.h>
#include <conio.h>

/**
*default constructor
*/
Unit::Unit(){}

/**
*default destructor
*/
Unit::~Unit(){}

/** 
* constructor
*/
Unit::Unit(std::string name){
	this->name=name;
	this->bias=2.1972; //the inverse of the lower Pk constraint with T=1
	this->T=1;
	this->stateChanged=true;
	this->connectionChanged=true;
	this->evolveBinary=true;
	this->evolveStochastic=true;
	yarp::os::Random *rnd=new Random();
	yarp::math::Rand *rndGenerator=new Rand;
	double value=double(rand())/RAND_MAX;
	rndGenerator->init(2);
	rndGenerator->scalar(0,1);
	state=(int)ceil(value-0.8);
	//state=0;
	//cout<<"RnDGenerator"<<rndGenerator->scalar()<<endl;
	//state=(int)ceil(rndGenerator->scalar()-0.5);
	cout<<"Unit state:"<<state<<endl;
	stochasticThrHIGH=0.8;
	stochasticThrLOW=0.2;
}

/**
* constructor
*/
Unit::Unit(std::string name,int state){
	this->name=name;
	this->bias=2.1972; //the inverse of the lower Pk constraint with T=1
	this->T=1;
	this->stateChanged=true;
	this->connectionChanged=true;
	this->evolveBinary=true;
	this->evolveStochastic=true;
	yarp::os::Random *rnd=new Random();
	yarp::math::Rand *rndGenerator=new Rand;
	double value=double(rand())/RAND_MAX;
	//rndGenerator->init(value);
	state=state;
	//cout<<"RnDGenerator"<<rndGenerator->scalar()<<endl;
	//state=(int)ceil(rndGenerator->scalar()-0.5);
	
	stochasticThrHIGH=0.8;
	stochasticThrLOW=0.2;

	//initialise the row of connection with the number of internal connections
	//this->addConnectionWeight(connectionsTOT);
}

/*constructor*/
/*_DEPRECATED Unit::Unit(std::string name,int state){
	this->name=name;
	this->bias=2.1972; //the inverse of the lower Pk constraint with T=1
	this->T=1;
	this->stateChanged=true;
	this->connectionChanged=true;
	this->evolveBinary=true;
	this->evolveStochastic=true;
	this->state=state;
	stochasticThrHIGH=0.8;
	stochasticThrLOW=0.2;
}*/


std::string Unit::getName(){
	return name;
}

void Unit::addConnection(Connection *connection){
	connectionList[connection->getName()]=*connection;
	//cout<<"unit "<<this->name<<" notifies connection name: "<< connection->getName()<<" number of WEIGHT: "<<connectionList[connection->getName()].getWeight()<<endl;
}

/**
* add a series of connection Weight to the listWeight
*/
void Unit::addConnectionWeight(int nConnectionWeight){
	for(int i=0;i<nConnectionWeight;i++)
		weightList.push_back(0);
}

/**
* add a series of connection Weight to the listWeight
*/
void Unit::addConnectionWeight(int nConnectionWeight,int weight){
	for(int i=0;i<nConnectionWeight;i++)
		weightList.push_back(weight);
}


/**
* set the weight(int) in the row of connections in a precise position
*/
void Unit::setConnectionWeight(int pos, int weight){
	list<int>::iterator iter=this->weightList.begin();
	for(int i=0;i<pos;i++){
		iter++;
	}
	*iter=weight;
}
/**
* get the weight(int) in the row of connections in a precise position
*/
int Unit::getConnectionWeight(int pos){
	list<int>::iterator iter=this->weightList.begin();
	for(int i=0;i<pos;i++){
		iter++;
	}
	return *iter;
}

int Unit::getBias(){
	return bias;
}

int Unit::getState(){
	return state;
}

/**
* calculates the local energy of a single unit
*/
int Unit::getDEnergy(){
	//if(connectionChanged){
		int sumConnections=0;
		map<std::string,Connection>::iterator iterC;
		//cout<<"unit "<<this->name<<" number of connections: "<<this->connectionList.size()<<endl;
		for(iterC=connectionList.begin(); iterC!=connectionList.end();iterC++){		
			sumConnections+=iterC->second.getWeight()*this->state;
			//cout<<"sumConnection+= "<<iterC->second.getWeight()*this->state<<"="<<sumConnections<<endl;
		}
		DEnergy=sumConnections-bias;
		/*if(evolveBinary){
			if(DEnergy>0){
				this->state=1;
				this->stateChanged=true;
			}
		}*/
		//this->connectionChanged=false; //set the flag to false in order to prevent over computation of this value
	//}
	return DEnergy; 
}

void Unit::calculateDEnergy(){
	
}

double Unit::getProbFired(){
	//if(connectionChanged){
		int E=this->getDEnergy();
		double A=exp((double)E/(T*-1));
		double D=1+A;
		probFired=1/D;
		//cout<<"T:"<<T<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;
		if(evolveStochastic){
			if(probFired>stochasticThrHIGH){
				this->state=1;
				this->stateChanged=true;
			}
			if(probFired<stochasticThrLOW){
				this->state=0;
				this->stateChanged=true;
			}
		}
	//}
	return probFired;
}

/**
* code the class into a std::string
*/
std::string Unit::toString(){
	//_____
	char number[3];
	int n=sprintf(number,"%d",this->getState());
	std::string i_str(number,n);
	//_____
	return this->name+"("+i_str+")";
}

/**
* the state of this units is updated based on the choosen rules
*/
void Unit::evolve(int rule){
	if(connectionChanged){
		if(rule==BINARY_THR){
			this->evolveBinary=true;
			this->getDEnergy();
		}
		else if(rule==PROBAB_THR){
			this->evolveStochastic=true;
			this->getProbFired();
		}
		else if(rule==BOTH_RULES){
			this->evolveStochastic=true;
			this->evolveBinary=true;
			this->getDEnergy();
			this->getProbFired();
		}
		else{
			cout<<"No Rule identified!"<<endl;
		}
	}
}