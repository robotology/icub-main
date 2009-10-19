// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -
#include <iCub/Row.h>
//#include <conio.h>



/**
*default constructor
*/
Row::Row(){}
/**
*default destructor
*/
Row::~Row(){}

/**
*constructor of n stardard units and null connections 
* @param name of row
* @param  number of units
*/
Row::Row(std::string name,int nUnits){
	this->name=name;
	//allocation of n units
	for(int i=0;i<nUnits;i++){
		//_____
		char number[3];
		int n=sprintf(number,"%d",i);
		std::string i_str(number,n);
		//_____
		//create a new unit
		Unit* unit=new Unit(this->getName()+"U"+i_str); // L1R1U2 means Row1-Unit2 or L1R1U2 Layer1-Row1-Unit2
		unitList.insert(pair<std::string,Unit>(unit->getName(),*unit));
	}
	//allocation of all the possible connection between n units (n!/2) with nill weight
	/*map<std::string,Unit>::iterator iterI;
	map<std::string,Unit>::iterator iterJ;
	//map<std::string,Connection>::iterator findC;
	for(iterI=unitList.begin(); iterI!=unitList.end();iterI++){
		for(iterJ=iterI,iterJ++; iterJ!=unitList.end();iterJ++){
			//CR1U1R1U2 means Connection(Row1-Unit1,Row1-Unit2)
			double value=double(rand())/RAND_MAX;
			int weight_rnd=(int)(value*200);
			//cout<<"WEight "<<weight_rnd<<endl;
			//create a new connection
			Connection *connection=new Connection("C"+iterI->second.getName()+iterJ->second.getName(),weight_rnd); 
			//set the references to the connection in the units
			iterI->second.addConnection(connection);
			iterJ->second.addConnection(connection);
			connectionList.insert(pair<std::string,Connection>(connection->getName(),*connection));	
		}
	}*/
}

void Row::addUnit(Unit unitA){
	unitList.insert(pair<std::string,Unit>(unitA.getName(),unitA));
}

void Row::addConnection(Unit unitA, Unit unitB, int weight){
	Connection* connection=new Connection(unitA.getName(),unitB.getName(), weight);
	unitA.addConnection(connection);
	unitB.addConnection(connection);
	connectionList.insert(pair<std::string, Connection>(unitA.getName()+unitB.getName(),*connection));
}


int Row::getEnergy(){
	int sumBiases=0;
	int sumConnections=0;
	map<std::string,Unit>::iterator iterI,findI;
	map<std::string,Unit>::iterator iterJ,findJ;
	map<std::string,Connection>::iterator findC;
	for(iterI=unitList.begin(); iterI!=unitList.end();iterI++){
		for(iterJ=iterI; iterJ!=unitList.end();iterJ++){
			findC=connectionList.find(iterI->second.getName()+iterJ->second.getName());
			//findI=unitList.find(iterI->second.getName());
			//findJ=unitList.find(iterJ->second.getName());
			sumConnections+=iterI->second.getState()*iterJ->second.getState()*findC->second.getWeight();
			sumBiases+=iterI->second.getBias()*iterI->second.getState();
		}
	}
	energy=sumBiases-sumConnections;
	return energy; 
}

/*get the iterator pointing at the beginning of thr connectionList*/
map<std::string,Connection>::iterator Row::getConnectionListIteratorBegin(){
	return connectionList.begin();
}

 /*get the iterator pointing at the end of thr connectionList*/
map<std::string,Connection>::iterator Row::getConnectionListIteratorEnd(){
	return connectionList.end();
}

/*get the iterator pointing at the beginning of the unitList*/
map<std::string,Unit>::iterator Row::getUnitListIteratorBegin(){
	return unitList.begin();
}

/*get the iterator pointing at the end of the unitList*/
map<std::string,Unit>::iterator Row::getUnitListIteratorEnd(){
	return unitList.end();
}

/*code the class into a std::string*/
std::string Row::toString(){
	std::string output="";
	//print all the unit out
	map<std::string,Unit>::iterator iterU;
	for(iterU=unitList.begin(); iterU!=unitList.end();iterU++){
		output+=iterU->second.toString()+"|";
	}
	//print all the connection out
	map<std::string,Connection>::iterator iterC;
	for(iterC=connectionList.begin(); iterC!=connectionList.end();iterC++){
		output+=iterC->second.toString()+"|";
	}
	output.append(std::string(1,'\n'));
	return output;
}

/*returns the name of this row*/
std::string Row::getName(){
	return this->name;
}

/*the states of the units in the Row are updated based on the choosen rules*/
void Row::evolve(int rule){
	map<std::string,Unit>::iterator iterU;
	for(iterU=unitList.begin(); iterU!=unitList.end();iterU++){
		iterU->second.evolve(rule);
	}	
}