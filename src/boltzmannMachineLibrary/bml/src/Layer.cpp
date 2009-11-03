// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//#include <conio.h>
#include <iCub/Layer.h>


#define MEAN_WEIGHT 50

//default constructor
Layer::Layer(){}
//default destructor
Layer::~Layer(){}

/**
*default constructor of a layer composed of nRows 
*/
Layer::Layer(std::string name,int nRows,int nUnits){
	stateVector=new Vector(nRows*nUnits);
	this->name=name;
	this->row=nRows;
	this->col=nUnits;
	for(int i=0;i<nRows;i++){
		//_____
		char number[3];
		int n=sprintf(number,"%d",i);
		std::string i_str(number,n);
		//_____
		cout<<"Creating the Row n "<<i_str<<endl;
		//create a new Row composed of nUnits
		Row *row=new Row(this->name+"R"+i_str,nUnits); //L1R1U1 means Layer1-Row1-Unit1		
		this->addRow(*row);
	}
	//this->interConnectUnits();
	iterEvolve=unitList.begin();
}

/**
*constructor given just the name
*/
Layer::Layer(std::string name){
	this->name=name;
	//iterEvolve=NULL;
}

void Layer::addUnit(Unit unit){
	unitList.insert(pair<std::string,Unit>(unit.getName(),unit));
}

int Layer::getRow(){
	return row;
}

double* Layer::getData(){
	//return this->stateMatrix.data;
	return 0;
}

/**
* returns the number of Columns
*/
int Layer::getCol(){
	return col;
}

/**
* function that creates connections within the layer
*/
void Layer::interConnectUnits(){
	map<std::string,Unit>::iterator iterI,findI;
	map<std::string,Unit>::iterator iterJ,findJ;
	map<std::string,Connection>::iterator findC;
	printf("Number of units allocated %d \n",unitList.size());
	for(iterI=unitList.begin(); iterI!=unitList.end();iterI++){
		for(iterJ=unitList.begin(); iterJ!=unitList.end();iterJ++){
			string name("C");
			name.append(iterI->second.getName());
			name.append(iterJ->second.getName());
			printf("connection name in interconnectUnits() %s",name.c_str());
			//double value=double(rand())/RAND_MAX;
			double value=1;
			double weight_rnd=(value)*MEAN_WEIGHT;
			addConnection(name,weight_rnd);

		}
	}
}

/**
*add a unit to the ConnectionList
*/
void Layer::addConnection(Connection connection){
	connectionList.insert(pair<std::string, Connection>(connection.getName(),connection));
}

void Layer::addConnection(Unit unitA, Unit unitB, double weight){
	Connection* connection=new Connection(unitA.getName(),unitB.getName(), weight);
	unitA.addConnection(connection);
	unitB.addConnection(connection);
	connectionList.insert(pair<std::string, Connection>(unitA.getName()+unitB.getName(),*connection));
}

/*add a unit to the ConnectionList*/
void Layer::addConnection(std::string unitName, double weight){
	Connection* connection=new Connection(unitName, weight);
	connectionList.insert(pair<std::string, Connection>(unitName,*connection));
}

int Layer::getEnergy(){
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

void Layer::addRow(Row row){
	//insert the row in the list
	rowList.insert(pair<std::string,Row>(row.getName(),row));	
	//migrate the connections to the connectionList of this Layer
	//cannot be possible to migrate just the connection inside the row
	/*map<std::string,Connection>::iterator iterC;
	for(iterC=row.getConnectionListIteratorBegin(); iterC!=row.getConnectionListIteratorEnd();iterC++){
		this->addConnection(iterC->second);
		cout<<"migration Connection Connection Weight:"<<iterC->second.getWeight()<<endl;
	}*/
	
	//migrate the units to the unitList of this Layer
	map<std::string,Unit>::iterator iterU;
	int i=1;
	for(iterU=row.getUnitListIteratorBegin(); iterU!=row.getUnitListIteratorEnd();iterU++){
		this->addUnit(iterU->second);	
		//add the state of the unit in the stateVector
		int pos=(rowList.size()-1)*row.unitList.size()+i;
		printf("pos: %d ",pos);
		(*stateVector)(pos)=iterU->second.getState();
		i++;
	}
}

/*code the class into a std::string*/
std::string Layer::toString(){
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

/*load the configuration of a new layer from the binary files*/
void Layer::loadConfiguration(){
	string filename;
	filename.append(this->getName()+".bin");
	cout<<"try to open the file:"<<filename<<endl;
	ifstream in (filename.c_str(), ios::in | ios::binary);
	if (!in.is_open()){  
		cout<<"Error:not able to open the file"<<endl;
		//getch();
		return;
	}
	cout<<"file open correctly"<<endl;
	//getch();
	char c;
	int i=0;
	do{
		//_____
		char number[3];
		int n=sprintf(number,"%d",i);
		std::string i_str(number,n);
		//_____
		string unitName(this->getName()+"U"+i_str);
		Unit *unit=new Unit(unitName,10);
		cout<<"created the unit:"<<unitName<<endl;
		//getch();
		int j=0;
		do{
			in.get(c);
			cout<<"C:"<<c;
			unit->setConnectionWeight(j,(int) c);
			j++;
		}while(c=='|');
		i++;
	}while(in.good());
}

/*code the class to be saved  into IOS::BINARY file*/
void Layer::saveConfiguration(std::string filename){
	ofstream pFile (filename.c_str(), ios::out | ios::app | ios::binary);
	if (pFile.is_open()){  
		/* ok, proceed with output */ 
		cout<<"The file has been correctly opened"<<endl;
		//getch();
	}
	else{
		cout<<"Error:not able to open the file"<<endl;
		return;
	}

	//print all the unit out
	map<std::string,Unit>::iterator iterU;
	map<std::string,Connection>::iterator iterC;
	list<int>::iterator iterConnection;
	
	for(iterU=unitList.begin(); iterU!=unitList.end();iterU++){
		//get the connections of every unit
		for(iterConnection=iterU->second.weightList.begin(); iterConnection!=iterU->second.weightList.end();iterConnection++){
			pFile.put((char) *iterConnection);
			cout<<"putChar:"<<*iterConnection<<endl;
			//getch();
		}
		pFile.put('|');
	}
	pFile.close();
}

/*returns the name of this row*/
std::string Layer::getName(){
	return this->name;
}

/*the states of the units in the Layer are updated based on the choosen rules*/
void Layer::evolveFreely(int rule){
	map<std::string,Unit>::iterator iterU;
	/*_DEPRECATED for(iterU=unitList.begin(); iterU!=unitList.end();iterU++){
		iterU->second.evolve(rule);
		cout<<" evolve the unit: "<<iterU->second.getName();
	}*/
	//iterU=iterEvolve; //this will create assert errors; try not to use it;
	iterU=unitList.find(iterEvolve->first);
	iterU->second.evolve(rule);
	iterU++;
	if(iterU==unitList.end()){
		//cout<<"evolve Unit restart"<<endl;
		iterEvolve=unitList.begin();
	}
	else{
		//cout<<"evolve Unit next Unit";
		iterEvolve=unitList.find(iterU->first);
	}
}

/*the states of the units in the Layer are updated based on the choosen rules*/
void Layer::evolveClamped(int rule){
	map<std::string,Unit>::iterator iterU;
	map<std::string,Unit>::iterator iterCU;
	/*_DEPRECATED for(iterU=unitList.begin(); iterU!=unitList.end();iterU++){
		iterU->second.evolve(rule);
		cout<<" evolve the unit: "<<iterU->second.getName();
	}*/
	//iterU=iterEvolve; //this will create assert errors; try not to use it;
	iterU=unitList.find(iterEvolve->first);
	iterCU=clampedList.find(iterEvolve->first);
	if(iterCU==clampedList.end()){
		//it is not one of the clampedUnits
		iterU->second.evolve(rule);
	}
	// point to the next Unit in the unitList
	iterU++;
	if(iterU==unitList.end()){
		//cout<<"evolve Unit restart"<<endl;
		iterEvolve=unitList.begin();
	}
	else{
		//cout<<"evolve Unit next Unit";
		iterEvolve=unitList.find(iterU->first);
	}
}

/*set the iterator need for the evolution to the beginning of the listUnit*/
void Layer::setIterEvolve(){
	iterEvolve=unitList.begin();
}