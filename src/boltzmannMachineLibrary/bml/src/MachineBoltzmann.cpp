// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/MachineBoltzmann.h>



#define MEAN_WEIGHT 50 //-->check the variable in layer.cpp
#define BIAS 1300
#define T_PARAM 2000
#define TH_HIGH 0.6
#define TH_LOW 0.4

/**
*default constructor that sets just mean weight and temperature
*/
MachineBoltzmann::MachineBoltzmann(){
	this->countElements=0;
	meanWeight=MEAN_WEIGHT;
	T=T_PARAM;
	epsilon=1;
}
/**
*default destructor
*/
MachineBoltzmann::~MachineBoltzmann(){
}

/**
*constructor that creates a Boltzmann machine of n Layers
*/
MachineBoltzmann::MachineBoltzmann(int nLayers){
	T=T_PARAM;
	meanWeight=MEAN_WEIGHT;
	countElements=0;
	epsilon=1;
	

	//creates the layers
	for(int i=0;i<nLayers;i++){
		//std::string i_str(1,(char)this->countElements+i);
		//_____
		char number[3];
		int n=sprintf(number,"%d",i);
		std::string i_str(number,n);
		//_____
		cout<<"Creating the Layer number "<<i_str<<endl;
		cout<<"The layer is rowsXunits: "<<standard_layer_dim_rows<<"X"<<standard_layer_dim_units<<endl;
		//create a new Layer
		Layer *layer=new Layer("L"+i_str,standard_layer_dim_rows+2,standard_layer_dim_units+2);
		//the layer is added in list inside the addLayer function!!!!
		//elementList.insert(pair<std::string,Layer>(layer->getName(),*layer));
		//elementList[(std::string)layer->getName()]=*layer;
		this->addLayer(*layer);
		this->migrateLayer(*layer);
		//cout<<"just inserted another layer "<<elementList[layer->getName()].toString()<<endl;
		countElements++;
	}
	cout<<"interconnections between layers taking place ..."<<endl;
	//getch();
	//this->interconnectLayers();
	iterEvolve=elementList.begin();
}

/**
*Add a layer to the Boltzmann Machine
*/
void MachineBoltzmann::addLayer(Layer layer){
	//finally add the layer to the list of layer
	elementList.insert(pair<std::string,Layer>(layer.getName(),layer));
	countElements++;
}

int MachineBoltzmann::getLayer(int name){ 
	//Layer layer=elementList.find("L1");
	return NULL;
}

void MachineBoltzmann::migrateLayer(Layer layer){
	int count=0;
	//migrate all the connections in the connectionList of the Boltzmann Machine
	/*map<std::string,Connection>::iterator iterC;
	for(iterC=layer.connectionList.begin(); iterC!=layer.connectionList.end();iterC++){
		this->addConnection(iterC->second);
		count++;	
	}
	printf("CountCOnnections: %d",count);*/

	//migrate all the units in the unitList of the Boltzmann Machine
	// after the insertion of the all new connections coming from other layers in every unit
	map<std::string,Unit>::iterator iterU;
	list<Unit>::iterator _iterU;
	count=0;
	for(iterU=layer.unitList.begin(); iterU!=layer.unitList.end();iterU++){
		this->addUnit(iterU->second);
		count++;
		//this->_unitList.push_back(iterU->second);
	}
	printf("CountConnections: %d",count);

}
/**
*creates the Connections between two different layers and save'em into the connectionList of Boltzmann Machine
*/
void MachineBoltzmann::interconnectLayers(Layer layerA, Layer layerB){
	int numdims=layerA.getCol()*layerA.getRow();
	int numhid=layerB.getCol()*layerB.getRow();

	//1. just using matrix
	Matrix ma=(*layerA.vishid);
	layerA.vishid=new Matrix(numdims,numhid);
	for (int i=0;i<numdims;i++)
		for (int j=0;j<numhid;j++)
			ma(i,j)=0.001;
	//printf("vishid is: (%s)\n", layerA.vishid->toString().c_str());

	//2. using the connection and unit list
	/*int k=0;
	map<std::string,Unit>::iterator iterU1,iterU2;
	for(iterU1=layerA.unitList.begin(); iterU1!=layerA.unitList.end();iterU1++){
		this->addUnit2Matrix(iterU1->second);
		for(iterU2=layerB.unitList.begin(); iterU2!=layerB.unitList.end();iterU2++){					
			//cout<<"k:"<<k;
			//double value=double(rand())/RAND_MAX;
			double value=1;
			double weight_rnd=(value)*MEAN_WEIGHT;
			
			//iterU1->second.addConnectionWeight(1,weight_rnd);
			string name("C");
			name.append(iterU1->second.getName());
			name.append(iterU2->second.getName());
			printf("connection name in interconnectlayers %s",name.c_str());
			Connection *c=new Connection(name,weight_rnd);
			this->addConnection2Matrix(*c);
			//creates two lists of the same lenght of the connection list
			this->addProbFreely2Matrix(0);
			this->addProbClamped2Matrix(0);
			k++;
		}		
	}*/
}


/**
*creates the Connections between two different layers and save'em into the connectionList of Boltzmann Machine
*/
void MachineBoltzmann::interconnectLayers(){
	//allocate the interconnection between this layer and the previously allocated layers
	map<std::string,Unit>::iterator iterI;
	map<std::string,Unit>::iterator iterJ;
	map<std::string,Layer>::iterator iterK;
	map<std::string,Layer>::iterator iterL;
	// visit all the layers in the elementList( this layer has not been added yet!)
	cout<<"number of layer in the elementList"<<elementList.size();
	//getch();
	map<std::string,Layer>::iterator iterE1,iterE2;
	map<std::string,Unit>::iterator iterU1,iterU2;
	int k=0;
	//reset the _connectionList because it changes completely with every layer
	_connectionList.clear();
	_unitList.clear();
	printf("the _connectionList has got size: %d \n",_connectionList.size());
	for(iterE1=elementList.begin(); iterE1!=elementList.end();iterE1++){
		for(iterU1=iterE1->second.unitList.begin(); iterU1!=iterE1->second.unitList.end();iterU1++){
			this->addUnit2Matrix(iterU1->second);
			for(iterE2=elementList.begin(); iterE2!=elementList.end();iterE2++){
				int k=0;
				for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){					
					//cout<<"k:"<<k;
					//double value=double(rand())/RAND_MAX;
					double value=1;
					double weight_rnd=(value)*MEAN_WEIGHT;
					
					//iterU1->second.addConnectionWeight(1,weight_rnd);
					string name("C");
					name.append(iterU1->second.getName());
					name.append(iterU2->second.getName());
					printf("connection name in interconnectlayers %s",name.c_str());
					Connection *c=new Connection(name,weight_rnd);
					this->addConnection2Matrix(*c);
					//creates two lists of the same lenght of the connection list
					this->addProbFreely2Matrix(0);
					this->addProbClamped2Matrix(0);
					k++;
				}
			}	
		}
	}
}
/* DEPRECATED void MachineBoltzmann::interconnectLayers(){
	//allocate the interconnection between this layer and the previously allocated layers
	map<std::string,Unit>::iterator iterI;
	map<std::string,Unit>::iterator iterJ;
	map<std::string,Layer>::iterator iterK;
	map<std::string,Layer>::iterator iterL;
	// visit all the layers in the elementList( this layer has not been added yet!)
	cout<<"number of layer in the elementList"<<elementList.size();
	//getch();
	map<std::string,Layer>::iterator iterE1,iterE2;
	map<std::string,Unit>::iterator iterU1,iterU2;
	int k=0;
	for(iterE1=elementList.begin(); iterE1!=elementList.end();iterE1++){
		for(iterE2=elementList.begin(); iterE2!=elementList.end();iterE2++){
			if(iterE2->first==iterE1->first){
				//no need for adding connectionWeight inside the layer
				
			}
			else
			for(iterU1=iterE1->second.unitList.begin(); iterU1!=iterE1->second.unitList.end();iterU1++){
				int k=0;
				for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){
					
					//cout<<"k:"<<k;
					double value=double(rand())/RAND_MAX;
					int weight_rnd=(int)((value)*MEAN_WEIGHT);
					//iterU1->second.addConnectionWeight(1,weight_rnd);
					string name("C");
					name.append(iterU1->second.getName());
					name.append(iterU2->second.getName());
					printf("connection name %s",name.c_str());
					Connection *c=new Connection(name,weight_rnd);
					this->addConnection(*c);
					k++;
				}
			}
			//getch();
		}
	}
	
}*/

/**
* creates the Connections between the layer passed as parameter and the already present layers
* for every layer that has to be interconnected the structure of the array of connection changes radically
* the values in the array have to be ordered in relation to the list of units
*/
void MachineBoltzmann::interconnectLayer(int layerNumber){
	//allocate the interconnection between this layer and the previously allocated layers
	map<std::string,Unit>::iterator iterI;
	map<std::string,Unit>::iterator iterJ;
	map<std::string,Layer>::iterator iterK;
	map<std::string,Layer>::iterator iterL;
	// visit all the layers in the elementList( this layer has not been added yet!)
	cout<<"number of layer in the elementList"<<elementList.size();
	//getch();
	map<std::string,Layer>::iterator iterE1,iterE2;
	map<std::string,Unit>::iterator iterU1,iterU2;
	list<Connection>::iterator iterC1;
	list<double>::iterator iterFreely,iterClamped;
	int k=0;

	//1.reference to the new Layer
	int countLayer=0;
	for(iterE2=elementList.begin(); countLayer!=layerNumber;iterE2++){
		countLayer++;
	}

    //2. finds the position for every row grouped as layer and adds the connection between this layer and the new one
	int countConnections=0;
	iterE1=elementList.begin();
	iterU1=iterE1->second.unitList.begin(); //iterU1!=iterE1->second.unitList.end();){
	iterC1=_connectionList.begin();
	iterFreely=_probFreely.begin();
	iterClamped=_probClamped.begin();
	int insertionPoint=_unitList.size();	
	while(iterC1!=_connectionList.end()){
		//insertion point is where the new connections from the allocated layers to this have to be inserted
		if((countConnections%insertionPoint==0)&&(countConnections!=0)){
			//add the new connection from already present units to new units
			for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){								
				//insertion of the connectio
				//double value=double(rand())/RAND_MAX;
				double value=1;
				int weight_rnd=(int)((value)*MEAN_WEIGHT);
				//iterU1->second.addConnectionWeight(1,weight_rnd);
				string name("C");
				name.append(iterU1->first);
				name.append(iterU2->first);
				printf("connection name in interconnectLayer %s",name.c_str());
				Connection *c=new Connection(name,weight_rnd);
				_connectionList.insert(iterC1,*c);	
				_probFreely.insert(iterFreely,0);
				_probClamped.insert(iterClamped,0);
			}
			iterU1++; //increments the pointer to the unit
			//if the pointer has reached the end, every unit of the next layer is referenced
			if(iterU1==iterE1->second.unitList.end()){
				
				printf("End of the previous layer and start of the new one _unitList.size %d",_unitList.size());
				iterE1++;
				iterU1=iterE1->second.unitList.begin();
			}
		}
		iterC1++;
		iterFreely++;iterClamped++;
		countConnections++;
		if(iterC1==_connectionList.end()){
			for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){								
				//insertion of the connectio
				//double value=double(rand())/RAND_MAX;
				double value=1;
				int weight_rnd=(int)((value)*MEAN_WEIGHT);
				//iterU1->second.addConnectionWeight(1,weight_rnd);
				string name("C");
				name.append(iterU1->first);
				name.append(iterU2->first);
				printf("connection name second part interconnectLayer %s",name.c_str());
				Connection *c=new Connection(name,weight_rnd);
				_connectionList.insert(iterC1,*c);
				//adds elements to the list of probability
				_probFreely.insert(iterFreely,0);
				_probClamped.insert(iterClamped,0);
			}
		}
	}


	//3. adds the final rows of connections between this layer and the already allocated layers 
	for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){
		//adding these units to the _unitList where there are the rest of units
		printf("Insertion of unit in _unitList %s",iterU2->first.c_str());
		_unitList.push_back(iterU2->second);
		for(iterE1=elementList.begin(); iterE1!=elementList.end();iterE1++){
			for(iterU1=iterE1->second.unitList.begin(); iterU1!=iterE1->second.unitList.end();iterU1++){
				//double value=double(rand())/RAND_MAX;
				double value=1;
				int weight_rnd=(int)((value)*MEAN_WEIGHT);
				//iterU1->second.addConnectionWeight(1,weight_rnd);
				string name("C");
				name.append(iterU2->first);
				name.append(iterU1->first);
				printf("connection name in 3 part interconnectLayer %s",name.c_str());
				Connection *c=new Connection(name,weight_rnd);
				_connectionList.push_back(*c);
				_probFreely.push_back(0);
				_probClamped.push_back(0);
			}
		}
	}
	
	printf("_unitList dimension  %d", _unitList.size());
	printf("_connectionList dimension %d",_connectionList.size());
	/*

	for(iterE1=elementList.begin(); iterE1!=elementList.end();iterE1++){
	for(iterE1=elementList.begin(); iterE1!=elementList.end();iterE1++){
		for(iterU1=iterE1->second.unitList.begin(); iterU1!=iterE1->second.unitList.end();iterU1++){
			this->addUnit2Matrix(iterU1->second);
			for(iterE2=elementList.begin(); iterE2!=elementList.end();iterE2++){
				int k=0;
				for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){					
					//cout<<"k:"<<k;
					double value=double(rand())/RAND_MAX;
					int weight_rnd=(int)((value)*MEAN_WEIGHT);
					//iterU1->second.addConnectionWeight(1,weight_rnd);
					string name("C");
					name.append(iterU1->second.getName());
					name.append(iterU2->second.getName());
					printf("connection name %s",name.c_str());
					Connection *c=new Connection(name,weight_rnd);
					this->addConnection2Matrix(*c);
					k++;
				}
			}	
		}
	} */
}

/**
* returns the temperature of the Boltzmann Machine
*/
int MachineBoltzmann::getTemperature(){
	return this->T;
}
/**
* sets the temperature of the Boltzmann Machine
* @param value new temperature of the Boltzmann Machine
*/
void MachineBoltzmann::setTemperature(int value){
	this->T=value;
}



/**
*Add a connection just to the map named connectionList
* the map connectionList is not what will be used during the calculation
*/
void MachineBoltzmann::addConnection(Connection connection){
	connectionList.insert(pair<std::string, Connection>(connection.getName(),connection));
	//_connectionList.push_back(connection);
}
/**
*Add an unit just to the map named unitList
* the map unitList is not what will be used during the calculation
*/	
void MachineBoltzmann::addUnit(Unit unit){
	unitList.insert(pair<std::string,Unit>(unit.getName(),unit));
	///_unitList.push_back(unit);
}

/**
*Add a connection to the array named _connectionList
* the _connectionList is  what will be used during the calculation
*/
void MachineBoltzmann::addConnection2Matrix(Connection connection){
	_connectionList.push_back(connection);
}

/**
*Add an unit just to the array named _unitList
* the _unitList is what will be used during the calculation
*/	
void MachineBoltzmann::addUnit2Matrix(Unit unit){
	_unitList.push_back(unit);
}

/**
*Add a probability value to the array named _probFreely
* the _probFreely is  where the probability of a unit to be fired during the freely state
* would be saved
*/
void MachineBoltzmann::addProbFreely2Matrix(double value){
	_probFreely.push_back(value);
}

/**
*Add a probability value to the array named _probFreely
* the _probFreely is  where the probability of a unit to be fired during the freely state
* would be saved
*/
void MachineBoltzmann::addProbClamped2Matrix(double value){
	_probClamped.push_back(value);
}




int MachineBoltzmann::getCountElements(){
	return this->countElements;
}

/**
* load the configuration of the Boltzmann Machine from the FILE named configuration.ini
* it loads two binaries called connections.bin and units.bin, which store the connection and units
*/
void MachineBoltzmann::loadConfiguration(){
	ifstream inConfiguration("configuration.ini",ios::in);
	ifstream inConnections("connections.bin",ios::in | ios::binary);
	ifstream inUnits("units.bin",ios::in | ios::binary);
	cout<<"input file stream opened"<<endl;
	//getch();
	if(!inConfiguration)
	{  
		cout<<"Cannot open Configuration file\n";
		//getch();
		return;
	}
	if(!inConnections)
	{  
		cout<<"Cannot open Connections file\n";
		//getch();
		return;
	}
	if(!inUnits)
	{  
		cout<<"Cannot open Units file\n";
		//getch();
		return;
	}
	char c;
	string readLine;
	string nameLayer;
	string str("");
	string layerName("");
	string fileName("");
	string dimension_str("");
	int parameterNumber=0;
	//reading the parameter of the boltzmann machine
	while(inConfiguration.good()){
		c = inConfiguration.get();       // get character from file
		if (inConfiguration.good()){
			//cout<<c;
		if(c=='|'){
				if(parameterNumber==0){
					//layerName
					layerName=str;
				}
				if(parameterNumber==1){
					//fileName
					fileName=str;
				}
				if(parameterNumber==2){
					//dimension of the Layer ROWxUNITS
					dimension_str=str;
				}
				parameterNumber++;
				str.clear();
			}
			else if(c=='\n'){
				string rowStr("");
				string columnStr("");
				size_t found;
				found=dimension_str.find('x');
				rowStr=dimension_str.substr(1,found-1);
				int row=atoi(rowStr.c_str());
				columnStr=dimension_str.substr(found,dimension_str.size()-found);
				int column=atoi(columnStr.c_str());
				Layer *layer=new Layer(layerName,row,column);
				this->addLayer(*layer);
				str.clear();
				parameterNumber=0;
			}
			else{
				str.append(1,c);
			}
		}//end if inConfiguration.good()
	}
	inConfiguration.close();

	// loading in all the units which are saved in one-dimension array
	int elementCount=0,rowCount=0;
	//cout<<"the number of layers"<<elementList.size()<<endl;
	//getch();
	map<std::string,Layer>::iterator iterK;
	iterK=elementList.begin();
	while (inUnits.good())     // loop while extraction from file is possible
	{
			c = inUnits.get();       // get character from file
			if(elementCount==10){
				rowCount++;
				elementCount=0;
			}
			if(rowCount==10){
				iterK++;
				rowCount=0;
				elementCount=0;
			}
			if(iterK==elementList.end())	
				break;
			//_____
			char number[3];
			int n=sprintf(number,"%d",rowCount);
			std::string rowCount_str(number,n);
			n=sprintf(number,"%d",elementCount);
			std::string elementCount_str(number,n);
			//_____
			cout<<"Name of the unit:"<<iterK->second.getName()+"U"+rowCount_str+"-"+elementCount_str<<"---->";
			cout <<(int)c<<endl;
			Unit *unit=new Unit(iterK->second.getName()+"U"+rowCount_str+elementCount_str,(int)c);
			this->addUnit(*unit);
			elementCount++;
	}
	inUnits.close();
	cout<<"_unitList size:"<<this->_unitList.size()<<" elementCount:"<<elementCount<<endl;
	//getch();
	//loading in all the connection which was saved as a matrix
	cout<<"loading in all the connection which was saved as a matrix"<<endl;
	//getch();
	list<Unit>::iterator iter;
	iter=this->_unitList.begin(); //iterator for the units previously uploaded
	int row=0;
	int element=0;
	while ((!inConnections.eof())&&(iter!=_unitList.end()))     // loop while extraction from file is possible
	{
			c = inConnections.get();       // get character from file
			if(c=='|'){
				iter++;  // moves to the next unit
				element=0;
				row++; //the next unit correspond to the next row
			}
			else{
				cout<<"ConnectionWeight"<<"("<<row<<"x"<<element<<")"<<(int)c<<endl;
				element++; 
				iter->addConnectionWeight(1,(int)c);
			}
	}
	if(iter==_unitList.end())
		cout<<"UnitList end!"<<endl;
	if(inConnections.eof())
		cout<<"Connections eof"<<endl;
	/*do{
		in.get((char*)readLine.c_str(),2);
		//in.getline((char*)readLine.c_str(),256);
		if(in.eof()){
			cout<<"getLine has failed"<<endl;
			getch();
			return;
		}
		else{

			cout<<"getLine has extracted a new line:"<<readLine<<endl;
			//getch();
		}
		//extract the name where the layer data is
		//the form of this line is L0.bin
		size_t found;
		found=readLine.find('.');
		if(found==readLine.size()){
			cout<<"char not found!"<<endl;
			//getch();
		}
		readLine.substr(1,found-1);
		cout<<"nameLayer:"<<readLine<<endl;
		//getch();
		Layer *layer=new Layer(nameLayer);
		layer->loadConfiguration();
	}while(in.eof());*/
	
}

/*DEPRECATED void MachineBoltzmann::loadConfiguration(std::string fileName){
	FILE * pFile;
	int n;
	char name [100];
	cout<<"Checking for the Boltzmann Machine file "<<fileName<<"............"<<endl;
	pFile = fopen (fileName.c_str(),"r");
	if(pFile==NULL){
		cout<<"Error:File not Found"<<endl;
		return;
	}
	else{
		cout<<"Reading the file"<<endl;
		char c;
		string entity("");
		do //cycle until the END_OF_FILE
		{	
			bool EOL_bool=false;
			do //cycle for a single class(Connection or Unit) 
			{ 
				do //cycle for a single entity(Connection or Unit) 
				{
				  c = fgetc (pFile);
				  if(c=='\n'){
					  EOL_bool=true;
					  break;
				  }
				  if(c=='*'){
					  EOL_bool=true;
					  break;
				  }
				  entity.append(1,c);
				} while (c != '|');
				if(!EOL_bool){
					//saving of the entity based on the presence of char 'C'-connection
					string::iterator iterator=entity.begin();
					if(*iterator=='C'){
						
						//extract the state of the unit
						size_t foundOpen,foundClose;
						foundOpen=entity.find('(');
						foundClose=entity.find(')');
						string weight_str=entity.substr(foundOpen+1,foundClose-foundOpen-1);
						//erase what is not necessary
						entity.erase(foundOpen,entity.size());

						//what remains is the name of the unit
						int entityWeight=atoi(weight_str.c_str());
						cout<<"-->Connection Name:"<<entity<<" weight:"<<entityWeight<<endl;
						 
						Connection *connection=new Connection(entity,entityWeight);
						//SPLIT THE NAME of CONNECTION in 2 PARTS
						string entity1=entity.substr(1,6);
						string entity2=entity.substr(7,6);
						//cout<<"Entity1:"<<entity1<<" Entity2:"<<entity2;


						//------------extract the connection information from the ENTITY1
						foundOpen=entity1.find('L');
						foundClose=entity1.find('R');
						string layer_name=entity1.substr(foundOpen,foundClose-foundOpen);
						//cout<<" Layer Name:"<<layer_name<<endl;
						
						 
						//look for the layer into the elementList
						map<std::string,Layer>::iterator iterElement;
						iterElement=elementList.find(layer_name);
						if(iterElement==elementList.end()){
							//the layer is not present
							//cout<<"New Layer found; ADDED to the elementList"<<endl;
							Layer *layer=new Layer(layer_name);
							//allocate an empty layer
							this->addLayer(*layer);
							this->iterEvolve=elementList.begin();
						}
						else{
							//the layer has been already stored in the elementList
							//cout<<"ALREADY ADDED to the elementList"<<endl;
						}
						//add the connection to the connectionList of the Layer
						elementList[layer_name].addConnection(*connection);
						//extract the name of the linked units to reference this connection into the units
						elementList[layer_name].unitList[entity1].addConnection(connection);
						//cout<<"Add Connection"<<connection->getName()<<"to the unit"<<elementList[layer_name].unitList[entity1].getName()<<endl;
						//add the layer again to update the connection in the Boltzmann Machine
						//this->addLayer(elementList[layer_name]);
						
						//-----------------extract the connection information from the ENTITY2
						foundOpen=entity2.find('L');
						foundClose=entity2.find('R');
						layer_name=entity2.substr(foundOpen,foundClose-foundOpen);
						//cout<<" Layer Name:"<<layer_name<<endl;
						 
						//look for the layer into the elementList
						//map<std::string,Layer>::iterator iterElement;
						iterElement=elementList.find(layer_name);
						if(iterElement==elementList.end()){
							//the layer is not present
							//cout<<"New Layer found; ADDED to the elementList"<<endl;
							Layer *layer=new Layer(layer_name);
							//allocate an empty layer
							this->addLayer(*layer);
							this->iterEvolve=elementList.begin();
						}
						else{
							//the layer has been already stored in the elementList
							//cout<<"ALREADY ADDED to the elementList"<<endl;
						}
						//add the connection to the connectionList of the Layer
						elementList[entity2].addConnection(*connection);
						//extract the name of the linked units to reference this connection into the units
						elementList[layer_name].unitList[entity2].addConnection(connection);
						//cout<<"Add Connection"<<connection->getName()<<"to the unit"<<elementList[layer_name].unitList[entity2].getName()<<endl;
						//add the layer again to update the connection in the Boltzmann Machine
						//this->addLayer(elementList[layer_name]);
					}
					else{
						//extract the state of the unit
						size_t foundOpen,foundClose;
						foundOpen=entity.find('(');
						foundClose=entity.find(')');
						string state_str=entity.substr(foundOpen+1,foundClose-foundOpen-1);
						//erase what is not necessary
						entity.erase(foundOpen,entity.size());
						//what remains is the name of the unit
						int entityState=atoi(state_str.c_str());
						cout<<"-->Unit Name:"<<entity<<" state:"<<entityState<<endl;
						 
						Unit *unit=new Unit(entity,entityState);
						//extract the unit information from the FILE
						foundOpen=entity.find('L');
						foundClose=entity.find('R');
						string layer_name=entity.substr(foundOpen,foundClose-foundOpen);
						//cout<<"Layer Name:"<<layer_name<<endl;
						//look for the layer into the elementList
						map<std::string,Layer>::iterator iterElement;
						iterElement=elementList.find(layer_name);
						if(iterElement==elementList.end()){
							//the layer is not present
							//cout<<"New Layer found; ADDED to the elementList"<<endl;
							Layer *layer=new Layer(layer_name);
							//allocate an empty layer
							this->addLayer(*layer);
							this->iterEvolve=elementList.begin();
						}
						else{
							//the layer has been already stored in the elementList
							//cout<<"ALREADY ADDED to the elementList"<<endl;
							//cout<<"found with the name"<<iterElement->first<<endl;
						}
						//add the unit to the elementList of the Layer
						elementList[layer_name].addUnit(*unit);
						//cout<<"added unit:"<<unit->getName()<<"to the Layer"<<elementList[layer_name].getName()<<endl;
						elementList[layer_name].setIterEvolve(); 
						//add the layer again to update the connection in the Boltzmann Machine
						//this->addLayer(elementList[layer_name]);
					}
					//migrate all Connections and Units of the instantiated layers to the Boltzmann Machine
					map<std::string,Layer>::iterator iterL;
					for(iterL=elementList.begin();iterL!=elementList.end();iterL++){
						migrateLayer(iterL->second);
					}
				}
				entity.clear();
			} while ((c != '\n')&&(c!='*'));
		}while(c!='*');
	}
}*/


/*_DEPRECATED void MachineBoltzmann::saveConfiguration(std::string fileName){
	FILE * pFile;
	int n;
	char name [100];
	cout<<"Saving the current Boltzmann Machine on file "<<fileName<<"............"<<endl;
	pFile = fopen (fileName.c_str(),"w");
	if(pFile!=NULL)
		cout<<"Saving the file correctly opened"<<endl;
	else
		cout<<"Error:not able to open the file"<<endl;
	//getch();
	
	/*DEPRECATEDmap<std::string,Layer>::iterator iterE;
	for(iterE=elementList.begin(); iterE!=elementList.end();iterE++){
		fprintf (pFile, iterE->second.toString().c_str());
		cout<<iterE->second.toString();
	}----//
	
	//print all the units of the Boltzmann Machine out
	map<std::string,Unit>::iterator iterU;
	for(iterU=unitList.begin(); iterU!=unitList.end();iterU++){
		string str_out=iterU->second.toString()+"|";
		fprintf(pFile,str_out.c_str());
	}
	fprintf(pFile,"\n");
	cout<<"Units saved correctly"<<endl;
	//getch();
	//print all the connections of the Boltzmann Machine out
	map<std::string,Connection>::iterator iterC;
	for(iterC=connectionList.begin(); iterC!=connectionList.end();iterC++){
		string str_out=iterC->second.toString()+"|";
		fprintf (pFile,str_out.c_str());
	}
	fprintf(pFile,"\n");
	cout<<"Connections saved correctly"<<endl;
	//getch();
	fprintf(pFile,"*");
	fclose (pFile);
}*/

/**
* save the configuration into two files connections.bin and units.bin
*/
void MachineBoltzmann::saveConfiguration(){
	ofstream outConnections ("connections.bin", ios::out | ios::binary);
	ofstream outUnits ("units.bin", ios::out | ios::app | ios::binary);
	if (outConnections.is_open()){  
		/* ok, proceed with output */ 
		cout<<"The file has been correctly opened."<<endl;
	}
	else{
		cout<<"Error:not able to open the file"<<endl;
		return;
	}
	map<std::string,Layer>::iterator iterE1,iterE2;
	map<std::string,Unit>::iterator iterU1,iterU2;

	//MATRIX representation of the CONNECTIONS
	int k=0; //initialise the counter of rows
	if(elementList.size()==0){
		return;
	}
	for(iterE1=elementList.begin(); iterE1!=elementList.end();iterE1++){
		for(iterU1=iterE1->second.unitList.begin(); iterU1!=iterE1->second.unitList.end();iterU1++){
			//Iteration over the rows of the matrix
			cout<<"."<<k;
			int j=0; //initialise the counter of elements
			for(iterE2=elementList.begin(); iterE2!=elementList.end();iterE2++){
				for(iterU2=iterE2->second.unitList.begin(); iterU2!=iterE2->second.unitList.end();iterU2++){
					cout<<"Row of Maxtrix N:"<<k<<" Element N:"<<j<<"of "<<iterE2->second.unitList.size()*elementList.size()<<endl;
					//getch();
					//iteration over the element of one row
					outConnections.put((char) iterU2->second.getConnectionWeight(j-1));
					//cout<<"."<<iterU2->second.getConnectionWeight(j-1);
					j++; //increment the counter of element
					//cout<<"putChar:"<<iterU2->second.getConnectionWeight(j-1)<<endl;
					//getch();
				}
			}
			outConnections.put((char)'|');
			outUnits.put((char) iterU1->second.getState());
			k++;
		}
		//getch();
	}
	//cout<<"this layer has successfully been saved"<<endl;
	//getch();
	outConnections.close();
	outUnits.close();
	/*ifstream inConnections("connections.bin",ios::in | ios::binary);
	char c=inConnections.get();
	if(!inConnections)
		cout<<"Error";
	else
		cout<<"c char after saving:"<<(int)c<<endl;
	//getch();*/
}

/*the states of the units in the Boltzmann Machine are updated based on the choosen rules*/
/*void MachineBoltzmann::evolveFreely(int rule){
	map<std::string,Layer>::iterator iterL;
	
	//__DEPRECATED for(iterL=elementList.begin(); iterL!=elementList.end();iterL++){
	//	iterL->second.evolve(rule);
	//}

	iterL=iterEvolve;
	//cout<<"evolve the layer: "<<iterL->second.getName()<<endl;
	iterL->second.evolveFreely(rule);
	iterL++;
	if(iterL==elementList.end()){
		iterEvolve=elementList.begin();
		//cout<<"evolveLayer restart"<<endl;
	}
	else{
		iterEvolve=iterL;
		//cout<<"evolveLayer next LAyer"<<endl;
	}
}*/

/**
*the states of the units in the Boltzmann Machine are updated based on the choosen rules
*/
/*void MachineBoltzmann::evolveFreely(int rule){
	double value_d=(((double)(rand())/RAND_MAX)*this->_unitList.size());
	int value=(int) floor(value_d);
	list<Unit>::iterator iter;
	for(iter=this->_unitList.begin();iter!=_unitList.end();iter++){
		iter->evolve(rule);
	}
}*/
void MachineBoltzmann::evolveFreely(int rule,int random){
	list<Unit>::iterator iter;
	list<Unit>::iterator iterU;
	int countConn=0;
	//printf("nConnection %d",_connectionList.size());
	list<Connection>::iterator iterC;
	iterC=this->_connectionList.begin();
	if(random){
		int limit=this->_unitList.size();
		for(int j=0;j<2;j++){
			int countConn=0;
			iterC=this->_connectionList.begin();
			double value=double(rand())/RAND_MAX;
			int choice=(int)(value*limit-1);
			//printf("value %f choice: %d \n",value,choice);
			//set the pointer to the choice
			iter=this->_unitList.begin();
			for(int i=0;i<choice;i++){
				iter++;
				for (int j=0;j<_unitList.size();j++){
					iterC++;
					countConn++;
				}
			}
			iterU=this->_unitList.begin();
			double sum=iterC->getWeight()*iterU->getState();
			//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			iterC++;
			iterU++;
			countConn++;
			
			//printf("countConn: %d \n",countConn);
			while(!(countConn%limit)==0){
			//while(countConn!=limit){
				sum+=iterC->getWeight()*iterU->getState();
				//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC++;
				iterU++;
				countConn++;
			}
			//printf("countConn: %d \n",countConn);
			double E=sum-BIAS;
			double A=exp(E/(T*-1));
			double D=1+A;
			double probFired=1/D;
			//cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;
			
			if(probFired>TH_HIGH){
				iter->state=1;
				iter->stateChanged=true;
			}
			else if(probFired<TH_LOW){
				iter->state=0;
				iter->stateChanged=true;
			}
		} //end of the for
	}
	else{
		for(iter=this->_unitList.begin();iter!=_unitList.end();iter++){
			iterU=this->_unitList.begin();
			long sum=iterC->getWeight()*iterU->getState();
			iterC++;
			iterU++;
			countConn++;
			int limit=this->_unitList.size();
			printf("countConn: %d \n",countConn);
			while(!(countConn%limit)==0){
				sum+=iterC->getWeight()*iterU->getState();
				iterC++;
				iterU++;
				countConn++;
			}
			//printf("countConn: %d \n",countConn);
			double E=sum-BIAS;
			double A=exp(E/(T*-1));
			double D=1+A;
			double probFired=1/D;
			cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;
			
			if(probFired>TH_HIGH){
				iter->state=1;
				iter->stateChanged=true;
			}
			else if(probFired<TH_LOW){
				iter->state=0;
				iter->stateChanged=true;
			}
		}
	}
}

/**
*The states that are not clamped can evolve. The random units evolves if it is not clamped
*/
void MachineBoltzmann::evolveClamped(int rule,int random){
	list<Unit>::iterator iter;
	list<Unit>::iterator iterU;
	list<int>::iterator iterValue;
	list<Connection>::iterator iterC=this->_connectionList.begin();
	//printf("nConnection %d",_connectionList.size());
	list<int>::iterator iterClamped;
	bool found=false;
	int choice;
	int limit;
	int countConn=0;
	if(random){
		int limit=this->_unitList.size();
		
		iterClamped=_clampedList.begin();
		iter=this->_unitList.begin();
		iterValue=this->_clampedListValue.begin();

		//updates the value of clamped units	
		int countIter=0;
		
		while((iterClamped!=_clampedList.end())&&(!found)){
			if(countIter==*iterClamped){
				//printf("clamped %d", countIter);
				iter->state=*iterValue;
				iterValue++;
				iterClamped++;
			}
			else{
				countIter++;
				iter++;
				if(countIter>*iterClamped){
					iterClamped++;
					iterValue++;
				}
			}
		}


		//extracts the random unit and checks if it is clamped
		double value=double(rand())/RAND_MAX;
		choice=(int)((value)*limit-1);
		//---
		
		iterClamped=_clampedList.begin();
		found=false;
		int current=0;
		while((iterClamped!=_clampedList.end())&&(!found)){
			if(choice==*iterClamped)
				found=true;
			if(choice<*iterClamped)
				break;
			iterClamped++;			
		}
		
		
		// calculates prob only if unit is not clamped
		if(!found){
			iter=this->_unitList.begin();
			//make the pointer point to right position iter to the unit to evolve e iterC to first connection of the unit
			for(int i=0;i<choice;i++){
				iter++;
				for (int j=0;j<_unitList.size();j++){
					iterC++;
					countConn++;
				}
			}
			iterU=this->_unitList.begin();
			long sum=iterC->getWeight()*iterU->getState();
			iterC++;
			iterU++;
			countConn++;
			
			//printf("countConn: %d \n",countConn);
			while(!(countConn%limit)==0){
				sum+=iterC->getWeight()*iterU->getState();
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC++;
				iterU++;
				countConn++;
			}
			//printf("countConn: %d \n",countConn);
			double E=sum-BIAS;
			double A=exp(E/(T*-1));
			double D=1+A;
			double probFired=1/D;
			cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;
			
			if(probFired>TH_HIGH){
				iter->state=1;
				iter->stateChanged=true;
			}
			else if(probFired<TH_LOW){
				iter->state=0;
				iter->stateChanged=true;
			}
		}
	
	} //closing if random
	else
	{ //still to implement the clamped researche for non-random evolution
		for(iter=this->_unitList.begin();iter!=_unitList.end();iter++){
			iterClamped=_clampedList.begin();
			iterValue=this->_clampedListValue.begin();
			found=false;
			int current=0;
			while((iterClamped!=_clampedList.end())&&(!found)){
				if(choice==*iterClamped){
					//printf("clamped %d", countIter);
					iter->state=*iterValue;
					iterValue++;
					iterClamped++;
				}
				if(choice<*iterClamped)
					break;
				iterClamped++;
				iterValue++;
			}
			
			
			// calculates prob only if unit is not clamped
			if(!found){
				iter=this->_unitList.begin();
				//make the pointer point to right position iter to the unit to evolve e iterC to first connection of the unit
				for(int i=0;i<choice;i++){
					iter++;
					for (int j=0;j<_unitList.size();j++){
						iterC++;
						countConn++;
					}
				}
				iterU=this->_unitList.begin();
				long sum=iterC->getWeight()*iterU->getState();
				iterC++;
				iterU++;
				countConn++;
				
				
				//printf("countConn: %d \n",countConn);
				while(!(countConn%limit)==0){
					sum+=iterC->getWeight()*iterU->getState();
					//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
					iterC++;
					iterU++;

					countConn++;
				}
				//printf("countConn: %d \n",countConn);
				double E=sum-BIAS;
				double A=exp(E/(T*-1));
				double D=1+A;
				double probFired=1/D;
				cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;
				
				if(probFired>TH_HIGH){
					iter->state=1;
					iter->stateChanged=true;
				}
				else if(probFired<TH_LOW){
					iter->state=0;
					iter->stateChanged=true;
				}
			}

			
			
			/*iterU=this->_unitList.begin();
			long sum=iterC->getWeight()*iterU->getState();
			iterC++;
			iterU++;
			countConn++;
			int limit=this->_unitList.size();
			printf("countConn: %d \n",countConn);
			while(!(countConn%limit)==0){
				sum+=iterC->getWeight()*iterU->getState();
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC++;
				iterU++;
				countConn++;
			}
			//printf("countConn: %d \n",countConn);
			double E=sum-BIAS;
			double A=exp(E/(T*-1));
			double D=1+A;
			double probFired=1/D;
			cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;
			
			if(probFired>TH_HIGH){
				iter->state=1;
				iter->stateChanged=true;
			}
			else if(probFired<TH_LOW){
				iter->state=0;
				iter->stateChanged=true;
			}*/
		}
	}
}
/*void MachineBoltzmann::evolveClamped(int rule){
	//evolve the random unit if not clamped
	list<Unit>::iterator iter;
	iter=this->_unitList.begin();				
	int value=0; //get the position of the unit
	for(iter=this->_unitList.begin();iter!=_unitList.end();iter++){
		list<int>::iterator iterClamped;
		iterClamped=_clampedList.begin();
		bool found=false;
		int current=0;
		while((iterClamped!=_clampedList.end())&&(!found)){
			if(value==*iterClamped)
				found=true;
			if(value<*iterClamped)
				break;
			iterClamped++;			
		}
		if(found)
			iter->state=1;
		else
			iter->evolve(rule);	
		value++;
	}
}*/


/*The states that are not clamped can evolve. The random units evolves if it is not clamped*/
/*DEPRECATED void MachineBoltzmann::evolveClamped(int rule){
	double value=double(rand())/RAND_MAX;
	list<int>::iterator iterClamped;
	iterClamped=_clampedList.begin();
	bool found=false;
	int current=0;
	while((iterClamped!=_clampedList.end())&&(!found)){
		if(value==*iterClamped)
			found=true;
		if(value<*iterClamped){
			break;
		}
		iterClamped++;
	}
	
	//evolve the random unit if not clamped
	list<Unit>::iterator iter;
	iter=this->_unitList.begin();				
	for(int i=0;i<value;i++){
		iter++;
	}
	if(found)
		iter->state=1;
	else
		iter->evolve(rule);	
}*/

/*The states that are not clamped can evolve*/
/*void MachineBoltzmann::evolveClamped(int rule){
	map<std::string,Layer>::iterator iterL;
	
	//__DEPRECATED for(iterL=elementList.begin(); iterL!=elementList.end();iterL++){
	//	iterL->second.evolve(rule);
	//}
	iterL=iterEvolve;
	//cout<<"evolve the layer: "<<iterL->second.getName()<<endl;
	iterL->second.evolveClamped(rule);
	iterL++;
	if(iterL==elementList.end()){
		iterEvolve=elementList.begin();
		//cout<<"evolveLayer restart"<<endl;
	}
	else{
		iterEvolve=iterL;
		//cout<<"evolveLayer next LAyer"<<endl;
	}
}*/




/**
* set the weights of the connections based on the energy in clamped and freely mode
*/
void MachineBoltzmann::learn(){
	list<Connection>::iterator iterC;
	list<double>::iterator iterFreely=this->_probFreely.begin();
	list<double>::iterator iterClamped=this->_probClamped.begin();
	int count=0;
	double difference,weight_previous;
	for(iterC=_connectionList.begin();iterC!=_connectionList.end();iterC++){
		//printf("iterFreely: %f iterClamped %f \n", *iterFreely,*iterClamped);
		difference=epsilon*(*iterFreely-*iterClamped);
		iterC->setWeight(iterC->getWeight()+difference);
		if(difference!=0){
			printf("count: %d , variation weight %f \n",count,difference);
		}
		count++;
		iterFreely++;
		iterClamped++;
	}
	printf("end learning \n");
}


/**
* averages the probability of two units both being in on-state when the system is clamped
*/
void MachineBoltzmann::setProbabilityClamped(){
	list<Unit>::iterator iter,iter2;
	list<Unit>::iterator iterU;
	int countConn=0,countConn2=0;
	//printf("nConnection %d",_connectionList.size());
	list<Connection>::iterator iterC,iterC2;
	iterC=this->_connectionList.begin();
	iterC2=this->_connectionList.begin();
	int limit=this->_unitList.size();
	for(int j=0;j<2;j++){
			int countConn=0;
			int countConn2=0;
			iterC=this->_connectionList.begin();
			iterC2=this->_connectionList.begin();
			double value=double(rand())/RAND_MAX;
			int choice=(int)(value*limit-1);
			double value2=double(rand())/RAND_MAX;
			int choice2=(int)(value2*limit-1);
			//printf("value %f choice:%d choice2:%d \n",value,choice,choice2);
			//set the pointers to the choices
			iter=this->_unitList.begin();
			for(int i=0;i<choice;i++){
				iter++;
				for (int j=0;j<_unitList.size();j++){
					iterC++;
					countConn++;
				}
			}
			iter2=this->_unitList.begin();
			for(int i=0;i<choice2;i++){
				iter2++;
				for (int j=0;j<_unitList.size();j++){
					iterC2++;
					countConn2++;
				}
			}

			// calculates the probability first unit
			iterU=this->_unitList.begin();
			double sum=iterC->getWeight()*iterU->getState();
			//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			iterC++;
			iterU++;
			countConn++;
			
			//printf("countConn: %d \n",countConn);
			while(!(countConn%limit)==0){
			//while(countConn!=limit){
				sum+=iterC->getWeight()*iterU->getState();
				//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC++;
				iterU++;
				countConn++;
			}
			double E=sum-BIAS;
			double A=exp(E/(T*-1));
			double D=1+A;
			double probFired=1/D;
			//cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;

			// calculates the probability second unit
			iterU=this->_unitList.begin();
			sum=iterC2->getWeight()*iterU->getState();
			//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			iterC2++;
			iterU++;
			countConn2++;
			
			//printf("countConn: %d \n",countConn);
			while(!(countConn2%limit)==0){
			//while(countConn!=limit){
				sum+=iterC2->getWeight()*iterU->getState();
				//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC2++;
				iterU++;
				countConn2++;
			}
			E=sum-BIAS;
			A=exp(E/(T*-1));
			D=1+A;
			double probFired2=1/D;
			//cout<<iter2->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired2:"<<probFired2<<endl;

			//set the value in the list of clamped probability
			list<double>::iterator iterClamped=_probClamped.begin();
			int limit=_unitList.size();
			for (int j=0;j<choice;j++){
				for(int i=0;i<limit;i++){
					iterClamped++;
				}
			}
			for(int x=0;x<choice2;x++){
				iterClamped++;
			}

			*iterClamped=(probFired+probFired2)/2;
			//_probFreely.assign(iterFreely, (probFired+probFired2)/2);
			//printf("Probability Clamped %f \n",*iterClamped );

	} //end of the for
	printf("ClampedProb");
}

/**
* average probability of two units both being in on-state when the netowork is running freely
*/
void MachineBoltzmann::setProbabilityFreely(){
	list<Unit>::iterator iter,iter2;
	list<Unit>::iterator iterU;
	int countConn=0,countConn2=0;
	//printf("nConnection %d",_connectionList.size());
	list<Connection>::iterator iterC,iterC2;
	iterC=this->_connectionList.begin();
	iterC2=this->_connectionList.begin();
	int limit=this->_unitList.size();
	for(int j=0;j<2;j++){
			int countConn=0;
			int countConn2=0;
			iterC=this->_connectionList.begin();
			iterC2=this->_connectionList.begin();
			double value=double(rand())/RAND_MAX;
			int choice=(int)(value*limit-1);
			double value2=double(rand())/RAND_MAX;
			int choice2=(int)(value2*limit-1);
			//printf("value %f choice:%d choice2:%d \n",value,choice,choice2);
			//set the pointers to the choices
			iter=this->_unitList.begin();
			for(int i=0;i<choice;i++){
				iter++;
				for (int j=0;j<_unitList.size();j++){
					iterC++;
					countConn++;
				}
			}
			iter2=this->_unitList.begin();
			for(int i=0;i<choice2;i++){
				iter2++;
				for (int j=0;j<_unitList.size();j++){
					iterC2++;
					countConn2++;
				}
			}

			// calculates the probability first unit
			iterU=this->_unitList.begin();
			double sum=iterC->getWeight()*iterU->getState();
			//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			iterC++;
			iterU++;
			countConn++;
			
			//printf("countConn: %d \n",countConn);
			while(!(countConn%limit)==0){
			//while(countConn!=limit){
				sum+=iterC->getWeight()*iterU->getState();
				//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC++;
				iterU++;
				countConn++;
			}
			double E=sum-BIAS;
			double A=exp(E/(T*-1));
			double D=1+A;
			double probFired=1/D;
			//cout<<iter->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired"<<probFired<<endl;

			// calculates the probability second unit
			iterU=this->_unitList.begin();
			sum=iterC2->getWeight()*iterU->getState();
			//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			iterC2++;
			iterU++;
			countConn2++;
			
			//printf("countConn: %d \n",countConn);
			while(!(countConn2%limit)==0){
			//while(countConn!=limit){
				sum+=iterC2->getWeight()*iterU->getState();
				//cout<<"Connection weight: Unit state: "<<iterC->getWeight()<<"  "<<iterU->getState();
			
				//printf(" dsum:%d ",iterC->getWeight()*iterU->getState());
				iterC2++;
				iterU++;
				countConn2++;
			}
			E=sum-BIAS;
			A=exp(E/(T*-1));
			D=1+A;
			double probFired2=1/D;
			//cout<<iter2->getName()<<" T:"<<T<<" sum"<<sum<<" DEnergy:"<<E<<" A:"<<A<<" probFired2:"<<probFired2<<endl;

			//set the value in the list of freely probability
			list<double>::iterator iterFreely=_probFreely.begin();
			int limit=_unitList.size();
			for (int j=0;j<choice;j++){
				for(int i=0;i<limit;i++)
					iterFreely++;
			}
			for(int x=0;x<choice2;x++){
				iterFreely++;
			}

			*iterFreely=(probFired+probFired2)/2;
			//_probFreely.assign(iterFreely, (probFired+probFired2)/2);
			//printf("Probability Freely %f \n",*iterFreely);

	} //end of the for
	printf("FreelyProb");
}
/*void MachineBoltzmann::setProbabilityFreely(){
	list<Unit>::iterator iterJ;
	list<Unit>::iterator iterI;
	double probValue;
	for(iterJ=_unitList.begin(); iterJ!=_unitList.end();iterJ++){
		for(iterI=iterJ,iterI++; iterI!=_unitList.end();iterI++){
			//both the units must be in the ON state
			if((!iterJ->getState())||(!iterJ->getState()))
				break;
			probValue=(iterJ->getProbFired()+iterI->getProbFired())/2;
			//this->probabilityFreely[iterJ->first+iterI->first]=probValue;
		}
	}
}*/

/**
* set the unit of the Boltzmann Machine which is going to be clamped
* the reference to this unit is indicated using the position of it in the list of units
*/
void MachineBoltzmann::addClampedUnit(int position,int value){
	if(_clampedList.empty()){
		cout<<"first element added"<<endl;
		_clampedList.push_front(position);
		_clampedListValue.push_front(value);
		return;
	}
	/*cout<<"the value:"<<position<<"is not the first one"<<endl;
	//getch();*/
	list<int>::iterator iter;
	list<int>::iterator iterValue;
	int min=this->_clampedList.front();
	int max=this->_clampedList.back();
	if((position==max)||(position==min)){
		return;
	}
	if(position>=max){
		_clampedList.push_back(position);
		_clampedListValue.push_back(value);
		return;
	}
	else if(position<=min){
		_clampedList.push_front(position);
		_clampedListValue.push_front(value);
		return;
	}
	else{
		iter=_clampedList.begin();
		iterValue=_clampedListValue.begin();
		bool found=false;
		int current=0;
		while((iter!=_clampedList.end())&&(!found)){
			if(position==*iter)
				return;
			if(position<*iter){
				_clampedList.insert(iter,position);
				_clampedListValue.insert(iterValue,value);
				found=true;
			}
			printf("%d",position);
			iter++;
			iterValue++;
		}
		if(iter==_clampedList.end()){
			cout<<"Error"<<endl;
		}
	}
}