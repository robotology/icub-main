// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _MACHINEBOLTZMANN_H_
#define _MACHINEBOLTZMANN_H_

#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Row.h>
#include <iCub/Layer.h>
#include <iCub/Element.h>

#include <stdlib.h>
#include <iostream>
#include <list>
#include <map>
#include <string>
using namespace std;

static int standard_layer_dim_rows=10;
static int standard_layer_dim_units=10;

class MachineBoltzmann{
private:
	/**
	* temperature of the machine
	*/
    double T; //
	int countElements;
	static const int BINARY_THR=1;
	static const int PROBAB_THR=2;
	static const int BOTH_RULES=3;
	int meanWeight;
	map<std::string,Layer>::iterator iterEvolve;
	map<std::string,double> probabilityFreely;
	/**
	* scales the difference of probabilities in the learning process
	*/
	double epsilon; 
public:
	//-------- methods
	/**
	* default constructor
	*/
	MachineBoltzmann();//
	/**
	 * default destructor
	 */
	~MachineBoltzmann();//
	/**
	* constructor that creates a Boltzmann machine of n Layers
	* @param nLayers number of layers in the initialization of Machine Boltzmann
	*/
	MachineBoltzmann(int nLayers);//
	/**
	*list of Layer in the Boltzmann Machine
	*/
	map<std::string,Layer> elementList; //
	/**
	* Add a Layer to the Boltzmann Machine
	* @param layer reference to the layer to be added
	*/
	void addLayer(Layer layer); //
	/**
	* return a Layer to the Boltzmann Machine
	* @param name name of the layer to be returned
	*/
	int getLayer(int name); 
	/**
	* returns the temperature of the Boltzmann Machine
	*/
	int getTemperature();
	/**
	* sets the temperature of the Boltzmann Machine
	* @param value new temperature of the Boltzmann Machine
	*/
	void setTemperature(int value);
	/**
	* all the connections and units of a layer are trasfered to the Bolzmann Machine lists
	* @param reference to the layer to be migrated
	*/
	void migrateLayer(Layer layer); // 
	/**
	* creates the Connections between two different layers and save'em into the connectionList of Boltzmann Machine
	*/
	void interconnectLayers(); //
	/**
	*creates the Connections between the layers passed as parameter with the already present layers
	* @param layerNumber the reference to the allocated layer that has to be interconnected
	*/
	void interconnectLayer(int layerNumber);
	/**
	* Add a connection to the connectionList
	* @param connection reference to the connection to be added
	*/
	void addConnection(Connection connection); //
	/**
	* Add an unit to the unitList
	* @param unit reference to the unit to be added
	*/
	void addUnit(Unit unit); //
	/**
	* Add an unit to the _unitList
	* @param unit reference to the unit to be added
	*/
	void addUnit2Matrix(Unit unit); //
	/**
	* Add a connection to the _connectionList
	* @param connection reference to the connection to be added
	*/
	void addConnection2Matrix(Connection connection);
	/**
	* add a value to the _probFreely
	*/
	void addProbFreely2Matrix(double value);
	/**
	* add a value to the _probClamped
	*/
	void addProbClamped2Matrix(double value);
	/**
	* save the configuration of the Boltzmann Machine into a FILE
	*/
	void saveConfiguration();  //
	/**
	* load the configuration of the Boltzmann Machine from a FILE
	*/
	void loadConfiguration(); //
	/**
	* returns the number of elements
	*/
	int getCountElements(); //
	/**
	* average probability of two units both being in on-state when the system is clamped
	*/
	void setProbabilityClamped(); //
	/**
	* average probability of two units both being in on-state when the netowork is running freely
	*/
	void setProbabilityFreely(); //
	/**
	* ALL the states of the units in the Boltzmann Machine are updated based on the choosen rules
	* @param rule constant integer indicating the decision rule (,probabil)
	* @param random indicates if the unit to evolve is randomly choosen or is one step of a loop over units
	*/
	void evolveFreely(int rule,int random); //
	/**
	* The states that are not clamped can evolve
	* @param rule integer number indicating the decision rule for evolution
	* @param random indicates if the unit to evolve is randomly choosen or is one step of a loop over units
	*/
	void evolveClamped(int rule, int random); //
	/**
	*add a reference to the unit of the Boltzmann Machine which is going to be clamped
	* @param position position in relation to the unit to be clamped
	* @param value state of the clamped unit
	*/
	void addClampedUnit(int position,int value);   //
	/**
	* set the weights of the connections based on the energy in clamped and freely mode
	*/
	void learn(); 
    //------- attributes
	map<std::string,Unit> unitList;
	/**
	* list of all the units present in the Boltzmann Machine
	*/
	list<Unit> _unitList;
	/**
	* list of the units that are clamped in the Boltzmann Machine
	*/
	list<int> _clampedList;
	/**
	* value of the clamped unit in the corrispective position
	*/
	list<int>_clampedListValue;
	/**
	* map of all the connections present in the Boltzmann Machine
	*/
	map<std::string,Connection> connectionList;
	/**
	* list of all the connections present in the Boltzmann Machine
	*/
	list<Connection> _connectionList;

	list<double> _probFreely;
	list<double> _probClamped;
};
#endif //_MACHINEBOLTZMANN_H_
