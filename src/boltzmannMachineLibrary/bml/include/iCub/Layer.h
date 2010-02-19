// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _LAYER_H_
#define _LAYER_H_


#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Row.h>
#include <iCub/Element.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>


#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <string>

using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

/**
* Class that represent a layer of units.
* @author Francesco Rea
*/

class Layer : public Element{
private:
	/**
	* name of the layer
	*/
	std::string name;
	/** 
	* evolution rule 1
	*/
	static const int BINARY_THR=1;
	/** 
	* evolution rule 2
	*/
	static const int PROBAB_THR=2;
	/** 
	* evolution rule 3
	*/
	static const int BOTH_RULES=3;
	/**
	* iterator for the evolution process
	*/
	map<std::string,Unit>::iterator iterEvolve;
	/**
	* number of rows
	*/
	int row;
	/**
	* number of columns
	*/
	int col;
	
public:
	/**
	* matrix storing the state of every single unit in the layer
	*/
	Matrix* vishid;
	/**
	* vector storing the state of every single unit in the layer
	*/
	Vector* stateVector;
	//_____________ METHODS ___________________
	/**
	*default constructor
	*/
	Layer();//
	/**
	* default destructor
	*/
	~Layer();//
	/**
	* default constructor of a layer composed of nRows of nUnits
	*/
	Layer(std::string name,int nRows,int nUnits);//
	Layer(std::string name);
	/**
	* returns the value of the internal energy
	*/
	int getEnergy();
	/**
	*
	*/
	double* getData();
	/**
	* returns the number of Rows
	*/
	int getRow();
	/**
	* returns the number of Columns
	*/
	int getCol();
	/**
	* returns the name of this row
	*/
	std::string getName();//
	/**
	* returns the number of units present in the layer
	*/
	int getNumUnits(); 
	/**
	* add a unit to the unitList
	*/
	void addUnit(Unit unitA); //
	/**
	* add a unit to the ConnectionList
	*/
	void addConnection(Unit unitA, Unit unitB, double weight); //
	/**
	* add a connection 
	* @param unitName name of the unit
	* @param weight weight of the connection
	*/
	void addConnection(std::string unitName, double weight); //
	/**
	* add a connection 
	* @param connection reference to the connection
	*/
	void addConnection(Connection connection);
	/**
	* add a row 
	* @param row reference to the row
	*/
	void addRow(Row row);
	/**
	* function that creates the connection within the layer
	*/
	void interConnectUnits();
	/**
	* set the iterator need for the evolution to the beginning of the listUnit
	*/
	void setIterEvolve(); //
	/**
	* code the class into a std::string
	*/
	std::string toString(); //
	/**
	* save the layer into IOS::BINARY file
	* @param filename name of the file where the configuration will be saved
	*/
	void saveConfiguration(std::string filename); //
	/**
	* load the configuration of a new layer from the binary files
	*/
	void loadConfiguration(); //
	/**
	* the states of the units in the Layer are updated based on the choosen rules
	* @param rule reference to the rule used for evolution
	*/
	void evolveFreely(int rule); //
	/**
	* The states that are not clamped can evolve
	* @param rule the modality by which the evolution is executed
	*/
	void evolveClamped(int rule); //
	//______________________ ATTRIBUTES __________________________________
	/**
	* map of all the units
	*/
	map<std::string,Unit> unitList;
	/**
	* map of the clamped units
	*/
	map<std::string,Unit> clampedList;
	/**
	* map of all the connections
	*/
	map<std::string,Connection> connectionList;
	/**
	* map of all the row of units present in the layer
	*/
	map<std::string,Row> rowList;
};

#endif //_LAYER_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
