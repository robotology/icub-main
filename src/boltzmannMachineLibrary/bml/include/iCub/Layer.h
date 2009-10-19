// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _LAYER_H_
#define _LAYER_H_


#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Row.h>
#include <iCub/Element.h>

#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <string>

using namespace std;

class Layer : public Element{
private:
	std::string name;
	static const int BINARY_THR=1;
	static const int PROBAB_THR=2;
	static const int BOTH_RULES=3;
	map<std::string,Unit>::iterator iterEvolve;
	int row;
	int col;
public:
	//------- methods 
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
	* add a unit to the ConnectionList
	*/
	void addConnection(std::string unitName, double weight); //
	void addConnection(Connection connection);
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
	*/
	void saveConfiguration(std::string filename); //
	/**
	* load the configuration of a new layer from the binary files
	*/
	void loadConfiguration(); //
	/**
	* the states of the units in the Layer are updated based on the choosen rules
	*/
	void evolveFreely(int rule); //
	/**
	* The states that are not clamped can evolve
	* @param rule the modality by which the evolution is executed
	*/
	void evolveClamped(int rule); //
	//------ attributes
	map<std::string,Unit> unitList;
	map<std::string,Unit> clampedList;
	map<std::string,Connection> connectionList;
	map<std::string,Row> rowList;
};

#endif //_LAYER_H_