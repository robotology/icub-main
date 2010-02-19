// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _UNIT_H_
#define _UNIT_H_



#include <iCub/Connection.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Random.h>

#include <iostream>
#include <map>
#include <string>
#include <math.h>
using namespace std;
using namespace yarp::math;
using namespace yarp::os;

class Unit{
private:
	int T; //"temperature" parameter direct referenced to the temperature of the Boltzmann Machine
	int bias;
	long DEnergy; //the Energy GAP of the unit locally determined
	std::string name; //the coded name of the Unit
	static const int BINARY_THR=1;
	static const int PROBAB_THR=2;
	static const int BOTH_RULES=3;
	double probFired; //probability of the unit to be ON
	bool evolveBinary;
	bool evolveStochastic;
	double stochasticThrHIGH;
	double stochasticThrLOW;
public:
	//------ methods
	/**
	* default constructor
	*/
	Unit();
	/**
	* default destructor
	*/
	~Unit();//default destructor
	/**
	* constructor
	* @param name of the unit
	*/
	Unit(std::string name);
	/**
	* constructor
	* @param name of the unit
	* @param state of the unit
	*/
	Unit(std::string name,int state);
	/**
	* return the probability that the unit is fired
	*/
	double getProbFired();
	std::string getName();
	int getBias();
	/**
	* returns the state of the unit
	*/
	int getState();
	/**
	* add a connection to the unit
	*/
	void addConnection(Connection* connection);
	/**
	* set the weight(int) in the row of connections in a precise position
	*/
	void setConnectionWeight(int pos, int weight); 
	/**
	* get the weight(int) in the row of connections in a precise position
	*/
	int getConnectionWeight(int pos); //
	/**
	* add a series of connection Weight to the listWeight
	*/
	void addConnectionWeight(int nConnectionWeight); // 
	/**
	* add a series of connection Weight to the listWeight
	*/
	void addConnectionWeight(int nConnectionWeight,int weight); // 
	/**
	*calculates the local energy of a single unit
	*/
	void calculateDEnergy();
	/**
	*returns the local energy of a single unit
	*/
	int getDEnergy(); 
	std::string toString(); //code the class into a std::string
	/**
	* the state of this units is updated based on the choosen rules
	*/
	void evolve(int rule); 
	//------ attributes
	map<std::string,Connection> connectionList; 
	list<int> weightList;
	bool stateChanged;
	bool connectionChanged;
	/**
	* state of the unit; it can assume either value 0 or  value 1
	*/
	int state; 
};

#endif //_UNIT_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
