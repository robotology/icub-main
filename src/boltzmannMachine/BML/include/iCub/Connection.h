// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _CONNECTION_H_
#define _CONNECTION_H_


//#include <iCub/Unit.h>
#include <list>
#include <string>
#include <iostream>

using namespace std;



class Connection{
private:
	/**
	*weight of the this connection
	*/
    double weight; //
	/**
	*name of A-end of this connection
	*/
	std::string unitAName; //
	/**
	*name of B-end of this connection
	*/
	std::string unitBName; //
	/**
	* name of the connection
	*/
	std::string name; //
public:
	/**
	* default constructor
	*/
	Connection();//default constructor
	/**
	* default destructor
	*/
	~Connection();//default destructor
	/**
	* constructor
	* @param weight
	*/
	Connection(double weight); //contructor
	/**
	* constructor
	* @param name of the unit A side
	* @param name of the unit B side
	* @param weight
	*/
	Connection(std::string unitAName, std::string unitBName, double weight); //constructor
	/**
	* constructor
	* @param name of the unit A side
	* @param weight
	*/
	Connection(std::string unitABName, double weight); //constructor
	/**
	* get the weight of the connection
	*/
	double getWeight(); //
	/**
	* get the  the A-end of this connection
	*/
	std::string getUnitAName(); 
	/**
	* get the  the B-end of this connection
	*/
	std::string getUnitBName(); 
	/**
	* get the complete name composition of A-end name and B-end name
	*/
	std::string getName(); 
	/**
	* set the A-end of this connection
	* @param unitAName the name of the unitAName
	*/
	void setUnitA(std::string unitAName); 
	/**
	* set the B-end of this connection
	* @param unitBName name of the B unit
	*/
	void setUnitB(std::string unitBName); 
	/**
	* set the Name of the connection
	* @param unitABName name of the connection as whole
	*/
	void setName(std::string unitABName);
	/**
	* set the weight of the connection
	* @param weight new weight of the connection
	*/
	void setWeight(double weight);
	/**
	* code the class into a std::string
	*/
	std::string toString(); //
};

#endif //_CONNECTION_H_