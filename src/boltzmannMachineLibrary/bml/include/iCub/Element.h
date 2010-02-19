// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _ELEMENT_H_
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#define _ELEMENT_H_


/*#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Row.h>*/

#include <iostream>
#include <list>
#include <map>
#include <string>

using namespace std;

class Element{
protected:
	std::string name; //specific name of the row
	long energy; //enrgy of the row globally calculated
	//map<std::string,Unit> unitList; 
	//map<std::string,Connection> connectionList;
public:
	Element();
	~Element();//default destructor
	virtual std::string toString();
};

#endif //_ELEMENT_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
