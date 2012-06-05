/*
 * FeatureInterface.cpp
 *
 *  Created on: May 31, 2012
 *      Author: Alberto Cardellino
 */


#include "FeatureInterface.h"
#include "FeatureInterface_hid.h"
#include "IRobotInterface.h"
#include <ethManager.h>


 IRobotInterface *iRobotInterface;

void *getRobotFeatureList_C(FEAT_ID *id)
{

	printf("hello!!\n");
//	ethResCreator::instance();
	IRobotInterface *iRobot;
	iRobot = iRobotInterface;
	iRobot->abort();
	iRobot->initialize(std::string("ciao"));
	iRobot->getRobot();
	IiCubFeatureList *list = iRobot->getRobotFeatureList(id);
	list->findus(id);
	//return (void*) getList();
}

