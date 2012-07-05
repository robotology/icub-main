/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "FeatureInterface.h"
#include "FeatureInterface_hid.h"
#include "IRobotInterface.h"
#include <ethManager.h>


 IRobotInterface *iRobotInterface;

void *getRobotFeatureList_C(FEAT_ID *id)
{
//	ethResCreator::instance();
	IRobotInterface *iRobot;
	iRobot = iRobotInterface;
//	iRobot->abort();
//	iRobot->initialize(std::string("ciao"));
	iRobot->getRobot();
	//IiCubFeatureList *list = iRobot->getRobotFeatureList(id);
	//list->findus(id);
	//return (void*) getList();
}

bool findAndFill(FEAT_ID *id, char *sk_array)
{
	IiCubFeatureList *list = iRobotInterface->getRobotSkinList(id);
	IiCubFeature * skin = list->findus(id);
	if(NULL == skin)
	{
//		printf(	"/************************************\\\n"
//				"			Parte non trovata!!!\n"
//				"\\***********************************/\n");
	}
	else
	{
		skin->fillData(sk_array);
		Vector *v = skin->getData();
		skin->pushData(*v);
	}
}
