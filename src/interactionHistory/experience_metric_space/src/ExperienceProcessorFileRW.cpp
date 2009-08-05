// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Assif Mirza
 * email:   assif.mirza@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <stdio.h>

#include <iCub/iha/debug.h>
#include <vector>
#include <map>

#include <iCub/iha/DataFrame.h>

#include <iCub/iha/ExperienceProcessor.h>
#include <iCub/iha/Experience.h>

#include <iCub/iha/serialization.h>


/**
 * methods to write/read the whole experience space
 * to/from a file 
 * */
void ExperienceProcessor::writeToStream(ostream& ofs) {
	// head details
	ofs << expIndex <<FSEP<< currexp_dsi <<FSEP<< horizon <<FSEP<< numbins <<FSEP<< dimension <<FSEP<< merge_threshold <<endl;
	// expToDSMap
	for (map<int,int>::iterator it=expToDSMap.begin(); it!=expToDSMap.end(); it++) {
		ofs << it->first <<FSEP<< it->second <<FSEP;
	}
	ofs<<endl;
	// experience map
	ofs<<"EXPSTART"<<endl;
	for (map<int,Experience*>::iterator it=experiences.begin(); it!=experiences.end(); it++) {
		ofs << it->first <<endl;
		Experience* exp = it->second;
		exp->writeToStream(ofs);
	}
	ofs<<"EXPEND"<<endl;
	// distance space
	distspace->writeToStream(ofs);

}

void ExperienceProcessor::readFromStream(istream& ifs, int& numexps, int& maxexp){
	IhaDebug::pmesg(DBGL_STATUS2,"Reading Experience Processor data\n");
	string line;
	istringstream iss;

	// head
	getline(ifs,line);
	fprintf(stderr,"%s\n",line.c_str());
	iss.str(line);
	iss >> expIndex >> currexp_dsi >> horizon >> numbins >> dimension >> merge_threshold;
	IhaDebug::pmesg(DBGL_DEBUG1,"expIndex: %d currexp_dsi: %d horizon: %d numbins: %d dimension: %d merge_threshold %f\n"
			,expIndex , currexp_dsi , horizon , numbins , dimension , merge_threshold);

	// expToDSMap
	expToDSMap.clear();
	int a,b;
	getline(ifs,line);
	iss.clear();
	iss.str(line);
	while (iss >> a >> b) {
		expToDSMap.insert(pair<int,int>(a,b));
	}

	// experience map
	IhaDebug::pmesg(DBGL_STATUS2,"ExperienceProcessor::readFromStream experience map\n");
	experiences.clear();
	getline(ifs,line);
	if (line!="EXPSTART") {
		fprintf(stderr,"Error: EXPSTART not found\n");
		return;
	}
	int exp;
	Experience* e;
	int expcnt=0;
	
	getline(ifs,line);
	while (line!="EXPEND") {
		istringstream iss(line);
		iss >> exp;
		if (exp>maxexp) maxexp=exp;
		e = new Experience();
		e->readFromStream(ifs);
		getline(ifs,line);
		experiences.insert(pair<int,Experience*>(exp,e));
		expcnt++;
	}
	numexps=expcnt;

	// distance space
	IhaDebug::pmesg(DBGL_STATUS2,"ExperienceProcessor::readFromStream distance space\n");
	distspace->clear();
	distspace->readFromStream(ifs);

	IhaDebug::pmesg(DBGL_STATUS2,"Reading Experience Processor data, done.\n");

}


