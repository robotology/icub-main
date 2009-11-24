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

#include <iCub/iha/ExperienceProcessor.h>

#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <vector>
#include <map>
#include <set>
#include <cmath>

#include <iCub/iha/debug.h>
#include <iCub/iha/DataFrame.h>
#include <iCub/iha/DistanceSpaceClass.h>
#include <iCub/iha/WindowIDCalc.h>
#include <iCub/iha/Experience.h>


using namespace yarp::os;

/**
 * Compute Information Distance between t1-h..t1 and t2-h..t2
 */
void ExperienceProcessor::getInfoDists(int t1, int t2, float *dists) {
	// Distances are stored in an array of floats, 1 for each dimension
	// passed as a parameter

	// Create N objects to compute the ID (N=dimension)
	WindowIDCalc **idcalc;
	idcalc = new WindowIDCalc*[dimension];
	for (int i=0;i<dimension;i++) idcalc[i] = new WindowIDCalc(numbins, horizon);

	// set up two starting points
	// recently changed from t1-horizon+1
	// now experiences go from t-horizon to t-1
	// this first with the first experience at dsIndex=granularity
	// ie. if gran=10, 1st exp will go from 0-9 but dsIndex with exp
	// is marked as 10
	int p1 = t1-horizon; 
	int p2 = t2-horizon;
	if (p1<0) {
		p2 += 0 - p1;
		p1=0;
	}
	IhaDebug::pmesg(DBGL_DEBUG2,"Calc distance from p1=%d p2=%d  to  p1=%d p2=%d\n",p1,p2,t1,t2);
	// send data to the ID calculators
	while (p1 <= t1 && p2 <= t2) {
		//IhaDebug::pmesg(DBGL_DEBUG3,"p1=%d p2=%d\n",p1,p2);
		DataFrame fr1 = *(*datastore)[p1];
		DataFrame fr2 = *(*datastore)[p2];

		//IhaDebug::pmesg(DBGL_DEBUG3, "DataPairs: ");
		for (int i=0;i<dimension;i++) {
			//IhaDebug::pmesg(DBGL_DEBUG3,"%d,%d ",fr1.getBinnedValue(i), fr2.getBinnedValue(i));
			idcalc[i]->putNextDataPair( fr1.getBinnedValue(i), fr2.getBinnedValue(i) );
		}
		//IhaDebug::pmesg(DBGL_DEBUG3, "\n");
		p1++;
		p2++;
	}

	IhaDebug::pmesg(DBGL_DEBUG3, "SensorDistances: ");
	for (int i=0;i<dimension;i++) {
		float dist = idcalc[i]->getInformationDist();
		IhaDebug::pmesg(DBGL_DEBUG3, "S:%d =%f, ", i, dist);
		dists[i] = dist;
	}
	IhaDebug::pmesg(DBGL_DEBUG3, "\n");
	
	for (int i=0;i<dimension;i++) delete idcalc[i];
	delete [] idcalc;
}

/**
 * Compute linear average info dist over all sensors
 * between t1-h..t1 and t2-h..t2
 */
float ExperienceProcessor::getInfoDist(int t1, int t2) {
	float dists[dimension];

	// get all the information distances for all sensors
	// over the time horizon given
	getInfoDists(t1,t2,dists);

	// Add up the distances
	float dist=0;
	for (int i=0;i<dimension;i++) dist+=dists[i];

	IhaDebug::pmesg(DBGL_DEBUG3,"getInfoDist: t1=%d, t2=%d dist=%f \n",t1,t2,dist/(float)dimension);
	// return the average
	return dist / (float) dimension;
}


int ExperienceProcessor::createExperience(int dsIndex, int action, double value, int& total_num_experiences, int experience_action_gap ) {
	IhaDebug::pmesg(DBGL_STATUS1, "ExpProc: createExperience ds=%d dim=%d numbins=%d, ea_gap=%d\n",dsIndex,dimension,numbins,experience_action_gap);

	// check there is enough data in the ds for one experience of
	// this horizon length
	if (dsIndex+1 < horizon) {
		return -1;
	}

	// creating an experience involves creating a mapping
	// from expid to a position in the data store and creating
	// an Experience object. The sensor data is kept in the 
	// data frames
	expIndex++;
	// add an entry to the ds->exp number map
	IhaDebug::pmesg(DBGL_STATUS1, "ExpProc: Adding Exp %d at ds=%d\n",expIndex, dsIndex);
	expToDSMap[expIndex]=dsIndex;
	currexp_dsi=dsIndex;

	// create an experience object 
	// place the experience in the experiences map
	experiences[expIndex] = new Experience();

	experiences[expIndex]->eid = expIndex;
	experiences[expIndex]->horizon = horizon;
	experiences[expIndex]->dsIndex = dsIndex;

	// add the reward value to the experience
	experiences[expIndex]->setValue(value);

	//// add the action to the experience
	//experiences[expIndex]->addAction(action);
	
	// add the action to the experience before!
	// actually - go back experience_action_gap experiences - allows tuning of gap
	if (expIndex >= experience_action_gap) {
        // A candidate experience
        int expForAction=expIndex-experience_action_gap;
        // check that the experience exists
        while (experiences.find(expForAction)==experiences.end()) {
            expForAction--;
            if (expForAction<0) {
                IhaDebug::pmesg(DBGL_STATUS2,"Cant find experience to assign action to\n");
                break;
            }
        }
        if (expForAction>=0) {
            IhaDebug::pmesg(DBGL_STATUS1, "ExpProc: Adding Action %d to Exp %d\n",action,expForAction);
            printExperienceList(DBGL_DEBUG2);
            experiences[expForAction]->addAction(action);
        }
	}


	// additionally note that this new experience is
	// using certain data items
	markUsed(dsIndex,horizon);

	total_num_experiences += experiences.size();
	return expIndex;
}
	
/**
 * Find the distance from the current experiences to all others in the space
 */
void ExperienceProcessor::processExperiences(int &num_comparisons) {
	IhaDebug::pmesg(DBGL_STATUS2,"ExpProcess: From %d Horizon %d. Starting.\n",expIndex, horizon);
	
	// Calculate Distances
	// iterate through the experiences from the start to just before the ref exp
	// use a look-up to find DS index 


	//map<int, int>::iterator refExpIt = expToDSMap.find(expIndex);

	IhaDebug::pmesg(DBGL_DEBUG3,"ExperienceToDataStore Map::\n");
	for (map<int,int>::iterator it = expToDSMap.begin(); it!=expToDSMap.end(); it++) {
		IhaDebug::pmesg(DBGL_DEBUG3,"%d -> %d\n",it->first,it->second);
	}
	for (map<int,int>::iterator it = expToDSMap.begin(); it!=expToDSMap.end(); it++) {
		
		// Exp2 will always be current exp, Exp1 the one we are comparing
		int expid1 = it->first;
		int dsi_expid1 = it->second;

		// only calculate distances from other "full" experiences
		// not including this one
		if (dsi_expid1+1 >= horizon && expid1!=expIndex) {
			// calculate the experience distance
			float dist = getInfoDist( dsi_expid1,	// DS index of Exp1
									  currexp_dsi	// DS index of current exp
									 );

			// distance space - ordered by distance
			// add both ways
			distspace->addDistance(dist, expid1, expIndex);
			distspace->addDistance(dist, expIndex, expid1);

			num_comparisons++;
		}
	}
	
	IhaDebug::pmesg(DBGL_STATUS2,"ExpProcess: From %d Horizon %d. Done.\n",expIndex, horizon);
	//distspace->printDistanceList(DBGL_DEBUG2,expIndex);

}

/**
 * Place an experience approximately in the space.
 * Instead of working out the distance to all experiences in the space, use a heuristic
 * to reduce the number of comparisons.
 * The heuristc depends on the Triangle Inequality.
 * This variation finds the intital close experience by random search.
 */
void ExperienceProcessor::processExperiencesTree(double neighbourRadius, int &num_comparisons) {
	IhaDebug::pmesg(DBGL_STATUS1,"ExpProcessTree: From %d Horizon %d. Starting.\n",expIndex, horizon);

	map<int, Experience *> checkedList;
	map<int, Experience *> toCheckList;

	IhaDebug::pmesg(DBGL_DEBUG2,"Currently %d experiences\n",experiences.size());
	if (experiences.size()<2) {
		return;
	}

	int eid;
	float dist = 0;

	// create a randomly sorted picklist
	multimap<int,int> toPickList;
	map<int,Experience*>::iterator eit=experiences.begin();
	while (eit != experiences.end()) {
		if (eit->first != expIndex) {
			toPickList.insert( pair<int,int>(rand() % experiences.size(), eit->first) );
		}
		eit++;
	}
	int randompicks=0;
	multimap<int,int>::iterator rit=toPickList.begin();
	do {
		// Get the next random exp
		eid = rit->second;
		IhaDebug::pmesg(DBGL_DEBUG3,"random exp %d\n",eid);

		// get distance to this random experience from current exp
		dist = getInfoDist( expToDSMap[eid], currexp_dsi);	
		num_comparisons++;

		// add to distance space
		distspace->addDistance(dist, eid, expIndex);
		distspace->addDistance(dist, expIndex, eid);

		// add to checked list
		checkedList[eid]=experiences[eid];
		IhaDebug::pmesg(DBGL_DEBUG2,"add %d to checkedList\n",eid);

		randompicks++;

		rit++;
	} while (dist > neighbourRadius && rit!=toPickList.end());

	if (dist > neighbourRadius) {
		IhaDebug::pmesg(DBGL_DEBUG1,"Could not find exp near to %d\n",eid); 
		return;
	} 
	// found a near experience
	IhaDebug::pmesg(DBGL_DEBUG1,"Found exp near to current = %d (%d attempts)\n",eid,checkedList.size()); 

	// Now add all of its neighbours (i.e. <radius) to the toCheckList
	DistSpaceT* dsp = distspace->getDistanceSpace();
	multimap<float,int>* dl = (*dsp)[eid];

	multimap<float,int>::iterator dlit = dl->begin(); // from begining
	while (dlit != dl->end() ) {                      // to end of distance list
		if ( dlit->first <= neighbourRadius &&        // within radius
		 checkedList.count(dlit->second)==0 &&        // not already checked
		 toCheckList.count(dlit->second)==0 &&        // not already added to toCheck
		 dlit->second != expIndex )                   // not current exp
		{
			toCheckList[dlit->second] = experiences[dlit->second];
			IhaDebug::pmesg(DBGL_DEBUG2,"add %d to toCheckList\n",dlit->second);
		}
		dlit++;
	}
	IhaDebug::pmesg(DBGL_DEBUG1,"Added %d neighbours of %d\n",toCheckList.size(),eid); 
	
	// also add previous experience (if it is not already there)
	// get id of previous experience
	map<int, Experience* >::reverse_iterator eitr=experiences.rbegin();
	eitr++;
	if (eitr==experiences.rend()) {
		IhaDebug::pmesg(DBGL_STATUS1,"Cant use processExperiencesTreeNeighbours with only 1 exp\n");
	} else {
		int previd = eitr->first;
		IhaDebug::pmesg(DBGL_DEBUG1,"Prev exp = %d",previd);
		if (checkedList.count(previd)==0 && toCheckList.count(previd)==0) {
			toCheckList[previd] = experiences[previd];
			IhaDebug::pmesg(DBGL_DEBUG1," added.");
		}
			IhaDebug::pmesg(DBGL_DEBUG1,"\n");
	}

	
	int added;
	do {
		added=0;
		// get distances to all in toCheckList (removing as we go)

		while ( ! toCheckList.empty() ) {

			eid = toCheckList.begin()->first;

			// get distance to this experience from current exp
			dist = getInfoDist( expToDSMap[eid], currexp_dsi);	
			num_comparisons++;

			// add to distance space
			distspace->addDistance(dist, eid, expIndex);
			distspace->addDistance(dist, expIndex, eid);
				
			IhaDebug::pmesg(DBGL_DEBUG2,"move %d from toCheckList to checkedList\n",eid);
			// add to checked list
			checkedList[eid]=experiences[eid];
			// remove from toCheck
			toCheckList.erase( toCheckList.begin() );

			// if the distance < radius, 
			// add its neighbours to toCheck (keeping count)
			// that are not in checked or toCheck and that are within radius

			if (dist <= neighbourRadius) {
				int thisadded=0;
				multimap<float,int>* dl = (*dsp)[eid];

				for (multimap<float,int>::iterator dlit = dl->begin(); // from begining
					 dlit != dl->end() &&                              // to end of distance list
					 dlit->first <= neighbourRadius &&                 // within radius
					 checkedList.count(dlit->second)==0 &&             // not already checked
					 toCheckList.count(dlit->second)==0 &&             // not already added to toCheck
					 dlit->second != expIndex;                         // not current exp
					 dlit++) 
				{
					toCheckList[dlit->second] = experiences[dlit->second];
					IhaDebug::pmesg(DBGL_DEBUG2,"add %d to toCheckList\n",dlit->second);
					added++; thisadded++;
				}
				IhaDebug::pmesg(DBGL_DEBUG2,"Added %d neighbours of %d\n",thisadded,eid);
			}

		}
		IhaDebug::pmesg(DBGL_DEBUG2,"Added %d further neighbours\n",added); 

	} while (added > 0);


	IhaDebug::pmesg(DBGL_STATUS1,"ExpProcessTree: From %d Horizon %d. Ended. NumComp=%d\n",expIndex, horizon, num_comparisons);

}

/**
 * Place an experience approximately in the space.
 * Instead of working out the distance to all experiences in the space, use a heuristic
 * to reduce the number of comparisons.
 * The heuristc depends on the Triangle Inequality.
 * This variation finds the intital close experience by using the neighbours of the
 * immediately preceeding experience as a seed.
 */
void ExperienceProcessor::processExperiencesTreeNeighbours(double neighbourRadius, int &num_comparisons) {
	IhaDebug::pmesg(DBGL_STATUS1,"ExpProcessTreeNeighbours: From %d Horizon %d. Starting.\n",expIndex, horizon);

	map<int, Experience *> checkedList;
	map<int, Experience *> toCheckList;

	DistSpaceT* dsp = distspace->getDistanceSpace();

	IhaDebug::pmesg(DBGL_DEBUG2,"Currently %d experiences\n",experiences.size());
	if (experiences.size()<2) {
		return;
	}

	int eid;
	float dist = 0;

	// get id of previous experience
	map<int, Experience* >::reverse_iterator eitr=experiences.rbegin();
	eitr++;
	if (eitr==experiences.rend()) {
		IhaDebug::pmesg(DBGL_STATUS1,"Cant use processExperiencesTreeNeighbours with only 1 exp\n");
		processExperiences(num_comparisons);
		return;
	}
	int previd = eitr->first;
	IhaDebug::pmesg(DBGL_STATUS1,"Prev exp = %d\n",previd);

	/*
	// Now add all of previous experiences' neighbours (i.e. <radius) to the toCheckList
	multimap<float,int>* dl = (*dsp)[previd];

	multimap<float,int>::iterator dlit = dl->begin(); // from begining
	while (dlit != dl->end() ) {                      // to end of distance list
		//IhaDebug::pmesg(DBGL_STATUS1,"%d dist %f\n",dlit->second,dlit->first);
		if ( dlit->first <= neighbourRadius &&        // within radius
		 dlit->second != expIndex )                   // not current exp
		{
			toCheckList[dlit->second] = experiences[dlit->second];
			IhaDebug::pmesg(DBGL_DEBUG2,"add %d to toCheckList\n",dlit->second);
		}
		dlit++;
	}
	IhaDebug::pmesg(DBGL_STATUS1,"Added %d neighbours of %d\n",toCheckList.size(),previd); 
	*/

	// get distance to previous exp
	dist = getInfoDist( expToDSMap[previd], currexp_dsi);	
	num_comparisons++;
				
	if (dist > neighbourRadius) {
		IhaDebug::pmesg(DBGL_STATUS1,"neighbour not close - calling random list tree\n");
		processExperiencesTree(neighbourRadius, num_comparisons);
		return;
	}
	// add to toCheckList
	toCheckList[previd] = experiences[previd];
	
	/*if (toCheckList.size()==0) {
		IhaDebug::pmesg(DBGL_STATUS1,"Got a zero sized neighbout list - calling random list tree\n");
		processExperiencesTree(neighbourRadius, num_comparisons);
		return;
	}*/
	
	int added;
	do {
		added=0;
		// get distances to all in toCheckList (removing as we go)

		while ( ! toCheckList.empty() ) {

			eid = toCheckList.begin()->first;

			// get distance to this experience from current exp
			dist = getInfoDist( expToDSMap[eid], currexp_dsi);	
			num_comparisons++;

			// add to distance space
			distspace->addDistance(dist, eid, expIndex);
			distspace->addDistance(dist, expIndex, eid);
				
			IhaDebug::pmesg(DBGL_DEBUG2,"move %d from toCheckList to checkedList\n",eid);
			// add to checked list
			checkedList[eid]=experiences[eid];
			// remove from toCheck
			toCheckList.erase( toCheckList.begin() );

			// if the distance < radius, 
			// add its neighbours to toCheck (keeping count)
			// that are not in checked or toCheck and that are within radius

			if (dist <= neighbourRadius) {
				int thisadded=0;
				multimap<float,int>* dl = (*dsp)[eid];

				for (multimap<float,int>::iterator dlit = dl->begin(); // from begining
					 dlit != dl->end() &&                              // to end of distance list
					 dlit->first <= neighbourRadius &&                 // within radius
					 checkedList.count(dlit->second)==0 &&             // not already checked
					 toCheckList.count(dlit->second)==0 &&             // not already added to toCheck
					 dlit->second != expIndex;                         // not current exp
					 dlit++) 
				{
					toCheckList[dlit->second] = experiences[dlit->second];
					IhaDebug::pmesg(DBGL_DEBUG2,"add %d to toCheckList\n",dlit->second);
					added++; thisadded++;
				}
				IhaDebug::pmesg(DBGL_DEBUG2,"Added %d neighbours of %d\n",thisadded,eid);
			}

		}
		IhaDebug::pmesg(DBGL_DEBUG2,"Added %d further neighbours\n",added); 

	} while (added > 0);


	IhaDebug::pmesg(DBGL_STATUS1,"ExpProcessTree: From %d Horizon %d. Ended. NumComp=%d\n",expIndex, horizon, num_comparisons);

}

void ExperienceProcessor::verifyDistanceList() {
	IhaDebug::pmesg(DBGL_STATUS1,"Calculated:\n");
	distspace->printDistanceList(DBGL_STATUS1,expIndex);

	IhaDebug::pmesg(DBGL_STATUS1,"Verifying ....\n");
	
	// build a fully examined distance list
	multimap<float, int> verlist;

	for (map<int,int>::iterator it = expToDSMap.begin(); it!=expToDSMap.end(); it++) {
		// Exp2 will always be current exp, Exp1 the one we are comparing
		int expid1 = it->first;
		int dsi_expid1 = it->second;

		// only calculate distances from other "full" experiences
		// not including this one
		if (dsi_expid1 >= horizon && expid1!=expIndex) {
			// calculate the experience distance
			float dist = getInfoDist( dsi_expid1,	// DS index of Exp1
									  currexp_dsi	// DS index of current exp
									 );

			verlist.insert(pair<float,int>(dist,expid1));
		}
	}
	// print first 20
	int cnt=0;
	for (multimap<float,int>::iterator dit=verlist.begin(); dit!=verlist.end() && cnt<20; dit++) {
		IhaDebug::pmesg(DBGL_STATUS1,"%d=%f ",dit->second,dit->first);
		cnt++;
	}
	IhaDebug::pmesg(DBGL_STATUS1,"\n");

	// see which are missing
	multimap<float,int>::iterator itfull=verlist.begin();
	DistSpaceT* dsp = distspace->getDistanceSpace();
	multimap<float,int>* treedl = (*dsp)[expIndex];
	multimap<float,int>::iterator ittree=treedl->begin();

	IhaDebug::pmesg(DBGL_STATUS1,"Missing from top 20:\n");
	int full_cnt=0;
	int missing_cnt=0;
	while (itfull != verlist.end() && ittree != treedl->end() && full_cnt<20 ) {
		if (itfull->second == ittree->second) {
			itfull++; full_cnt++;
			ittree++;
		} else {
			IhaDebug::pmesg(DBGL_STATUS1,"(%d) %d %f\n",full_cnt,itfull->second,itfull->first);
			itfull++; full_cnt++;
			missing_cnt++;
		}
	}
	if (missing_cnt==0) IhaDebug::pmesg(DBGL_STATUS1,"None\n");
}

/**
 * Calculate the future value of an experience in the past
 * based on the value of subsequent experiences.
 * This variation just returns the maximum future value
 */
double ExperienceProcessor::getMaxFutureValue(int dsi1, int dsi2) {
	double fv=0;
	double value=0;

	int topend = dsi2;
	if (topend>currexp_dsi) topend=currexp_dsi;


	for (int i=topend;i>=dsi1;i--) {
		value=(*datastore)[i]->getValue();
		//IhaDebug::pmesg(DBGL_DEBUG3,"i=%d timestep=%d value=%f\n",i,(*(*datastore))[i].getTimestep(),value);
		if ( fabs( value ) > fabs(fv) ) {
			fv = value;
		}
	}

	IhaDebug::pmesg(DBGL_DEBUG3,"get max fv from %d to %d (ds size=%d) = %f\n",dsi1,topend,datastore->size(), fv);

	return fv;
}

/**
 * Calculate the future value of an experience in the past
 * based on the value of subsequent experiences.
 * This variation returns low if a low valued exp is found, high if any high one is found
 * and a middle value otherwise. (Proximal experiences are more relevant)
 */
double ExperienceProcessor::getBiasedFutureValue(int dsi1, int dsi2) {
	double fv=0.5;
	double value=0;

	// move back throgh ds from dsi2 (in future) to ds1
	// so exps nearer to the current one are most significant
	int topend = dsi2;
	if (topend>currexp_dsi) topend=currexp_dsi;

	for (int i=dsi1;i<=topend;i++) {
		value=(*datastore)[i]->getValue();
		//IhaDebug::pmesg(DBGL_DEBUG3,"i=%d timestep=%d value=%f\n",i,(*datastore)[i]->getTimestep(),value);
		
		// the following only works where we know low/middle reward and there are only
		// three possible values
		// Doesn't work for a continuous reward - but it is fast
		if ( value > 0.5 || value == 0 ) {
			return value;
		}
	}

	IhaDebug::pmesg(DBGL_DEBUG3,"get min/max fv from %d to %d (ds size=%d) = %f\n",dsi1,topend,datastore->size(), fv);

	return fv;
}

/**
 * Update the future value of all past experiences up to a particular horizon.
 */
void ExperienceProcessor::updateFutureValues(int futureHorizon, ConstString future_value_update_type) {
	if (future_value_update_type=="NONE") return;
	if (experiences.empty()) return;
	IhaDebug::pmesg(DBGL_DEBUG1, "ExpProc: updateFuture Values futureHorizon:%f\n",futureHorizon);

	// iterate backwards through the experiences
	map<int, Experience*>::reverse_iterator it = experiences.rbegin();
	int eid = it->first;
	int dsi = expToDSMap[eid];
	Experience* exp = it->second;
	IhaDebug::pmesg(DBGL_DEBUG1,"FV (FutHor:%d): ",futureHorizon);
	while(it != experiences.rend() && (currexp_dsi - dsi) < futureHorizon) {

		//double fv =  getMaxFutureValue(dsi, dsi + (horizon * multiplier) - 1);
		double fv;
		if (future_value_update_type == "MAX") {
			fv =  getMaxFutureValue(dsi, (int) (dsi + futureHorizon - 1) );
		} else if (future_value_update_type == "BIASED") {
			fv =  getBiasedFutureValue(dsi, (int) (dsi + futureHorizon - 1) );
		} else {
			return;
		}
		IhaDebug::pmesg(DBGL_DEBUG1,"%d:%f ",eid,fv);
		exp->setValue(fv);
        lastExpUpdatingWithFuture = eid;

		it++;
		eid = it->first;
		dsi = expToDSMap[eid];
		exp = it->second;
	}
	IhaDebug::pmesg(DBGL_DEBUG1,"\n");
    IhaDebug::pmesg(DBGL_DEBUG1,"lastExpUpdatingWithFuture = %d\n",lastExpUpdatingWithFuture);
}

/**
 * Update all experiences to reduce their value
 */
void ExperienceProcessor::updateOldValues(double decrement) {
    IhaDebug::pmesg(DBGL_DEBUG1,"Decrement old values by %f\n",decrement);
	map<int, Experience*>::iterator it = experiences.begin();
	int eid = it->first;
	int dsi = expToDSMap[eid];
	Experience* exp = it->second;
	while(it != experiences.end()) {
        exp->addValue(-1*decrement);
		it++;
		eid = it->first;
		dsi = expToDSMap[eid];
		exp = it->second;
    }
}

void ExperienceProcessor::printExperienceList(int dbgl) {
	IhaDebug::pmesg(dbgl,"Experiences ");
	for (map<int, Experience*>::iterator it = experiences.begin() ; it !=experiences.end() ; it++ ) 
	{
		IhaDebug::pmesg(dbgl, "%d:%d(%f) ",it->first, it->second->eid, it->second->getValue());
	}
	IhaDebug::pmesg(dbgl,".\n");
}
void ExperienceProcessor::deleteExperience(int expid) {
	if (IhaDebug::getLevel()>=DBGL_DEBUG1) printExperienceList(DBGL_DEBUG1);
	IhaDebug::pmesg(DBGL_DEBUG1,"Delete experience %d hor %d\n",expid,horizon);
	releaseDSUsage(expid);

	// delete the mapping entry
	if (expToDSMap.find( expid)!=expToDSMap.end()) {
		expToDSMap.erase( expid );
	//} else if (expToDSMapMerged.find( expid)!=expToDSMapMerged.end()) {
	//	expToDSMapMerged.erase( expid );
	} else {
		fprintf(stderr,"Error deleting experience %d horizon %d\n",expid,horizon);
	}

	// delete the distances in the metric distance space
	distspace->deleteDistanceEntries( expid );

	// delete the experience
	delete experiences[expid];
	experiences.erase ( expid );
	if (IhaDebug::getLevel()>=DBGL_DEBUG1) printExperienceList(DBGL_DEBUG1);
}

void ExperienceProcessor::mergeExperiences(int expid1, int expid2) {
	IhaDebug::pmesg(DBGL_DEBUG1,"Merge exp %d into %d\n",expid2,expid1);
	experiences[expid1]->addActions(experiences[expid2]->getActions());

	double v1 = experiences[expid1]->getValue();
	double v2 = experiences[expid2]->getValue();
	// use max value
	double maxval = (v1 > v2) ? v1 : v2;
	experiences[expid1]->setValue(maxval);
	
	// or could use average value ...
	//double avgval = (v1+v2)/2;
	//experiences[expid1]->setValue(avgval);

	// make exp 2 a child of exp 1
	//experiences[expid2]->setParent(experiences[expid1]);

	// move to merged list
	//expToDSMapMerged[expid2]=expToDSMap[expid2];
	//expToDSMap.erase(expid2);

	deleteExperience(expid2);
	IhaDebug::pmesg(DBGL_DEBUG1,"Merged exp=> ");
	IhaDebug::pmesg(DBGL_DEBUG1,(char*)experiences[expid1]->toString().c_str());
}

void ExperienceProcessor::doMergeExperiences(int &numMerged, bool onlyMergeSameActions, int futureHorizon) {
	IhaDebug::pmesg(DBGL_STATUS2,"doMergeExperiences: horizon %d SameActions?%s, threshold %f\n",horizon,onlyMergeSameActions?"ON":"OFF",merge_threshold);

	// (1) merge experiences with distance <= merge_threshold

	
	//IhaDebug::pmesg(DBGL_STATUS2,"merge close experiences. Threshold %f\n",merge_threshold);
	
	// create a merge list: map should remain unique if we always
	// make the index the smaller value
	// this is a multi map as we may have mutiple experiences merged into a single one
	multimap<int,int> merge_list;

	// for each experience
	for (DistSpaceT::iterator dspit = distspace->getDistanceSpace()->begin(); dspit != distspace->getDistanceSpace()->end(); dspit++) 
	{
		IhaDebug::pmesg(DBGL_DEBUG1,"Experience %d: ---------------------------------------\n",dspit->first);

		int e1 = dspit->first;
		if (e1 == expIndex) { 
			// don't consider current exp
			continue;
		}
		int dsi1 = expToDSMap[e1];
		if (dsi1 >= currexp_dsi - futureHorizon) { 
			// skip if exp isn't old enough
			continue;
		}

		// the distance list for this experience
		multimap<float,int>* distlist = dspit->second;

		if (distlist->size()==0) { 
			// make sure we have some distances from this experience
			continue;
		}

		// iterate through distance list, shortest first
		multimap<float,int>::iterator dlit=distlist->begin(); 
		IhaDebug::pmesg(DBGL_DEBUG1,"dlit->first : %f   ", dlit->first);
		IhaDebug::pmesg(DBGL_DEBUG1,"dlit->second : %d\n", dlit->second);

		// the distance - we check this against the threshold
		float dist=dlit->first;
		IhaDebug::pmesg(DBGL_DEBUG1,"  Dist exp %d-->%d : %f\n",e1,dlit->second,dist);

		// actions: only look at most recent action of an experience
		int act_e1 = experiences[e1]->getAction();
		IhaDebug::pmesg(DBGL_DEBUG1,"  Action for exp %d : %d, ",e1,act_e1);
		int act_e2 = experiences[dlit->second]->getAction();
		IhaDebug::pmesg(DBGL_DEBUG1,"exp %d : %d\n",dlit->second,act_e2);

		int dsi2 = expToDSMap[dlit->second];

		while (dist < merge_threshold 
				&& dlit!=distlist->end() 
				&& dsi2 >= currexp_dsi - futureHorizon
			  ) 
		{
			if (dlit->second != expIndex 
				&& dlit->second!=e1 
				&& (!onlyMergeSameActions || act_e1 == act_e2 )
			   ) 
			{
				IhaDebug::pmesg(DBGL_DEBUG1,"Inserting exp %d into merge list\n",dlit->second);
				int A,B;
				if (e1<dlit->second) {
					A=e1;
					B=dlit->second;
				} else {
					A=dlit->second;
					B=e1;
				}

				// check for duplicate
				// have to check key - and then all possible values
				multimap<int,int>::iterator mmit = merge_list.find(A);
				int val; bool found_match=false;

				if (mmit != merge_list.end()) // a match for A has been found
				{
					val=mmit->second; // val is the second half of this pair
					while ( !found_match && // while a full match hasnt been found
						    mmit->first==A  // and this is still the right part of the map
						  )  
					{
						if (val=B) found_match=true; // found a match
						mmit++;                      // otherwise continue
						if (mmit!=merge_list.end()) 
							val=mmit->second;
					}

					if (!found_match) // no match so insert away
						merge_list.insert(pair<int,int>(A, B));

				} 
				else // no match even for the first part
				{
					merge_list.insert(pair<int,int>(A, B));
				}
			}

			dlit++;
			if (dlit!=distlist->end()) {
			   	dist=dlit->first;
				IhaDebug::pmesg(DBGL_DEBUG1,"  Dist exp %d-->%d : %f\n",e1,dlit->second,dist);
				act_e2 = experiences[dlit->second]->getAction();
				IhaDebug::pmesg(DBGL_DEBUG1,"  Action for exp %d : %d\n",dlit->second,act_e2);
			}
		}


	}
	
	// show the merge list
	if (IhaDebug::getLevel()>=DBGL_DEBUG1) {
		if (merge_list.size()>0) {
			IhaDebug::pmesg(DBGL_DEBUG1,"Merge list ");
			for (multimap<int,int>::iterator mlit=merge_list.begin();mlit!=merge_list.end();mlit++) 
			{
				IhaDebug::pmesg(DBGL_DEBUG1,"%d:%d ",mlit->first,mlit->second);
			}
			IhaDebug::pmesg(DBGL_DEBUG1,"\n");
		}
		else 
		{
			IhaDebug::pmesg(DBGL_DEBUG1,"nothing to merge\n");
		}
	}
	
	// keep track of what we merge in case there is a cascade
	// note the order is reversed first has been merged into second
	map<int,int> merged;

	// iterate through the merge list - merging
	multimap<int,int>::iterator head;
	while (merge_list.size()>0) {
		head = merge_list.begin();
		int a=head->first;
		int b=head->second;
        
        int newA=a;
        int newB=b;

        while (!checkMerged(merged, a, b, newA, newB)) {
                a=newA;
                b=newB;
        }

		mergeExperiences(a, b);
		merged[b]=a;

		merge_list.erase(head);

		numMerged++;
	}

	//IhaDebug::pmesg(DBGL_STATUS2,"(3) Forgetting\n");

	IhaDebug::pmesg(DBGL_STATUS2,"mergeExperiences: ended\n");
}

bool ExperienceProcessor::checkMerged(map<int,int> &merged, int A, int B, int &newA, int &newB) {
    bool check1=false, check2=false;
    // check if the experience has already been merged
    // e.g list is 2:10 , 4:10, 7:10
    //     then we would merge 2:10 (10 disappears) then merge 4 into 2 and 7 into 2
    if ( merged.find(B) != merged.end() ) {
        IhaDebug::pmesg(DBGL_DEBUG1,"Replacing1 (%d,%d) with (%d,%d)\n",A,B,merged[B],A);
        newA=merged[B]; // replace the merged item with what it has been merged with
        newB=A; // and merge the experience with that one instead
    } else {
        check1=true;
    }

    // opposite sense
    // e.g list is 2:4 , 4:6
    //     we should merge 4 with 2 (4 disappears) then merge 6 with 2 (instead of 4)
    if ( merged.find(A) != merged.end() ) {
        IhaDebug::pmesg(DBGL_DEBUG1,"Replacing2 (%d,%d) with (%d,%d)\n",A,B,merged[A],B);
        newA=merged[A];
        newB=B;
    } else {
        check2=true;
    }
    return check1 && check2;

}

void ExperienceProcessor::purgeExperiences(int &numDeleted, int futureHorizon, double threshold) {
	IhaDebug::pmesg(DBGL_STATUS2,"purgeExperiences: delete experiences with value < %f. futureHorizon=%d \n", threshold, futureHorizon);
	// purge experiences with value < threshold
	// experience has to be a certain age (i.e. has
	// to have been updated by the future value process)
		
	// for each experience
	for (map<int, Experience*>::iterator expit = experiences.begin(); expit != experiences.end(); expit++) {
		int e1 = expit->first;
		
		int dsi1 = expToDSMap[e1];
		if (dsi1 >= currexp_dsi - futureHorizon) { 
			// don't consider exp
			continue;
		}
		if (expit->second->getValue() < threshold) {
			// delete
			IhaDebug::pmesg(DBGL_STATUS1, "Delete experience %d\n",e1);
			deleteExperience(e1);
			numDeleted++;
		}
	}
	IhaDebug::pmesg(DBGL_STATUS2,"purgeExperiences: ended\n");
}


void ExperienceProcessor::releaseDSUsage(int expid) {
	IhaDebug::pmesg(DBGL_DEBUG2,"Release DS usage exp %d hor %d\n",expid,horizon);
	// note down in ds usage that we are no longer using these
	// data items
	int dsi = expToDSMap[expid];
	for (int i=dsi-horizon+1; i<=dsi; i++) {
		(*datastore)[i]->usage--;
	}
}

/**
 * mark h items before dsi in datastore as used 
 */
void ExperienceProcessor::markUsed(int dsi, int h) {
	for (int i=dsi-h+1; i<=dsi; i++) {
		(*datastore)[i]->usage++;
	}
}

