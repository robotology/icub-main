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

#include <iCub/iha/DistanceSpaceClass.h>

#include <stdio.h>

#include <yarp/os/all.h>

#include <iCub/iha/debug.h>
#include <vector>
#include <map>
#include <iCub/iha/Experience.h>

#include <iCub/iha/serialization.h>
#include <iostream>


DistanceSpaceClass::DistanceSpaceClass(int hor) : horizon(hor) {
	distSpace = new DistSpaceT;
}
DistanceSpaceClass::~DistanceSpaceClass() {
	clear();
	delete distSpace;
}

void DistanceSpaceClass::deleteDistanceList(int expid) {
	IhaDebug::pmesg(DBGL_DEBUG2,"Delete distances for exp %d:\n",expid);
	multimap <float,int>* dl = (*distSpace)[expid];

	dl->erase(dl->begin(), dl->end());
	distSpace->erase( expid );
}

void DistanceSpaceClass::deleteDistanceEntries(int exp)
{
	IhaDebug::pmesg(DBGL_DEBUG1,"delete dist list for Exp %d\n",exp);
	deleteDistanceList(exp);
	
	IhaDebug::pmesg(DBGL_DEBUG1,"and exp %d entries in:",exp);
	for (DistSpaceT::iterator dspit = distSpace->begin(); dspit != distSpace->end(); dspit++) {
		multimap<float,int>* distlist = dspit->second;
		multimap<float,int>::iterator dlit = distlist->begin();
		bool deleted=false;
		while (!deleted && dlit!=distlist->end() ) {
			if ( dlit->second == exp ) {
				IhaDebug::pmesg(DBGL_DEBUG1,"%d, ",dspit->first);
				distlist->erase(dlit);
				deleted=true;
			}
			dlit++;
		}
	}

	IhaDebug::pmesg(DBGL_DEBUG1,"\n");
}

void DistanceSpaceClass::clear() {
	while (!distSpace->empty()) {
		multimap <float,int>* dl = distSpace->begin()->second;
		IhaDebug::pmesg(DBGL_DEBUG2,"Delete %d distances for exp %d:\n",dl->size(),distSpace->begin()->first);
		dl->erase(dl->begin(), dl->end());
		distSpace->erase( distSpace->begin() );
	}
}


/**
 * Add a new list to the distance space to hold distances
 * from an experience expno.
 */
void DistanceSpaceClass::addExperience(int expno) {
	// only add the list if there is not already one there
	if (distSpace->count(expno)==0) {
		IhaDebug::pmesg(DBGL_DEBUG1, "DS: DSP Adding new dist list for exp %d\n",expno);
		multimap<float,int> *distlist = new multimap<float,int>;
		(*distSpace)[expno] = distlist;
	}
}
/**
 * Add a distance d to this distance space from expno1 to expno2.
 */
void DistanceSpaceClass::addDistance(float d, int expno1, int expno2) {
	// First make sure that this experience has a list in the distance space
	addExperience(expno2);
	
	// add the distance
	IhaDebug::pmesg(DBGL_DEBUG2, "DS: DSP Add dist %d->%d=%f\n",expno1,expno2,d);
	(*distSpace)[expno2]->insert( pair<float,int>(d,expno1) );

	/*
	// if we are configured to keep a limited number of distances in the neighbour
	// list then delete the furthest away
	if (maxDSPNeighbours>0) {
		if ( (*dsp)[expno2].size() > maxDSPNeighbours ) {
			(*dsp)[expno2].erase( --(*dsp)[expno2].end() );
		}
	}
	*/
}

/**
 * Print a single distlist in this distance space
 */
void DistanceSpaceClass::printDistanceList(int dbgl, int expid) {
	IhaDebug::pmesg(dbgl,"Exp %d : ",expid);
	multimap<float,int>* dl = (*distSpace)[expid];
	for (multimap<float,int>::iterator dit=dl->begin(); dit!=dl->end(); dit++) {
		IhaDebug::pmesg(dbgl,"%d=%f ",dit->second,dit->first);
	}
	IhaDebug::pmesg(dbgl,"\n");
}

/**
 * Print out this distance space
 */
void DistanceSpaceClass::printDistanceSpace(int dbgl) {
	for (DistSpaceT::iterator dit=distSpace->begin(); dit!=distSpace->end(); dit++) {
		printDistanceList(dbgl, dit->first);
	}
}

/**
 * Writes out to a port a structured list of the neighbouring experiences to the
 * current experience ordered by distance.  The (future) value and action 
 * frequencies are also writen.
 *
 * @param writeMaxDSPNeighbours maximum number of neighbours to write (0==unlimited)
 * @param writeMaxDSPRadius maximum dist of neighbours to write (0==unlimited)
 * @param experiences map of experiences
 *
 * The structure of the data written to the port is as follows:
 * Item  0 horizon
 * Item  1 expid from which all these distances are calculated
 *
 * Items 2 ... (n+2) the distances and associated information, n=writeMaxDSPNeighbours
 *
 * Item  (n+2)   a list:
 * Item  (n+2).0   - expid
 * Item  (n+2).1   - distance
 * Item  (n+2).2   - (future) value
 *
 * Items (n+2).3 .. (n+2).(m+3) a list of actions and frequencies
 *                              only lists actions with non-zero freqs
 *
 * Item  (n+2).(m+3)    a list (pair):
 * Item  (n+2).(m+3).0    - action number
 * Item  (n+2).(m+3).1    - frequency (0..1)
 */
void DistanceSpaceClass::writeCurrDistToPorts(int writeMaxDSPNeighbours, double writeMaxDSPRadius, map< int, Experience* >  &experiences, int lastExpUpdatingWithFuture) {

	if ( distSpace->size() > 0) {

		IhaDebug::pmesg(DBGL_DEBUG2,"Current Distance Space:\n");
		printDistanceSpace(DBGL_DEBUG2);

		// current experience in distance space will be the
		// most recent (not expid!)
		DistSpaceT::reverse_iterator currit = distSpace->rbegin();
		int curr_expid = (*currit).first;

		IhaDebug::pmesg(DBGL_DEBUG2,"Write current (expid=%d) dists for space horizon %d to port\n",curr_expid,horizon);

		yarp::os::Bottle &b = currDistPort->prepare();
		IhaDebug::pmesg(DBGL_DEBUG2,"Bottle prepared\n");
		b.clear();
		IhaDebug::pmesg(DBGL_DEBUG2,"Bottle cleared\n");

		b.addInt(horizon);	  // item 0 horizon
		IhaDebug::pmesg(DBGL_DEBUG2,"Horizon added\n");
		b.addInt(curr_expid);	// item 1 expid from which all these distances are calcd
		IhaDebug::pmesg(DBGL_DEBUG2,"Current expid added\n");
		
							// now follows items 2 ... (n+2), the dists and assoc info
						// there will be a maximum of n=writeMaxDSPNeighbours of these
		int n=0;
		IhaDebug::pmesg(DBGL_DEBUG2,"before loop\n");
		multimap<float,int>* dl = (*distSpace)[curr_expid];
		for ( multimap<float,int>::iterator dit=dl->begin(); 
				dit!=dl->end() && 
                (writeMaxDSPNeighbours==0 || n < writeMaxDSPNeighbours) && 
                (writeMaxDSPRadius==0 || dit->first < writeMaxDSPRadius);
				dit++) {
			
            if (dit->second > lastExpUpdatingWithFuture)  {
                IhaDebug::pmesg(DBGL_DEBUG2,"Exp %d newer than %d - skipping\n",dit->second, lastExpUpdatingWithFuture);
                continue;
            }
			IhaDebug::pmesg(DBGL_DEBUG2,"dlist %f %d\n",dit->first, dit->second);
			// list of distances and associated info
			yarp::os::Bottle &bl = b.addList();	// item (n+2) a list

			bl.addInt(dit->second);		// item (n+2).0 expid 
			bl.addDouble(dit->first);	// item (n+2).1 distance
			bl.addDouble(experiences[dit->second]->getValue()); // item (n+2).2 (future) value

			/* This code is commented out because if the experience space is not fully
			 * populated (ie some are deleted) then you can't use this method of lookahead
			 *
			// Get the *next* experience to this one
			Experience* nextexp = experiences[dit->second];
			map<int, Experience*>::iterator nextexp_it = experiences.find(dit->second);
			if (nextexp_it==experiences.end()) {
				fprintf(stderr,"Error: could not find experience %d in experience map\n",dit->second);
				return;
			}
			nextexp_it++;
			if (nextexp_it != experiences.end()) {
				nextexp = nextexp_it->second;
				IhaDebug::pmesg(DBGL_DEBUG2,"Exp %d getting actions from next exp %d\n",dit->second, nextexp_it->first);
			}

			// Retrieve the action frequencies associated with the experience
			vector< int >* actionfreqs = nextexp->getActionFrequencies();
			*/

			// Retrieve the action frequencies associated with the experience
			vector< int >* actionfreqs = experiences[dit->second]->getActionFrequencies();

			yarp::os::Bottle &actl = bl.addList(); // item (n+2).3 a list of actions and frequencies
			for (int i=0;i<actionfreqs->size();i++) {
				if ((*actionfreqs)[i] > 0) {		// only insert non-zero freqs
					yarp::os::Bottle &actlpair = actl.addList();        	// item (n+2).3.i   a list (pair)
					actlpair.addInt( i );                     	// item (n+2).3.i.0 action no
					actlpair.addInt( (int) (*actionfreqs)[i] ); // item (n+2).3.i.1 frequency 
				}
			}

			n++;
		}

		IhaDebug::pmesg(DBGL_DEBUG2,"Bottle: %s\n",b.toString().c_str());
		currDistPort->writeStrict();
			
		IhaDebug::pmesg(DBGL_STATUS2,"Wrote %d neighbours to port\n",n);
	}
}

void DistanceSpaceClass::distPortSendClose() {
	yarp::os::Bottle &b = currDistPort->prepare();
	b.clear();
	b.addInt(-1);
	currDistPort->writeStrict();
	IhaDebug::pmesg(DBGL_STATUS2,"Sent close to dist port hor %d\n",horizon);
}

void DistanceSpaceClass::writeToStream(ostream& ofs) {
	ofs << "DSPSTART" << endl;
	for (DistSpaceT::iterator dit=distSpace->begin(); dit!=distSpace->end(); dit++) {
		ofs << dit->first ;
		multimap<float,int>* dl = dit->second;
		for (multimap<float,int>::iterator dlit=dl->begin(); dlit!=dl->end(); dlit++) {
			ofs << FSEP << dlit->first << FSEP << dlit->second ;
		}
		ofs << endl;
	}
	ofs << "DSPEND" << endl;
}

void DistanceSpaceClass::readFromStream(istream& ifs) {
	std::string line;

	getline(ifs,line);
	if (line!="DSPSTART") {
		fprintf(stderr,"Error: DSPSTART not found\n");
		return;
	}
	
	int e1,e2;
	float d;

	getline(ifs,line);
	while (line!="DSPEND") {
		istringstream iss(line);
		iss >> e1;
		multimap<float,int> *distlist = new multimap<float,int>;
		while ( iss >> d >> e2 ) {
			distlist->insert(pair<float,int>(d,e2));
		}
		(*distSpace)[e1] = distlist;
		getline(ifs,line);
	}
}

