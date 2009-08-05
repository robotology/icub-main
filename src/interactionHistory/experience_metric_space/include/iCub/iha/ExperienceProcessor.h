#ifndef IHA_EXPERIENCE_PROCESSOR__H__
#define IHA_EXPERIENCE_PROCESSOR__H__

/*
 * Copyright (C) 2006 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <vector>
#include <map>
#include <set>

#include <iCub/iha/debug.h>
#include <iCub/iha/DataFrame.h>
#include <iCub/iha/DistanceSpaceClass.h>
#include <iCub/iha/Experience.h>
#include <iCub/iha/serialization.h>

namespace iCub {
	namespace iha {
		class ExperienceProcessor;
	}
}

using namespace yarp::os;
using namespace iCub::iha;

/** 
 * @ingroup icub_iha_ExperienceMetricSpace
 *
 * \brief Experience Processor.
 *
 * Holds a metric space (ordered by distance) for one horizon length.
 *
 * Contains all procesing for experiences in this space including forgetting and merging.
 */
class iCub::iha::ExperienceProcessor
{
	//-----------------------------------------------------------------------
	// Members
	public:
		// Map of experience numbers to ds indexes
		map< int, int > expToDSMap;  /**< Map of experience numbers to ds indexes */
		
	private:
		// data store that this processor works on
		map<int,DataFrame*>* datastore;  /**< Data store that this processor works on */

		// unique experience index for this datastore
		// == current (most recently created) experience id
		int expIndex; /**< unique experience index for this datastore = current (most recently created) experience id */
		// and datastore index of the current experience
		int currexp_dsi;  /**< datastore index of the current experience */

		// Map of experiences
		map< int, Experience* > experiences; /**< A Map of experiences to their ID references */

		// distance space (single horizon)
		DistanceSpaceClass * distspace;  /**< A Distance space (single horizon) */

		float merge_threshold;  /**< The distance threshold below which experiences may be merged */

		int horizon;
		int numbins;
		int dimension;

		int lastExpUpdatingWithFuture;

	//-----------------------------------------------------------------------
	// Methods
	public:
		ExperienceProcessor(map<int,DataFrame*>* ds, int h, int nb, int dim) : 
			datastore(ds), 
			expIndex(-1), 
			merge_threshold(0),
			horizon(h),
			numbins(nb),
			dimension(dim)
		{ 
		   distspace = new DistanceSpaceClass(h);
		   lastExpUpdatingWithFuture = 0;
		}
		~ExperienceProcessor() 
		{
			delete distspace;
		}

		void setMergeThreshold(double mt) { 
			merge_threshold=mt;
		}
		/**
		 * Compute Information Distance between t1-h..t1 and t2-h..t2
		 */
		void getInfoDists(int t1, int t2, float *dists);

		/**
		 * Compute linear average info dist over all sensors
		 * between t1-h..t1 and t2-h..t2
		 */
		float getInfoDist(int t1, int t2);

		/**
		 * Create a new experience
		 */
		int createExperience(int dsIndex, int action, double value, int& total_num_experiences, int experience_action_gap);

		/**
		 * Process experiences known by this processor
		 */
		void processExperiences(int &num_comparisons);

		/**
		 * Tree based experience processor.
		 * The algorithm tries to traverse as few experiences as possible
		 * in order to construct a finite nearest neighbour list.
		 */
		void processExperiencesTree(double neighbourRadius, int &num_comparisons);

		void processExperiencesTreeNeighbours(double neighbourRadius, int &num_comparisons);

		void verifyDistanceList();

		/**
		 * Look forward for future value from a point in the datastore
		 * (should be in a datastore object)
		 */
		double getMaxFutureValue(int currentdsi, int dsi);

		/** Get the future value with min and max biased.
		 * If either min or max is encountered, that must be returned
		 * with a bias toward nearset to this exp
		 */
		double getBiasedFutureValue(int currentdsi, int dsi);

		/**
		 * Set future value for an experience
		 * type can be "NONE", "MAX", "BIASED"
		 */
		void updateFutureValues(int futureHorizon, ConstString future_value_update_type);

		void updateOldValues(double decrement);

		/**
		 * Get reward value for a given experience id
		 */
		double getExperienceValue(int expid) {
			return experiences[expid]->getValue();
		}

		/**
		 * Print the list of experiences stored in the map
		 * along with their map no. (DEBUG)
		 */
		void printExperienceList(int dbgl);
		/**
		 * delete an experience from the experience map
		 */
		void deleteExperience(int expid);

		/**
		 * Merge one experience into another
		 * @param int expid1 remaining experience after merge
		 * @param int expid2 experience that is deleted after merge
		 */
		void mergeExperiences(int expid1, int expid2);

		/**
		 * Main loop for finding experiences to merge
		 */
		void doMergeExperiences(int &numMerged, bool onlyMergeSameActions, int futureHorizon);
		
		/** 
		 * check for merged pair in merged list
		 * used by doMergeExperiences
		 **/
		bool checkMerged(map<int,int> &merged, int A, int B, int &newA, int &newB); 

		/**
		 * delete zero valued experiences
		 */
		void purgeExperiences(int &numDeleted, int futureHorizon, double threshold);

		void setMergeThreshold(float mt) {
			merge_threshold=mt;
		}
		float getMergeThreshold() {
			return merge_threshold;
		}
		/**
		 * write out the distance space for the current experience only
		 * invokes identical method in the distance space class
		 */
		void writeCurrDistToPorts(int writeMaxDSPNeighbours, double writeMaxDSPRadius) {
			distspace->writeCurrDistToPorts(writeMaxDSPNeighbours, writeMaxDSPRadius, experiences, lastExpUpdatingWithFuture);
		}

		void distPortSendClose() {
			distspace->distPortSendClose();
		}

		/**
		 * invokes identical method in the distance space class
		 */
		void printDistanceSpace(int dbgl) {
			distspace->printDistanceSpace(dbgl);
		}

		/**
		 * invokes identical method in the distance space class
		 */
		void setMSPort(BufferedPort<Bottle> *p) {
			distspace->setMSPort(p);
		}

		/**
		 * invokes identical method in the distance space class
		 */
		void setCDPort(BufferedPort<Bottle> *p) {
			distspace->setCDPort(p);
		}

		/**
		 * invokes identical method in the distance space class
		 */
		void closeMSPort() {
			distspace->closeMSPort();
		}

		/**
		 * invokes identical method in the distance space class
		 */
		void closeCDPort() {
			distspace->closeCDPort();
		}

		/**
		 * methods to write/read the whole experience space
		 * to/from a file stream
		 */
		void writeToStream(ostream& ofs);
		void readFromStream(istream& ifs, int& numexps, int& maxexp);

	private:
		/**
		 * mark that ds entries are no longer used for a
		 * given experience
		 */
		void releaseDSUsage(int expid);

		/**
		 * mark h items before dsi in datastore as used 
		 */
		void markUsed(int dsi, int h);

};




#endif
