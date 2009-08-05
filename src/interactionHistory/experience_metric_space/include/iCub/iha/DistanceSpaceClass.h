#ifndef IHA_DISTANCE_SPACE_CLASS__H__
#define IHA_DISTANCE_SPACE_CLASS__H__

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

#include <yarp/os/all.h>
#include <vector>
#include <map>
#include <iCub/iha/Experience.h>
#include <iCub/iha/serialization.h>

using namespace yarp::os;
using namespace std;

namespace iCub {
	namespace iha {

		/**
		 * Distance Space definition
		 * Alternative view of metric space
		 * distances from an experience stored
		 * in distance order
		 * indexes are experience numbers
		 */
		typedef map< int, multimap< float, int >* > DistSpaceT;
		class DistanceSpaceClass;
	}
}

using namespace iCub::iha;

/**
 * @ingroup icub_iha_ExperienceMetricSpace
 *
 * \brief Distance Space Class holds a single distance space and local processing
 * 
 */
class iCub::iha::DistanceSpaceClass 
{
	private:
		DistSpaceT* distSpace;
		int horizon;

		// a buffered port on which to write out the metric space
		yarp::os::BufferedPort<yarp::os::Bottle> *dsPort;

		// and one to write current distance list
		yarp::os::BufferedPort<yarp::os::Bottle> *currDistPort;

	public:
		DistanceSpaceClass(int hor);
		~DistanceSpaceClass();

		/**
		 * clear distance space
		 */
		void clear();

		/**
		 * delete a single experience's distance list 
		 */
		void deleteDistanceList(int expid);

		/**
		 * Add a new list to this space to hold distances
		 * from an experience expno.
		 */
		void addExperience(int expno);

		/**
		 * Add a distance d to this space from expno1 to expno2.
		 */
		void addDistance(float d, int expno1, int expno2);

		/**
		 * Print a single distlist in a distance space
		 */
		void printDistanceList(int dbgl, int expid);

		/**
		 * Print out one distance space
		 */
		void printDistanceSpace(int dbgl);

		void setMSPort(yarp::os::BufferedPort<yarp::os::Bottle> *p) {
			dsPort = p;
		}
		void setCDPort(yarp::os::BufferedPort<yarp::os::Bottle> *p) {
			currDistPort = p;
		}

		void closeMSPort() {
			dsPort->close();
		}
		
		void closeCDPort() {
			currDistPort->close();
		}

		/**
		 *
		 * Not used
		 */
		void writeDSPToPorts() { }

		/**
		 * Writes out to a port a structured list of the neighbouring experiences to the
		 * current experience ordered by distance.  The (future) value and action 
		 * frequencies are also writen.
		 *
		 * @param writeMaxDSPNeighbours maximum number of neighbours to write (0==unlimited)
		 * @param writeMaxDSPRadius maximum dist of neighbours to write (0==unlimited)
		 * @param experiences map of experiences
		 *
		 *
		 * The structure of the data written to the port is as follows:
		 * \verbatim
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
		 * \endverbatim
		 */
		void writeCurrDistToPorts(int writeMaxDSPNeighbours, double writeMaxDSPRadius, map< int, Experience* >  &experiences, int lastExpUpdatingWithFuture);

		void distPortSendClose();
		/**
		 * delete occurences of an exprience in the distance space
		 */
		void deleteDistanceEntries(int exp);

		DistSpaceT* getDistanceSpace() {
			return distSpace;
		}

		void writeToStream(ostream& ofs);
		void readFromStream(istream& ofs);

};

#endif
