#ifndef IHA_DATA_FRAME__H
#define IHA_DATA_FRAME__H

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
#include <vector>

#include <iCub/iha/debug.h>
#include <iCub/iha/serialization.h>

using namespace std;


namespace iCub {
	namespace iha {
		class DataFrame;
	}
}
using namespace iCub::iha;

/**
 * @ingroup icub_iha_ExperienceMetricSpace
 *
 * \brief Class to hold one frame of data from the sensors
 * 
 * Data is held in a vector of unsigned chars.
 *
 * A dData store is constructed from a map of these data frames.
 */
class iCub::iha::DataFrame 
{
public:
	/**
	 * create an empty data frame
	 */
	DataFrame() : timestep(0), action(-1), usage(0) {};

	/**
	 * Create a populated data frame 
	 * data is in an array
	 */
	DataFrame(int ts, unsigned char *bv, int dim) : action(-1), timestep(ts), usage(0)
	{
		for (int i=0;i<dim;i++) {
			bvalues.push_back(bv[i]);
		}
	};
	
	~DataFrame() { }

	/**
	 * Set the timestep for this data
	 */
	void setTimestep(int ts) { timestep = ts; };
	
	/**
	 * return the timestep of this data
	 */
	int getTimestep() { return timestep; };

	// actions
	/**
	 * Set the action asociated with this data frame (co-occuring)
	 */
	void setAction(int act) { action = act; };

	/**
	 * Return the action associated with this data frame
	 */
	int getAction() { return action; };

	/** 
	 * Set the value associated with this data frame
	 */
	void setValue(double v) { 
		value = v; 
	};
	/**
	 * Return the value associated with this data frame
	 */
	double getValue() {
		return value;
	}

	/**
	 * Add binned values to data frame
	 */
	void addBinnedValue(unsigned char dv) {
		bvalues.push_back(dv);
	}

	/**
	 * get an individual binned value from the data vector
	 */
	unsigned char getBinnedValue(int i) {
		if (i>=bvalues.size()) {
			fprintf(stderr,"Error: trying to access item %d in vector size %d\n",i,bvalues.size());
		}
		return bvalues.at(i);
	};

	/**
	 * Debug printing
	 */
	void print(int dbgl) {
		if (IhaDebug::getLevel()<dbgl) return;
		
		IhaDebug::pmesg(dbgl,"----------------------------------------->>\n");
		IhaDebug::pmesg(dbgl,"TS: %d ACT: %d REW: %f DATA: ",timestep,action,value);
		//printf("TS: %d ACT: %d DATA: ",timestep,action);
		vector<unsigned char>::iterator it;
		it = bvalues.begin();
		while (it != bvalues.end()) {
			IhaDebug::pmesg(dbgl,"%d ",*it);
			it++;
		}
		IhaDebug::pmesg(dbgl,"\n>>-----------------------------------------\n");
	};

	/**
	 * Serialization: writing
	 */
	void writeToStream(ostream& ofs) {
		ofs << timestep << FSEP << action << FSEP << value << FSEP << usage << FSEP;
		for (vector<unsigned char>::iterator it=bvalues.begin();it!=bvalues.end();it++) {
			ofs << FSEP << (int)*it;
		}
		ofs << endl;
	}
	
	/**
	 * Serialization: reading
	 */
	void readFromStream(istream& ifs) {
		ifs >> timestep ;
		//fprintf(stderr,"timestep: %d\n",timestep);
		ifs >> action;
		//fprintf(stderr,"action: %d\n",action);
		ifs >> value;
		//fprintf(stderr,"value: %f\n",value);
		ifs >> usage;
		//fprintf(stderr,"usage: %d\n",usage);

		bvalues.clear();

		string line;
		getline(ifs,line); 
		//fprintf(stderr,"data: %s",line.c_str());
		istringstream iss(line);
		int c;
		while (iss >> c) {
			bvalues.push_back((unsigned char) c);
		}
		//print();
	}

// Data members
public:
	int usage; // counts how many experiences are using this data frame

private:
	vector <unsigned char> bvalues; // vector of binned sensor values
	int timestep;
	int action;
	double value;
};
#endif
