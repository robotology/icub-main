#ifndef _IHA_EXPERIENCE__H__
#define _IHA_EXPERIENCE__H__

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
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

/** \class
 */

#include <sstream>
#include <vector>

#include <iCub/iha/serialization.h>
#include <iCub/iha/debug.h>

using namespace std;

namespace iCub {
	namespace iha {
		class Experience;
	}
}

/**
 * @ingroup icub_iha_ExperienceMetricSpace
 *
 * \brief A single Experience.  This holds a reference to sensor data in a DataStore along with ID information.
 *
 * Also stored are associated actions (and action frequencies for merged Experiences) and the associated Motivation/Reward value.
 *
 */
class iCub::iha::Experience
{
public:
	int eid;
	int dsIndex;
	int horizon;

	static int numActions;

private:
	double value;
	int action;
	vector< int >* actions;
	vector< int >* actionfreqs;
	int actioncount;

	//Experience* parent;
	//double distToParent;

public:
	Experience() {
		//parent=NULL;
		actions = new vector < int >;
		actionfreqs = new vector < int >;
		for (int i=0;i<numActions;i++) {
			actionfreqs->push_back(0);
		}
		actioncount=0;
		eid=-1;
	}

	~Experience() { 
		delete actions;
		delete actionfreqs;
	}

	//void setEID(int id) { eid=id; }
	//int getEID() { return eid; }
	
	void addAction(int act) {
		IhaDebug::pmesg(DBGL_DEBUG1,"addAction: %d\n",act);
		IhaDebug::pmesg(DBGL_DEBUG2,"addAction: actions->size()=%d\n",actions->size());
		if (actions->size()==0) {
			action = act;
		}
		actions->push_back(act);
		IhaDebug::pmesg(DBGL_DEBUG2,"addAction: actionfreqs->size()=%d\n",actionfreqs->size());
		(*actionfreqs)[ (int) act ]++;
		actioncount++;
		IhaDebug::pmesg(DBGL_DEBUG2,"addAction: new action count %d\n",actioncount);
	}
	void addActions(vector < int > * acts) {
		for (int i=0;i<acts->size();i++) {
			addAction( (*acts)[i] );
		}
	}

	int getAction() { return action; }

	vector< int >* getActions() { return actions; }

	vector< int >* getActionFrequencies() { return actionfreqs; }

	int getActionCount() {return actioncount;}


	//bool isChild() {
	//	return parent==NULL?false:true;
	//}

	//Experience* getParent() { return parent; }

	//void setParent(Experience* exp) {
	//	parent = exp;
	//}

	double getValue() { return value; }

	void setValue(double v) { value=v; }

	void addValue(double v) { value+=v; }

	std::string toString() {
		ostringstream oss;
		oss << "Exp:" << eid
			<< " Dsi:" << dsIndex 
			<< " Hor:" << horizon 
			<< " Value: " << value
			<< " AF:";
		for (vector<int>::iterator it=actionfreqs->begin(); it!=actionfreqs->end(); it++) {
			oss << " " << *it;
		}
		//if (isChild()) {
		//	oss << " isChild of " << getParent()->eid;
		//}
		oss << endl;
		return oss.str();
	}

	void writeToStream(ostream &ofs) {
		ofs << eid <<FSEP<< horizon <<FSEP<< dsIndex <<FSEP<< value << endl;

		vector<int>::iterator it=actions->begin();
		while (it!=actions->end()) {
			ofs << *it <<FSEP;
			it++;
		}
		ofs << endl;
		
		it=actionfreqs->begin(); 
		while (it!=actionfreqs->end()) {
			ofs << FSEP << *it;
			it++;
		}
		ofs << endl;
		
	}

	void readFromStream(istream &ifs) {
		ifs >> eid;
		ifs >> horizon;
		ifs >> dsIndex;
		ifs >> value;

		string line;
		getline(ifs,line); // end of previous line

		int a;
		int c=0;

		{
			actions->clear();
			getline(ifs,line); // line with actions
			istringstream iss(line);
			while (iss >> a) {
				actions->push_back(a);
				c++;
			}
		}
		actioncount=c;

		{
			actionfreqs->clear();
			getline(ifs,line); // line with actionfreqs
			istringstream iss(line);
			while (iss >> a) {
				actionfreqs->push_back(a);
			}
		}

	}

};

#endif

