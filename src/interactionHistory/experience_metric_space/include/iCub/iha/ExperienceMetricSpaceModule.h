// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

#ifndef __EXPERIENCE_METRIC_SPACE_MODULE_H__
#define __EXPERIENCE_METRIC_SPACE_MODULE_H__

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

#include <stdio.h>
#include <string>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/Limits.h>
#include <map>
#include <iCub/iha/DataFrame.h>
#include <iCub/iha/ExperienceProcessor.h>
#include <iCub/iha/BinWindowMaxEntropy.h>

namespace iCub {
    namespace contrib {
        class ExperienceMetricSpaceModule;
    }
}


/**
 * Experience Metric Space Module class
 *
 * \brief See \ref icub_iha_ExperienceMetricSpace
 */
class iCub::contrib::ExperienceMetricSpaceModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::BufferedPort<yarp::os::Bottle> sensorsPort;
    yarp::os::Port statusPort;

    //-------------------------------------------
    // variables and switches read from 
    // configuration file

    int numBins; 
    int dimension;
    int granularity;
    int experience_action_gap;

    bool regular_experiences;
    bool action_experiences;
    bool value_experiences;

    int numImageSensorsX;
    int numImageSensorsY;

    bool adaptiveBinning;
    int adaptiveBinningWindowSize;
    int histogramResolution;

    bool writeMetricSpaceToPortFlag;
    bool writeCurrDistToPortFlag;
    int writeMaxDSPNeighbours;
    double writeMaxDSPRadius;

    double neighbourRadius;

    yarp::os::ConstString mergeAdaptType;
    double mergeThreshold, startingMergeThreshold;
    double mergeIncrement;
    int mergeExpThreshold;
    int mergeCycleTimeThreshold;

    bool onlyMergeSameActions;

    bool purgeExpSwitch;
    double purgeExpThreshold;

    int reward_index;
    int action_index;
    bool use_reward_action_in_exp;

    int futureHorizon;
    yarp::os::ConstString futureValueUpdateType;

    yarp::os::ConstString metricSpaceHeuristic;
    bool verifyHeuristic;
    int heuristicStartThreshold;
    double heuristicTreeRadius;

    double valueDecrement;

    bool loadArchive;
    bool saveArchive;
    std::string archiveLoadFile;
    std::string archiveSaveFile;
    //-------------------------------------------

    Limits limits;

	int dsnumber;

    int* horizons;
    int numHorizons;
    int longestHorizon;
    int numActions;

    // Long-term storage for (binned) sensor data
    // we use a map to keep a constant reference 
    // even if elements are deleted later
    std::map< int, DataFrame* > ds;

    // Experience Processors, one per horizon
    std::map< int, ExperienceProcessor* > exp_proc_map;

    // Counters for the main loop
	double lasttime;
	int num_comparisons;
	int num_deleted;
	int num_merged;
	int last_cycle_time;
	int act;
	int last_act;
	double value;
	double last_value;

    int dsIndex;
    int expid;
    int total_num_experiences;

	// adaptive binning calculators
	BinWindowMaxEntropy** ems;

public:

    ExperienceMetricSpaceModule();
    virtual ~ExperienceMetricSpaceModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

    void initDataStore();
    bool loadMetricSpace();
    void saveMetricSpace();
    void createMetricSpaces();
    void deleteMetricSpaces();
    
};


#endif
