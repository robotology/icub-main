// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

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


#include "iCub/iha/ExperienceMetricSpaceModule.h"
#include <iostream>
#include <iCub/iha/iha_utils.h>

using iCub::iha::Limits;
using iCub::contrib::ExperienceMetricSpaceModule;

/**
\addtogroup icub_iha_ExperienceMetricSpace

\section icub_iha_ems_intro_sec Metric Spaces of Experience

The metric space is constructed continuously as the robot experiences its environment.  
- A new experience is created every \e Granularity \f$G\f$ timesteps, and consists of Horizon \f$h\f$ timesteps counting back from the current timestep. 
 - Where \f$h>G\f$ the experiences will overlap.  
- Each sensor reading is quantized into \f$Q\f$ evenly-sized bins.
- Each new quantized experience is compared to other experiences in order to determine its neighbours. 
This process, if all experiences are compared, results in a distance matrix between experiences which defines the structure of the metric space as it is experienced by an individual robot.

A \e quality value is assigned to the quantized experience, determined by factors such as environmental reward/punishment, internal drive and affective state.
- The actual formula for calculation of \e quality is specific to the application and goal and can be a determining factor in the eventual behaviour and course of development, although it can be fairly general and thus applicable to a wide range of situations.

Finally, the last action executed during the experience is also noted and stored with the quantized experience.

Thus the metric space of experience in the Interaction History Architecture, the \b interaction \b history \b space, can be described by the tuple \b \f$(\epsilon, D, q, a)\f$, where \b \f$\epsilon\f$ is a collection of quantized "experiences", \b \f$D\f$ is the a matrix of distances between elements of \b \f$\epsilon\f$, \b \f$q\f$ is a vector of quality values and \b \f$ a \f$ a vector of actions.

\subsection Finding Nearest Neighbours
Say one is interested in finding all nearest neighbours of an experience \f$E^{new}\f$ within a "ball" of radius \f$r\f$, then the triangle inequality can be employed to reduce the number of distances that need to be measured. Specifically:

-  Given an experience \f$E^k\f$ that is distance \f$d(E^{new},E^k)\leq r\f$ from \f$E^{new}\f$, then any neighbours of \f$E^k\f$ that are further away than \f$2r\f$ are not within distance \f$r\f$ of \f$E^{new}\f$.
 \end{theorem}

(for proof see Mirza:Thesis)

This fact can be used discard experiences from consideration when finding nearest neighbours within a specified radius. Of course, this requires first finding an experience with radius \f$r\f$ of the new experience.  
One approach to this problem is to simply randomly sample the experience space until one is found.  
Other strategies exist, for example: using the continuous nature of the environment to start the search for near experiences (in terms of information distance) with those experiences near in terms of time. 
 - These can be configured using the \c --metricSpaceHeuristic switch and associated parameters

\subsection icub_iha_ems_merging  Clustering Experiences

The merging strategy in the Interaction History Architecture is to merge any two experiences closer than a threshold \f$T_{merge}\f$ (parameter \c --mergeThreshold).

When two experiences are merged, the resulting experience takes the experience number of the oldest experience and that experience's data frame (i.e. in terms of sensory data and comparison to other experiences it is identical to that experience).
It's "mass" is the sum of the two experience masses and the action frequencies are also merged.

- \f$T_{merge}\f$ can be fixed or can be adapted.  This is controlled by the \c --mergeAdaptType switch and the \c --mergeIncrement parameter.
  - \c --mergeAdaptType \c NONE No adaptation of threshold: always merge experience closer than \c --mergeThreshold
  - \c --mergeAdaptType \c CYCLE_TIME Increase/decrease the merge threshold to maintain a given cycle time set in \c --mergeCycleTimeThreshold
  - \c --mergeAdaptType \c NUM_COMPARISONS Increase/decrease the merge threshold to maintain a given number of comparisons set in \c --mergeExpThreshold
- A further switch \c --onlyMergeSameActions will restrict merging to experiences having the same action

\subsection icub_iha_ems_forgetting  Forgetting Experiences
Experiences may also be deleted, that is, forgotten.  
This serves two particular purposes in the present architecture.  
- The first is to provide a mechanism where the interaction history space can be continually modified and so be adaptive to changes in the environmental interaction.  
- The second, more practically, is to reduce the number of experiences in the space and so reduce computational complexity in estimating distances to new experiences inserted into the space.

The deletion is controlled using the two parameters \c --purgeExpSwitch and \c --purgeExpThreshold

\subsection icub_iha_ems_future Update of Environmental Reward

Each experience in the interaction history space is associated with a quality value \f$q\f$.  
This value has bearing on the selection of the experience, and in turn on the action-selection process.  
The quality value is intended to reflect how useful the experience is in terms of positive or negative environmental feedback, and is derived directly from the internal reward function or an external reward measured by the robot's sensors.

In the simplest case, the immediate (instantaneous) reward received from the environment is associated with the current experience.  
An alternative scheme is for the quality associated with an experience to be dependent not only on the current reward, but also on the future reward. 

The \e future reward for an experience \f$E_{t,h}\f$ for some given horizon \f$h_{future}\f$ (set by the \c --futureHorizon parameter) is a function \f${\mathcal F}()\f$ on all reward values received for \f$h_{future}\f$ timesteps after time \f$t\f$. Of course, this value cannot be known completely until at least \f$h_{future}\f$ timesteps have passed, but it is estimated until that point.  

Two functions have been used in the implementation (Controlled by the \c --futureValueUpdateType switch:
- The first \c --futureValueUpdateType=MAX, \f${\mathcal F}_{min\_max}()\f$, returns the most proximal maximum or minimum reward.  
- The second \c --futureValueUpdateType=BIASED, \f${\mathcal F}_{max}\f$ simply returns the maximum reward over the horizon.


\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
---------------------------------------------------------------------------
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : config file
--save [STR] : Save data to file at end of process
--load [STR] : Load complete data store from file  before starting
---------------------------------------------------------------------------
--dsnumber [INT]  : Number of this data store
--horizon [INT]  : Horizon length
--num_bins [INT]  : number of bins for quantizing data
--granularity [INT]  : number of timesteps between experiences
---------------------------------------------------------------------------
--connect_to_sensors [STR]       : connect to specified port for sensors
---------------------------------------------------------------------------
--experience_action_gap [INT]       : delay to associated action
--regular_experiences [T/F]         : create exp at regular timestep
--action_experiences [T/F]          : create exp when action changes
--value_experiences [T/F]           : create exp when feedback reward value changes
--numActions [INT]                  : number of actions defined

--writeCurrDistToPortFlag [T/F]     : Write current distance to port
--writeMaxDSPNeighbours [INT]       : Max number of neighbours to write
--writeMaxDSPRadius [FLT]           : Max radius of experiences that are written in neighbour list
--neighbourRadius [FLT]             : Radius for experience to be considered as a neighbour

--mergeAdaptType [STR]              : Type of adaptation: NONE, CYCLE_TIME or NUM_COMPARISONS
--mergeThreshold [FLT]              : merge experiences closer than this threshold distance
--mergeIncrement [FLT]              : the amount that the threshold is inc/dec when adapting
--mergeExpThreshold [INT]           : for mergeAdaptType=NUM_COMPARISONS this is threshold
--mergeCycleTimeThreshold [INT]     : for mergeAdaptType=CYCLE_TIME Merge experiences if cycle time exceeds this
--onlyMergeSameActions [T/F]        : Only merge experiences associated with the same actions

--purgeExpSwitch [T/F]              : Carry out purging
--purgeExpThreshold [FLT]           : Purge experiences with reward value less than this

--adaptiveBinning [T/F]             : Use Adaptive Binning
--adaptiveBinningWindowSize [INT]   : Adaptive Binning: Window size
--histogramResolution [INT]         : Adaptive Binning: Histogram resolution

--futureHorizon [INT]               : Horizon over which Reward feeds back
--futureValueUpdateType [STR]       : MAX (feedback Max horizon) or BIASED (Feeback Lowest or Highest)

--metricSpaceHeuristic [STR]        : Heuristic used to speed up.  NONE, TREE or TREENEIGHBOUR
--verifyHeuristic [T/F]             : Output check data for Heuristic
--heuristicStartThreshold [INT]     : Parameter for TREE Heuristic
--heuristicTreeRadius [FLT]         : Parameter for TREE Heuristic
--------------------------------------------------------------------------
\endverbatim

\section portsa_sec Ports Accessed
- /iha/sm/sensor:out  - IHA Module SensorMotorInterfaceModule sensor output port
 - can be automatically connected using the --connect_to_sensors parameter

- /iha/status/monitor:in - status information collection port

\section portsc_sec Ports Created
- /iha/ds/data:in  - for reading the sensor data
 
- /iha/as/status:out - to write status info for monitor process
- /iha/as/quit  - module quit port

\section conf_file_sec Configuration Files
conf/ihaExperienceMetricSpace.ini

Sample INI file:
\verbatim
# The data store number
dsnumber 1

############################################################
# Actions
#

num_actions 21

#
############################################################

############################################################
# section has the names of the sensors and the hi/lo range
# to allow binning calculations
# remember to put reward and action as last two items

SENSORS HEAD_PITCH HEAD_YAW HEAD_PAN EYES_UD EYES_RL EYES_CD LSH_ROT LSH_ELV LSH_TWST LELB_FLX LELB_TWST LWR_ABD LWR_FLX LDIG_1 LDIG_2 LDIG_3 LDIG_4 LDIG_5 LDIG_6 LDIG_7 LDIG_8 LDIG_9 RSH_ROT RSH_ELV RSH_TWST RELB_FLX RELB_TWST RWR_ABD RWR_FLX RDIG_1 RDIG_2 RDIG_3 RDIG_4 RDIG_5 RDIG_6 RDIG_7 RDIG_8 RDIG_9 FACE SOUNDS ACTION REWARD

#          0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41

LIMIT_HI  30  60  55  15  52  45  90 161 100 106  90  10  40  30 105  90  90  90  90  90  90 120  90 161 100 106  90  10  40  30 105  90  90  90  90  90  90 120  1   1  21   1
LIMIT_LO -40 -70 -55 -35 -50  -1 -96  -5 -39  -2 -90 -90 -20 -10 -15  -2  -2  -2  -2  -2  -2  -2 -96  -5 -39  -2 -90 -90 -20 -10 -15  -2  -2  -2  -2  -2  -2  -2  0   0   0   0

reward_index 41
action_index 40

num_image_sensors_x 4
num_image_sensors_y 4

#
###########################################################

############################################################
# Main configuration settings for experience creation
numBins 5
granularity 2
HORIZONS 20
experience_action_gap 2
#
############################################################

############################################################
# Updating Future Value
# can be MAX which always gives max value
# or BIASED which gives 0 or MAX

future_value_update_type MAX
future_horizon 40
#
############################################################

############################################################
# Writing of metric space or neighbour list

# just send current neighbours
write_curr_dist_to_port TRUE
# send the whole space (I have never used this it is silly)
write_ms_to_port FALSE

# These set how many neighbours are sent in a neighbour list
# You can choose to limit number directly or the maximum 
# radius or both
write_max_dsp_neighbours 6
write_max_dsp_radius 3.0

# This setting gives a max radius for the neighbour list for
# the purposes of working out relative probablities in the
# experience selection
neighbour_radius 3.0
#
############################################################

############################################################
# TEMPERATURE
# start temp
#temperature 3.0
temperature 4.0
# cooling (use instead of adapt_temp)
# 0 = no cooling
temp_dec 0.01

# set to adapt temp according to reward
adapt_temp FALSE

# adaptation rates and limits
# so far only used in the pioneer code
adapt_rate_inc 0.4
adapt_rate_dec 0.2
# if there is a low reward, the minimum temperature is this
min_temp_lo_reward 1.0
# if there is a high reward, the maximum temperature is this
max_temp_hi_reward 0.2
max_temperature 3.0
#
############################################################

###########################################################
# Merging
#
merge_threshold 0.20
only_merge_same_actions FALSE
#merge_threshold 1.60
#only_merge_same_actions FALSE

# NONE, CYCLE_TIME, NUM_COMPARISIONS
merge_adapt_type NONE

merge_increment 0.01
merge_exp_threshold 300
merge_cycle_time_threshold 600
#
###########################################################

###########################################################
# Forgetting
#
# purge experiences based on value
purge_experiences TRUE
purge_threshold 0.8


###########################################################
# Adaptive Binning
#
adaptive_binning FALSE
adaptive_binning_window_size 96

histogram_resolution 256
#
###########################################################

###########################################################
# Metric Space calculation speed-up
#
# Types available : NONE TREE TREENEIGHBOUR

metric_space_heuristic NONE
heuristic_start_threshold 20
heuristic_tree_radius 1.5
verify_heuristic FALSE
#
###########################################################

############################################################
# Experience selection feedback
# controls modification of experience space depending on
# reward feedback as a result of action taken from the
# selected experience
#
# Currently not implemented due to delayed reward updating
# Instead merging plays a role in this
#
experience_feedback TRUE
ef_value_increment 0.1
ef_value_decrement 0.1
#
############################################################
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaExperienceMetricSpace --name /iha/ds --file conf/ihaExperienceMetricSpace.ini --connect_to_sensors /iha/sm/sensor:out --dbg 30

See script $ICUB_ROOT/app/iha_manual/iha_datastore.sh

\see iCub::iha::ExperienceProcessor
\see iCub::iha::DistanceSpaceClass
\see iCub::iha::DataFrame
\see iCub::iha::Experience
\see iCub::iha::Limits
\see iCub::iha::WindowIDCalc
\see iCub::iha::BinWindowMaxEntropy
\see iCub::contrib::ExperienceMetricSpaceModule

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/experience_metric_space/src/ExperienceMetricSpaceModule.cpp.
 */

ExperienceMetricSpaceModule::ExperienceMetricSpaceModule(){
}

ExperienceMetricSpaceModule::~ExperienceMetricSpaceModule(){ 
}


bool ExperienceMetricSpaceModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
        << "  --save [STR] : Save data to file at end of process" << "\n"
        << "  --load [STR] : Load complete data store from file  before starting" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "  --dsnumber [INT]  : Number of this data store" << "\n"
		<< "  --horizon [INT]  : Horizon length" << "\n"
		<< "  --num_bins [INT]  : number of bins for quantizing data" << "\n"
		<< "  --granularity [INT]  : number of timesteps between experiences" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --connect_to_sensors [STR]       : connect to specified port for sensors" << "\n"
		<< "---------------------------------------------------------------------------------------------------------" << "\n"
        << "  --experience_action_gap [INT]       : delay to associated action" << "\n"
        << "  --regular_experiences [T/F]         : create exp at regular timestep" << "\n"
        << "  --action_experiences [T/F]          : create exp when action changes" << "\n"
        << "  --value_experiences [T/F]           : create exp when feedback reward value changes" << "\n"
        << "  --numActions [INT]                  : number of actions defined" << "\n"
        << "  --writeCurrDistToPortFlag [T/F]     : Write current distance to port" << "\n"
        << "  --writeMaxDSPNeighbours [INT]       : Max number of neighbours to write" << "\n"
        << "  --writeMaxDSPRadius [FLT]           : Max radius of experiences that are written in neighbour list" << "\n"
        << "  --neighbourRadius [FLT]             : Radius for experience to be considered as a neighbour" << "\n"
        << "  --mergeAdaptType [STR]              : Type of adaptation: NONE, CYCLE_TIME or NUM_COMPARISONS" << "\n"
        << "  --mergeThreshold [FLT]              : merge experiences closer than this threshold distance" << "\n"
        << "  --mergeIncrement [FLT]              : the amount that the threshold is inc/dec when adapting" << "\n"
        << "  --mergeExpThreshold [INT]           : for mergeAdaptType=NUM_COMPARISONS this is threshold" << "\n"
        << "  --mergeCycleTimeThreshold [INT]     : for mergeAdaptType=CYCLE_TIME Merge experiences if cycle time exceeds this" << "\n"
        << "  --onlyMergeSameActions [T/F]        : Only merge experiences associated with the same actions" << "\n"
        << "  --purgeExpSwitch [T/F]              : Carry out purging" << "\n"
        << "  --purgeExpThreshold [FLT]           : Purge experiences with reward value less than this" << "\n"
        << "  --adaptiveBinning [T/F]             : Use Adaptive Binning" << "\n"
        << "  --adaptiveBinningWindowSize [INT]   : Adaptive Binning: Window size" << "\n"
        << "  --histogramResolution [INT]         : Adaptive Binning: Histogram resolution" << "\n"
        << "  --futureHorizon [INT]               : Horizon over which Reward feeds back" << "\n"
        << "  --futureValueUpdateType [STR]       : MAX (feedback Max horizon) or BIASED (Feeback Lowest or Highest)" << "\n"
        << "  --metricSpaceHeuristic [STR]        : Heuristic used to speed up.  NONE, TREE or TREENEIGHBOUR" << "\n"
        << "  --verifyHeuristic [T/F]             : Output check data for Heuristic" << "\n"
        << "  --heuristicStartThreshold [INT]     : Parameter for TREE Heuristic" << "\n"
        << "  --heuristicTreeRadius [FLT]         : Parameter for TREE Heuristic" << "\n"
		<< "---------------------------------------------------------------------------------------------------------" << "\n"

		<< "For further information on these parameters see the ini file" << "\n"

		<< "\n";
        return false;
    }

    bool ok = true;

	//------------------------------------------------------
	// config file read: switches
	IhaDebug::pmesg(DBGL_INFO,"Configuration:\n");

	dsnumber = config.check("dsnumber",Value(1)).asInt();
    IhaDebug::pmesg(DBGL_INFO,"Data Store number %d\n",dsnumber);

	saveArchive=false;
	loadArchive=false;
	if (config.check("save")) { 
        saveArchive = true; archiveSaveFile = config.find("save").asString(); 
        IhaDebug::pmesg(DBGL_INFO,"Will save datastore to %s\n",archiveSaveFile.c_str());
    }
	if (config.check("load")) { 
        loadArchive = true; archiveLoadFile = config.find("load").asString(); 
        IhaDebug::pmesg(DBGL_INFO,"Will load datastore from %s\n",archiveLoadFile.c_str());
    }
	
	numBins = config.check("num_bins",Value(5)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"number of bins %d\n",numBins);
	granularity = config.check("granularity",Value(1)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"granularity %d\n",granularity);

	experience_action_gap = config.check("experience_action_gap",Value(1)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"experience_action_gap %d\n",experience_action_gap);

	regular_experiences = boolStringTest(config.check("regular_experiences",Value("TRUE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"regular_experiences %s\n",regular_experiences?"TRUE":"FALSE");
	action_experiences = boolStringTest(config.check("action_experiences",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"action_experiences %s\n",action_experiences?"TRUE":"FALSE");
	value_experiences = boolStringTest(config.check("value_experiences",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"value_experiences %s\n",value_experiences?"TRUE":"FALSE");

    //load num actions from action defs file to make sure it is correct value
	ConstString action_defs_file = config.check("action_defs",Value("action_defs.txt")).asString();
	Property actiondefs_props;
	actiondefs_props.fromConfigFile(action_defs_file.c_str()); 
    if (!actiondefs_props.check("NUM_ACTIONS")) {
		return false;
	}
	numActions = actiondefs_props.find("NUM_ACTIONS").asInt();
	//numActions = config.check("num_actions",Value(4)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"num_actions %d\n",numActions);
	iCub::iha::Experience::numActions = numActions;
	
	writeMetricSpaceToPortFlag = boolStringTest(config.check("write_ms_to_port",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"write_ms_to_port %s\n",writeMetricSpaceToPortFlag?"TRUE":"FALSE");

	writeCurrDistToPortFlag = boolStringTest(config.check("write_curr_dist_to_port",Value("TRUE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"write_curr_dist_to_port %s\n",writeCurrDistToPortFlag?"TRUE":"FALSE");

	writeMaxDSPNeighbours = config.check("write_max_dsp_neighbours",Value(0)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"write_max_dsp_neighbours %d\n",writeMaxDSPNeighbours);

	writeMaxDSPRadius = config.check("write_max_dsp_radius",Value(0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"write_max_dsp_radius %f\n",writeMaxDSPRadius);

	neighbourRadius = config.check("neighbour_radius",Value(1.0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"neighbour_radius %f\n",neighbourRadius);

	mergeAdaptType = config.check("merge_adapt_type",Value("NONE")).asString();
	IhaDebug::pmesg(DBGL_INFO,"Merge Adapt Type:  %s\n",mergeAdaptType.c_str());
	mergeThreshold = config.check("merge_threshold",Value(0.0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"    Threshold =  %f\n",mergeThreshold);
	startingMergeThreshold=mergeThreshold;
	mergeIncrement = config.check("merge_increment",Value(0.01)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"    Increment =  %f\n",mergeIncrement);
	
	onlyMergeSameActions = boolStringTest(config.check("only_merge_same_actions",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"only_merge_same_actions %s\n",onlyMergeSameActions?"TRUE":"FALSE");

	purgeExpSwitch = boolStringTest(config.check("purge_experiences",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"purge_experiences %s\n",purgeExpSwitch?"TRUE":"FALSE");
	purgeExpThreshold = config.check("purge_threshold",Value(0.0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"    Threshold =  %f\n",purgeExpThreshold);

	if (mergeAdaptType=="NUM_COMPARISONS") {
		mergeExpThreshold = config.check("merge_exp_threshold",Value(400)).asInt();
		IhaDebug::pmesg(DBGL_INFO,"    Target # Comparisons =  %d\n",mergeExpThreshold);
	}
	if (mergeAdaptType=="CYCLE_TIME") {
		mergeCycleTimeThreshold = config.check("merge_cycle_time_threshold",Value(400)).asInt();
		IhaDebug::pmesg(DBGL_INFO,"    Target Cycle Time =  %d\n",mergeCycleTimeThreshold);
	}

	adaptiveBinning = boolStringTest(config.check("adaptive_binning",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"adaptive_binning %s\n",adaptiveBinning?"TRUE":"FALSE");
	adaptiveBinningWindowSize = config.check("adaptive_binning_window_size",Value(32)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"adaptive_binning_window_size %d\n",adaptiveBinningWindowSize);
	histogramResolution = config.check("histogram_resolution",Value(256)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"histogram_resolution %d\n",histogramResolution);

	futureHorizon = config.check("future_horizon",Value(200)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"future_horizon%d\n",futureHorizon);
	futureValueUpdateType = config.check("future_value_update_type",Value("MAX")).asString();
	IhaDebug::pmesg(DBGL_INFO,"future_value_update_type %s\n",futureValueUpdateType.c_str());

	metricSpaceHeuristic = config.check("metric_space_heuristic",Value("NONE")).asString();
	IhaDebug::pmesg(DBGL_INFO,"metric_space_heuristic %s\n",metricSpaceHeuristic.c_str());
	verifyHeuristic = boolStringTest(config.check("verify_heuristic",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"verify_heuristic %s\n",verifyHeuristic?"TRUE":"FALSE");
	heuristicStartThreshold = config.check("heuristic_start_threshold",Value(40)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"heuristic_start_threshold %d\n",heuristicStartThreshold);
	heuristicTreeRadius = config.check("heuristic_tree_radius",Value(1.0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"heuristic_tree_radius %f\n",heuristicTreeRadius);

	valueDecrement = config.check("value_decrement",Value(0.001)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"value_decrement %f\n",valueDecrement);

	// config file read: image pixelation
	//
	numImageSensorsX = config.check("num_image_sensors_x",Value(8)).asInt();
	numImageSensorsY = config.check("num_image_sensors_y",Value(8)).asInt();
	
	IhaDebug::pmesg(DBGL_INFO,"numBins: %d Granularity: %d ImgXY: %dx%d EAGap %d\n",numBins,granularity,numImageSensorsX,numImageSensorsY, experience_action_gap);
	
	use_reward_action_in_exp = boolStringTest(config.check("use_reward_action_in_exp",Value("TRUE")).asString());
	
	// read the number of the reward and action data fields
    int ts_offset = config.find("ts_offset").asInt();
	reward_index = config.find("reward_offset").asInt() + ts_offset;
	action_index = reward_index - 1; //must compute this way index in config no longer valid

	//------------------------------------------------------
	// read limits
	dimension = limits.readLimits(config, numImageSensorsX, numImageSensorsY);
	// if we don't want to use the reward and action fields
	// in the experience then reduce the dimension by 2
	// NOTE: these must always be the last two fields!!
	if (!use_reward_action_in_exp) {
		dimension = dimension - 2;
	}
	
	//------------------------------------------------------
	// Get the horizons from the config file
	Bottle& horizonsConfig =  config.findGroup("HORIZONS");
	
    horizons = new int [horizonsConfig.size()-1];

	IhaDebug::pmesg(DBGL_INFO,"Horizons :");
	int h=0;
	longestHorizon=0;
	for (int i=1;i<horizonsConfig.size();i++) {
		IhaDebug::pmesg(DBGL_INFO," %d", horizonsConfig.get(i).asInt());
		horizons[h] = horizonsConfig.get(i).asInt();
		if (horizons[h] > longestHorizon) {
			longestHorizon = horizons[h];
		}
		h++;
	}
	numHorizons=h;
	IhaDebug::pmesg(DBGL_INFO,". Total %d\n",numHorizons);

	//------------------------------------------------------
	// create names of ports
	char portpostfix[30];
	sprintf(portpostfix,"%d/data:in",dsnumber);
    ConstString sensorsPortName = getName(portpostfix);
	
	// open the coordinates reading port
	sensorsPort.open(sensorsPortName.c_str());

	// if required we can connect to the sensors
	if (config.check("connect_to_sensors")) {
		if (connectToParam(config,"connect_to_sensors",sensorsPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Sensors\n");
        } else {
            ok = false;
        }
	}

	//------------------------------------------------------
    // Create a port to write status to
    ConstString statusPortName = getName("status:out");
    statusPort.open(statusPortName.c_str());

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool ExperienceMetricSpaceModule::close(){

	if (saveArchive) {
        IhaDebug::pmesg(DBGL_INFO,"Saving Experience Space\n");
        saveMetricSpace();
        IhaDebug::pmesg(DBGL_INFO,"Save complete.\n");
    }

    sensorsPort.close();

	//----------------------------------------------------------------------
	// Cleanup
	
	// for each experience processor
	// send out a close signal (-1)
	for (map<int,ExperienceProcessor*>::iterator epit=exp_proc_map.begin(); 
		 epit!=exp_proc_map.end(); 
		 epit++) 
	{
		int hor = epit->first;
		ExperienceProcessor* ep = epit->second;
		if (writeCurrDistToPortFlag) ep->distPortSendClose();
	}
	
	// for each experience processor
	for (map<int,ExperienceProcessor*>::iterator epit=exp_proc_map.begin(); 
		 epit!=exp_proc_map.end(); 
		 epit++) 
	{
		// for the record
		epit->second->printDistanceSpace(DBGL_INFO);

		// close ports
		if (writeMetricSpaceToPortFlag) {
			epit->second->closeMSPort();
		}
		if (writeCurrDistToPortFlag) {
			epit->second->closeCDPort();
		}
		
		// delete experience processor
		IhaDebug::pmesg(DBGL_STATUS1,"data_store: Delete Distance Space hor=%d\n",epit->first);
		if (exp_proc_map[epit->first]) delete exp_proc_map[epit->first];
	}


	// delete the data store
	while (!ds.empty()) {
		delete(ds.begin()->second);
		ds.erase( ds.begin() );
	}
	
    return true;
}

bool ExperienceMetricSpaceModule::interruptModule(){
    sensorsPort.interrupt();
    return true;
}

bool ExperienceMetricSpaceModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	


void ExperienceMetricSpaceModule::initDataStore() {

	lasttime = Time::now();
	num_comparisons=0;
	num_deleted=0;
	num_merged=0;
	last_cycle_time=0;

	act=0;
	last_act=0;

	value=0;
	last_value=0;

    dsIndex=0;
    expid=0;
    total_num_experiences=0;

	// create adaptive binning calculators
	ems = allocAndCheck<BinWindowMaxEntropy*> (dimension);

	for (int d=0;d<dimension;d++) {
		ems[d] = new BinWindowMaxEntropy(numBins,adaptiveBinningWindowSize,histogramResolution);
	}

	//------------------------------------------------------
	// create some metric spaces
	IhaDebug::pmesg(DBGL_STATUS1,"data_store: Create MetricSpaces\n");
	
	// create the experience processors
	for (int i=0;i<numHorizons;i++) {
		ExperienceProcessor* ep = new ExperienceProcessor( &ds, 
		                                                  horizons[i], 
		                                                  numBins, 
		                                                  dimension);

		// store the EP reference against its horizon
		exp_proc_map[horizons[i]] = ep;
	}
	
	// create the ports in the processors
	for (int i=0;i<numHorizons;i++) {

		if (writeCurrDistToPortFlag) {
			//------------------------------------------------------
			// create a port for writing current distances 
            char portpostfix[30];
            sprintf(portpostfix,"currdist:out:%d",horizons[i]);
            ConstString distPortName = getName(portpostfix);

			IhaDebug::pmesg(DBGL_INFO,"Sending current distances (h=%d) data to port %s\n",horizons[i],distPortName.c_str());
			BufferedPort<Bottle> *cdport = new BufferedPort<Bottle>;
			cdport->open(distPortName.c_str());
			cdport->setStrict();
			exp_proc_map[horizons[i]]->setCDPort(cdport);
		}

		// set the initial merge threshold
		exp_proc_map[horizons[i]]->setMergeThreshold(mergeThreshold);

	}
	
}

void ExperienceMetricSpaceModule::deleteMetricSpaces() {
	for (int d=0;d<dimension;d++) {
		delete ems[d];
	}
	checkAndDestroy<BinWindowMaxEntropy*> (ems);

    // for each experience processor
    for (map<int,ExperienceProcessor*>::iterator epit=exp_proc_map.begin(); 
         epit!=exp_proc_map.end(); 
         epit++) 
    {
        ExperienceProcessor* ep = epit->second;
        delete ep;
    }


    delete [] horizons;
}

bool ExperienceMetricSpaceModule::updateModule(){

	// Read from the port and extract the timestep field
	//b = dataPortIn.read(true);
	Bottle* b = sensorsPort.read(true);

    if (isStopping()) {
        ACE_OS::fprintf(stderr,"ExperienceMetricSpaceModule: stopping\n");
        return false;
    }

	IhaDebug::pmesg(DBGL_DEBUG2,"RAW:%s\n",b->toString().c_str());
	int timestep = b->get(0).asInt();

	double cycletime = Time::now() - lasttime;
	last_cycle_time = (int) (cycletime*1000);

	// prepare a frame of data in the data store
	// Note ds is a map so this assignes a new mapping from a dsIndex to a DataFrame
	ds[dsIndex] =  new DataFrame();

	// put in the timestep of this data frame
	ds[dsIndex]->setTimestep(timestep);


	IhaDebug::pmesg(DBGL_STATUS1, "TS:%d Cycletime:%03.0f ms DSIndex:%d ExpId:%d #exp:%d #comp:%d #del:%d #merge:%d Th:%f\n",
			timestep, cycletime*1000, dsIndex, expid, total_num_experiences, num_comparisons, num_deleted, num_merged, mergeThreshold);
	IhaDebug::pmesg(DBGL_STATUSLINE, "TS:%d Cycletime:%03.0f ms DSIndex:%d ExpId:%d #exp:%d #comp:%d #del:%d #merge:%d Th:%f    \r",
			timestep, cycletime*1000, dsIndex, expid, total_num_experiences, num_comparisons, num_deleted, num_merged, mergeThreshold);

    writeStatus(statusPort,"ExpMetSpace","TS",timestep);
    writeStatus(statusPort,"ExpMetSpace","ExpID",expid);
    writeStatus(statusPort,"ExpMetSpace","DSIndex",dsIndex);

	// Loop through the rest of the fields. Normalize the sensor reading.
	// Add to an array
	int dim=0;

	//fprintf(stderr,"b->size()=%d 1=%f \n",b->size(),b->get(1).asDouble());
	
	// Quantize the data using simple linear binning or adaptive binning
	if (adaptiveBinning) {
		for (int i=1;i<b->size() && dim<dimension;i++) {
			// check range
			if (!limits.inRange(b->get(i).asDouble(), i-1)) ACE_OS::fprintf(stderr,"Limits Error : Value out of range S=%d : %f\n",i-1,b->get(i).asDouble());

			// normalise 0..1
			double norm = limits.normalize(b->get(i).asDouble(), i-1);

			// add data to the entropy maximization calculator
			ems[i-1]->putNextData(norm);

			// extract the latest max-entropy weighted binning for this data
			int abin = ems[i-1]->getBinMaxEntropy(norm);

			// put into the data frame
			ds[dsIndex]->addBinnedValue( (unsigned char) abin );

			//IhaDebug::pmesg(DBGL_DEBUG3,"Value: %f Norm %f ABinned %d\n",b->get(i).asDouble(), norm, ds[dsIndex]->getBinnedValue(i-1));
			dim++;
		}
	} else {
		for (int i=1;i<b->size() && dim<dimension;i++) {
			// check range
			if (!limits.inRange(b->get(i).asDouble(), i-1)) ACE_OS::fprintf(stderr,"Limits Error : Value out of range S=%d : %f\n",i-1,b->get(i).asDouble());

			// normalise 0..1
			double norm = limits.normalize(b->get(i).asDouble(), i-1);

			// add the binned value of this sensor value to the data frame
			ds[dsIndex]->addBinnedValue( getBin(norm,numBins) );

			IhaDebug::pmesg(DBGL_DEBUG3,"Value: %f Norm %f UBinned %d\n",b->get(i).asDouble(), norm, ds[dsIndex]->getBinnedValue(i-1));
			dim++;
		}
	}


	IhaDebug::pmesg(DBGL_DEBUG3, "dim %d\n",dim);
	// check that the data lines are complete
	// dimension will have been set from configuration file
	if (dim!=dimension) {
		ACE_OS::fprintf(stderr,"DS: Error dimension of data on port (%d) does not match configuration (%d)\n",dim,dimension);
	}

	// get reward value
	last_value = value;
	value=b->get(reward_index).asDouble();
	// and the action 
	last_act = act;
	act = b->get(action_index).asInt();

	IhaDebug::pmesg(DBGL_DEBUG2,"DS: index=%d action=%d value=%f\n",dsIndex,act,value);


	// add the action and value to the data frame
	ds[dsIndex]->setAction(act);
	ds[dsIndex]->setValue(value);

	// print the data frame for debug
	ds[dsIndex]->print(DBGL_DEBUG2);

	// Experiences are created EITHER every 'granularity' timesteps
	// OR when the action changes 
	// AND/OR when the value changes
	// also add the very first experience possible - ensures a distance list at 
	// the first proper experience
	// check granularity 
	if ( (regular_experiences && (dsIndex % granularity) == granularity-1)
		 ||
		 (action_experiences && (last_act!=act || dsIndex==horizons[0]) )
		 ||
		 (value_experiences && last_value!=value )
	   ) 
	{
		lasttime = Time::now();
		num_comparisons=0;
		//num_deleted=0;
		total_num_experiences=0;

		// for each experience processor
		for (map<int,ExperienceProcessor*>::iterator epit=exp_proc_map.begin(); 
			 epit!=exp_proc_map.end(); 
			 epit++) 
		{
			int hor = epit->first;
			ExperienceProcessor* ep = epit->second;

			ep->updateFutureValues(futureHorizon,futureValueUpdateType); // look forward futureHorizon ts
			ep->updateOldValues(valueDecrement); // look forward futureHorizon ts

			// create the experiences
			// and place in a metric space
			expid = ep->createExperience(dsIndex, act, value, total_num_experiences, experience_action_gap);

			if (expid==-1) continue;

			// process the experiences (i.e. populate metric space)
			if (metricSpaceHeuristic=="NONE") {
					ep->processExperiences(num_comparisons);
			} else {
				if (expid < heuristicStartThreshold) {
					ep->processExperiences(num_comparisons);
				} else {
					if (metricSpaceHeuristic=="TREE") {
						ep->processExperiencesTree(neighbourRadius*heuristicTreeRadius, num_comparisons);
					}
					if (metricSpaceHeuristic=="TREENEIGHBOUR") {
						ep->processExperiencesTreeNeighbours(neighbourRadius*heuristicTreeRadius, num_comparisons);
					}
					if (verifyHeuristic) {
						ep->verifyDistanceList();
					}
				}
			}

			if (expid>=0) {
				// write the Current Experience's Distance List to Ports
				if (writeCurrDistToPortFlag) ep->writeCurrDistToPorts(writeMaxDSPNeighbours, writeMaxDSPRadius);
			}

			// carry out the deletion strategy
			
			// Strategy: (ideal)
			// 1) if the distance is zero, we can always merge the experiences.
			// 2) always delete the experience with the lowest "value"
			// 3) if values are equal, delete the oldest
			// 4) Merging: if experiences are closer than some (dynamic) threshold, one can be deleted
			//    keeping its value and action information with the merged experience
			// 5) Forgetting: if experiences are of low value then they should be deleted 
			//    preferentially.
			//    This rule may need some work ... delete rarely used experiences? Should proximity
			//    to other experiences matter??


			// (2) Merge experiences close together
			// this should *not* delete the current experience
			ep->doMergeExperiences(num_merged, onlyMergeSameActions, futureHorizon);

			// (1) Purge experiences with QUALITY VALUE less than a threshold 
			if (purgeExpSwitch) ep->purgeExperiences(num_deleted, futureHorizon, purgeExpThreshold);

		}

		// adapt the merge threshold
		//
		// if there are too many comparisons, increase the threshold
		// which should bring the number of comparisons down
		if (mergeAdaptType=="NUM_COMPARISONS") {
			if (num_comparisons > mergeExpThreshold) {
				mergeThreshold+=mergeIncrement;
			} 
			if (num_comparisons < mergeExpThreshold && mergeThreshold > startingMergeThreshold) {
				mergeThreshold-=mergeIncrement;
			}
		}
		//
		// use cycle time ....
		if (mergeAdaptType=="CYCLE_TIME") {
			if (last_cycle_time > mergeCycleTimeThreshold) {
				mergeThreshold+=mergeIncrement;
			} 
			if (last_cycle_time < mergeCycleTimeThreshold && mergeThreshold > startingMergeThreshold) {
				mergeThreshold-=mergeIncrement;
			}
		}

		// and set it in the exp processors
		if (mergeAdaptType!="NONE") {
			for (map<int,ExperienceProcessor*>::iterator epit=exp_proc_map.begin(); 
				 epit!=exp_proc_map.end(); 
				 epit++) 
			{
				ExperienceProcessor* ep = epit->second;
				ep->setMergeThreshold(mergeThreshold); 
			}
		}
			
	}

	dsIndex++;

	return true;
}


bool ExperienceMetricSpaceModule::loadMetricSpace() {

	if (loadArchive) {
        string line;
        istringstream iss;
        string name;
        double version;
        ifstream ifs(archiveLoadFile.c_str());
        ifs >> name >> version;
        if (name != "EXPERIENCE_PROCESSOR_ARCHIVE_FILE") {
            ACE_OS::fprintf(stderr,"Error incorrect data file\n");
            return false;
        }
        if (version != 1.1) {
            ACE_OS::fprintf(stderr,"Error: incorrect version of data file (%f)\n",version);
            return false;
        }
        getline(ifs,line);// read the end of the line

        // get the header details
        // make sure the same number of horizons is in the data file
        int nh = 0;
        int h;
        getline(ifs,line);
        iss.str(line);
        while (iss >> h) {
            printf("h=%d\n",h);
            if (h!=horizons[nh]) {
                ACE_OS::fprintf(stderr,"Error: incorrect horizons\n");
                return false;
            }
            nh++;
        }
        if (nh!=numHorizons) {
            ACE_OS::fprintf(stderr,"Error: incorrect horizons\n");
            return false;
        }
        // OK - correct number of horizons ...
        // now data store
        getline(ifs,line);
        if (line!="DS_START") {
            ACE_OS::fprintf(stderr,"DataFile incorrect : no DS_START\n");
            return false;
        }
        getline(ifs,line);
        while (line != "DS_END") {
            iss.clear();
            iss.str(line);
            iss >> dsIndex;
            DataFrame *df = new DataFrame();
            df->readFromStream(iss);
            ds.insert(pair<int,DataFrame*>(dsIndex,df));
            getline(ifs,line);
        }
        dsIndex++; // this is the next index available for new data 

        // Experience Processors
        getline(ifs,line);
        if (line!="EP_START") {
            ACE_OS::fprintf(stderr,"DataFile incorrect : no EP_START\n");
            return false;
        }
        int epnum=0;
        getline(ifs,line);
        while (line != "EP_END") {
            int numexps=0, maxexp=0;
            ifs >> h;
            exp_proc_map[h]->readFromStream(ifs,numexps,maxexp);
            IhaDebug::pmesg(DBGL_STATUS1,"ExpProc #%d h=%d numexps: %d maxexps: %d\n",epnum,h,numexps,maxexp);
            epnum++;
            getline(ifs,line);
            if (maxexp>expid) expid=maxexp;
            total_num_experiences+=numexps;
        }
        ifs.close();
    }
    return true;
}

void ExperienceMetricSpaceModule::saveMetricSpace() {
    IhaDebug::pmesg(DBGL_INFO, "saveMetricSpace called\n");
	if (saveArchive) {
        ofstream ofs(archiveSaveFile.c_str());
        // start by saving a title line
        ofs << "EXPERIENCE_PROCESSOR_ARCHIVE_FILE 1.1" <<endl;
        // horizons
        for (int h=0;h<numHorizons;h++) {
            ofs << horizons[h] << FSEP;
        }
        ofs << endl;
        // data store
        ofs << "DS_START" << endl;
        for (map<int,DataFrame*>::iterator it=ds.begin(); it!=ds.end(); it++) {
            ofs << it->first << FSEP;
            it->second->writeToStream(ofs);
        }
        ofs << "DS_END" << endl;
        // Experience Processors
        ofs << "EP_START" << endl;
        for (map<int,ExperienceProcessor*>::iterator it=exp_proc_map.begin(); 
             it!=exp_proc_map.end(); 
             it++) 
        {
            ofs << endl << it->first << FSEP;
            it->second->writeToStream(ofs);
        }
        ofs << "EP_END" << endl;

        ofs.close();
    }

}
