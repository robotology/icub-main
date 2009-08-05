// include header
#include <includes.h>

// ***************************************************************************

/**
 *
 * Implementation for including and providing paramaters to all running modules/classes
 *
 */

Parameters::Parameters(string _config_file) {
	//printf("try to locate your config file here: %s\n", config_file.c_str());
	string filename = _config_file;

	this->properties.fromConfigFile(_config_file.c_str());
	if (!this->properties.check("EXECUTION")) {
		filename = string("../") + _config_file;
		this->properties.fromConfigFile(filename.c_str());
		if (!this->properties.check("EXECUTION")) {
			printf("Cannot understand config file\n");
			exit(1);
		}
		else {
			this->linuxFlag = false;
		}
	}
	else {
		this->linuxFlag = true;
	}


	this->config_file = filename;
}

Parameters::~Parameters() {
}


int Parameters::getIntParameter(const char *_groupName_ptr, const char *_paramName_ptr) {

    int param = this->properties.findGroup(_groupName_ptr).find(_paramName_ptr).asInt();
	//printf("get parameter [%s]\t\t%s \t\t %d\n", _groupName_ptr, _paramName_ptr,  param);
	return param;
	
}

double Parameters::getDoubleParameter(const char *_groupName_ptr, const char *_paramName_ptr) {

    double param = this->properties.findGroup(_groupName_ptr).find(_paramName_ptr).asDouble();
	//printf("get parameter [%s]\t\t%s \t\t %f\n", _groupName_ptr, _paramName_ptr,  param);
	return param;
	
}

string Parameters::getTextParameter(const char *_groupName_ptr, const char *_paramName_ptr) {
	
    string param = this->properties.findGroup(_groupName_ptr).find(_paramName_ptr).toString().c_str();
	//printf("get parameter [%s]\t\t%s \t\t <%s>\n", _groupName_ptr, _paramName_ptr,  param.c_str());
	return param; //.c_str();
}

bool Parameters::getBool(const char *_onOff_ptr) {
	if (string(_onOff_ptr) == "on") {
		return true;
	}
	else {
		return false;
	}
}

void Parameters::setDoubleParameter(double *_paramName_ptr, double _paramValue) {
	*_paramName_ptr	= _paramValue;
}

void Parameters::setIntParameter(int *_paramName_ptr, int _paramValue){
	*_paramName_ptr = _paramValue;
}


void Parameters::changeParameter(string _paramName_ptr) {
	// display & recording settings
	printf("change parameter %s\n", _paramName_ptr.c_str());
	if (_paramName_ptr == "display") {
		this->printAndAsk(_paramName_ptr.c_str(), &this->display);
	}
	else if (_paramName_ptr == "perception") {
		this->printAndAsk(_paramName_ptr.c_str(), &this->rec_perception);
	}
	else if (_paramName_ptr == "result") {
		this->printAndAsk(_paramName_ptr.c_str(), &this->rec_result);
	}
	else if (_paramName_ptr == "stats") {
		this->printAndAsk(_paramName_ptr.c_str(), &this->rec_statistics);
	}
}

void Parameters::printAndAsk(const char *_param_ptr, bool *_value_ptr) {
	char	tmp [100]	= "";
	string	value;
	if (*_value_ptr) {
		value = "on";
	}
	else {
		value = "off";
	}

	sprintf(tmp, "Current %s value is now: %s. Enter your new choice:", _param_ptr, value);
	cout << tmp << endl;
	cin >> value;
	//tmp = "";
	sprintf(tmp, "Now, %s is: %s.", _param_ptr, *_value_ptr);
	if (value == "on") {
		*_value_ptr = true;
	}
	else {
		*_value_ptr = false;
	}

	cout << tmp << endl;
}


void Parameters::printAndAsk(const char *_param_ptr, int *_value_ptr) {
	char	tmp [100]	= "";
	int		value;
	sprintf(tmp, "Current %s value is now: %d. Enter your new choice:", _param_ptr, *_value_ptr);
	cout << tmp << endl;
	cin >> value;
	*_value_ptr = value;
	sprintf(tmp, "Now, %s is: %d.", _param_ptr, *_value_ptr);
	cout << tmp << endl;
}

void Parameters::printAndAsk(const char *_param_ptr, double *_value_ptr) {
	char	tmp [100]	= "";
	int		value;
	sprintf(tmp, "Current %s value is now: %f. Enter your new choice:", _param_ptr, *_value_ptr);
	cout << tmp << endl;
	cin >> value;
	*_value_ptr = value;
	sprintf(tmp, "Now, %s is: %f.", _param_ptr, *_value_ptr);
	cout << tmp << endl;
}

void Parameters::printAndAsk(const char *_param_ptr, string *_value_ptr) {
	char	tmp [100]	= "";
	int		value;
	sprintf(tmp, "Current %s value is now: %s. Enter your new choice:", _param_ptr, *_value_ptr);
	cout << tmp << endl;
	cin >> value;
	*_value_ptr = value;
	sprintf(tmp, "Now, %s is: %s.", _param_ptr, *_value_ptr);
	cout << tmp << endl;
}

void Parameters::loadParameters() {

	// *****************************************************************************
	//
	//                             Execution settings
	//
	// *****************************************************************************

	this->execution			= this->getTextParameter("EXECUTION", "Source");
	this->stereovision		= this->getBool(this->getTextParameter("EXECUTION", "Stereovision").c_str());

	if (this->execution == "robot") {
		this->cam1_src		= this->getTextParameter("DATASOURCES", "Robot_cam1");
		this->cam2_src		= this->getTextParameter("DATASOURCES", "Robot_cam2");
	}
	else {
		this->cam1_src		= "/dump/cam/left";
		this->cam2_src		= "/dump/cam/right";
	}

	this->viewer1_dst		= this->getTextParameter("OUTPORTS", "viewer1_out_port");
	this->viewer2_dst		= this->getTextParameter("OUTPORTS", "viewer2_out_port");

	this->trackerLeft_out_port		= this->getTextParameter("OUTPORTS", "tracker_pos_left_out_port").c_str();
	this->trackerRight_out_port		= this->getTextParameter("OUTPORTS", "tracker_pos_right_out_port").c_str();

	this->nof_loops			= this->getIntParameter("NOF_EXECUTIONS", "Nof_loops");

	// port names (not to configure)
	this->cam1_out_port		= "/markerTracker/images/out/1";
	this->cam2_out_port		= "/markerTracker/images/out/2";

	this->cam1_in_port		= "/cam/in1";
	this->cam2_in_port		= "/cam/in2";
	//this->enc_in_port		= "/encs/in";


	// *****************************************************************************
	//
	//                  Recording and Display settings
	//
	// *****************************************************************************

	this->rec_perception	= this->getBool(this->getTextParameter("RECORDING", "Rec_perception").c_str());
	this->rec_result		= this->getBool(this->getTextParameter("RECORDING", "Rec_results").c_str());
	this->rec_statistics	= this->getBool(this->getTextParameter("RECORDING", "Rec_stats").c_str());
	this->rec_steps			= this->getBool(this->getTextParameter("RECORDING", "Rec_steps").c_str());

	this->display			= this->getBool(this->getTextParameter("DISPLAY", "display").c_str());

	// *****************************************************************************
	//
	//                  I/O settings (files and locations)
	//
	// *****************************************************************************
	this->stat_str				= this->getTextParameter("STATS", "FileOut_stat");
	this->halo_str				= this->getTextParameter("RESULT_LOC", "viewer1_record_dir");
	this->hade_str				= this->getTextParameter("RESULT_LOC", "viewer2_record_dir");
	
	this->statisticsFile		= this->stat_str.c_str();
	this->result_dir_1			= this->halo_str.c_str();
	this->result_dir_2			= this->hade_str.c_str();

	// *****************************************************************************
	//
	//                              Parameter settings
	//
	// *****************************************************************************


	// *****************************************************************************
	//
	//                      File mode settings (source definition)
	//
	// *****************************************************************************

	if (this->execution == "file") {
		this->dump1_source			= this->getTextParameter("SOURCE", "CamDump2Use_left");
		this->dump1_dir				= this->getTextParameter(this->dump1_source.c_str(), "dump_DIR");
		this->dump1_name			= this->getTextParameter(this->dump1_source.c_str(), "dump_NAME");
		this->dump1_end				= this->getTextParameter(this->dump1_source.c_str(), "dump_ENDING");
		this->dump1_cnt_len			= this->getIntParameter(this->dump1_source.c_str(), "dump_NUMLEN");
		this->dump1_zerofilling		= this->getBool(this->getTextParameter(this->dump1_source.c_str(), "dump_ZEROFILLING").c_str());
		this->dump1_min_filenum		= this->getIntParameter(this->dump1_source.c_str(), "dump_MINFILENUM");
		this->dump1_max_filenum		= this->getIntParameter(this->dump1_source.c_str(), "dump_MAXFILENUM");

		this->dump2_source			= this->getTextParameter("SOURCE", "CamDump2Use_right");
		this->dump2_dir				= this->getTextParameter(this->dump2_source.c_str(), "dump_DIR");
		this->dump2_name			= this->getTextParameter(this->dump2_source.c_str(), "dump_NAME");
		this->dump2_end				= this->getTextParameter(this->dump2_source.c_str(), "dump_ENDING");
		this->dump2_cnt_len			= this->getIntParameter(this->dump2_source.c_str(), "dump_NUMLEN");
		this->dump2_zerofilling		= this->getBool(this->getTextParameter(this->dump2_source.c_str(), "dump_ZEROFILLING").c_str());
		this->dump2_min_filenum		= this->getIntParameter(this->dump2_source.c_str(), "dump_MINFILENUM");
		this->dump2_max_filenum		= this->getIntParameter(this->dump2_source.c_str(), "dump_MAXFILENUM");
	}
	// *****************************************************************************
	//
	//                               runtime settings
	//
	// *****************************************************************************

	this->startTime			= 1;
	this->netCounter		= 0;
	this->exec_loop			= 0;
	this->merger_counter	= 0;
	this->interrupt_count	= 0;

}
