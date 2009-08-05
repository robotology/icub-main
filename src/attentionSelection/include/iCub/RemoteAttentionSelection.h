// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2009 Jonas Ruesch
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef ICUB_REMOTEATTENTIONSELECTION_INC
#define ICUB_REMOTEATTENTIONSELECTION_INC

// std
#include <iostream>

// yarp
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Port.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Network.h>

// iCub
#include <iCub/AttentionSelectionInterfaces.h>

namespace iCub {
	namespace contrib {
		class RemoteAttentionSelection;
	}
}

/**
* Remote access to AttentionSelection module
*/
class iCub::contrib::RemoteAttentionSelection : 
	public yarp::dev::DeviceDriver,
	public IAttentionSelectionControls{

public:

	RemoteAttentionSelection();
	virtual ~RemoteAttentionSelection();

	virtual bool open(const char *name);
	virtual bool close();

	// IAttentionSelectionInterfaces
	virtual void setInhibitOutput(bool on);
	virtual bool getInhibitOutput();

private:

	yarp::os::Port configPort;
	void setCommand(int code);
	void setCommand(int code, double v);
	void setCommand(int code, int v);
	void setCommand(int code, std::string s);
	bool getCommand(int code, double& v) const;
	bool getCommand(int code, int& v) const;
	bool getCommand(int code, std::string& s) const;
	void setDouble (int code, int j, double val);
	void setDoubleBottle(int v, yarp::os::Bottle &bot);
	bool getDouble(int v, int j, double *val);
	bool getDoubleBottle(int v, yarp::os::Bottle &bot);
	bool getString(int code, int j, std::string& s);
	void setString(int code, int j, std::string s);
};

#endif

