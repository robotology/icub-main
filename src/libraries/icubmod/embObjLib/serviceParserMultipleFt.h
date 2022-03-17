#pragma once

#include <yarp/os/Bottle.h>

#include <map>
#include <string>

using namespace yarp::os;

class servAScollector_t;

class ServiceParserMultipleFt
{
   public:
	ServiceParserMultipleFt(servAScollector_t& asService);
	bool parse(yarp::os::Searchable& config);

   protected:
	virtual bool checkPropertyCanBoards(const Bottle& bPropertiesCanBoards, bool& formaterror);
	virtual bool checkPropertySensors(const Bottle& property, bool& formaterror);
	virtual bool checkSettings(const Bottle& settings, bool& formaterror);
	virtual bool checkServiceType(const Bottle& service, bool& formaterror);
	virtual bool checkCanMonitor(const Bottle& service, bool& formaterror);

   private:
	const std::map<std::string, eObrd_canmonitor_reportmode_t> stringToReport = {{"NEVER", eobrd_canmonitor_reportmode_NEVER},
																				 {"LOSTFOUND", eobrd_canmonitor_reportmode_justLOSTjustFOUND},
																				 {"LOSTFOUNDLOST", eobrd_canmonitor_reportmode_justLOSTjustFOUNDstillLOST},
																				 {"ALL", eobrd_canmonitor_reportmode_ALL}};

	servAScollector_t& asService_;
};