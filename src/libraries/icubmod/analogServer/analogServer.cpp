


#include "analogServer.h"

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace std;

/**
  * Handler of the rpc port related to an analog sensor.
  * Manage the calibration command received on the rpc port.
  **/

AnalogServerHandler::AnalogServerHandler(const char* n)
{
    yTrace();
    rpcPort.open(n);
    rpcPort.setReader(*this);
}

AnalogServerHandler::~AnalogServerHandler()
{
    yTrace();
    rpcPort.close();
    is = 0;
}

void AnalogServerHandler::setInterface(yarp::dev::IAnalogSensor *is)
{
    yTrace();
    this->is = is;
}

bool AnalogServerHandler::_handleIAnalog(yarp::os::Bottle &cmd, yarp::os::Bottle &reply)
{
    yTrace();
    if (is==0)
      return false;

    int msgsize=cmd.size();

    int code=cmd.get(1).asVocab();
    switch (code)
    {
    case VOCAB_CALIBRATE:
      if (msgsize==2)
        is->calibrateSensor();
      else
      {
        //read Vector of values and pass to is->calibrate();
      }
      return true;
      break;
    case VOCAB_CALIBRATE_CHANNEL:
      if (msgsize==3)
      {
        int ch=cmd.get(2).asInt();
        is->calibrateChannel(ch);
      }
      if (msgsize==4)
      {
        int ch=cmd.get(2).asInt();
        double v=cmd.get(3).asDouble();
        is->calibrateChannel(ch, v);
      }

      return true;
      break;
    default:
      return false;
    }
}

bool AnalogServerHandler::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok=in.read(connection);
    if (!ok) return false;

    // parse in, prepare out
    int code = in.get(0).asVocab();
    bool ret=false;
    if (code==VOCAB_IANALOG)
    {
        ret=_handleIAnalog(in, out);
    }

    if (!ret)
    {
        out.clear();
        out.addVocab(VOCAB_FAILED);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender!=NULL) {
        out.write(*returnToSender);
    }
    return true;
}


/**
  * A yarp port that output data read from an analog sensor.
  * It contains information about which data of the analog sensor are sent
  * on the port, i.e. an offset and a length.
  */

AnalogPortEntry::AnalogPortEntry(void)
{
    yTrace();
}

AnalogPortEntry::AnalogPortEntry(const AnalogPortEntry &alt)
{
    yTrace();
    this->length = alt.length;
    this->offset = alt.offset;
    this->port_name = alt.port_name;
}

AnalogPortEntry &AnalogPortEntry::operator =(const AnalogPortEntry &alt)
{
    yTrace();
    this->length = alt.length;
    this->offset = alt.offset;
    this->port_name = alt.port_name;
    return *this;
}

/**
  * It reads the data from an analog sensor and sends them on one or more ports.
  * It creates one rpc port and its related handler for every output port.
  */

// Constructor used when there is only one output port
AnalogServer::AnalogServer(const char* name, int rate): RateThread(rate)
{
    yTrace();
    createPort(name, rate);
}

bool AnalogServer::createPort(const char* name, int rate)
{
    yTrace();
    analogSensor_p=0;
    analogPorts.resize(1);
    analogPorts[0].offset = 0;
    analogPorts[0].length = -1; // max length
    analogPorts[0].port_name = std::string(name);
    setHandlers();
    setRate(rate);
    return true;
}

// Contructor used when one or more output ports are specified
AnalogServer::AnalogServer(const std::vector<AnalogPortEntry>& _analogPorts, int rate): RateThread(rate)
{
  yTrace();
  createPorts(_analogPorts, rate);
}
bool AnalogServer::createPorts(const std::vector<AnalogPortEntry>& _analogPorts, int rate)
{
    yTrace();
    analogSensor_p=0;
    this->analogPorts=_analogPorts;
    setHandlers();
    setRate(rate);
    return true;
}

AnalogServer::AnalogServer(): RateThread(0)
{
    yTrace();
    _rate = 0;
    analogSensor_p = NULL;
}

AnalogServer::~AnalogServer()
{
    yTrace();
    _rate = 0;
    analogSensor_p = NULL;
}

void AnalogServer::setHandlers()
{
  yTrace();
  for(unsigned int i=0;i<analogPorts.size(); i++)
  {
	std::string rpcPortName = analogPorts[i].port_name;
	rpcPortName += "/rpc:i";
    AnalogServerHandler* ash = new AnalogServerHandler(rpcPortName.c_str());
    handlers.push_back(ash);
  }
}

void AnalogServer::removeHandlers()
{
    for(unsigned int i=0; i<handlers.size(); i++)
    {
        if (handlers[i]!=NULL)
            delete handlers[i];
    }
   
    handlers.clear();
}


/**
  * Specify which analog sensor this thread has to read from.
  */

bool AnalogServer::attachAll(const PolyDriverList &analog2attach)
{
    yTrace();
    if (analog2attach.size() != 1)
    {
        yError()<<"AnalogServer: cannot attach more than one device\n";
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach=analog2attach[0]->poly;

    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(analogSensor_p);
    }

    if(NULL == analogSensor_p)
    {
        yError() << "AnalogServer: subdevice passed to attach method is invalid!!!";
        return false;
    }
    attach(analogSensor_p);
    start();

    return true;
}

bool AnalogServer::detachAll()
{
    yTrace();
    analogSensor_p = NULL;
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        handlers[i]->setInterface(analogSensor_p);
    }
    return true;
}

void AnalogServer::attach(yarp::dev::IAnalogSensor *s)
{
    yTrace();
    analogSensor_p=s;
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        handlers[i]->setInterface(analogSensor_p);
    }
}

void AnalogServer::detach()
{
    yTrace();
    analogSensor_p = NULL;;
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        handlers[i]->setInterface(analogSensor_p);
    }
}

bool AnalogServer::threadInit()
{
    yTrace();
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        // open data port
        if (!analogPorts[i].port.open(analogPorts[i].port_name.c_str()))
            return false;
    }
    return true;
}

void AnalogServer::setId(const std::string &i)
{
    id=i;
}

std::string AnalogServer::getId()
{
    return id;
}

bool AnalogServer::open(yarp::os::Searchable &config)
{
     yTrace() << "AnalogServer param = " << config.toString().c_str();

    Property params;
    params.fromString(config.toString().c_str());
    bool correct=true;

//     if(params.check("ports"))
//     {
//       Bottle *ports=params.find("ports").asList();
//       setId(ports->get(0).asString().c_str());
//     }

    // Verify minimum set of parameters required
    if(!params.check("robotName") )   // ?? qui dentro, da dove lo pesco ??
    {
        correct=false;
        yError() << "AnalogServer missing robot Name, check your configuration file!! Quitting\n";
        return false;
    }

    if(params.check("deviceId"))
    {
      string tmp(params.find("deviceId").asString());// .asList();
//       string tmp(deviceId->get(0).asString());
      cout << tmp;
      setId(tmp);
    }

    if (params.check("period"))
    {
        _rate=params.find("period").asInt();
    }
    else
    {
        _rate=20;
        yWarning() <<"Warning: part "<< id <<" using default period ("<<_rate<<")\n";
    }

    // Read the list of ports
    std::string robotName=params.find("robotName").asString().c_str();
    std::string root_name;
    root_name+="/";
    root_name+=robotName;
    root_name+= "/" + this->id + "/analog";
    rpcPortName = root_name + "/rpc:i";

    // port names are optional, do not check for correctness.
    if(!params.check("ports"))
    {
     // if there is no "ports" section take the name of the "skin" group as the only port name
        createPort((root_name+":o" ).c_str(), _rate );

//         tmpPorts.resize( (size_t) 1);
//         tmpPorts[0].offset = 0;
//         tmpPorts[0].length = -1;
//         tmpPorts[0].port_name = root_name + this->id;
    }
    else
    {
        Bottle *ports=params.find("ports").asList();

        if (!params.check("total_taxels", "number of taxels of the part"))
            return false;
        int total_taxels=params.find("total_taxels").asInt();
        int nports=ports->size();
        int totalT = 0;
        std::vector<AnalogPortEntry> tmpPorts;
        tmpPorts.resize(nports);

        for(int k=0;k<ports->size();k++)
        {
            Bottle parameters=params.findGroup(ports->get(k).asString().c_str());

            if (parameters.size()!=5)
            {
                yError () <<"check skin port parameters in part description";
                yError() << "--> I was expecting "<<ports->get(k).asString().c_str() << " followed by four integers";
                return false;
            }

            int wBase=parameters.get(1).asInt();
            int wTop=parameters.get(2).asInt();
            int base=parameters.get(3).asInt();
            int top=parameters.get(4).asInt();

            cout<<"--> "<<wBase<<" "<<wTop<<" "<<base<<" "<<top<<endl;

            //check consistenty
            if(wTop-wBase != top-base){
                yError() << "Error: check skin port parameters in part description\n";
                yError() << "Numbers of mapped taxels do not match.\n";
                return false;
            }
            int taxels=top-base+1;

            tmpPorts[k].length = taxels;
            tmpPorts[k].offset = wBase;
            tmpPorts[k].port_name = root_name+":o"+string(ports->get(k).asString().c_str());

            createPorts(tmpPorts, _rate);
            totalT+=taxels;
        }

        if (totalT!=total_taxels)
        {
            yError() << "Error total number of mapped taxels does not correspond to total taxels";
            return false;
        }
    }
    return true;
}

void AnalogServer::threadRelease()
{
    yTrace();
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        analogPorts[i].port.close();
    }
    close();
}

void AnalogServer::run()
{
    int first, last, ret;
    if (analogSensor_p!=0)
    {
        // read from the analog sensor
        yarp::sig::Vector v;

        ret=analogSensor_p->read(v);

        if (ret==yarp::dev::IAnalogSensor::AS_OK)
        {
            if (v.size()>0)
            {
                lastStateStamp.update();
                // send the data on the port(s), splitting them as specified in the config file
                for(unsigned int i=0; i<analogPorts.size(); i++){
                    yarp::sig::Vector &pv = analogPorts[i].port.prepare();
                    first = analogPorts[i].offset;
                    if(analogPorts[i].length==-1)   // read the max length available
                        last = v.size()-1;
                    else
                        last = analogPorts[i].offset + analogPorts[i].length - 1;
                    // check vector limit
                    if(last>=(int)v.size()){
                        yError() << "Error while sending analog sensor output on port "<< analogPorts[i].port_name;
                        yError() << "Vector size expected to be at least "<<last<<" whereas it is "<< v.size();
                        continue;
                    }
                    pv = v.subVector(first, last);
                    analogPorts[i].port.setEnvelope(lastStateStamp);
                    analogPorts[i].port.write();
                }
            }
        }
        else
        {
            //todo release
        }
    }
}

bool AnalogServer::close()
{
    yTrace() << "AnalogServer::Close";
    RateThread::stop();
    detachAll();
    removeHandlers();
    return true;
}

// eof



