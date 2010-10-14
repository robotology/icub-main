class iCubNetwork
{
public:
    iCubNetwork(){}
    iCubNetwork(std::string &name,
                std::string &file,
                std::string &device,
                std::string &canbusdevice)
        : name(name),file(file),device(device),canbusdevice(canbusdevice) 
    {
    }

    inline bool operator==(iCubNetwork& n)
    {
        return name==n.name;
    }

    ~iCubNetwork(){}

    std::string name;
    std::string file;
    std::string device;
    std::string canbusdevice;
};

/*
class iCubNetworkEntry
{
public:
    enum { NOT_USED=-1 };
    
    iCubNetworkEntry(iCubNetwork *net=NULL,int j0=NOT_USED,int j1=NOT_USED,int j2=NOT_USED,int j3=NOT_USED)
    {
        pNet=net;

        joints[0]=j0;
        joints[1]=j1;
        joints[2]=j2;
        joints[3]=j3;
    }
    ~iCubNetworkEntry(){}

protected:
    int joints[4];
    iCubNetwork *pNet;
};

class iCubPart
{
public:
    iCubPart(){}
    iCubPart(std::string name,int joints,int threadrate)
        : name(name),joints(joints),threadrate(threadrate)
    {
    }
    ~iCubPart(){}
    
    void addNetwork(iCubNetwork *pNet,int j0,int j1,int j2,int j3)
    {
        networks.push_back(iCubNetworkEntry(pNet,j0,j1,j2,j3));
    }

protected:
    std::string name;
    int joints;
    int threadrate;
    yarp::sig::VectorOf<iCubNetworkEntry> networks;
};

class iCubCanNetworkMap
{
public:
    iCubCanNetworkMap(){}
    ~iCubCanNetworkMap(){}

    void addPart(iCubPart &part)
    {
        parts.push_back(part);
    }
    void addNetwork(iCubNetwork& net)
    {
        for (int i=0; i<networks.size(); ++i)
        {
            if (networks[i]==net) return;
        }

        networks.push_back(net);
    }

    bool config(yarp::os::Property &robot)
    {
        yarp::os::Bottle general=robot.findGroup("GENERAL");
        yarp::os::Bottle *parts=general.find("parts").asList();

        for (int t=0; t<parts->size(); ++t)
        {
            yarp::os::ConstString partName=parts->get(t).asString();
            yarp::os::Bottle part=robot.findGroup(partName.c_str());
            
            yarp::os::Bottle *networks=general.find("networks").asList();

            for (int n=0; n<networks->size(); ++n)
            {
                yarp::os::ConstString netName=networks->get(t).asString();
                yarp::os::Bottle net=robot.findGroup(netName.c_str());

                addNetwork(iCubNetwork(std::string(netName.c_str()),
                                       std::string(net.find("file").asString().c_str()),
                                       std::string(net.find("device").asString().c_str()),
                                       std::string(net.find("canbusdevice").asString().c_str())));
            }
        }


    }

protected:
    std::string name;
    yarp::sig::VectorOf<iCubPart>    parts;
    yarp::sig::VectorOf<iCubNetwork> networks;
    //yarp::sig::VectorOf<iCubSkin*>    skins;
    //yarp::sig::VectorOf<iCubAnalog*>  analogs;
};

class iCubSkin
{
public:
    iCubSkin(){}
    ~iCubSkin(){}

protected:
    std::string name;
};

    void addSkin(iCubSkin *skin)
    {
        skins.push_back(skin);
    }
    void addAnalog(iCubAnalog *anal)
    {
        analogs.push_back(anal);
    }

class iCubAnalog
{
public:
    iCubAnalog(std::string name,int threadrate,iCubNetwork *network)
        : name(name),period(period),network(network)
    {
    }
    ~iCubAnalog(){}

    void addPart(iCubPart *deviceId)
    {
        deviceIds.push_back(deviceId);
    }

protected:
    std::string name;
    int period;
    iCubNetwork *network;
    yarp::sig::VectorOf<iCubPart*> deviceIds; 
};
*/