/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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

#include <cstdio>
#include <cstdarg>
#include <algorithm>

#include <gsl/gsl_math.h>

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/d4c/d4c_server.h>
#include <iCub/d4c/private/d4c_helpers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::d4c;


/************************************************************************/
Item::Item()
{
    name="";
    type="undefined";
    active=false;
    center.resize(3,0.0);
    orientation.resize(4,0.0);
    radius.resize(3,0.0);
    color.resize(3,0.0);
}


/************************************************************************/
bool Item::fromProperty(const Property &options)
{
    if (options.check("name"))
        name=options.find("name").asString().c_str();    

    if (options.check("active"))
        active=options.find("active").asString()=="on";    

    extractVector(options,"center",center);
    extractVector(options,"orientation",orientation);
    extractVector(options,"radius",radius);
    extractVector(options,"color",color);

    return true;
}


/************************************************************************/
Property Item::toProperty() const
{
    Value vcenter;      vcenter.fromString(("("+string(center.toString().c_str())+")").c_str());
    Value vorientation; vorientation.fromString(("("+string(orientation.toString().c_str())+")").c_str());
    Value vradius;      vradius.fromString(("("+string(radius.toString().c_str())+")").c_str());
    Value vcolor;       vcolor.fromString(("("+string(color.toString().c_str())+")").c_str());

    Property prop;
    prop.put("name",name.c_str());
    prop.put("type",type.c_str());
    prop.put("active",active?"on":"off");
    prop.put("center",vcenter);
    prop.put("orientation",vorientation);
    prop.put("radius",vradius);
    prop.put("color",vcolor);

    return prop;
}


/************************************************************************/
Target_MSD::Target_MSD() : K(0.0), D(0.0)
{
    type="target_msd";
    color[0]=0.0;
    color[1]=255.0;
    color[2]=0.0;
}


/************************************************************************/
bool Target_MSD::fromProperty(const Property &options)
{
    // call the father's method
    Item::fromProperty(options);

    if (options.check("K"))
        K=options.find("K").asDouble();

    if (options.check("D"))
        D=options.find("D").asDouble();

    return true;
}


/************************************************************************/
Property Target_MSD::toProperty() const
{
    // call the father's method
    Property prop=Item::toProperty();

    prop.put("K",K);
    prop.put("D",D);

    return prop;
}


/************************************************************************/
Vector Target_MSD::getField(const Vector &x, const Vector &xdot)
{
    Vector F;

    if (active)
    {
        F=K*(center-getVectorPos(x))-D*getVectorPos(xdot);
        F.push_back(0.0);
        F.push_back(0.0);
        F.push_back(0.0);
        F.push_back(0.0);
    }
    else
        F.resize(x.length(),0.0);

    return F;
}


/************************************************************************/
Obstacle_Gaussian::Obstacle_Gaussian() : G(0.0), cut_tails(false)
{
    type="obstacle_gaussian";
    color[0]=255.0;
    color[1]=0.0;
    color[2]=0.0;
}


/************************************************************************/
bool Obstacle_Gaussian::fromProperty(const Property &options)
{
    // call the father's method
    Item::fromProperty(options);

    if (options.check("G"))
        G=options.find("G").asDouble();

    if (options.check("cut_tails"))
        cut_tails=options.find("cut_tails").asString()=="on";

    return true;
}


/************************************************************************/
Property Obstacle_Gaussian::toProperty() const
{
    // call the father's method
    Property prop=Item::toProperty();

    prop.put("G",G);
    prop.put("cut_tails",cut_tails?"on":"off");

    return prop;
}


/************************************************************************/
Vector Obstacle_Gaussian::getField(const Vector &x, const Vector &xdot)
{
    Vector zeros(x.length(),0.0);
    if (!active)
        return zeros;

    Matrix Hro=axis2dcm(orientation);
    Hro(0,3)=center[0];
    Hro(1,3)=center[1];
    Hro(2,3)=center[2];

    Vector xr=getVectorPos(x);
    xr.push_back(1.0);
    
    Vector xo=SE3inv(Hro)*xr;

    xr.pop_back();
    xo.pop_back();

    Vector ell=(xo*xo)/(radius*radius);
    double dist=ell[0]+ell[1]+ell[2];

    if (cut_tails && (dist>1.0))
        return zeros;

    Vector dx=xr-center;
    Vector F=(G*exp(-dist)/norm(dx))*dx;
    F.push_back(0.0);
    F.push_back(0.0);
    F.push_back(0.0);
    F.push_back(0.0);

    return F;
}


/************************************************************************/
GuiReporter::GuiReporter()
{
    server=NULL;
}


/************************************************************************/
void GuiReporter::setServer(D4CServer *server)
{
    this->server=server;
}


/************************************************************************/
void GuiReporter::report(const PortInfo &info)
{
    if ((server!=NULL) && info.created && !info.incoming)
        server->scheduleInitGuiTrajectory();
}


/************************************************************************/
D4CServer::D4CServer() : RateThread(20), If(0.0,Vector(7)), Iv(0.0,Vector(7))
{
    isOpen=false;
    fieldEnabled=false;
    controlEnabled=false;
    simulationEnabled=false;
    simulationFirstStep=false;
    doInitGuiTrajectory=false;
    offlineMode=false;
    initIntegration=true;
    verbosity=0;
    name="";
    activeIF="";
    iCtrlRight=NULL;
    iCtrlLeft=NULL;
    itCnt=0;
    doTrajectoryCnt=0;

    xdot.resize(7,0.0);
    x.resize(7,0.0);
    xhat.resize(7,0.0);

    toolFrame=invToolFrame=eye(4,4);
}


/************************************************************************/
D4CServer::~D4CServer()
{
    close();
}


/************************************************************************/
void D4CServer::printMessage(const int logtype, const int level,
                             const char *format, ...) const
{
    if (verbosity>=level)
    {
        string str;
        str="*** "+name+": ";

        va_list arg;
        char buf[512];
        va_start(arg,format);        
        vsnprintf(buf,sizeof(buf),format,arg);
        va_end(arg);

        str+=buf;
        switch (logtype)
        {
        case log::error:
            yError(str.c_str());
            break;
        case log::warning:
            yWarning(str.c_str());
            break;
        case log::info:
            yInfo(str.c_str());
            break;
        default:
            printf("%s\n",str.c_str());
        }
    }
}


/************************************************************************/
bool D4CServer::open(const Property &options)
{
    verbosity=options.check("verbosity",Value(0)).asInt();
    device=options.check("device",Value("cartesiancontrollerclient")).asString().c_str();
    name=options.check("name",Value("d4c_server")).asString().c_str();
    robot=options.check("robot",Value("icub")).asString().c_str();
    part=options.check("part",Value("both_arms")).asString().c_str();
    offlineMode=options.check("offline");

    if ((part!="right_arm") && (part!="left_arm") && (part!="both_arms"))
    {
        printMessage(log::error,1,"server failed to open, unknown part specified");
        return false;
    }

    period=options.check("period",Value(20)).asInt();

    double Ts=(double)period/1000.0;

    If.setTs(Ts);
    Iv.setTs(Ts);

    If.reset(xdot);
    Iv.reset(x);

    reporter.setServer(this);
    gui.setReporter(reporter);

    data.open(("/"+name+"/data:o").c_str());    
    gui.open(("/"+name+"/gui:o").c_str());
    rpc.open(("/"+name+"/rpc").c_str());
    rpc.setReader(*this);

    if (!offlineMode)
    {
        if ((part=="right_arm") || (part=="both_arms"))
        {
            Property optCtrlRight;
            optCtrlRight.put("device",device.c_str());
            optCtrlRight.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
            optCtrlRight.put("local",("/"+name+"_right/cartesian").c_str());

            if (dCtrlRight.open(optCtrlRight))
                dCtrlRight.view(iCtrlRight);
            else
                return false;
        }

        if ((part=="left_arm") || (part=="both_arms"))
        {
            Property optCtrlLeft;
            optCtrlLeft.put("device",device.c_str());
            optCtrlLeft.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
            optCtrlLeft.put("local",("/"+name+"_left/cartesian").c_str());

            if (dCtrlLeft.open(optCtrlLeft))
                dCtrlLeft.view(iCtrlLeft);
            else
                return false;

        }

        if ((part=="both_arms") || (part=="right_arm"))
        {
            iCtrlActive=iCtrlRight;
            activeIF="right";
        }
        else
        {
            iCtrlActive=iCtrlLeft;
            activeIF="left";
        }

        Vector dof;
        iCtrlActive->getDOF(dof);
        qhat.resize(dof.length(),0.0);

        t0=Time::now();
    }

    // request high resolution scheduling straightaway
    Time::turboBoost();
    
    setRate(period);
    start();

    printMessage(log::info,1,"server successfully open%s", offlineMode?" (offline mode)":"");

    return isOpen=true;
}


/************************************************************************/
void D4CServer::close()
{
    if (isOpen)
    {
        // close prior before any other stuff
        rpc.interrupt();
        rpc.close();

        if (isRunning())
            stop();

        if (!offlineMode)
        {
            if ((part=="right_arm") || (part=="both_arms"))
                dCtrlRight.close();
            if ((part=="left_arm") || (part=="both_arms"))
                dCtrlLeft.close();
        }

        mutex.lock();

        guiQueue.clear();

        for (map<int,Item*>::iterator it=table.begin(); it!=table.end(); it++)
        {            
            eraseGuiItem(GuiRequest("erase",it));

            delete it->second;

            printMessage(log::no_info,1,"erasing item %d",it->first);

            Time::delay(0.02);
        }

        table.clear();

        eraseGuiTrajectory();

        data.interrupt();
        data.close();

        gui.interrupt();
        gui.close();

        isOpen=false;

        mutex.unlock();

        printMessage(log::info,1,"server closed");
    }
    else
        printMessage(log::warning,3,"server is already closed");
}


/************************************************************************/
Item* D4CServer::itemFactory(const Property &options)
{
    Item *item=NULL;    

    if (options.check("type"))
    {
        string name=options.check("name",Value("")).asString().c_str();
        string type=options.find("type").asString().c_str();

        printMessage(log::no_info,2,"creating \"%s\" of type %s ...",name.c_str(),type.c_str());

        if (type=="target_msd")
            item=new Target_MSD;
        else if (type=="obstacle_gaussian")
            item=new Obstacle_Gaussian;
    }
    else
        printMessage(log::warning,2,"option \"type\" not found");

    if (item!=NULL)
        printMessage(log::no_info,2,"successfully created");
    else
        printMessage(log::warning,2,"unknown type");

    return item;
}


/************************************************************************/
bool D4CServer::addItem(const Property &options, int &item)
{
    if (isOpen)
    {
        mutex.lock();
        printMessage(log::no_info,2,"received request for adding item: %s",
                     options.toString().c_str());

        bool ret=false;
        if (Item *pItem=itemFactory(options))
        {
            // configure item
            pItem->fromProperty(options);
            table[item=itCnt++]=pItem;
            map<int,Item*>::iterator it=table.find(item);
            pushUpdateGuiItem(it);

            printMessage(log::no_info,1,"added item %s",
                         pItem->toProperty().toString().c_str());
            ret=true;
        }
        else
            printMessage(log::error,1,"wrong request detected!");

        mutex.unlock();
        return ret;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::eraseItem(const int item)
{
    if (isOpen)
    {
        mutex.lock();
        printMessage(log::no_info,2,"received request for erasing item %d",item);

        bool ret=false;
        map<int,Item*>::iterator it=table.find(item);
        if (it!=table.end())
        {
            pushEraseGuiItem(it);

            delete it->second;
            table.erase(it);

            printMessage(log::no_info,1,"item %d scheduled for erasing",item);
            ret=true;
        }
        else
            printMessage(log::warning,1,"item %d not found!",item);

        mutex.unlock();
        return ret;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::clearItems()
{
    if (isOpen)
    {
        mutex.lock();
        for (map<int,Item*>::iterator it=table.begin(); it!=table.end(); it++)
        {
            pushEraseGuiItem(it);
            delete it->second;
        }

        table.clear();
        printMessage(log::no_info,1,"all items have been scheduled for erasing");

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getItems(Bottle &items)
{
    if (isOpen)
    {
        mutex.lock();
        items.clear();
        for (map<int,Item*>::const_iterator it=table.begin(); it!=table.end(); it++)
            items.addInt(it->first);

        printMessage(log::no_info,1,"list of items ids prepared for sending: (%s)",
                     items.toString().c_str());

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::setProperty(const int item, const Property &options)
{
    if (isOpen)
    {
        mutex.lock();
        printMessage(log::no_info,2,"received request for setting item %d property: %s",
                     item,options.toString().c_str());

        bool ret=false;
        map<int,Item*>::iterator it=table.find(item);
        if (it!=table.end())
        {
            it->second->fromProperty(options);
            pushUpdateGuiItem(it);

            printMessage(log::no_info,1,"item %d property successfully updated: %s",
                         item,it->second->toProperty().toString().c_str());
            ret=true;
        }
        else
            printMessage(log::warning,1,"item %d not found!",item);

        mutex.unlock();
        return ret;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getProperty(const int item, Property &options)
{
    if (isOpen)
    {
        mutex.lock();
        printMessage(log::no_info,2,"received request for getting item %d property",item);

        bool ret=false;
        map<int,Item*>::const_iterator it=table.find(item);
        if (it!=table.end())
        {
            options=it->second->toProperty();

            printMessage(log::no_info,1,"item %d property successfully retrieved: %s",
                         item,options.toString().c_str());
            ret=true;
        }
        else
            printMessage(log::warning,1,"item %d not found!",item);

        mutex.unlock();
        return ret;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::enableField()
{
    if (isOpen)
    {
        if (initIntegration)
        {
            Vector pos,orien;
            iCtrlActive->getPose(pos,orien);
            mutex.lock();
            copyVectorData(pos,this->x);
            copyVectorData(orien,this->x);
            Iv.reset(x);
            initIntegration=false;
            mutex.unlock();
        }
        fieldEnabled=true;
        printMessage(log::no_info,1,"field enabled");
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::disableField()
{
    if (isOpen)
    {
        mutex.lock();
        if (!offlineMode)
            iCtrlActive->stopControl();

        fieldEnabled=false;
        printMessage(log::no_info,1,"field disabled");

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getFieldStatus(bool &status)
{
    if (isOpen)
    {
        status=fieldEnabled;
        printMessage(log::no_info,1,"field status = %s",status?"on":"off");
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::enableControl()
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            mutex.lock();
            controlEnabled=true;
            printMessage(log::no_info,1,"control enabled");

            if (simulationEnabled)
            {
                simulationEnabled=false;
                printMessage(log::warning,2,"simulation gets automatically disabled");
            }

            mutex.unlock();
            return true;
        }
        else
        {
            printMessage(log::warning,1,"it is not possible to enable control in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::disableControl()
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            mutex.lock();
            iCtrlActive->stopControl();
            controlEnabled=false;
            printMessage(log::no_info,1,"control disabled");
            mutex.unlock();
            return true;
        }
        else
        {
            printMessage(log::warning,1,"it is not possible to disable control in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getControlStatus(bool &status)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            status=controlEnabled;
            printMessage(log::no_info,1,"control status = %s",status?"on":"off");
            return true;
        }
        else
        {
            printMessage(log::warning,1,"there is no possibility to enable or disable control in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::enableSimulation()
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            mutex.lock();
            simulationEnabled=true;
            simulationFirstStep=true;
            printMessage(log::no_info,1,"simulation enabled");

            if (controlEnabled)
            {
                controlEnabled=false;
                printMessage(log::warning,2,"control gets automatically disabled");
            }

            mutex.unlock();
            return true;
        }
        else
        {
            printMessage(log::warning,1,"it is not possible to enable simulation in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::disableSimulation()
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            simulationEnabled=false;
            printMessage(log::no_info,1,"simulation disabled");
            return true;
        }
        else
        {
            printMessage(log::warning,1,"it is not possible to disable simulation in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getSimulationStatus(bool &status)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            status=simulationEnabled;
            printMessage(log::no_info,1,"simulation status = %s",status?"on":"off");
            return true;
        }
        else
        {
            printMessage(log::warning,1,"there is no possibility to enable or disable simulation in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::setPeriod(const int period)
{
    if (isOpen)
    {
        mutex.lock();
        this->period=period;
        double Ts=(double)this->period/1000.0;

        If.setTs(Ts);
        Iv.setTs(Ts);

        if (!offlineMode)
        {        
            setRate(this->period);
            printMessage(log::no_info,1,"thread period changed to %d [ms]",period);
        }

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getPeriod(int &period)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            period=(int)const_cast<D4CServer*>(this)->getRate();
            printMessage(log::no_info,1,"thread period is %d [ms]",period);
        }
        else
        {
            period=this->period;
            printMessage(log::no_info,1,"integration time is %d [ms]",this->period);
        }

        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::setPointState(const Vector &x, const Vector &o,
                              const Vector &xdot, const Vector &odot)
{
    if (isOpen)
    {
        mutex.lock();
        copyVectorData(x,this->x);
        copyVectorData(o,this->x);
        copyVectorData(xdot,this->xdot);
        copyVectorData(odot,this->xdot);

        If.reset(this->xdot);
        Iv.reset(this->x);

        initIntegration=false;

        printMessage(log::no_info,1,"point state changed to x = %s; xdot = %s",
                     this->x.toString().c_str(),this->xdot.toString().c_str());

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::setPointOrientation(const Vector &o, const Vector &odot)
{
    if (isOpen)
    {
        mutex.lock();
        copyVectorData(o,this->x);
        copyVectorData(odot,this->xdot);

        If.reset(this->xdot);
        Iv.reset(this->x);

        printMessage(log::no_info,1,"point state changed to x = %s; xdot = %s",
                     this->x.toString().c_str(),this->xdot.toString().c_str());

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::setPointStateToTool()
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            Vector x,o,xdot,odot;
            getTool(x,o);
            iCtrlActive->getTaskVelocities(xdot,odot);
            odot=0.0;

            mutex.lock();
            copyVectorData(x,this->x);
            copyVectorData(o,this->x);
            copyVectorData(xdot,this->xdot);
            copyVectorData(odot,this->xdot);

            If.reset(this->xdot);
            Iv.reset(this->x);

            initIntegration=false;

            printMessage(log::no_info,1,"point state changed to x = %s; xdot = %s",
                         this->x.toString().c_str(),this->xdot.toString().c_str());

            mutex.unlock();
            return true;
        }
        else
        {
            printMessage(log::warning,1,"no connection with the robot in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::attachToolFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            if ((x.length()<3) || (o.length()<4))
            {
                printMessage(log::error,1,"problem with vector lengths");
                return false;
            }
            else
            {
                mutex.lock();
                toolFrame=axis2dcm(o);
                toolFrame(0,3)=x[0];
                toolFrame(1,3)=x[1];
                toolFrame(2,3)=x[2];

                invToolFrame=SE3inv(toolFrame);
                printMessage(log::no_info,1,"attach tool frame = %s",toolFrame.toString().c_str());

                mutex.unlock();
                return true;
            }
        }
        else
        {
            printMessage(log::warning,1,"it is not possible to attach frame in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getToolFrame(yarp::sig::Vector &x, yarp::sig::Vector &o)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            mutex.lock();
            x.resize(3);            
            x[0]=toolFrame(0,3);
            x[1]=toolFrame(1,3);
            x[2]=toolFrame(2,3);
            o=dcm2axis(toolFrame);

            printMessage(log::no_info,1,"tool frame currently attached is = %s",
                         toolFrame.toString().c_str());

            mutex.unlock();
            return true;
        }
        else
        {
            printMessage(log::warning,1,"there is no tool in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::removeToolFrame()
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            mutex.lock();
            toolFrame=invToolFrame=eye(4,4);
            printMessage(log::no_info,1,"tool frame removed");
            mutex.unlock();
            return true;
        }
        else
        {
            printMessage(log::warning,1,"there is no tool in offline mode");
            return false;
        }   
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getTool(yarp::sig::Vector &x, yarp::sig::Vector &o)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            Vector pos,orien;
            iCtrlActive->getPose(pos,orien);

            Matrix frame1=axis2dcm(orien);
            frame1(0,3)=pos[0];
            frame1(1,3)=pos[1];
            frame1(2,3)=pos[2];

            Matrix frame2=frame1*toolFrame;
            x.resize(3);
            x[0]=frame2(0,3);
            x[1]=frame2(1,3);
            x[2]=frame2(2,3);
            o=dcm2axis(frame2);

            printMessage(log::no_info,1,"tool state is = %s",
                         frame2.toString().c_str());
            return true;
        }
        else
        {
            printMessage(log::warning,1,"there is no tool in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getPointState(Vector &x, Vector &o, Vector &xdot, Vector &odot)
{
    if (isOpen)
    {
        x=getVectorPos(this->x);
        o=getVectorOrien(this->x);
        xdot=getVectorPos(this->xdot);
        odot=getVectorOrien(this->xdot);

        printMessage(log::no_info,1,"point state is x = %s; xdot = %s",
                     x.toString().c_str(),xdot.toString().c_str());
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getField(Vector &field)
{
    if (isOpen)
    {
        field.resize(x.length(),0.0);
        for (map<int,Item*>::const_iterator it=table.begin(); it!=table.end(); it++) 
            field=field+it->second->getField(x,xdot);

        printMessage(log::no_info,1,"field = %s",field.toString().c_str());
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::getSimulation(Vector &xhat, Vector &ohat, Vector &qhat)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            xhat=getVectorPos(this->xhat);
            ohat=getVectorOrien(this->xhat);
            qhat=this->qhat;

            printMessage(log::no_info,1,"simulated end-effector pose is xhat = %s; part configuration is qhat = %s",
                         xhat.toString().c_str(),qhat.toString().c_str());
            return true;
        }
        else
        {
            printMessage(log::warning,1,"there is no possibility to enable simulation in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::read(ConnectionReader &connection)
{
    Bottle cmd, reply;
    cmd.read(connection);

    if (cmd.size()==0)
        reply.addVocab(D4C_VOCAB_CMD_NACK);
    else switch(cmd.get(0).asVocab())
    {
        //-----------------
         case D4C_VOCAB_CMD_PING:
         {
             reply.addVocab(D4C_VOCAB_CMD_ACK);
             break;
         }         

         //-----------------
         case D4C_VOCAB_CMD_ADD:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 int item;
                 if (addItem(extractProperty(cmd.get(1)),item))
                 {
                     reply.addVocab(D4C_VOCAB_CMD_ACK);
                     reply.addInt(item);
                 }
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
             }

             break;
         }         

         //-----------------
         case D4C_VOCAB_CMD_DEL:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 if (eraseItem(cmd.get(1).asInt()))
                     reply.addVocab(D4C_VOCAB_CMD_ACK);
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_SET:
         {
             if (cmd.size()<3)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else if (setProperty(cmd.get(1).asInt(),extractProperty(cmd.get(2))))
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_GET:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 Property options;
                 if (getProperty(cmd.get(1).asInt(),options))
                 {
                     Value val;
                     val.fromString(("("+string(options.toString().c_str())+")").c_str());

                     reply.addVocab(D4C_VOCAB_CMD_ACK);
                     reply.add(val);
                 }
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_LIST:
         {
             Bottle items;
             if (getItems(items))
             {
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.addList().append(items);
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_CLEAR:
         {
             if (clearItems())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_ENFIELD:
         {
             if (enableField())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_DISFIELD:
         {
             if (disableField())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_STATFIELD:
         {
             bool status;
             if (getFieldStatus(status))
             {
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.addString(status?"on":"off");
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_ENCTRL:
         {
             if (enableControl())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_DISCTRL:
         {
             if (disableControl())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_STATCTRL:
         {
             bool status;
             if (getControlStatus(status))
             {
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.addString(status?"on":"off");
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_ENSIM:
         {
             if (enableSimulation())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_DISSIM:
         {
             if (disableSimulation())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_STATSIM:
         {
             bool status;
             if (getSimulationStatus(status))
             {
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.addString(status?"on":"off");
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_SETPER:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else if (setPeriod(cmd.get(1).asInt()))
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }         

         //-----------------
         case D4C_VOCAB_CMD_GETPER:
         {
             int period;
             if (getPeriod(period))
             {
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.addInt(period);
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_SETACTIF:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else if (setActiveIF(cmd.get(1).asString().c_str()))
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_GETACTIF:
         {
             string _activeIF;
             if (getActiveIF(_activeIF))
             {
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.addString(_activeIF.c_str());
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_GETTRAJ:
         {
             Property options=extractProperty(cmd.get(1));
             if (options.isNull())
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 int maxIterations=options.find("maxIterations").asInt();
                 double Ts=options.find("Ts").asDouble();
                 deque<Vector> trajPos;
                 deque<Vector> trajOrien;
                 if (getTrajectory(trajPos,trajOrien,maxIterations,Ts))
                 {
                     reply.addVocab(D4C_VOCAB_CMD_ACK);

                     Bottle &bPos=reply.addList();
                     bPos.addString("trajPos");
                     for (unsigned int i=0; i<trajPos.size(); i++)
                     {
                         Bottle &point=bPos.addList();
                         for (size_t j=0; j<trajPos[i].length(); j++)
                            point.addDouble(trajPos[i][j]);
                     }

                     Bottle &bOrien=reply.addList();
                     bOrien.addString("trajOrien");
                     for (unsigned int i=0; i<trajOrien.size(); i++)
                     {
                         Bottle &point=bOrien.addList();
                         for (size_t j=0; j<trajOrien[i].length(); j++)
                            point.addDouble(trajOrien[i][j]);
                     }
                 }
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_EXECTRAJ:
         {
             Bottle *options=cmd.get(1).asList();
             if (options->isNull())
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 double trajTime=-1.0;
                 deque<Vector> trajPos;
                 deque<Vector> trajOrien;

                 if (Bottle *BtrajTime=options->get(0).asList())
                     if (BtrajTime->get(0).asString()=="trajTime")
                         trajTime=BtrajTime->get(1).asDouble();

                 bool okPos=false;
                 if (Bottle *BtrajPos=options->get(1).asList())
                 {
                     if (BtrajPos->get(0).asString()=="trajPos")
                     {
                         for (int i=1; i<BtrajPos->size(); i++)
                         {                            
                             if (Bottle *point=BtrajPos->get(i).asList())
                             {
                                 Vector pos(point->size());
                                 for (int j=0; j<point->size(); j++)
                                     pos[j]=point->get(j).asDouble();

                                 trajPos.push_back(pos);
                             }
                         }

                         okPos=true;
                     }
                 }

                 bool okOrien=false;
                 if (Bottle *BtrajOrien=reply.get(2).asList())
                 {
                     if (BtrajOrien->get(0).asString()=="trajOrien")
                     {
                         for (int i=1; i<BtrajOrien->size(); i++)
                         {                            
                             if (Bottle *point=BtrajOrien->get(i).asList())
                             {                                
                                 Vector orien(point->size());
                                 for (int j=0; j<point->size(); j++)
                                     orien[j]=point->get(j).asDouble();

                                 trajOrien.push_back(orien);
                             }
                         }

                         okOrien=true;
                     }
                 }

                 if (!okPos || !okOrien)
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
                 else if (executeTrajectory(trajPos,trajOrien,trajTime))
                     reply.addVocab(D4C_VOCAB_CMD_ACK);
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_SETSTATETOTOOL:
         {
             if (cmd.size()<1)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 if (setPointStateToTool())
                     reply.addVocab(D4C_VOCAB_CMD_ACK);
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_SETSTATE:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 Property options=extractProperty(cmd.get(1));
                 if (options.isNull())
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
                 else
                 {
                     Vector pos,vel;
                     if (extractVector(options,"x",pos) && extractVector(options,"xdot",vel))
                     {
                         if (setPointState(getVectorPos(pos),getVectorOrien(pos),
                                           getVectorPos(vel),getVectorOrien(vel)))
                             reply.addVocab(D4C_VOCAB_CMD_ACK);
                         else
                             reply.addVocab(D4C_VOCAB_CMD_NACK);
                     }
                     else
                         reply.addVocab(D4C_VOCAB_CMD_NACK);
                 }
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_SETORIEN:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 Property options=extractProperty(cmd.get(1));
                 if (options.isNull())
                     reply.addVocab(D4C_VOCAB_CMD_NACK);
                 else
                 {
                     Vector o,odot;
                     if (extractVector(options,"o",o) && extractVector(options,"odot",odot))
                     {
                         if (setPointOrientation(o,odot))
                             reply.addVocab(D4C_VOCAB_CMD_ACK);
                         else
                             reply.addVocab(D4C_VOCAB_CMD_NACK);
                     }
                     else
                         reply.addVocab(D4C_VOCAB_CMD_NACK);
                 }
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_GETSTATE:
         {
             if (isOpen)
             {
                 mutex.lock();
                 Property state=prepareData();
                 mutex.unlock();

                 Value val_state;
                 val_state.fromString(("("+string(state.toString().c_str())+")").c_str());

                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.add(val_state);
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_ATTACHTOOLFRAME:
         {
             if (cmd.size()<2)
                 reply.addVocab(D4C_VOCAB_CMD_NACK);
             else
             {
                 Vector x,o;
                 Property options=extractProperty(cmd.get(1));
                 extractVector(options,"x",x);
                 extractVector(options,"o",o);

                 if (attachToolFrame(x,o))
                     reply.addVocab(D4C_VOCAB_CMD_ACK);
                 else
                     reply.addVocab(D4C_VOCAB_CMD_NACK);                 
             }

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_GETTOOLFRAME:
         {
             Vector x,o;
             if (getToolFrame(x,o))
             {
                 Value val_x; val_x.fromString(("("+string(x.toString().c_str())+")").c_str());
                 Value val_o; val_o.fromString(("("+string(o.toString().c_str())+")").c_str());

                 Property options;
                 options.put("x",val_x);
                 options.put("o",val_o);

                 Value val;
                 val.fromString(("("+string(options.toString().c_str())+")").c_str());

                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.add(val);
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);                 

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_REMOVETOOLFRAME:
         {
             if (removeToolFrame())
                 reply.addVocab(D4C_VOCAB_CMD_ACK);
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);                 

             break;
         }

         //-----------------
         case D4C_VOCAB_CMD_GETTOOL:
         {
             Vector x,o;
             if (getTool(x,o))
             {
                 Value val_x; val_x.fromString(("("+string(x.toString().c_str())+")").c_str());
                 Value val_o; val_o.fromString(("("+string(o.toString().c_str())+")").c_str());

                 Property options;
                 options.put("x",val_x);
                 options.put("o",val_o);

                 Value val;
                 val.fromString(("("+string(options.toString().c_str())+")").c_str());

                 reply.addVocab(D4C_VOCAB_CMD_ACK);
                 reply.add(val);
             }
             else
                 reply.addVocab(D4C_VOCAB_CMD_NACK);                 

             break;
         }

         //-----------------
         default:
             reply.addVocab(D4C_VOCAB_CMD_NACK);
    }

    ConnectionWriter *returnToSender=connection.getWriter();
    if (returnToSender!=NULL)
        reply.write(*returnToSender);    

    return true;
}


/************************************************************************/
Property D4CServer::prepareData()
{
    Vector field;
    getField(field);

    Value val_field; val_field.fromString(("("+string(field.toString().c_str())+")").c_str());
    Value val_xdot;  val_xdot.fromString(("("+string(xdot.toString().c_str())+")").c_str());  
    Value val_x;     val_x.fromString(("("+string(x.toString().c_str())+")").c_str());        
    Value val_xhat;  val_xhat.fromString(("("+string(xhat.toString().c_str())+")").c_str());  
    Value val_qhat;  val_qhat.fromString(("("+string(qhat.toString().c_str())+")").c_str());  

    Property out;
    out.put("field",val_field);
    out.put("xdot",val_xdot);
    out.put("x",val_x);
    out.put("xhat",val_xhat);
    out.put("qhat",val_qhat);

    return out;
}


/************************************************************************/
void D4CServer::getTargetForCartesianIF(Vector &pos, Vector &orien)
{
    pos=getVectorPos(x);
    orien=getVectorOrien(x);

    Matrix frame1=axis2dcm(orien);
    frame1(0,3)=pos[0];
    frame1(1,3)=pos[1];
    frame1(2,3)=pos[2];

    Matrix frame2=frame1*invToolFrame;
    pos[0]=frame2(0,3);
    pos[1]=frame2(1,3);
    pos[2]=frame2(2,3);
    orien=dcm2axis(frame2);
}


/************************************************************************/
bool D4CServer::getActiveIF(string &activeIF)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            activeIF=this->activeIF;
            printMessage(log::no_info,1,"active interface: %s",activeIF.c_str());
            return true;
        }
        else
        {
            activeIF="";
            printMessage(log::warning,1,"no connection with the robot in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::setActiveIF(const string &activeIF)
{
    if (isOpen)
    {
        if (!offlineMode)
        {
            if (part=="both_arms")
            {
                if ((activeIF=="right") || (activeIF=="left"))
                {
                    mutex.lock();
                    iCtrlActive->stopControl();
                    if (activeIF=="right")
                        iCtrlActive=iCtrlRight;
                    else
                        iCtrlActive=iCtrlLeft;
                    
                    this->activeIF=activeIF;
                    printMessage(log::no_info,1,"active interface successfully set to %s",
                                 activeIF.c_str());

                    mutex.unlock();
                    return true;
                }
                else
                {
                    printMessage(log::error,1,"wrong value specified, it should be \"right\" or \"left\"");
                    return false;
                }
            }
            else
            {
                printMessage(log::warning,1,"cannot swap arm");
                return false;
            }
        }
        else
        {            
            printMessage(log::warning,1,"no connection with the robot in offline mode");
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
void D4CServer::scheduleInitGuiTrajectory()
{
    doInitGuiTrajectory=true;
}


/************************************************************************/
void D4CServer::initGuiTrajectory()
{
    if (gui.getOutputCount()>0)
    {
        Bottle &obj=gui.prepare();
        obj.clear();

        obj.addString("trajectory");
        obj.addString(part.c_str());    // trajectory identifier
        obj.addString(part.c_str());    // trajectory name
        obj.addInt(512);                // max samples in circular queue
        obj.addDouble(5.0);             // lifetime of samples
        obj.addInt(0);                  // col R
        obj.addInt(0);                  // col G
        obj.addInt(255);                // col B
        obj.addDouble(0.5);             // alpha [0,1]
        obj.addDouble(5.0);             // line width

        gui.writeStrict();
        printMessage(log::no_info,4,"initializing gui trajectory point");
    }
}


/************************************************************************/
void D4CServer::updateGuiTrajectory()
{
    if (gui.getOutputCount()>0)
    {
        Bottle &obj=gui.prepare();
        obj.clear();

        obj.addString("addpoint");
        obj.addString(part.c_str());    // trajectory identifier
        obj.addDouble(1000.0*x[0]);     // posX [mm]
        obj.addDouble(1000.0*x[1]);     // posY [mm]
        obj.addDouble(1000.0*x[2]);     // posZ [mm]

        gui.writeStrict();
        printMessage(log::no_info,4,"updating gui trajectory point");
    }
}


/************************************************************************/
void D4CServer::eraseGuiTrajectory()
{
    if (gui.getOutputCount()>0)
    {
        Bottle &obj=gui.prepare();
        obj.clear();

        obj.addString("delete");
        obj.addString(part.c_str());

        gui.writeStrict();
        printMessage(log::no_info,4,"erasing gui tracjectory point");
    }
}


/************************************************************************/
void D4CServer::updateGuiItem(const GuiRequest &req)
{    
    if (gui.getOutputCount()>0)
    {
        map<int,Item*>::iterator it=req.getIter();
        string name=req.getName();
        int item=it->first;
        Item *pItem=it->second;

        Vector rpy=CTRL_RAD2DEG*dcm2rpy(axis2dcm(pItem->orientation));

        Bottle &obj=gui.prepare();
        obj.clear();

        obj.addString("object");        
        obj.addString(name.c_str());
        obj.addDouble(1000.0*pItem->radius[0]);     // dimX [mm]
        obj.addDouble(1000.0*pItem->radius[1]);     // dimY [mm]
        obj.addDouble(1000.0*pItem->radius[2]);     // dimZ [mm]
        obj.addDouble(1000.0*pItem->center[0]);     // posX [mm]
        obj.addDouble(1000.0*pItem->center[1]);     // posY [mm]
        obj.addDouble(1000.0*pItem->center[2]);     // posZ [mm]
        obj.addDouble(rpy[0]);                      // rotX [deg]
        obj.addDouble(rpy[1]);                      // rotY [deg]
        obj.addDouble(rpy[2]);                      // rotZ [deg]
        obj.addInt((int)pItem->color[0]);           // col R
        obj.addInt((int)pItem->color[1]);           // col G
        obj.addInt((int)pItem->color[2]);           // col B
        obj.addDouble(pItem->active?1.0:0.25);      // alpha [0,1]

        gui.writeStrict();
        printMessage(log::no_info,4,"updating gui item %s",name.c_str());
    }
}


/************************************************************************/
void D4CServer::eraseGuiItem(const GuiRequest &req)
{
    if (gui.getOutputCount()>0)
    {
        Bottle &obj=gui.prepare();
        obj.clear();

        obj.addString("delete");
        obj.addString(req.getName().c_str());

        gui.write();
        printMessage(log::no_info,1,"erasing gui item %s",name.c_str());
    }
}


/************************************************************************/
void D4CServer::pushUpdateGuiItem(map<int,Item*>::iterator &it)
{
    GuiRequest update("update",it);

    for (unsigned int i=0; i<guiQueue.size(); i++)
    {
        if (guiQueue[i]==update)
        {
            printMessage(log::warning,5,"gui item %d update is already scheduled",it->first);
            return;
        }
    }

    guiQueue.push_back(update);
    printMessage(log::no_info,5,"scheduled request for gui item %d update",it->first);
}


/************************************************************************/
void D4CServer::pushEraseGuiItem(map<int,Item*>::iterator &it)
{
    int item=it->first;
    GuiRequest update("update",it);

    for (unsigned int i=0; i<guiQueue.size(); i++)
    {
        if (guiQueue[i]==update)
        {
            guiQueue.erase(guiQueue.begin()+i);
            printMessage(log::no_info,5,"remove schedule for gui item %d update",item);
            break;
        }
    }

    guiQueue.push_back(GuiRequest("erase",it));
    printMessage(log::no_info,5,"scheduled request for gui item %d erase",item);
}


/************************************************************************/
void D4CServer::handleGuiQueue()
{
    if (doInitGuiTrajectory)
    {
        initGuiTrajectory();
        doInitGuiTrajectory=false;
    }
    // doTrajectoryCnt serves to avoid updating continuosly
    // the items and delaying the trajectory too much
    else if (guiQueue.size() && (doTrajectoryCnt<3))
    {
        GuiRequest guiReq=guiQueue.front();
        guiQueue.pop_front();

        if (guiReq.getType()=="update")
            updateGuiItem(guiReq);
        else if (guiReq.getType()=="erase")
            eraseGuiItem(guiReq);
        else
            printMessage(log::warning,1,"unknown gui request received!");

        doTrajectoryCnt++;
    }
    else
    {
        updateGuiTrajectory();
        doTrajectoryCnt=0;
    }
}


/************************************************************************/
bool D4CServer::getTrajectory(deque<Vector> &trajPos, deque<Vector> &trajOrien,
                              const unsigned int maxIterations, const double Ts)
{
    if (isOpen)
    {
        mutex.lock();
        printMessage(log::no_info,1,"request for trajectory simulation");
        Vector xdotOffline=xdot;
        Vector xOffline=x;

        double _Ts=(Ts<=D4C_DEFAULT_TS_DISABLED)?(double)period/1000.0:Ts;
        Integrator IfOffline(_Ts,xdotOffline);
        Integrator IvOffline(_Ts,xOffline);

        unsigned int iteration=0;
        while (iteration++<maxIterations)
        {
            Vector field(x.length(),0.0);
            for (map<int,Item*>::const_iterator it=table.begin(); it!=table.end(); it++) 
                field=field+it->second->getField(xOffline,xdotOffline);

            xdotOffline=IfOffline.integrate(field);
            xOffline=IvOffline.integrate(xdotOffline);

            trajPos.push_back(getVectorPos(xOffline));
            trajOrien.push_back(getVectorOrien(xOffline));
        }       

        mutex.unlock();
        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
bool D4CServer::executeTrajectory(const deque<Vector> &trajPos, const deque<Vector> &trajOrien,
                                  const double trajTime)
{
    if (isOpen)
    {
        printMessage(log::no_info,1,"request for trajectory execution");

        if (trajPos.size()!=trajOrien.size())
        {
            printMessage(log::error,1,"position and orientation data have different size!");
            return false;
        }
        else if (trajTime<0.0)
        {
            printMessage(log::error,1,"negative trajectory duration provided!");
            return false;
        }

        // define the tracker thread
        class TrackerThread : public RateThread
        {
            ICartesianControl *ctrl;
            const deque<Vector> *trajPos;
            const deque<Vector> *trajOrien;
            double trajTime;
            double t0,t;
            int pos,N;
            bool threadInit()
            {
                t=0.0;
                t0=Time::now();
                return true;
            }
            void run()
            {
                t=Time::now()-t0;
                int i=std::min(N,(int)((t*N)/trajTime));
                if (i>pos)
                {
                    ctrl->goToPose(trajPos->at(i),trajOrien->at(i));
                    pos=i;
                }
            }
            void threadRelease()
            {
                // make sure we attempt to reach the last point
                ctrl->goToPoseSync(trajPos->back(),trajOrien->back());
                ctrl->waitMotionDone();
            }
        public:
           TrackerThread() : RateThread(20), pos(-1) { }
           void setInfo(ICartesianControl *ctrl, const deque<Vector> *trajPos,
                        const deque<Vector> *trajOrien, const double trajTime)
           {
               this->trajPos=trajPos;
               this->trajOrien=trajOrien;
               this->trajTime=trajTime;
               N=(int)(trajPos->size()-1);
           }
           void wait()
           {
               while(t<trajTime)
                   Time::delay(0.1);
           }
        } trackerThread;

        trackerThread.setRate((int)getRate());
        trackerThread.setInfo(iCtrlActive,&trajPos,&trajOrien,trajTime);

        // lock the main run
        mutex.lock();

        // run the tracker
        trackerThread.start();
        trackerThread.wait();
        trackerThread.stop();

        // unlock the main run
        mutex.unlock();

        return true;
    }
    else
    {
        printMessage(log::warning,1,"server is not open");
        return false;
    }
}


/************************************************************************/
void D4CServer::run()
{
    mutex.lock();

    if (!offlineMode)
    {
        if (fieldEnabled)
        {
            printMessage(log::no_info,4,"processing %d items",table.size());
            
            Vector field;
            getField(field);

            xdot=If.integrate(field);
            x=Iv.integrate(xdot);

            if (controlEnabled)
            {
                Vector pos,orien;
                getTargetForCartesianIF(pos,orien);
                iCtrlActive->goToPose(pos,orien);
            }

            if (simulationEnabled)
            {            
                Vector pos,orien,xdhat,odhat;
                getTargetForCartesianIF(pos,orien);

                if (simulationFirstStep)
                {                
                    iCtrlActive->askForPose(pos,orien,xdhat,odhat,qhat);
                    simulationFirstStep=false;
                }
                else
                {
                    Vector dof;
                    iCtrlActive->getDOF(dof);

                    Vector q0;
                    for (size_t i=0; i<dof.length(); i++)
                        if (dof[i]>0.0)
                            q0.push_back(qhat[i]);

                    iCtrlActive->askForPose(q0,pos,orien,xdhat,odhat,qhat);
                }

                copyVectorData(xdhat,xhat);
                copyVectorData(odhat,xhat);
            }
        }
    }

    if (data.getOutputCount()>0)
    {
        data.prepare()=prepareData();
        data.write();
    }    

    // avoid running faster than what is necessary
    if (Time::now()-t0>0.03)
    {
        handleGuiQueue();
        t0=Time::now();
    }

    mutex.unlock();
}


