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

#include <iCub/d4c/d4c_client.h>
#include <iCub/d4c/private/d4c_helpers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::d4c;


/************************************************************************/
D4CClient::D4CClient() : isOpen(false), verbosity(0)
{
    remote="";
    local="";

    field.resize(7,0.0);
    xdot.resize(7,0.0);
    x.resize(7,0.0);
    xhat.resize(7,0.0);
    qhat.resize(10,0.0);
}


/************************************************************************/
D4CClient::~D4CClient()
{
    close();
}


/************************************************************************/
void D4CClient::printMessage(const int logtype, const int level,
                             const char *format, ...) const
{
    if (verbosity>=level)
    {
        string str;
        str="*** "+local+": ";

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
bool D4CClient::open(const Property &options)
{
    if (options.check("remote"))
        remote=options.find("remote").asString().c_str();
    else
    {
        printMessage(log::error,1,"\"remote\" option is mandatory to open the client!");
        return false;
    }

    if (options.check("local"))
        local=options.find("local").asString().c_str();
    else
    {
        printMessage(log::error,1,"\"local\" option is mandatory to open the client!");
        return false;
    }

    carrier=options.check("carrier",Value("udp")).asString().c_str();
    verbosity=options.check("verbosity",Value(0)).asInt();

    data.open((local+"/data:i").c_str());
    rpc.open((local+"/rpc").c_str());

    bool ok=true;

    ok&=Network::connect((remote+"/data:o").c_str(),data.getName().c_str(),carrier.c_str());
    ok&=Network::connect(rpc.getName().c_str(),(remote+"/rpc").c_str());

    if (ok)
    {        
        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_PING);

        if (rpc.write(cmd,reply))
        {
            if (reply.size()>0)
            {
                if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
                {
                    printMessage(log::info,1,"successfully connected with the server %s!",remote.c_str());
                    return isOpen=true;
                }
            }
        }

        printMessage(log::error,1,"unable to get correct reply from the server %s!",remote.c_str());
        close();

        return false;
    }
    else
    {
        printMessage(log::error,1,"unable to connect to the server %s!",remote.c_str());
        close();

        return false;
    }
}


/************************************************************************/
void D4CClient::close()
{
    if (isOpen)
    {
        data.interrupt();
        rpc.interrupt();

        data.close();
        rpc.close();

        isOpen=false;

        printMessage(log::info,1,"client closed");
    }
    else
        printMessage(log::warning,3,"client is already closed");
}


/************************************************************************/
bool D4CClient::addItem(const Property &options, int &item)
{
    if (isOpen)
    {
        string options_str=options.toString().c_str();
        printMessage(log::no_info,2,"request for adding new item: %s",options_str.c_str());
        
        Value val;
        val.fromString(("("+options_str+")").c_str());

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_ADD);
        cmd.add(val);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                item=reply.get(1).asInt();
                printMessage(log::no_info,1,"item %d successfully added",item);
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::eraseItem(const int item)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for erasing item %d",item);

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_DEL);
        cmd.add(item);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"item %d successfully erased",item);
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::clearItems()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for clearing item table");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_CLEAR);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"all items have been successfully erased");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getItems(Bottle &items)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving items ids list");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_LIST);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                items=*reply.get(1).asList();
                printMessage(log::no_info,1,"items ids successfully retrieved: %s",
                             items.toString().c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::setProperty(const int item, const Property &options)
{
    if (isOpen)
    {     
        string options_str=options.toString().c_str();
        printMessage(log::no_info,2,"request for setting item %d property: %s",
                     item,options_str.c_str());

        Value val;
        val.fromString(("("+options_str+")").c_str());

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_SET);
        cmd.addInt(item);
        cmd.add(val);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"item %d property successfully updated: %s",
                             item,options_str.c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getProperty(const int item, Property &options)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving item %d property",item);

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_GET);
        cmd.addInt(item);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                options=extractProperty(reply.get(1));
                printMessage(log::no_info,1,"item %d property successfully retrieved: %s",
                             item,options.toString().c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::enableField()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for enabling field");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_ENFIELD);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"field enabled");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::disableField()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for disabling field");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_DISFIELD);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"field disabled");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getFieldStatus(bool &status)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving field status");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_STATFIELD);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                string sw=reply.get(1).asString().c_str();
                status=sw=="on";
                printMessage(log::no_info,1,"field status = %s",sw.c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::enableControl()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for enabling control");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_ENCTRL);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"control enabled");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::disableControl()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for disabling control");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_DISCTRL);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"control disabled");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getControlStatus(bool &status)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving control status");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_STATCTRL);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                string sw=reply.get(1).asString().c_str();
                status=sw=="on";
                printMessage(log::no_info,1,"control status = %s",sw.c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::enableSimulation()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for enabling simulation");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_ENSIM);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"simulation enabled");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::disableSimulation()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for disabling simulation");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_DISSIM);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"simulation disabled");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getSimulationStatus(bool &status)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving simulation status");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_STATSIM);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                string sw=reply.get(1).asString().c_str();
                status=sw=="on";
                printMessage(log::no_info,1,"simulation status = %s",sw.c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::setPeriod(const int period)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for setting period to %s [ms]",period);

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_SETPER);
        cmd.addInt(period);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"period successfully updated");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getPeriod(int &period)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving period");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_GETPER);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                period=reply.get(1).asInt();
                printMessage(log::no_info,1,"period successfully retrieved: %d [ms]",period);
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::setPointStateToTool()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for setting state equal to tool state");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_SETSTATETOTOOL);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"state successfully updated");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::attachToolFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o)
{
    if (isOpen)
    {
        if ((x.length()<3) || (o.length()<4))
        {
            printMessage(log::error,1,"problem with vector lengths");
            return false;
        }

        Value val_x; val_x.fromString(("("+string(x.toString().c_str())+")").c_str());
        Value val_o; val_o.fromString(("("+string(o.toString().c_str())+")").c_str());

        Property options;
        options.put("x",val_x);
        options.put("o",val_o);

        string options_str=options.toString().c_str();
        printMessage(log::no_info,2,"request for attaching tool frame: %s",options_str.c_str());

        Value val;
        val.fromString(("("+options_str+")").c_str());

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_ATTACHTOOLFRAME);
        cmd.add(val);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"tool frame successfully attached");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getToolFrame(yarp::sig::Vector &x, yarp::sig::Vector &o)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving tool frame");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_GETTOOLFRAME);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                Property options=extractProperty(reply.get(1));
                extractVector(options,"x",x);
                extractVector(options,"o",o);

                printMessage(log::no_info,1,"tool frame successfully retrieved %s",
                             options.toString().c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::removeToolFrame()
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for removing tool frame");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_REMOVETOOLFRAME);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"tool successfully removed");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getTool(yarp::sig::Vector &x, yarp::sig::Vector &o)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request for retrieving tool");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_GETTOOL);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                Property options=extractProperty(reply.get(1));
                extractVector(options,"x",x);
                extractVector(options,"o",o);

                printMessage(log::no_info,1,"tool successfully retrieved %s",
                             options.toString().c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::setPointState(const Vector &x, const Vector &o,
                              const Vector &xdot, const Vector &odot)
{
    if (isOpen)
    {
        Vector pos(7),vel(7);
        copyVectorData(x,pos);
        copyVectorData(o,pos);
        copyVectorData(xdot,vel);
        copyVectorData(odot,vel);

        Value val_x;    val_x.fromString(("("+string(pos.toString().c_str())+")").c_str());
        Value val_xdot; val_xdot.fromString(("("+string(vel.toString().c_str())+")").c_str());

        Property options;
        options.put("x",val_x);
        options.put("xdot",val_xdot);

        string options_str=options.toString().c_str();
        printMessage(log::no_info,2,"request for setting state: %s",options_str.c_str());

        Value val;
        val.fromString(("("+options_str+")").c_str());

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_SETSTATE);
        cmd.add(val);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"state successfully updated");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::setPointOrientation(const Vector &o, const Vector &odot)
{
    if (isOpen)
    {
        Value val_o;    val_o.fromString(("("+string(o.toString().c_str())+")").c_str());
        Value val_odot; val_odot.fromString(("("+string(odot.toString().c_str())+")").c_str());

        Property options;
        options.put("o",val_o);
        options.put("odot",val_odot);

        string options_str=options.toString().c_str();
        printMessage(log::no_info,2,"request for setting orientation: %s",options_str.c_str());

        Value val;
        val.fromString(("("+options_str+")").c_str());

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_SETORIEN);
        cmd.add(val);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"orientation successfully updated");
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getPointState(Vector &x, Vector &o, Vector &xdot, Vector &odot)
{
    if (isOpen)
    {
        if (Property *prop=data.read(false))
        {
            extractVector(*prop,"x",this->x);
            extractVector(*prop,"xdot",this->xdot);
        }

        x=getVectorPos(this->x);
        o=getVectorOrien(this->x);
        xdot=getVectorPos(this->xdot);
        odot=getVectorOrien(this->xdot);

        printMessage(log::no_info,1,"got state: x = %s xdot = %s",
                     this->x.toString().c_str(),this->xdot.toString().c_str());
        return true;
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getField(Vector &field)
{
    if (isOpen)
    {
        if (Property *prop=data.read(false))
            extractVector(*prop,"field",this->field);

        field=this->field;

        printMessage(log::no_info,1,"got field: field=%s",
                     this->field.toString().c_str());
        return true;
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getSimulation(Vector &xhat, Vector &ohat, Vector &qhat)
{
    if (isOpen)
    {
        if (Property *prop=data.read(false))
        {
            extractVector(*prop,"xhat",this->xhat);
            extractVector(*prop,"qhat",this->qhat);
        }

        xhat=getVectorPos(this->xhat);
        ohat=getVectorOrien(this->xhat);
        qhat=this->qhat;

        printMessage(log::no_info,1,"got simulated end-effector pose: xhat = %s; got part configuration: qhat = %s",
                     this->xhat.toString().c_str(),this->qhat.toString().c_str());
        return true;
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getActiveIF(string &activeIF)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request to know the active interface");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_GETACTIF);

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                activeIF=reply.get(1).asString().c_str();
                printMessage(log::no_info,1,"active interface: %s",activeIF.c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::setActiveIF(const string &activeIF)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request to set the active interface");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_SETACTIF);
        cmd.addString(activeIF.c_str());

        if (rpc.write(cmd,reply))
        {
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"active interface successfully set to %s",activeIF.c_str());
                return true;
            }
            else
            {
                printMessage(log::error,1,"something went wrong: request rejected");
                return false;
            }
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::getTrajectory(deque<Vector> &trajPos, deque<Vector> &trajOrien,
                              const unsigned int maxIterations, const double Ts)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request to retrieve the whole trajectory");

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_GETTRAJ);
         
        Property options;
        options.put("maxIterations",(int)maxIterations);
        options.put("Ts",Ts);

        string options_str=options.toString().c_str();

        Value val;
        val.fromString(("("+options_str+")").c_str());

        cmd.add(val);
        if (rpc.write(cmd,reply))
        {            
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                bool okPos=false;
                if (Bottle *BtrajPos=reply.get(1).asList())
                {
                    if (BtrajPos->get(0).asString()=="trajPos")
                    {
                        trajPos.clear();
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
                        printMessage(log::no_info,1,"trajectory in position has been computed");
                    }
                }

                bool okOrien=false;
                if (Bottle *BtrajOrien=reply.get(2).asList())
                {
                    if (BtrajOrien->get(0).asString()=="trajOrien")
                    {
                        trajOrien.clear();
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
                        printMessage(log::no_info,1,"trajectory in orientation has been computed");                        
                    }
                }

                if (okPos && okOrien)
                    return true;
            }

            printMessage(log::error,1,"something went wrong: request rejected");
            return false;
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


/************************************************************************/
bool D4CClient::executeTrajectory(const deque<Vector> &trajPos, const deque<Vector> &trajOrien,
                                  const double trajTime)
{
    if (isOpen)
    {
        printMessage(log::no_info,2,"request to execute user trajectory");

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

        Bottle cmd,reply;
        cmd.addVocab(D4C_VOCAB_CMD_EXECTRAJ);
        Bottle &cmdOptions=cmd.addList();

        Bottle &bTrajTime=cmdOptions.addList();
        bTrajTime.addString("trajTime");
        bTrajTime.addDouble(trajTime);

        Bottle &bPos=cmdOptions.addList();
        bPos.addString("trajPos");
        for (unsigned int i=0; i<trajPos.size(); i++)
        {
            Bottle &point=bPos.addList();
            for (size_t j=0; j<trajPos[i].length(); j++)
               point.addDouble(trajPos[i][j]);
        }

        Bottle &bOrien=cmdOptions.addList();
        bOrien.addString("trajOrien");
        for (unsigned int i=0; i<trajOrien.size(); i++)
        {
            Bottle &point=bOrien.addList();
            for (size_t j=0; j<trajOrien[i].length(); j++)
               point.addDouble(trajOrien[i][j]);
        }

        if (rpc.write(cmd,reply))
        {            
            if (reply.get(0).asVocab()==D4C_VOCAB_CMD_ACK)
            {
                printMessage(log::no_info,1,"trajectory executed");
                return true;
            }

            printMessage(log::error,1,"something went wrong: request rejected");
            return false;
        }
        else
        {
            printMessage(log::error,1,"unable to get reply from the server %s!",remote.c_str());
            return false;
        }
    }
    else
    {
        printMessage(log::warning,1,"client is not open");
        return false;
    }
}


