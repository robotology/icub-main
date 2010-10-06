/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/**
@ingroup icub_module

\defgroup objectsPropertiesCollector objectsPropertiesCollector
 
Provides a on-line database to collect properties such as 
positions, colors, shapes, grasping points and so on about 
objects that are interesting for your specific application, i.e.
normally real objects that the robot can play with but ideally 
any kind of objects you can think of. 
The user can set, get, add, delete items and even make queries 
to the database. 
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 
 
Date: first release 06/10/2010

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section proto_sec Protocol
 
Notation: [.] is a Vocab, {.} is a string, <.> is a Value (i.e. 
string, double, int). 
 
Reserved properties tag: 
--id used to specify the unique integer identifier assigned to 
  each stored item.
--lifeTimer specify the forgetting factor given in seconds, 
  meaning that after <lifeTimer> seconds since its creation, the
  item is removed automatically from the server.
 
The commands sent as bottles to the module port /<modName>/rpc
are the following: 
 
<b>ADD</b> 
Format: [add] (({prop0} <val0>) ({prop1} <val1>) ...) 
Reply: [nack]; [ack] (id <num>) 
Action: a new item is added to the database with the given 
properties. A unique identifier is returned that is used to 
access to the item later on. 

<b>DELETE</b> 
Format: [del] ((id <num>)) 
Reply: [nack]; [ack] 
Action: remove from the database an item specified with the 
given identifier. 
 
<b>GET</b> 
Format: [get] ((id <num>)) 
Reply: [nack]; [ack] (({prop0} <val0>) ({prop1} <val1>) ...) 
Action: return all the properties assigned to the stored item.
 
<b>SET</b> 
Format: [get] ((id <num>) ({prop2} <val2>) ...) 
Reply: [nack]; [ack] 
Action: add/modify properties of the stored item.
 
<b>DUMP</b> 
Format: [dump] 
Reply: [ack] 
Action: ask the database to dump on the screen all the stored 
items along their properties. 
 
<b>ASK</b> 
Format: [ask] (({prop0} < <val0>) || ({prop1} >= <val1>) ...) 
Reply: [nack]; [ack] (id (id0 id1 ...))
Action: query the database to find all the items whose 
properties match the conditions given in the command. You can 
compose multiple conditions using the boolean operators such as 
''||' for 'or' and '&&' for 'and' and each condition has to be 
expressed giving the property name, the value to compare with 
and the correpsonding relational operator (e.g. >, <=, ==, ...).
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
--name \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix /<name>/. If not
  specified \e objectsPropertiesCollector is assumed.
 
\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
 
- \e <name>/rpc the remote procedure call port used to send 
  command to the database and receive replies.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example 
 
Several examples of the requests you may forward to the 
database: 
 
\code 
command: [add] ((prop0 0) (prop1 1)) 
reply: [ack] (id 0) 
 
command: [add] ((prop0 1) (prop2 2)) 
reply: [ack] (id 1) 
 
command: [set] ((id 1) (prop2 3)) 
reply: [ack] 
 
command: [get] ((id 1)) 
reply: [ack] ((id 1) (prop0 1) (prop2 3))
 
command: [ask] ((prop0 < 10) && (prop1 == 1)) 
reply: [ack] (id (0)) 
\endcode 
 
\author Ugo Pattacini
*/ 

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>

#include <stdio.h>
#include <string>
#include <deque>

using namespace yarp::os;
using namespace std;

#define CMD_ADD         VOCAB3('a','d','d')
#define CMD_DEL         VOCAB3('d','e','l')
#define CMD_GET         VOCAB3('g','e','t')
#define CMD_SET         VOCAB3('s','e','t')
#define CMD_DUMP        VOCAB4('d','u','m','p')
#define CMD_ASK         VOCAB3('a','s','k')

#define REP_ACK         VOCAB3('a','c','k')
#define REP_NACK        VOCAB4('n','a','c','k')
#define REP_UNKNOWN     VOCAB4('u','n','k','n')


/************************************************************************/
bool greater(Value &a, Value& b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()>b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()>b.asInt());
    else
        return false;
}


/************************************************************************/
bool greaterEqual(Value &a, Value& b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()>=b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()>=b.asInt());
    else
        return false;
}


/************************************************************************/
bool lower(Value &a, Value& b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()<b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()<b.asInt());
    else
        return false;
}


/************************************************************************/
bool lowerEqual(Value &a, Value& b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()<=b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()<=b.asInt());
    else
        return false;
}


/************************************************************************/
bool equal(Value &a, Value& b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()==b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()==b.asInt());
    else if (a.isString() && b.isString())
    {
        string aStr=a.asString().c_str();
        string bStr=b.asString().c_str();

        return (aStr==bStr);
    }
    else
        return false;
}


/************************************************************************/
class DataBase
{
protected:
    deque<Property> itemsList;
    Semaphore mutex;
    int idCnt;

    /************************************************************************/
    struct Condition
    {
        string prop;
        bool (*compare)(Value&,Value&);
        Value val;
    };

    /************************************************************************/
    void add(Bottle *content)
    {
        Property item(content->toString().c_str());
        item.unput("id");
        item.put("id",idCnt);
        itemsList.push_back(item);

        fprintf(stdout,"added item %s\n",item.toString().c_str());
    }

    /************************************************************************/
    bool remove(Bottle *content)
    {
        Property request(content->toString().c_str());

        if (!request.check("id"))
        {
            fprintf(stdout,"id field not present within the request!\n");
            return false;
        }
        
        int id=request.find("id").asInt();

        fprintf(stdout,"removing item %d ... ",id);

        for (deque<Property>::iterator it=itemsList.begin(); it!=itemsList.end(); it++)
        {
            if (it->find("id").asInt()==id)
            {
                fprintf(stdout,"successfully\n");
                itemsList.erase(it);
                return true;
            }
        }

        fprintf(stdout,"not present!\n");

        return false;
    }

    /************************************************************************/
    bool get(Bottle *content, Bottle &item)
    {
        Property request(content->toString().c_str());

        if (!request.check("id"))
        {
            fprintf(stdout,"id field not present within the request!\n");
            return false;
        }

        int id=request.find("id").asInt();

        fprintf(stdout,"getting item %d ... ",id);

        for (deque<Property>::iterator it=itemsList.begin(); it!=itemsList.end(); it++)
        {
            if (it->find("id").asInt()==id)
            {
                item.clear();
                item.fromString(it->toString().c_str());
                fprintf(stdout,"%s\n",item.toString().c_str());
                return true;
            }
        }

        fprintf(stdout,"not present!\n");

        return false;
    }

    /************************************************************************/
    bool set(Bottle *content)
    {
        Property request(content->toString().c_str());

        if (!request.check("id"))
        {
            fprintf(stdout,"id field not present within the request!\n");
            return false;
        }

        int id=request.find("id").asInt();

        fprintf(stdout,"setting item %d ... ",id);

        for (deque<Property>::iterator it=itemsList.begin(); it!=itemsList.end(); it++)
        {
            if (it->find("id").asInt()==id)
            {
                request.unput("id");
                Bottle b(request.toString().c_str());

                fprintf(stdout,"%s\n",b.toString().c_str());

                for (int i=0; i<b.size(); i++)
                {
                    Bottle *option=b.get(i).asList();

                    if (option->size()<2)
                    {
                        fprintf(stdout,"invalid property!\n");
                        continue;
                    }

                    string prop=option->get(0).asString().c_str();
                    Value  val=option->get(1);

                    it->unput(prop.c_str());
                    it->put(prop.c_str(),val);
                }

                return true;
            }
        }

        fprintf(stdout,"not present!\n");

        return false;
    }

    /************************************************************************/
    void dump()
    {
        fprintf(stdout,"dumping database content ... \n");

        if (itemsList.size()==0)
            fprintf(stdout,"empty\n");

        for (unsigned int i=0; i<itemsList.size(); i++)
            fprintf(stdout,"object %d: %s\n",i,itemsList[i].toString().c_str());
    }

    /************************************************************************/
    bool ask(Bottle *content, Bottle &items)
    {
        deque<Condition> condList;
        deque<string>    opList;

        // we cannot accept a conditions string ending with
        // a boolean operator
        if (!(content->size()&0x01))
        {
            fprintf(stdout,"uncorrect conditions received!\n");
            return false;
        }

        // parse the received conditions and build the lists
        for (int i=0; i<content->size(); i+=2)
        {
            Bottle *b=content->get(i).asList();
            Condition condition;
            string operation;

            if (b->size()<3)
            {
                fprintf(stdout,"condition given with less than 3 elements!\n");
                return false;
            }
            
            condition.prop=b->get(0).asString().c_str();
            condition.val=b->get(2);

            operation=b->get(1).asString().c_str();
            if (operation==">")
                condition.compare=&greater;
            else if (operation==">=")
                condition.compare=&greaterEqual;
            else if (operation=="<")
                condition.compare=&lower;
            else if (operation=="<=")
                condition.compare=&lowerEqual;
            else if (operation=="==")
                condition.compare=&equal;
            else
            {
                fprintf(stdout,"unknown relational operator '%s'!\n",operation.c_str());
                return false;
            }

            condList.push_back(condition);

            if ((i+1)<content->size())
            {
                operation=content->get(i+1).asString().c_str();
                if ((operation!="||") && (operation!="&&"))
                {
                    fprintf(stdout,"unknown boolean operator '%s'!\n",operation.c_str());
                    return false;
                }
                else
                    opList.push_back(operation);
            }
        }

        items.clear();

        // apply the conditions to each item
        for (unsigned int i=0; i<itemsList.size(); i++)
        {
            // there must be at least one condition to process
            string &prop=condList[0].prop;
            bool finalRes;

            if (itemsList[i].check(prop.c_str()))
            {
                Value &val=itemsList[i].find(prop.c_str());
                finalRes=(*condList[0].compare)(val,condList[0].val);
            }
            else
                finalRes=false;

            // if we're required to process more than one condition
            // we go ahead accumulating the temporary results
            for (unsigned int j=0; j<opList.size(); j++)
            {
                int k=j+1;

                string &prop=condList[k].prop;
                bool currentRes;

                if (itemsList[i].check(prop.c_str()))
                {
                    Value &val=itemsList[i].find(prop.c_str());
                    currentRes=(*condList[k].compare)(val,condList[k].val);
                }
                else
                    currentRes=false;

                if (opList[j]=="||")
                    finalRes=finalRes||currentRes;
                else if (opList[j]=="&&")
                    finalRes=finalRes&&currentRes;
            }

            // keep only the item that satisfies the whole list of conditions
            if (finalRes)
                items.addInt(itemsList[i].find("id").asInt());
        }

        return true;
    }

public:
    /************************************************************************/
    DataBase() : idCnt(0)
    {
        fprintf(stdout,"database ready ...\n");
    }

    /************************************************************************/
    void periodicHandler(const double dt)   // manage the items life-timers
    {
        mutex.wait();

        for (deque<Property>::iterator it=itemsList.begin(); it!=itemsList.end(); it++)
        {
            if (it->check("lifeTimer"))
            {
                double lifeTimer=it->find("lifeTimer").asDouble()-dt;

                if (lifeTimer<0.0)
                {
                    fprintf(stdout,"item %d expired\n",it->find("id").asInt());
                    itemsList.erase(it);

                    // bug ?? a run-time error occurs if we don't break here;
                    // never mind: at the next call we keep on checking
                    break;
                }
                else
                {
                    it->unput("lifeTimer");
                    it->put("lifeTimer",lifeTimer);
                }
            }
        }

        mutex.post();
    }

    /************************************************************************/
    void respond(const Bottle &command, Bottle &reply)
    {
        if (command.size()<1)
        {
            reply.addVocab(REP_NACK);
            return;
        }

        reply.clear();

        int cmd=command.get(0).asVocab();

        mutex.wait();

        switch(cmd)
        {
            case CMD_ADD:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();

                add(content);
                reply.addVocab(REP_ACK);
                Bottle &b=reply.addList();
                b.addString("id");
                b.addInt(idCnt);
                idCnt++;

                break;
            }

            case CMD_DEL:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();

                if (remove(content))
                    reply.addVocab(REP_ACK);
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            case CMD_GET:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();
                Bottle item;

                if (get(content,item))
                {
                    reply.addVocab(REP_ACK);
                    reply.addList()=item;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            case CMD_SET:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }
    
                Bottle *content=command.get(1).asList();
    
                if (set(content))
                    reply.addVocab(REP_ACK);
                else
                    reply.addVocab(REP_NACK);
    
                break;
            }
    
            case CMD_DUMP:
            {
                dump();
                reply.addVocab(REP_ACK);
                break;
            }

            case CMD_ASK:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();
                Bottle items;

                if (ask(content,items))
                {
                    reply.addVocab(REP_ACK);
                    Bottle &b1=reply.addList();
                    b1.addString("id");
                    Bottle &b2=b1.addList();
                    b2.addList()=items;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            default:
            {
                fprintf(stdout,"received unknown command!\n");
                reply.addVocab(REP_UNKNOWN);
            }
        }

        mutex.post();
    }
};


/************************************************************************/
class RpcProcessor : public PortReader
{
protected:
    DataBase *pDataBase;

    /************************************************************************/
    virtual bool read(ConnectionReader &connection)
    {
        Bottle command, reply;

        if (!command.read(connection))
            return false;

        pDataBase->respond(command,reply);

        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

        return true;
    }

public:
    /************************************************************************/
    void setDataBase(DataBase &dataBase)
    {
        pDataBase=&dataBase;
    }
};


/************************************************************************/
class objectsPropertiesCollectorModule: public RFModule
{
private:
    DataBase     dataBase;
    RpcProcessor rpcProcessor;
    Port         rpcPort;

public:
    /************************************************************************/
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        string name=rf.check("name",Value("objectsPropertiesCollector")).asString().c_str();

        rpcProcessor.setDataBase(dataBase);
        rpcPort.setReader(rpcProcessor);
        rpcPort.open(("/"+name+"/rpc").c_str());

        return true;
    }

    /************************************************************************/
    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    /************************************************************************/
    virtual bool updateModule()
    {        
        dataBase.periodicHandler(getPeriod());
        return true;
    }

    /************************************************************************/
    virtual double getPeriod()
    {
        return 1.0;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);

    if (rf.check("help"))
    {
        fprintf(stdout,"Options\n\n");
        fprintf(stdout,"\t--name <name>: collector name (default: objectsPropertiesCollector)\n");
        fprintf(stdout,"\n");

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    objectsPropertiesCollectorModule collector;

    return collector.runModule(rf);
}



