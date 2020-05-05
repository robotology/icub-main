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
 
Provides a on-line yarp-oriented database to collect properties 
of objects that are of interest for your specific application. 
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 
 
Date: first release 06/10/2010

CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 
\section intro_sec Description 
 
Provides a on-line yarp-oriented database to collect properties 
such as positions, colors, shapes, grasping points and so on 
about objects that are of interest for your specific 
application, i.e. normally real objects that the robot can play 
with but ideally any kind of objects you can think of. 
The user can set, get, add, remove items and make queries 
to the database. \n 
Importantly, the module is capable of running in real-time.

\section proto_sec Protocol
 
Notation used hereafter to explain available commands: [.] is a 
Vocab, "." is a string, <.> is a Value (i.e. string, double, 
int) or a List (so that also complex properties such as images 
can be attached to the objects). 
 
Reserved properties tags: 
 
- \e id is used to specify the unique integer identifier 
assigned to each stored item. 
 
- \e lifeTimer specifies the forgetting factor given in seconds, 
meaning that after \e lifeTimer seconds since its creation, the 
item is removed automatically from the database. The user can 
read this property at run-time, can even modify it or assign it
to an object after its creation. 
 
- \e propSet is used to specify a list of properties over which 
  execute the given command. 
 
The commands sent as bottles to the module port 
/<moduleName>/rpc are the following: 
 
<b>add</b> \n
<i>Format</i>: [add] (("prop0" <val0>) ("prop1" <val1>) ...) \n
<i>Reply</i>: [nack]; [ack] ("id" <num>) \n 
<i>Action</i>: a new item is added to the database with the 
given properties. \n 
A unique identifier is returned that is used to access the item.

<b>del</b> \n
<i>Format</i>: [del] (("id" <num>) (propSet ("prop0" "prop1" 
...))) \n
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: remove from the database the specified properties
belonging to the item specified with the given identifier. \n 
The special command "del ((id <num>))" removes the whole item.\n
The special command "[del] (all)" clears the current content of 
the database. 
 
<b>get</b> \n
<i>Format</i>: [get] (("id" <num>) (propSet ("prop0" "prop1" 
...))) \n
<i>Reply</i>: [nack]; [ack] (("prop0" <val0>) ("prop1" <val1>) 
...) \n 
<i>Action</i>: return the required properties assigned to the 
stored item. \n 
The special command "[get] ((id <num>))" returns all the 
properties. 
 
<b>set</b> \n
<i>Format</i>: [set] (("id" <num>) ("prop0" <val0>) ...) \n 
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: add/modify properties of the stored item. 
 
<b>lock</b> \n
<i>Format</i>: [lock] (("id" <num>)) \n 
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: lock the specified item; this way only the port 
owner can modify it later on through a [set] request.
 
<b>unlock</b> \n
<i>Format</i>: [unlock] (("id" <num>)) \n 
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: unlock the specified item.
 
<b>owner</b> \n
<i>Format</i>: [owner] (("id" <num>)) \n 
<i>Reply</i>: [nack]; [ack] ("owner_name") \n 
<i>Action</i>: ask for the port name of the item owner; a name
equal to "all" means that the item is not locked by any agent.
 
<b>time</b> \n
<i>Format</i>: [time] (("id" <num>)) \n 
<i>Reply</i>: [nack]; [ack] (<time>) \n 
<i>Action</i>: retrieve the time elapsed in seconds from the 
last change occured on the stored item. \n 
Negative values of <time> indicates that the item has not been 
modified since it was loaded within the database. 
 
<b>dump</b> \n 
<i>Format</i>: [dump] \n 
<i>Reply</i>: [ack] \n 
<i>Action</i>: ask the database to dump on the screen all the
stored items along with their properties. 
 
<b>synchronous broadcast</b> \n 
<i>Format</i>: [sync] [start] <T>/[stop] \n 
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: ask the database to start/stop broadcasting its 
content to a yarp port each <T> seconds. The parameter <T> is 
optional. 
 
<b>asynchronous broadcast</b> \n 
<i>Format</i>: [async] [on]/[off] \n 
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: ask the database to enable/disable the broadcast 
toward a yarp port whenever a change in the content occurs. 
 
<b>ask</b> \n
<i>Format</i>: [ask] (("prop0" "<" <val0>) || ("prop1" ">=" 
<val1>) ...) \n 
<i>Reply</i>: [nack]; [ack] ("id" (<num0> <num1> ...)) \n 
<i>Action</i>: query the database to find all the items whose 
properties match the conditions given in the command. You can 
compose multiple conditions using the boolean operators such as 
"||" for \e or and "&&" for \e and and each condition has to be 
expressed giving the property name, the value to compare with 
and the corresponding relational operator (e.g. ">", "<=", "==",
...).\n 
Commands such as "[ask] ((prop0) || (prop1))" will query whether
the properties exist or not. \n The special command "[ask] 
(all)" returns the whole set of ids present within the database. 
\n 
In order to simplify the implementation, nested conditions such 
as (cond1) && ((cond2) || (cond3)) are not handled; however, 
this is not a real limitation since nested conditions can be 
properly expanded: indeed, the previous example can be cast back 
to (cond1)&&(cond2) || (cond1)&&(cond3). 
 
<b>quit</b> \n 
<i>Format</i>: [quit] \n 
<i>Reply</i>: [ack] \n 
<i>Action</i>: quit the module.
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
--name \e moduleName 
- The parameter \e moduleName identifies the module's name; all 
  the open ports will be tagged with the prefix /<moduleName>/.
  If not specified \e objectsPropertiesCollector is assumed.
 
--db \e dbFileName 
- The parameter \e dbFileName specifies the name of the database 
  to load at startup (if already existing) and save at shutdown.
 
--context \e contextName 
- To specify the context where to search for the database file; 
  \e objectsPropertiesCollector is the default context.
 
--no-load-db 
- If this options is given then an empty database is started.
 
--no-save-db
- If this option is given then the content of database is not 
  saved at shutdown.
 
--sync-bc <T> 
- Broadcast the database content each \e T seconds. If not 
  specified, a period of 1.0 second is assumed.
 
--async-bc 
- Broadcast the database content whenever a change occurs. 
 
--stats 
- Enable statistics printouts.
 
\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
 
- \e /<moduleName>/rpc the remote procedure call port used to 
  send requests to the database and receive replies.

- \e /<moduleName>/broadcast:o the port used to broadcast the 
  database content in synchronous and asynchronous mode.
 
- \e /<moduleName>/modify:i the port used to modify the database
  content complying with the data format implemented for the
  broadcast port.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux and Windows.

\section reqexample_sec Examples 
 
Several examples of the requests you may forward to the 
database: 
 
\code 
command: [add] ((name ball) (color red) (x 1)) 
reply: [ack] (id 0) 
 
command: [add] ((name octopus) (color blue) (x 2)) 
reply: [ack] (id 1) 
 
command: [set] ((id 1) (x 3)) 
reply: [ack] 
 
command: [get] ((id 1)) 
reply: [ack] ((name octopus) (color blue) (x 3))
 
command: [ask] ((x < 10) && (color == blue)) 
reply: [ack] (id (1))
\endcode 
 
\author Ugo Pattacini
*/ 

#include <cstdio>
#include <cstdarg>
#include <mutex>
#include <sstream>
#include <string>
#include <map>
#include <deque>

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

#define CMD_ADD                         createVocab('a','d','d')
#define CMD_DEL                         createVocab('d','e','l')
#define CMD_GET                         createVocab('g','e','t')
#define CMD_SET                         createVocab('s','e','t')
#define CMD_LOCK                        createVocab('l','o','c','k')
#define CMD_UNLOCK                      createVocab('u','n','l','o')
#define CMD_OWNER                       createVocab('o','w','n','e')
#define CMD_TIME                        createVocab('t','i','m','e')
#define CMD_DUMP                        createVocab('d','u','m','p')
#define CMD_ASK                         createVocab('a','s','k')
#define CMD_SYNC                        createVocab('s','y','n','c')
#define CMD_ASYNC                       createVocab('a','s','y','n')
#define CMD_QUIT                        createVocab('q','u','i','t')
#define CMD_BYE                         createVocab('b','y','e')
                                        
#define REP_ACK                         createVocab('a','c','k')
#define REP_NACK                        createVocab('n','a','c','k')
#define REP_UNKNOWN                     createVocab('u','n','k','n')
                                        
#define OPT_ALL                         createVocab('a','l','l')
#define OPT_DISABLED                    (-1.0)
#define OPT_OWNERSHIP_ALL               ("all")
                                        
#define PROP_ID                         ("id")
#define PROP_LIFETIMER                  ("lifeTimer")
#define PROP_SET                        ("propSet")
#define BCTAG_EMPTY                     ("empty")
#define BCTAG_SYNC                      ("sync")
#define BCTAG_ASYNC                     ("async")


namespace relationalOperators
{

/************************************************************************/
bool alwaysTrue(Value &a, Value &b)
{
    return true;
}


/************************************************************************/
bool greater(Value &a, Value &b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()>b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()>b.asInt());
    else
        return false;
}


/************************************************************************/
bool greaterEqual(Value &a, Value &b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()>=b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()>=b.asInt());
    else
        return false;
}


/************************************************************************/
bool lower(Value &a, Value &b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()<b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()<b.asInt());
    else
        return false;
}


/************************************************************************/
bool lowerEqual(Value &a, Value &b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()<=b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()<=b.asInt());
    else
        return false;
}


/************************************************************************/
bool equal(Value &a, Value &b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()==b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()==b.asInt());
    else if (a.isString() && b.isString())
    {
        string aStr=a.asString();
        string bStr=b.asString();
        return (aStr==bStr);
    }
    else
        return false;
}


/************************************************************************/
bool notEqual(Value &a, Value &b)
{
    if (a.isDouble() && b.isDouble())
        return (a.asDouble()!=b.asDouble());
    else if (a.isInt() && b.isInt())
        return (a.asInt()!=b.asInt());
    else if (a.isString() && b.isString())
    {
        string aStr=a.asString();
        string bStr=b.asString();
        return (aStr!=bStr);
    }
    else
        return false;
}

}


/************************************************************************/
class DataBase : public PeriodicThread
{
protected:
    /************************************************************************/
    struct Item
    {
        Property *prop;
        double    lastUpdate;
        string    owner;

        Item() : prop(NULL),
                 lastUpdate(OPT_DISABLED),
                 owner(OPT_OWNERSHIP_ALL) { }
    };

    /************************************************************************/
    struct Condition
    {
        string prop;
        bool (*compare)(Value&,Value&);
        Value val;
    };

    ResourceFinder *rf;
    map<int,Item> itemsMap;
    mutex mtx;
    int  idCnt;
    bool initialized;
    bool nosavedb;
    bool quitting;

    BufferedPort<Bottle> *pBroadcastPort;
    bool asyncBroadcast;

    /************************************************************************/
    void clear()
    {
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
            delete it->second.prop;

        itemsMap.clear();
    }

    /************************************************************************/
    void eraseItem(map<int,Item>::iterator &it)
    {
        delete it->second.prop;
        itemsMap.erase(it);
    }

    /************************************************************************/
    void write(FILE *stream)
    {
        int i=0;
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
            fprintf(stream,"item_%d (%s %d) (%s)\n",
                    i++,PROP_ID,it->first,it->second.prop->toString().c_str());
    }

    /************************************************************************/
    bool recursiveCheck(Property *item, deque<Condition> &condList,
                        deque<string> &opList, const unsigned int i=0)
    {
        bool result;
        if (item->check(condList[i].prop))
        {
            // take the current value of the item's property under test
            Value &val=item->find(condList[i].prop);

            // compute the condition over the current value
            result=(*condList[i].compare)(val,condList[i].val);
        }
        else
            result=false;

        if ((i+1)>=condList.size())
            return result;
        else if (opList[i]=="||")
            return (result||recursiveCheck(item,condList,opList,i+1));
        else    // at this point we know we deal with "&&" operations
        {
            unsigned int j;
            for (j=i+1; j<condList.size(); j++)
            {
                if (result)
                {
                    if (item->check(condList[j].prop))
                    {
                        Value &val=item->find(condList[j].prop);
                        result=result&&(*condList[j].compare)(val,condList[j].val);
                    }
                    else
                        result=false;
                }

                if (j<opList.size())
                    if (opList[j]=="||")
                        break;
            }

            if (j>=condList.size())
                return result;
            else
                return (result||recursiveCheck(item,condList,opList,j+1));
        }
    }

    /************************************************************************/
    void run()
    {
        broadcast(BCTAG_SYNC);
    }

public:
    /************************************************************************/
    DataBase() : PeriodicThread(1.0)
    {
        pBroadcastPort=NULL;
        asyncBroadcast=false;
        initialized=false;
        nosavedb=false;
        quitting=false;
        idCnt=0;
    }

    /************************************************************************/
    ~DataBase()
    {
        if (isRunning())
            stop();

        save();
        clear();
    }

    /************************************************************************/
    void configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        if (initialized)
        {
            yWarning("database already initialized ...");
            return;
        }

        nosavedb=rf.check("no-save-db");
        if (!rf.check("no-load-db"))
            load();

        dump();
        initialized=true;
        yInfo("database ready ...");

        if (rf.check("sync-bc"))
        {
            setPeriod(rf.check("sync-bc",Value(1.0)).asDouble());
            start();
        }

        asyncBroadcast=rf.check("async-bc");
    }

    /************************************************************************/
    void setBroadcastPort(BufferedPort<Bottle> &broadcastPort)
    {
        pBroadcastPort=&broadcastPort;
    }

    /************************************************************************/
    void load()
    {
        string dbFileName=rf->findFile("db");
        if (dbFileName.empty())
        {
            yWarning("requested database to be loaded not found!");
            return;
        }

        yInfo("loading database from %s ...",dbFileName.c_str());

        lock_guard<mutex> lck(mtx);
        clear();
        idCnt=0;

        Property finProperty;
        finProperty.fromConfigFile(dbFileName);

        Bottle finBottle; finBottle.read(finProperty);
        for (int i=0; i<finBottle.size(); i++)
        {
            ostringstream tag;
            tag<<"item_"<<i;
            Bottle &b1=finBottle.findGroup(tag.str());

            if (b1.isNull())
                continue;

            if (b1.size()<3)
            {
                yWarning("error while loading %s!",tag.str().c_str());
                continue;
            }

            Bottle *b2=b1.get(1).asList();
            Bottle *b3=b1.get(2).asList();
            if ((b2==NULL) || (b3==NULL))
            {
                yWarning("error while loading %s!",tag.str().c_str());
                continue;
            }

            if (b2->size()<2)
            {
                yWarning("error while loading %s!",tag.str().c_str());
                continue;
            }

            int id=b2->get(1).asInt();
            itemsMap[id].prop=new Property(b3->toString().c_str());

            if (idCnt<=id)
                idCnt=id+1;
        }

        yInfo("database loaded");
    }

    /************************************************************************/
    void save()
    {
        if (nosavedb)
            return;

        lock_guard<mutex> lck(mtx);
        string dbFileName=rf->getHomeContextPath();
        dbFileName+="/";
        dbFileName+=rf->find("db").asString();
        yInfo("saving database in %s ...",dbFileName.c_str());

        FILE *fout=fopen(dbFileName.c_str(),"w");
        write(fout);
        fclose(fout);

        yInfo("database stored");
    }

    /************************************************************************/
    void dump()
    {
        lock_guard<mutex> lck(mtx);
        yInfo("dumping database content ...");

        if (itemsMap.size()==0)
            yInfo("empty");
        else
            write(stdout);
    }

    /************************************************************************/
    void broadcast(const string &type)
    {
        if (pBroadcastPort!=NULL)
        {
            if (pBroadcastPort->getOutputCount()>0)
            {
                lock_guard<mutex> lck(mtx);
                Bottle &bottle=pBroadcastPort->prepare();
                bottle.clear();

                bottle.addString(type);
                if (itemsMap.empty())
                    bottle.addString(BCTAG_EMPTY);
                else for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
                {
                    Bottle &item=bottle.addList();
                    item.read(*it->second.prop);

                    Bottle &idList=item.addList();
                    idList.addString(PROP_ID);
                    idList.addInt(it->first);
                }

                pBroadcastPort->writeStrict();
            }
        }
    }

    /************************************************************************/
    bool add(Bottle *content)
    {
        if (content==NULL)
            return false;

        if (content->check(PROP_ID))
        {
            yWarning("%s field cannot be specified as a property!",PROP_ID);
            return false;
        }

        lock_guard<mutex> lck(mtx);
        itemsMap[idCnt].prop=new Property(content->toString().c_str());
        itemsMap[idCnt].lastUpdate=Time::now();

        return true;
    }

    /************************************************************************/
    bool remove(Bottle *content)
    {
        if (content==NULL)
            return false;
        
        if (content->size()==1)
        {
            if (content->get(0).isVocab() || content->get(0).isString())
            {
                if (content->get(0).asVocab()==OPT_ALL)
                {
                    lock_guard<mutex> lck(mtx);
                    clear();
                    yInfo("database cleared");
                    return true;
                }
            }
        }

        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            Bottle *propSet=content->find(PROP_SET).asList();
            if (propSet!=NULL)
            {
                for (int i=0; i<propSet->size(); i++)
                    it->second.prop->unput(propSet->get(i).asString());

                it->second.lastUpdate=Time::now();
            }
            else
                eraseItem(it);

            return true;
        }

        return false;
    }

    /************************************************************************/
    bool get(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            Property *pProp=it->second.prop;
            response.clear();

            Bottle *propSet=content->find(PROP_SET).asList();
            if (propSet!=NULL)
            {
                Property prop;
                for (int i=0; i<propSet->size(); i++)
                {
                    string propName=propSet->get(i).asString();
                    if (pProp->check(propName))
                        prop.put(propName,pProp->find(propName));
                }

                response.read(prop);
            }
            else
                response.read(*pProp);

            return true;
        }

        return false;
    }

    /************************************************************************/
    bool set(Bottle *content, const string &agent)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            string owner=it->second.owner;
            if ((owner==OPT_OWNERSHIP_ALL) || (owner==agent))
            {
                Property *pProp=it->second.prop;
                for (int i=0; i<content->size(); i++)
                {
                    if (Bottle *option=content->get(i).asList())
                    {
                        if (option->size()<2)
                            continue;

                        string prop=option->get(0).asString();
                        Value  val=option->get(1);

                        if (prop==PROP_ID)
                            continue;

                        pProp->unput(prop);
                        pProp->put(prop,val);
                    }
                    else
                        continue;
                }

                it->second.lastUpdate=Time::now();
                return true;
            }
        }

        return false;
    }

    /************************************************************************/
    bool lock(Bottle *content, const string &agent)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            string owner=it->second.owner;
            if ((owner==OPT_OWNERSHIP_ALL) || (owner==agent))
            {
                it->second.owner=agent;
                return true;
            }
        }

        return false;
    }

    /************************************************************************/
    bool unlock(Bottle *content, const string &agent)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            string owner=it->second.owner;
            if ((owner==OPT_OWNERSHIP_ALL) || (owner==agent))
            {
                it->second.owner=OPT_OWNERSHIP_ALL;
                return true;
            }
        }

        return false;
    }

    /************************************************************************/
    bool owner(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;
        
        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            response.clear();
            string &owner=it->second.owner;
            response.addString(owner);
            return true;
        }

        return false;
    }

    /************************************************************************/
    bool time(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;
        
        if (!content->check(PROP_ID))
        {
            yWarning("%s field not present within the request!",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();

        lock_guard<mutex> lck(mtx);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            response.clear();
            if (it->second.lastUpdate<0.0)
                response.addDouble(it->second.lastUpdate);
            else
            {
                double dt=Time::now()-it->second.lastUpdate;
                response.addDouble(dt);
            }
            return true;
        }

        return false;
    }

    /************************************************************************/
    bool ask(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;

        lock_guard<mutex> lck(mtx);
        if (content->size()==1)
        {
            if (content->get(0).isVocab() || content->get(0).isString())
            {
                if (content->get(0).asVocab()==OPT_ALL)
                {
                    response.clear();
                
                    for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
                        response.addInt(it->first);

                    return true;
                }
            }
        }

        deque<Condition> condList;
        deque<string>    opList;

        // we cannot accept a conditions string ending with
        // a boolean operator
        if (!(content->size()&0x01))
        {
            yWarning("uncorrect conditions received!");
            return false;
        }

        // parse the received conditions and build the lists
        for (int i=0; i<content->size(); i+=2)
        {
            if (Bottle *b=content->get(i).asList())
            {
                Condition condition;
                string operation;

                if (b->size()==1)
                {
                    condition.prop=b->get(0).asString();
                    condition.compare=&relationalOperators::alwaysTrue;
                }
                else if (b->size()>2)
                {
                    condition.prop=b->get(0).asString();
                    operation=b->get(1).asString();
                    condition.val=b->get(2);

                    if (operation==">")
                        condition.compare=&relationalOperators::greater;
                    else if (operation==">=")
                        condition.compare=&relationalOperators::greaterEqual;
                    else if (operation=="<")
                        condition.compare=&relationalOperators::lower;
                    else if (operation=="<=")
                        condition.compare=&relationalOperators::lowerEqual;
                    else if (operation=="==")
                        condition.compare=&relationalOperators::equal;
                    else if (operation=="!=")
                        condition.compare=&relationalOperators::notEqual;
                    else
                    {
                        yWarning("unknown relational operator '%s'!",operation.c_str());
                        return false;
                    }
                }
                else
                {
                    yWarning("wrong condition given!");
                    return false;
                }

                condList.push_back(condition);

                if ((i+1)<content->size())
                {
                    operation=content->get(i+1).asString();
                    if ((operation!="||") && (operation!="&&"))
                    {
                        yWarning("unknown boolean operator '%s'!",operation.c_str());
                        return false;
                    }
                    else
                        opList.push_back(operation);
                }
            }
            else
            {
                yWarning("wrong condition given!");
                return false;
            }
        }

        response.clear();

        // apply the conditions to each item
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
        {
            // do recursion and keep only the item that
            // satisfies the whole list of conditions
            if (recursiveCheck(it->second.prop,condList,opList))
                response.addInt(it->first);
        }

        return true;
    }

    /************************************************************************/
    void periodicHandler(const double dt)   // manage the items life-timers
    {
        mtx.lock();
        bool erased=false;
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
        {
            Property *pProp=it->second.prop;
            if (pProp->check(PROP_LIFETIMER))
            {
                double lifeTimer=pProp->find(PROP_LIFETIMER).asDouble()-dt;
                if (lifeTimer<=0.0)
                {
                    eraseItem(it);
                    erased=true;
                    break;
                }
                else
                {
                    pProp->unput(PROP_LIFETIMER);
                    pProp->put(PROP_LIFETIMER,lifeTimer);
                }
            }
        }
        mtx.unlock();

        if (asyncBroadcast && erased)
            broadcast(BCTAG_ASYNC);
    }

    /************************************************************************/
    bool isQuitting() const
    {
        return quitting;
    }

    /************************************************************************/
    void respond(ConnectionReader &connection, const Bottle &command, Bottle &reply)
    {
        string agent=connection.getRemoteContact().getName();
        if (command.size()<1)
        {
            reply.addVocab(REP_NACK);
            return;
        }

        reply.clear();
        int cmd=command.get(0).asVocab();
        switch(cmd)
        {
            //-----------------
            case CMD_ADD:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();
                if (add(content))
                {
                    reply.addVocab(REP_ACK);
                    Bottle &b=reply.addList();
                    b.addString(PROP_ID);
                    b.addInt(idCnt);
                    idCnt++;

                    if (asyncBroadcast)
                        broadcast(BCTAG_ASYNC);
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_DEL:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();
                if (remove(content))
                {
                    reply.addVocab(REP_ACK);
                    if (asyncBroadcast)
                        broadcast(BCTAG_ASYNC);
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_GET:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle response;
                Bottle *content=command.get(1).asList();
                if (get(content,response))
                {
                    reply.addVocab(REP_ACK);
                    reply.addList()=response;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_SET:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }
    
                Bottle *content=command.get(1).asList();
                if (set(content,agent))
                {
                    reply.addVocab(REP_ACK);
                    if (asyncBroadcast)
                        broadcast(BCTAG_ASYNC);
                }
                else
                    reply.addVocab(REP_NACK);
    
                break;
            }

            //-----------------
            case CMD_LOCK:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();
                if (lock(content,agent))
                    reply.addVocab(REP_ACK);
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_UNLOCK:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle *content=command.get(1).asList();
                if (unlock(content,agent))
                    reply.addVocab(REP_ACK);
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_OWNER:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle response;
                Bottle *content=command.get(1).asList();
                if (owner(content,response))
                {
                    reply.addVocab(REP_ACK);
                    reply.addList()=response;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_TIME:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle response;
                Bottle *content=command.get(1).asList();
                if (time(content,response))
                {
                    reply.addVocab(REP_ACK);
                    reply.addList()=response;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_DUMP:
            {
                dump();
                reply.addVocab(REP_ACK);
                break;
            }

            //-----------------
            case CMD_SYNC:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                int opt=command.get(1).asVocab();
                if (opt==Vocab::encode("start"))
                {
                    if (command.size()>=3)
                        setPeriod(command.get(2).asDouble());

                    if (!isRunning())
                        start();
                    else if (isSuspended())
                        resume();

                    reply.addVocab(REP_ACK);
                }
                else if (opt==Vocab::encode("stop"))
                {
                    if (isRunning() && !isSuspended())
                        suspend();

                    reply.addVocab(REP_ACK);
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_ASYNC:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                int opt=command.get(1).asVocab();
                if (opt==Vocab::encode("on"))
                {
                    asyncBroadcast=true;
                    reply.addVocab(REP_ACK);
                }
                else if (opt==Vocab::encode("off"))
                {
                    asyncBroadcast=false;
                    reply.addVocab(REP_ACK);
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_ASK:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle response;
                Bottle *content=command.get(1).asList();
                if (ask(content,response))
                {
                    reply.addVocab(REP_ACK);
                    Bottle &b=reply.addList();
                    b.addString(PROP_ID);
                    b.addList()=response;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            case CMD_QUIT:
            case CMD_BYE:
            {
                quitting=true;
                reply.addVocab(REP_ACK);
                break;
            }

            //-----------------
            default:
            {
                yWarning("received unknown command!");
                reply.addVocab(REP_UNKNOWN);
            }
        }
    }

    /************************************************************************/
    bool modify(const Bottle &content)
    {
        if (content.size()==0)
            return false;

        string type=content.get(0).asString();
        if ((type!=BCTAG_EMPTY) && (type!=BCTAG_SYNC) && (type!=BCTAG_ASYNC))
            return false;

        mtx.lock();
        clear();

        if (type!=BCTAG_EMPTY)
        {
            idCnt=0;
            for (int i=1; i<content.size(); i++)
            {
                if (Bottle *item=content.get(i).asList())
                {
                    if (Bottle *idList=item->get(0).asList())
                    {
                        if (idList->size()==2)
                        {
                            if (idList->get(0).asString()==PROP_ID)
                            {
                                int id=idList->get(1).asInt();
                                itemsMap[id].prop=new Property(item->tail().toString().c_str());

                                if (idCnt<=id)
                                    idCnt=id+1;
                            }
                        }
                    }
                }
            }
        }

        mtx.unlock();

        if (asyncBroadcast)
            broadcast(BCTAG_ASYNC);

        return true;
    }
};


/************************************************************************/
class RpcProcessor : public PortReader
{
protected:
    DataBase *pDataBase;
    unsigned int nCalls;
    double cumTime;

    /************************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle command;
        if (!command.read(connection) || (pDataBase==NULL))
            return false;

        Bottle reply;
        double t0=Time::now();
        pDataBase->respond(connection,command,reply);
        cumTime+=Time::now()-t0;
        nCalls++;

        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

        return true;
    }

public:
    /************************************************************************/
    RpcProcessor()
    {
        pDataBase=NULL;
        nCalls=0;
        cumTime=0.0;
    }

    /************************************************************************/
    void setDataBase(DataBase &dataBase)
    {
        pDataBase=&dataBase;
    }

    /************************************************************************/
    void getStats(unsigned int &nCalls, double &cumTime) const
    {
        nCalls=this->nCalls;
        cumTime=this->cumTime;
    }
};


/************************************************************************/
class DataBaseModifyPort : public BufferedPort<Bottle>
{
protected:
    DataBase *pDataBase;

    /************************************************************************/
    void onRead(Bottle &content)
    {
        if (pDataBase!=NULL)
            pDataBase->modify(content);
    }

public:
    /************************************************************************/
    DataBaseModifyPort() : pDataBase(NULL) { }

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
    DataBase             dataBase;
    RpcProcessor         rpcProcessor;
    DataBaseModifyPort   modifyPort;
    RpcServer            rpcPort;
    BufferedPort<Bottle> bcPort;

    int cnt;
    bool stats;
    unsigned int nCallsOld;
    double cumTimeOld;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        dataBase.configure(rf);
        stats=rf.check("stats");

        string name=rf.check("name",Value("objectsPropertiesCollector")).asString();
        dataBase.setBroadcastPort(bcPort);
        rpcProcessor.setDataBase(dataBase);
        rpcPort.setReader(rpcProcessor);
        modifyPort.setDataBase(dataBase);
        modifyPort.useCallback();
        rpcPort.open("/"+name+"/rpc");
        bcPort.open("/"+name+"/broadcast:o");
        modifyPort.open("/"+name+"/modify:i");

        cnt=0;
        nCallsOld=0;
        cumTimeOld=0.0;

        return true;
    }

    /************************************************************************/
    bool close()
    {
        modifyPort.disableCallback();
        dataBase.stop();

        rpcPort.interrupt();
        bcPort.interrupt();
        modifyPort.interrupt();

        rpcPort.close();
        bcPort.close();
        modifyPort.close();

        return true;
    }

    /************************************************************************/
    bool updateModule()
    {
        dataBase.periodicHandler(getPeriod());

        // back-up straightaway the database each 15 minutes
        if ((++cnt)*getPeriod()>(15.0*60.0))
        {
            dataBase.save();
            cnt=0;
        }

        if (stats)
        {
            unsigned int nCalls; double cumTime;
            rpcProcessor.getStats(nCalls,cumTime);

            unsigned int calls=nCalls-nCallsOld;
            double timeSpent=cumTime-cumTimeOld;
            yInfo("*** Statistics: received %d/%g [requests/s]; %g [ms/request] spent on average",
                  calls,getPeriod(),timeSpent==0.0?0.0:(1e3*timeSpent)/(double)calls);

            nCallsOld=nCalls;
            cumTimeOld=cumTime;
        }

        return !dataBase.isQuitting();
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setDefaultContext("objectsPropertiesCollector");
    rf.setDefault("db","dataBase.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        printf("Options\n");
        printf("\t--name        <name>: collector name (default: objectsPropertiesCollector)\n");
        printf("\t--db      <fileName>: database file name to load at startup/save at shutdown (default: dataBase.ini)\n");
        printf("\t--context  <context>: context to search for database file (default: objectsPropertiesCollector)\n");
        printf("\t--no-load-db        : start an empty database\n");
        printf("\t--no-save-db        : prevent from saving the content of database at shutdown\n");
        printf("\t--sync-bc        <T>: broadcast the database content each T seconds\n");
        printf("\t--async-bc          : broadcast the database content whenever a change occurs\n");
        printf("\t--stats             : enable statistics printouts\n");
        printf("\n");
        return 0;
    }

    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    objectsPropertiesCollectorModule collector;
    return collector.runModule(rf);
}


