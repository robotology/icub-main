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
The user can set, get, add, remove items and even make queries 
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
 
--empty 
- If this options is given then an empty database is started.
 
--nosave
- If this option is given then the content of database is not 
  saved at shutdown.
 
--verbose 
- Enable some verbosity. 
 
--sync_bc <T> 
- Broadcast the database content each \e T seconds. If not 
  specified, a period of 1.0 second is assumed.
 
--async_bc 
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

#include <stdio.h>
#include <stdarg.h>
#include <sstream>
#include <string>
#include <map>
#include <deque>

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

#define CMD_ADD                         VOCAB3('a','d','d')
#define CMD_DEL                         VOCAB3('d','e','l')
#define CMD_GET                         VOCAB3('g','e','t')
#define CMD_SET                         VOCAB3('s','e','t')
#define CMD_LOCK                        VOCAB4('l','o','c','k')
#define CMD_UNLOCK                      VOCAB4('u','n','l','o')
#define CMD_OWNER                       VOCAB4('o','w','n','e')
#define CMD_TIME                        VOCAB4('t','i','m','e')
#define CMD_DUMP                        VOCAB4('d','u','m','p')
#define CMD_ASK                         VOCAB3('a','s','k')
#define CMD_SYNC                        VOCAB4('s','y','n','c')
#define CMD_ASYNC                       VOCAB4('a','s','y','n')
#define CMD_QUIT                        VOCAB4('q','u','i','t')
#define CMD_BYE                         VOCAB3('b','y','e')
                                        
#define REP_ACK                         VOCAB3('a','c','k')
#define REP_NACK                        VOCAB4('n','a','c','k')
#define REP_UNKNOWN                     VOCAB4('u','n','k','n')
                                        
#define OPT_ALL                         VOCAB3('a','l','l')
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
        string aStr=a.asString().c_str();
        string bStr=b.asString().c_str();
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
        string aStr=a.asString().c_str();
        string bStr=b.asString().c_str();
        return (aStr!=bStr);
    }
    else
        return false;
}


}


/************************************************************************/
class DataBase : public RateThread
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
    Mutex mutex;
    int  idCnt;
    bool initialized;
    bool nosave;
    bool verbose;
    bool quitting;

    BufferedPort<Bottle> *pBroadcastPort;
    bool asyncBroadcast;

    /************************************************************************/
    int printMessage(const char *format, ...)
    {
        if (verbose)
        {
            va_list ap;
            va_start(ap,format);
            int ret=vfprintf(stdout,format,ap);
            va_end(ap);

            return ret;
        }
        else
            return -1;
    }

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
        if (item->check(condList[i].prop.c_str()))
        {
            // take the current value of the item's property under test
            Value &val=item->find(condList[i].prop.c_str());

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
                    if (item->check(condList[j].prop.c_str()))
                    {
                        Value &val=item->find(condList[j].prop.c_str());
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
    DataBase() : RateThread(1000)
    {
        pBroadcastPort=NULL;
        asyncBroadcast=false;
        initialized=false;
        nosave=false;
        verbose=false;
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
        verbose=rf.check("verbose");
        if (initialized)
        {
            printMessage("database already initialized ...\n");
            return;
        }

        nosave=rf.check("nosave");
        if (!rf.check("empty"))
            load();

        dump();
        initialized=true;
        printMessage("database ready ...\n");

        if (rf.check("sync_bc"))
        {
            setRate((int)(1000.0*rf.check("sync_bc",Value(1.0)).asDouble()));
            start();
        }

        asyncBroadcast=rf.check("async_bc");
    }

    /************************************************************************/
    void setBroadcastPort(BufferedPort<Bottle> &broadcastPort)
    {
        pBroadcastPort=&broadcastPort;
    }

    /************************************************************************/
    void load()
    {
        string dbFileName=rf->findFile("db").c_str();
        if (dbFileName.empty())
        {
            printMessage("requested database to be loaded not found!\n");
            return;
        }

        printMessage("loading database from %s ...\n",dbFileName.c_str());

        LockGuard lg(mutex);
        clear();
        idCnt=0;

        Property finProperty;
        finProperty.fromConfigFile(dbFileName.c_str());

        Bottle finBottle; finBottle.read(finProperty);
        for (int i=0; i<finBottle.size(); i++)
        {
            ostringstream tag;
            tag<<"item_"<<i;
            Bottle &b1=finBottle.findGroup(tag.str().c_str());

            if (b1.isNull())
                continue;

            if (b1.size()<3)
            {
                printMessage("error while loading %s!\n",tag.str().c_str());
                continue;
            }

            Bottle *b2=b1.get(1).asList();
            Bottle *b3=b1.get(2).asList();
            if ((b2==NULL) || (b3==NULL))
            {
                printMessage("error while loading %s!\n",tag.str().c_str());
                continue;
            }

            if (b2->size()<2)
            {
                printMessage("error while loading %s!\n",tag.str().c_str());
                continue;
            }

            int id=b2->get(1).asInt();
            itemsMap[id].prop=new Property(b3->toString().c_str());

            if (idCnt<=id)
                idCnt=id+1;
        }

        printMessage("database loaded\n");
    }

    /************************************************************************/
    void save()
    {
        if (nosave)
            return;

        LockGuard lg(mutex);
        string dbFileName=rf->getHomeContextPath().c_str();
        dbFileName+="/";
        dbFileName+=rf->find("db").asString().c_str();
        printMessage("saving database in %s ...\n",dbFileName.c_str());

        FILE *fout=fopen(dbFileName.c_str(),"w");
        write(fout);
        fclose(fout);

        printMessage("database stored\n");
    }

    /************************************************************************/
    void dump()
    {
        LockGuard lg(mutex);
        printMessage("dumping database content ... \n");

        if (itemsMap.size()==0)
            printMessage("empty\n");
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
                LockGuard lg(mutex);
                Bottle bottle;

                bottle.addString(type.c_str());
                if (itemsMap.empty())
                    bottle.addString(BCTAG_EMPTY);
                else for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
                {
                    Bottle &item=bottle.addList();
                    Bottle &idList=item.addList();
                    idList.addString(PROP_ID);
                    idList.addInt(it->first);
                    item.read(*it->second.prop);
                }

                pBroadcastPort->prepare()=bottle;
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
            printMessage("%s field cannot be specified as a property!\n",PROP_ID);
            return false;
        }

        LockGuard lg(mutex);
        Property *item=new Property(content->toString().c_str());
        itemsMap[idCnt].prop=item;
        itemsMap[idCnt].lastUpdate=Time::now();

        printMessage("added item %s\n",item->toString().c_str());
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
                    LockGuard lg(mutex);
                    clear();
                    printMessage("database cleared\n");
                    return true;
                }
            }
        }

        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("removing item %d ... ",id);

        LockGuard lg(mutex);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            Bottle *propSet=content->find(PROP_SET).asList();
            if (propSet!=NULL)
            {
                for (int i=0; i<propSet->size(); i++)
                    it->second.prop->unput(propSet->get(i).asString().c_str());

                it->second.lastUpdate=Time::now();
            }
            else
                eraseItem(it);

            printMessage("successfully\n");
            return true;
        }

        printMessage("not present!\n");
        return false;
    }

    /************************************************************************/
    bool get(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("getting item %d ... ",id);

        LockGuard lg(mutex);
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
                    string propName=propSet->get(i).asString().c_str();
                    if (pProp->check(propName.c_str()))
                        prop.put(propName.c_str(),pProp->find(propName.c_str()));
                }

                response.read(prop);
            }
            else
                response.read(*pProp);

            printMessage("%s\n",response.toString().c_str());
            return true;
        }

        printMessage("not present!\n");
        return false;
    }

    /************************************************************************/
    bool set(Bottle *content, const string &agent)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("setting item %d ... ",id);

        LockGuard lg(mutex);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            string owner=it->second.owner;
            if ((owner==OPT_OWNERSHIP_ALL) || (owner==agent))
            {
                Property *pProp=it->second.prop;
                printMessage("%s\n",content->toString().c_str());

                for (int i=0; i<content->size(); i++)
                {
                    if (Bottle *option=content->get(i).asList())
                    {
                        if (option->size()<2)
                        {
                            printMessage("invalid property!\n");
                            continue;
                        }

                        string prop=option->get(0).asString().c_str();
                        Value  val=option->get(1);

                        if (prop==PROP_ID)
                            continue;

                        pProp->unput(prop.c_str());
                        pProp->put(prop.c_str(),val);
                    }
                    else
                    {
                        printMessage("invalid property!\n");
                        continue;
                    }
                }

                it->second.lastUpdate=Time::now();
                return true;
            }
            else
            {
                printMessage("locked by [%s]!\n",owner.c_str());
                return false;
            }
        }

        printMessage("not present!\n");
        return false;
    }

    /************************************************************************/
    bool lock(Bottle *content, const string &agent)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("locking item %d ... ",id);

        LockGuard lg(mutex);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            string owner=it->second.owner;
            if ((owner==OPT_OWNERSHIP_ALL) || (owner==agent))
            {
                printMessage("successfully locked by [%s]!\n",agent.c_str());
                it->second.owner=agent;
                return true;
            }
            else
            {
                printMessage("already locked by [%s]!\n",owner.c_str());
                return false;
            }
        }

        printMessage("not present!\n");
        return false;
    }

    /************************************************************************/
    bool unlock(Bottle *content, const string &agent)
    {
        if (content==NULL)
            return false;

        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("unlocking item %d ... ",id);

        LockGuard lg(mutex);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            string owner=it->second.owner;
            if ((owner==OPT_OWNERSHIP_ALL) || (owner==agent))
            {
                printMessage("successfully unlocked!\n");
                it->second.owner=OPT_OWNERSHIP_ALL;
                return true;
            }
            else
            {
                printMessage("already locked by [%s]!\n",owner.c_str());
                return false;
            }
        }

        printMessage("not present!\n");
        return false;
    }

    /************************************************************************/
    bool owner(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;
        
        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("getting owner of the item %d ... ",id);

        LockGuard lg(mutex);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            response.clear();
            string &owner=it->second.owner;
            response.addString(owner.c_str());
            printMessage("[%s]\n",owner.c_str());
            return true;
        }

        printMessage("item not present!\n");
        return false;
    }

    /************************************************************************/
    bool time(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;
        
        if (!content->check(PROP_ID))
        {
            printMessage("%s field not present within the request!\n",PROP_ID);
            return false;
        }

        int id=content->find(PROP_ID).asInt();
        printMessage("getting time elapsed from last update for item %d ... ",id);

        LockGuard lg(mutex);
        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            response.clear();
            if (it->second.lastUpdate<0.0)
            {
                response.addDouble(it->second.lastUpdate);
                printMessage("just loaded\n");
            }
            else
            {
                double dt=Time::now()-it->second.lastUpdate;
                response.addDouble(dt);
                printMessage("%g [s]\n",dt);
            }
            return true;
        }

        printMessage("item not present!\n");
        return false;
    }

    /************************************************************************/
    bool ask(Bottle *content, Bottle &response)
    {
        if (content==NULL)
            return false;

        LockGuard lg(mutex);
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
            printMessage("uncorrect conditions received!\n");
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
                    condition.prop=b->get(0).asString().c_str();
                    condition.compare=&relationalOperators::alwaysTrue;
                }
                else if (b->size()>2)
                {
                    condition.prop=b->get(0).asString().c_str();
                    operation=b->get(1).asString().c_str();
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
                        printMessage("unknown relational operator '%s'!\n",operation.c_str());
                        return false;
                    }
                }
                else
                {
                    printMessage("wrong condition given!\n");
                    return false;
                }

                condList.push_back(condition);

                if ((i+1)<content->size())
                {
                    operation=content->get(i+1).asString().c_str();
                    if ((operation!="||") && (operation!="&&"))
                    {
                        printMessage("unknown boolean operator '%s'!\n",operation.c_str());
                        return false;
                    }
                    else
                        opList.push_back(operation);
                }
            }
            else
            {
                printMessage("wrong condition given!\n");
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

        printMessage("found items matching received conditions: (%s)\n",response.toString().c_str());
        return true;
    }

    /************************************************************************/
    void periodicHandler(const double dt)   // manage the items life-timers
    {
        mutex.lock();
        bool erased=false;
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
        {
            Property *pProp=it->second.prop;
            if (pProp->check(PROP_LIFETIMER))
            {
                double lifeTimer=pProp->find(PROP_LIFETIMER).asDouble()-dt;
                if (lifeTimer<0.0)
                {
                    printMessage("item with id==%d expired\n",it->first);
                    eraseItem(it);
                    erased=true;

                    break;  // to avoid seg-fault
                }
                else
                {
                    pProp->unput(PROP_LIFETIMER);
                    pProp->put(PROP_LIFETIMER,lifeTimer);
                }
            }
        }
        mutex.unlock();

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
        string agent=connection.getRemoteContact().getName().c_str();
        printMessage("[%s]: %s\n",agent.c_str(),command.toString().c_str());

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
                        setRate((int)(1000.0*command.get(2).asDouble()));

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
                printMessage("received unknown command!\n");
                reply.addVocab(REP_UNKNOWN);
            }
        }
    }

    /************************************************************************/
    bool modify(const Bottle &content)
    {
        if (content.size()==0)
            return false;

        string type=content.get(0).asString().c_str();
        if ((type!=BCTAG_EMPTY) && (type!=BCTAG_SYNC) && (type!=BCTAG_ASYNC))
            return false;

        mutex.lock();
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

        mutex.unlock();

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
        Bottle command, reply;
        if (!command.read(connection) || (pDataBase==NULL))
            return false;

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
    DataBaseModifyPort()
    {
        pDataBase=NULL;
        useCallback();
    }

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
        // request high resolution scheduling
        Time::turboBoost();

        dataBase.configure(rf);
        stats=rf.check("stats");

        string name=rf.check("name",Value("objectsPropertiesCollector")).asString().c_str();
        dataBase.setBroadcastPort(bcPort);
        rpcProcessor.setDataBase(dataBase);
        rpcPort.setReader(rpcProcessor);
        modifyPort.setDataBase(dataBase);
        rpcPort.open(("/"+name+"/rpc").c_str());
        bcPort.open(("/"+name+"/broadcast:o").c_str());
        modifyPort.open(("/"+name+"/modify:i").c_str());

        cnt=0;
        nCallsOld=0;
        cumTimeOld=0.0;

        return true;
    }

    /************************************************************************/
    bool close()
    {
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
            fprintf(stdout,"*** Statistics: received %d/%g [requests/s]; %g [ms/request] spent on average\n",
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
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("objectsPropertiesCollector");
    rf.setDefault("db","dataBase.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        fprintf(stdout,"Options\n\n");
        fprintf(stdout,"\t--name        <name>: collector name (default: objectsPropertiesCollector)\n");
        fprintf(stdout,"\t--db      <fileName>: database file name to load at startup/save at shutdown (default: dataBase.ini)\n");
        fprintf(stdout,"\t--context  <context>: context to search for database file (default: objectsPropertiesCollector)\n");
        fprintf(stdout,"\t--empty             : start an empty database\n");
        fprintf(stdout,"\t--nosave            : prevent from saving the content of database at shutdown\n");
        fprintf(stdout,"\t--verbose           : enable some verbosity\n");
        fprintf(stdout,"\t--sync_bc        <T>: broadcast the database content each T seconds\n");
        fprintf(stdout,"\t--async_bc          : broadcast the database content whenever a change occurs\n");
        fprintf(stdout,"\t--stats             : enable statistics printouts\n");
        fprintf(stdout,"\n");

        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    objectsPropertiesCollectorModule collector;
    return collector.runModule(rf);
}



