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
to the database. 

\section proto_sec Protocol
 
Notation used hereafter to explain available commands: [.] is a 
Vocab, {.} is a string, <.> is a Value (i.e. string, double, 
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
<i>Format</i>: [add] (({prop0} <val0>) ({prop1} <val1>) ...) \n
<i>Reply</i>: [nack]; [ack] (id <num>) \n 
<i>Action</i>: a new item is added to the database with the 
given properties. \n 
A unique identifier is returned that is used to access the item.

<b>del</b> \n
<i>Format</i>: [del] ((id <num>) (propSet ({prop0} {prop1} 
...))) \n
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: remove from the database the specified properties
belonging to the item specified with the given identifier. \n 
The special command "del ((id <num>))" removes the whole item.\n
The special command "[del] (all)" clears the current content of 
the database. 
 
<b>get</b> \n
<i>Format</i>: [get] ((id <num>) (propSet ({prop0} {prop1} 
...))) \n
<i>Reply</i>: [nack]; [ack] (({prop0} <val0>) ({prop1} <val1>) 
...) \n 
<i>Action</i>: return the required properties assigned to the 
stored item. \n 
The special command "[get] ((id <num>))" returns all the 
properties. 
 
<b>set</b> \n
<i>Format</i>: [set] ((id <num>) ({prop2} <val2>) ...) \n 
<i>Reply</i>: [nack]; [ack] \n 
<i>Action</i>: add/modify properties of the stored item.
 
<b>time</b> \n
<i>Format</i>: [time] ((id <num>)) \n 
<i>Reply</i>: [nack]; [ack] (<time>) \n 
<i>Action</i>: retrieve the time elapsed in seconds from the 
last [add]/[del]/[set] operations on the stored item. \n 
Negative values of <time> indicates that the item has not been 
modified since it was loaded within the database. 
 
<b>dump</b> \n 
<i>Format</i>: [dump] \n 
<i>Reply</i>: [ack] \n 
<i>Action</i>: ask the database handler to dump on the screen 
all the stored items along with their properties. 
 
<b>ask</b> \n
<i>Format</i>: [ask] (({prop0} < <val0>) || ({prop1} >= <val1>) 
...) \n 
<i>Reply</i>: [nack]; [ack] (id (<num0> <num1> ...)) \n 
<i>Action</i>: query the database to find all the items whose 
properties match the conditions given in the command. You can 
compose multiple conditions using the boolean operators such as 
'||' for \e or and '&&' for \e and and each condition has to be 
expressed giving the property name, the value to compare with 
and the corresponding relational operator (e.g. >, <=, ==, 
...).\n 
The special command "[ask] (all)" returns the whole set of ids 
present within the database. \n 
In order to simplify the implementation, nested conditions such 
as (cond1) && ((cond2) || (cond3)) are not handled; by the way, 
this is not a real limitation since nested conditions can be 
properly expanded: indeed, the previous example can be cast back 
to (cond1)&&(cond2) || (cond1)&&(cond3). 
 
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
 
--empty
- If this options is given then an empty database is started.
 
\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
 
- \e /<moduleName>/rpc the remote procedure call port used to 
  send requests to the database and receive replies.

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

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>

#include <stdio.h>
#include <string>
#include <map>
#include <deque>

using namespace yarp::os;
using namespace std;

#define CMD_ADD             VOCAB3('a','d','d')
#define CMD_DEL             VOCAB3('d','e','l')
#define CMD_GET             VOCAB3('g','e','t')
#define CMD_SET             VOCAB3('s','e','t')
#define CMD_TIME            VOCAB4('t','i','m','e')
#define CMD_DUMP            VOCAB4('d','u','m','p')
#define CMD_ASK             VOCAB3('a','s','k')
                            
#define REP_ACK             VOCAB3('a','c','k')
#define REP_NACK            VOCAB4('n','a','c','k')
#define REP_UNKNOWN         VOCAB4('u','n','k','n')
                            
#define OPT_ALL             VOCAB3('a','l','l')
                            
#define PROP_ID             ("id")
#define PROP_LIFETIMER      ("lifeTimer")
#define PROP_SET            ("propSet")


namespace relationalOperators
{

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
class DataBase
{
protected:
    /************************************************************************/
    struct Item
    {
        Property *prop;
        double   lastUpdate;
    };

    /************************************************************************/
    struct Condition
    {
        string prop;
        bool (*compare)(Value&,Value&);
        Value val;
    };

    map<int,Item> itemsMap;
    Semaphore mutex;
    int idCnt;
    bool initialized;

    string dbFileName;

    /************************************************************************/
    void clearMap()
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
        else    // at this point we know we deal with the "&&" operations
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

public:
    /************************************************************************/
    DataBase()
    {
        initialized=false;
        idCnt=0;
    }

    /************************************************************************/
    ~DataBase()
    {
        save();
        clearMap();
    }

    /************************************************************************/
    void init(ResourceFinder &rf)
    {
        if (initialized)
        {
            fprintf(stdout,"database already initialized ...\n");
            return;
        }

        dbFileName=rf.getContextPath().c_str();
        dbFileName+="/";
        dbFileName+=rf.find("db").asString().c_str();

        if (!rf.check("empty"))
            load();

        dump();
        initialized=true;
        fprintf(stdout,"database ready ...\n");
    }

    /************************************************************************/
    void load()
    {
        mutex.wait();
        fprintf(stdout,"loading database from %s ...\n",dbFileName.c_str());

        clearMap();
        idCnt=0;

        Property finProperty;
        finProperty.fromConfigFile(dbFileName.c_str());
        Bottle finBottle(finProperty.toString().c_str());

        char tag[255];
        for (int i=0; i<finBottle.size(); i++)
        {
            sprintf(tag,"item_%d",i);
            Bottle &b1=finBottle.findGroup(tag);

            if (b1.isNull())
                continue;

            if (b1.size()<3)
            {
                fprintf(stdout,"error while loading %s!\n",tag);
                continue;
            }

            Bottle *b2=b1.get(1).asList();
            if (b2==NULL)
            {
                fprintf(stdout,"error while loading %s!\n",tag);
                continue;
            }

            if (b2->size()<2)
            {
                fprintf(stdout,"error while loading %s!\n",tag);
                continue;
            }

            int id=b2->get(1).asInt();
            itemsMap[id].prop=new Property(b1.get(2).asList()->toString().c_str());
            itemsMap[id].lastUpdate=-1.0;

            if (idCnt<id)
                idCnt=id+1;
        }

        fprintf(stdout,"database loaded\n");
        mutex.post();
    }

    /************************************************************************/
    void save()
    {
        mutex.wait();
        fprintf(stdout,"saving database in %s ...\n",dbFileName.c_str());

        FILE *fout=fopen(dbFileName.c_str(),"w");
        write(fout);
        fclose(fout);

        fprintf(stdout,"database stored\n");
        mutex.post();
    }

    /************************************************************************/
    void dump()
    {
        mutex.wait();
        fprintf(stdout,"dumping database content ... \n");        

        if (itemsMap.size()==0)
            fprintf(stdout,"empty\n");
        else
            write(stdout);
        mutex.post();
    }

    /************************************************************************/
    bool add(Bottle *content)
    {
        if (content==NULL)
            return false;

        mutex.wait();
        Property *item=new Property(content->toString().c_str());
        itemsMap[idCnt].prop=item;
        itemsMap[idCnt].lastUpdate=Time::now();

        fprintf(stdout,"added item %s\n",item->toString().c_str());
        mutex.post();
        return true;
    }

    /************************************************************************/
    bool remove(Bottle *content)
    {
        if (content==NULL)
            return false;

        mutex.wait();
        if (content->size()==1)
        {
            if (content->get(0).isVocab() || content->get(0).isString())
            {
                if (content->get(0).asVocab()==OPT_ALL)
                {
                    clearMap();
                    fprintf(stdout,"database cleared\n");
                    mutex.post();
                    return true;
                }
            }
        }

        Property request(content->toString().c_str());
        if (!request.check(PROP_ID))
        {
            fprintf(stdout,"%s field not present within the request!\n",PROP_ID);
            mutex.post();
            return false;
        }
        
        int id=request.find(PROP_ID).asInt();
        fprintf(stdout,"removing item %d ... ",id);

        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            Bottle *propSet=request.find(PROP_SET).asList();
            if (propSet!=NULL)
            {
                for (int i=0; i<propSet->size(); i++)
                    it->second.prop->unput(propSet->get(i).asString().c_str());

                it->second.lastUpdate=Time::now();
            }
            else
                eraseItem(it);

            fprintf(stdout,"successfully\n");
            mutex.post();
            return true;
        }

        fprintf(stdout,"not present!\n");
        mutex.post();
        return false;
    }

    /************************************************************************/
    bool get(Bottle *content, Bottle &item)
    {
        if (content==NULL)
            return false;

        mutex.wait();
        Property request(content->toString().c_str());
        if (!request.check(PROP_ID))
        {
            fprintf(stdout,"%s field not present within the request!\n",PROP_ID);
            mutex.post();
            return false;
        }

        int id=request.find(PROP_ID).asInt();
        fprintf(stdout,"getting item %d ... ",id);

        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            Property *pProp=it->second.prop;
            item.clear();

            Bottle *propSet=request.find(PROP_SET).asList();
            if (propSet!=NULL)
            {
                Property prop;
                for (int i=0; i<propSet->size(); i++)
                {
                    string propName=propSet->get(i).asString().c_str();
                    if (pProp->check(propName.c_str()))
                        prop.put(propName.c_str(),pProp->find(propName.c_str()));
                }

                item.fromString(prop.toString().c_str());
            }
            else
                item.fromString(pProp->toString().c_str());

            fprintf(stdout,"%s\n",item.toString().c_str());
            mutex.post();
            return true;
        }

        fprintf(stdout,"not present!\n");
        mutex.post();
        return false;
    }

    /************************************************************************/
    bool set(Bottle *content)
    {
        if (content==NULL)
            return false;

        mutex.wait();
        Property request(content->toString().c_str());
        if (!request.check(PROP_ID))
        {
            fprintf(stdout,"%s field not present within the request!\n",PROP_ID);
            mutex.post();
            return false;
        }

        int id=request.find(PROP_ID).asInt();
        fprintf(stdout,"setting item %d ... ",id);

        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            Property *pProp=it->second.prop;
            request.unput(PROP_ID);
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

                pProp->unput(prop.c_str());
                pProp->put(prop.c_str(),val);
            }

            it->second.lastUpdate=Time::now();
            mutex.post();
            return true;
        }

        fprintf(stdout,"not present!\n");
        mutex.post();
        return false;
    }

    /************************************************************************/
    bool time(Bottle *content, Bottle &item)
    {
        if (content==NULL)
            return false;

        mutex.wait();
        Property request(content->toString().c_str());
        if (!request.check(PROP_ID))
        {
            fprintf(stdout,"%s field not present within the request!\n",PROP_ID);
            mutex.post();
            return false;
        }

        int id=request.find(PROP_ID).asInt();
        fprintf(stdout,"getting time elapsed from last update for item %d ... ",id);

        map<int,Item>::iterator it=itemsMap.find(id);
        if (it!=itemsMap.end())
        {
            item.clear();
            if (it->second.lastUpdate<0.0)
            {
                item.addDouble(it->second.lastUpdate);
                fprintf(stdout,"just loaded\n");
            }
            else
            {
                double dt=Time::now()-it->second.lastUpdate;
                item.addDouble(dt);
                fprintf(stdout,"%g [s]\n",dt);
            }
            mutex.post();
            return true;
        }

        fprintf(stdout,"item not present!\n");
        mutex.post();
        return false;
    }

    /************************************************************************/
    bool ask(Bottle *content, Bottle &items)
    {
        if (content==NULL)
            return false;

        mutex.wait();
        if (content->size()==1)
        {
            if (content->get(0).isVocab() || content->get(0).isString())
            {
                if (content->get(0).asVocab()==OPT_ALL)
                {
                    items.clear();
                
                    for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
                        items.addInt(it->first);
                
                    mutex.post();
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
            fprintf(stdout,"uncorrect conditions received!\n");
            mutex.post();
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
                mutex.post();
                return false;
            }

            condition.prop=b->get(0).asString().c_str();
            condition.val=b->get(2);

            operation=b->get(1).asString().c_str();
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
                fprintf(stdout,"unknown relational operator '%s'!\n",operation.c_str());
                mutex.post();
                return false;
            }

            condList.push_back(condition);

            if ((i+1)<content->size())
            {
                operation=content->get(i+1).asString().c_str();
                if ((operation!="||") && (operation!="&&"))
                {
                    fprintf(stdout,"unknown boolean operator '%s'!\n",operation.c_str());
                    mutex.post();
                    return false;
                }
                else
                    opList.push_back(operation);
            }
        }

        items.clear();

        // apply the conditions to each item
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
        {
            // do recursion and keep only the item that
            // satisfies the whole list of conditions
            if (recursiveCheck(it->second.prop,condList,opList))
                items.addInt(it->first);
        }

        fprintf(stdout,"found items matching received conditions: (%s)\n",items.toString().c_str());
        mutex.post();
        return true;
    }

    /************************************************************************/
    void periodicHandler(const double dt)   // manage the items life-timers
    {
        mutex.wait();
        for (map<int,Item>::iterator it=itemsMap.begin(); it!=itemsMap.end(); it++)
        {
            Property *pProp=it->second.prop;
            if (pProp->check(PROP_LIFETIMER))
            {
                double lifeTimer=pProp->find(PROP_LIFETIMER).asDouble()-dt;

                if (lifeTimer<0.0)
                {
                    fprintf(stdout,"item with id==%d expired\n",it->first);
                    eraseItem(it);
                    break;  // to avoid seg-fault
                }
                else
                {
                    pProp->unput(PROP_LIFETIMER);
                    pProp->put(PROP_LIFETIMER,lifeTimer);
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
                    reply.addVocab(REP_ACK);
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

                Bottle item;
                Bottle *content=command.get(1).asList();                
                if (get(content,item))
                {
                    reply.addVocab(REP_ACK);
                    reply.addList()=item;
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
                if (set(content))
                    reply.addVocab(REP_ACK);
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

                Bottle item;
                Bottle *content=command.get(1).asList();                
                if (time(content,item))
                {
                    reply.addVocab(REP_ACK);
                    reply.addList()=item;
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
            case CMD_ASK:
            {
                if (command.size()<2)
                {
                    reply.addVocab(REP_NACK);
                    break;
                }

                Bottle items;
                Bottle *content=command.get(1).asList();
                if (ask(content,items))
                {
                    reply.addVocab(REP_ACK);
                    Bottle &b=reply.addList();
                    b.addString(PROP_ID);
                    b.addList()=items;
                }
                else
                    reply.addVocab(REP_NACK);

                break;
            }

            //-----------------
            default:
            {
                fprintf(stdout,"received unknown command!\n");
                reply.addVocab(REP_UNKNOWN);
            }
        }
    }
};


/************************************************************************/
class RpcProcessor : public PortReader
{
protected:
    DataBase *pDataBase;

    /************************************************************************/
    bool read(ConnectionReader &connection)
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

    int cnt;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        dataBase.init(rf);

        string name=rf.check("name",Value("objectsPropertiesCollector")).asString().c_str();
        rpcProcessor.setDataBase(dataBase);
        rpcPort.setReader(rpcProcessor);
        rpcPort.open(("/"+name+"/rpc").c_str());

        cnt=0;

        return true;
    }

    /************************************************************************/
    bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

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

        return true;
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
    rf.setDefaultContext("objectsPropertiesCollector/conf");
    rf.setDefault("db","dataBase.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        fprintf(stdout,"Options\n\n");
        fprintf(stdout,"\t--name        <name>: collector name (default: objectsPropertiesCollector)\n");
        fprintf(stdout,"\t--db      <fileName>: database file name to load at startup/save at shutdown (default: dataBase.ini)\n");
        fprintf(stdout,"\t--context  <context>: context to search for database file (default: objectsPropertiesCollector/conf)\n");
        fprintf(stdout,"\t--empty             : start an empty database\n");
        fprintf(stdout,"\n");

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    objectsPropertiesCollectorModule collector;

    return collector.runModule(rf);
}



