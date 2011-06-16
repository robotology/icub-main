/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <iCub/perception/nodes.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::perception;


/************************************************************************/
EventCallback::EventCallback()
{
    name="";
}


/************************************************************************/
Node::Node()
{
    name="";
}


/************************************************************************/
void Node::attachSensor(Sensor &sensor)
{
    sensors[sensor.getName()]=&sensor;
}


/************************************************************************/
void Node::attachCallback(EventCallback &callback)
{
    callbacks[callback.getName()]=&callback;
}


/************************************************************************/
void Node::addNeighbor(Node &node)
{
    neighbors[node.getName()]=&node;
}


/************************************************************************/
bool Node::removeNeighbor(const string &name)
{
    map<string,Node*>::iterator it=neighbors.find(name);
    if (it!=neighbors.end())
    {
        neighbors.erase(it);
        return true;
    }
    else
        return false;
}


/************************************************************************/
Node* Node::getNeighbor(const string &name) const
{
    map<string,Node*>::const_iterator it=neighbors.find(name);
    return (it!=neighbors.end()?it->second:NULL);
}



