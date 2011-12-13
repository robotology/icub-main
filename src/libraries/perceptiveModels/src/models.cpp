/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <iCub/perception/models.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::perception;


/************************************************************************/
Model::Model()
{
    name="";
}


/************************************************************************/
void Model::attachNode(Node &node)
{
    nodes[node.getName()]=&node;
}


/************************************************************************/
Node* Model::getNode(const string &name) const
{
    map<string,Node*>::const_iterator it=nodes.find(name);
    return (it!=nodes.end()?it->second:NULL);
}


