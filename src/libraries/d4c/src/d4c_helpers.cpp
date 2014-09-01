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

#include <stdio.h>

#include <iCub/d4c/private/d4c_helpers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/************************************************************************/
bool extractVector(Property &prop, const string &option, Vector &res)
{
    if (prop.check(option.c_str()))
    {
        if (Bottle *v=prop.find(option.c_str()).asList())
        {
            res.resize(v->size());
            for (size_t i=0; i<res.length(); i++)
                res[i]=v->get(i).asDouble();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool copyVectorData(const Vector &src, Vector &dest)
{
    size_t offs=0;
    if (src.length()>3)
        offs=3;

    if (src.length()<=dest.length()-offs)
    {
        for (size_t i=0; i<src.length(); i++)
            dest[offs+i]=src[i];

        return true;
    }
    else
        return false;
}


