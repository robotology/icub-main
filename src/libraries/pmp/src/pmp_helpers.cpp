/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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

#include <stdio.h>
#include <iCub/pmp/private/pmp_helpers.h>

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
            for (int i=0; i<res.length(); i++)
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
    int offs=0;
    if (src.length()>3)
        offs=3;

    if (src.length()<=dest.length()-offs)
    {
        for (int i=0; i<src.length(); i++)
            dest[offs+i]=src[i];

        return true;
    }
    else
        return false;
}


