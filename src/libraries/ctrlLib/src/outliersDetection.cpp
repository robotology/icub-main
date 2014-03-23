/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
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

#include <gsl/gsl_math.h>

#include <iCub/ctrl/outliersDetection.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;


/**********************************************************************/
ModifiedThompsonTau::ModifiedThompsonTau()
{
    tauLUP[3] =1.1511;
    tauLUP[4] =1.4250;
    tauLUP[5] =1.5712;
    tauLUP[6] =1.6563;
    tauLUP[7] =1.7110;
    tauLUP[8] =1.7491;
    tauLUP[9] =1.7770;
    tauLUP[10]=1.7984;
    tauLUP[11]=1.8153;
    tauLUP[12]=1.8290;
    tauLUP[13]=1.8403;
    tauLUP[14]=1.8498;
    tauLUP[15]=1.8579;
    tauLUP[16]=1.8649;
    tauLUP[17]=1.8710;
    tauLUP[18]=1.8764;
    tauLUP[19]=1.8811;
    tauLUP[20]=1.8853;
    tauLUP[21]=1.8891;
    tauLUP[22]=1.8926;
    tauLUP[23]=1.8957;
    tauLUP[24]=1.8985;
    tauLUP[25]=1.9011;
    tauLUP[26]=1.9035;
    tauLUP[27]=1.9057;
    tauLUP[28]=1.9078;
    tauLUP[29]=1.9096;
    tauLUP[30]=1.9114;
    tauLUP[31]=1.9130;
    tauLUP[32]=1.9146;
    tauLUP[33]=1.9160;
    tauLUP[34]=1.9174;
    tauLUP[35]=1.9186;
    tauLUP[36]=1.9198;
    tauLUP[37]=1.9209;
    tauLUP[38]=1.9220;
    tauLUP[39]=1.9230;
    tauLUP[40]=1.9240;
    tauLUP[41]=tauLUP[40];
    tauLUP[42]=1.9257;
    tauLUP[43]=tauLUP[42];
    tauLUP[44]=1.9273;
    tauLUP[45]=tauLUP[44];
    tauLUP[46]=1.9288;
    tauLUP[47]=tauLUP[46];
    tauLUP[48]=1.9301;
    tauLUP[49]=tauLUP[48];
    tauLUP[50]=1.9314;
    tauLUP[51]=tauLUP[50];
    tauLUP[52]=1.9325;
    tauLUP[53]=tauLUP[52];
    tauLUP[54]=1.9335;
    tauLUP[55]=tauLUP[54];
    tauLUP[56]=1.9345;
    tauLUP[57]=tauLUP[56];
    tauLUP[58]=1.9354;
    tauLUP[59]=tauLUP[58];
    tauLUP[60]=1.9362;
    for (int i=1; i<5; i++)
        tauLUP[60+i]=tauLUP[60];
    tauLUP[65]=1.9381;
    for (int i=1; i<5; i++)
        tauLUP[65+i]=tauLUP[65];
    tauLUP[70]=1.9397;
    for (int i=1; i<5; i++)
        tauLUP[70+i]=tauLUP[70];
    tauLUP[75]=1.9411;
    for (int i=1; i<5; i++)
        tauLUP[75+i]=tauLUP[75];
    tauLUP[80]=1.9423;
    for (int i=1; i<10; i++)
        tauLUP[80+i]=tauLUP[80];
    tauLUP[90]=1.9443;
    for (int i=1; i<10; i++)
        tauLUP[90+i]=tauLUP[90];
    tauLUP[100]=1.9459;

    tauLUP[150]=1.9506;
    tauLUP[200]=1.9530;
    tauLUP[500]=1.9572;
    tauLUP[1000]=1.9586;
    tauLUP[5000]=1.9597;
    //n -> inf => tau=1.96
}


/**********************************************************************/
VectorOf<int> ModifiedThompsonTau::detect(const Vector &data, const Property &options)
{
    VectorOf<int> res;
    if (data.length()<3)
        return res;

    double mean=0.0; 
    double stdev=0.0;

    Property &opt=const_cast<Property&>(options);
    if (opt.check("mean") && opt.check("std"))
    {
        mean=opt.find("mean").asDouble();
        stdev=opt.find("std").asDouble();
    }
    else
    {
        // find mean and standard deviation
        for (size_t i=0; i<data.length(); i++)
        {
            mean+=data[i];
            stdev+=data[i]*data[i];
        }

        mean/=data.length();
        stdev=sqrt(stdev/data.length()-mean*mean);
    }

    int i_check;
    double delta_check=0.0;
    if (opt.check("sorted"))
    {
        double delta_0=fabs(data[0]-mean);
        double delta_n1=fabs(data[data.length()-1]-mean);

        // default=last
        delta_check=delta_n1;
        i_check=data.length()-1;

        if (opt.check("check_outlier"))
        {
            if (opt.find("check_outlier").asString()=="first")
            {
                delta_check=delta_0;
                i_check=0;
            }
        }
        else if (delta_0>delta_n1)
        {
            delta_check=delta_0;
            i_check=0;
        }
    }
    else for (size_t i=0; i<data.length(); i++)
    {
        double delta=fabs(data[i]-mean);
        if (delta>delta_check)
        {
            delta_check=delta;
            i_check=i;
        }
    }

    // choose tau
    double tau=0.0;
    if (data.length()>5000)
        tau=1.96;
    else if (data.length()>1000)
        tau=tauLUP[5000];
    else if (data.length()>500)
        tau=tauLUP[1000];
    else if (data.length()>200)
        tau=tauLUP[500];
    else if (data.length()>150)
        tau=tauLUP[200];
    else if (data.length()>100)
        tau=tauLUP[150];
    else
        tauLUP[data.length()];

    // perform detection
    if (delta_check>tau*stdev)
        res.push_back(i_check);

    return res;
}


