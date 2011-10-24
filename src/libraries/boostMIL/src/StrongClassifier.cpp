
/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Carlo Ciliberto
 * email:  carlo.ciliberto@iit.it
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

#include <math.h>
#include <iCub/boostMIL/StrongClassifier.h>
#include <iCub/boostMIL/ClassifierFactory.h>


#include <iostream>
#include <fstream>

using namespace iCub::boostMIL;

void    StrongClassifier::clear     ()
{
    ready = false;

    alphas.clear();

    while(weak_classifiers.size())
    {
        delete weak_classifiers.back();
        weak_classifiers.pop_back();
    }

    while(function_space.size())
    {
        delete function_space.back();
        function_space.pop_back();
    }
}



double  StrongClassifier::margin    (const Inputs *input, Output *output) const
{
    double s = 0.0;
    double a=0.0;
    for(unsigned int i = 0; i < weak_classifiers.size(); i++)
    {
        //double ai=alphas[i]>0.0?alphas[i]:0.0;
        //s+=ai*weak_classifiers[i]->classify(input,output);
        s+=alphas[i]*weak_classifiers[i]->classify(input,output);
        //a+=ai;
        a+=fabs(alphas[i]);
    }

    return (s+a)/(2*a);
}

void    StrongClassifier::printAlphas(FILE *f)
{
    for(unsigned int i = 0; i < alphas.size(); i++)
        fprintf(f,"%f\n",alphas[i]);
}

int     StrongClassifier::classify  (const Inputs *input, Output *output) const
{
    if(!ready)
    {
        if(output!=NULL)
            output->setLabel(0);

        return 0;
    }

    int label;

    double m = margin(input,output);
    if(m == 0.0)
        label=0;

    if(m > 0.0)
        label=1;
    else
        label=-1;

    if(output!=NULL)
        output->setLabel(label);

    return label;
}



std::string  StrongClassifier::toString() const
{
    std::stringstream strstr;

    for(unsigned int i = 0; i < this->weak_classifiers.size(); i++)
        strstr << "((alpha " << alphas[i] << ") (weak_classifier (" << weak_classifiers[i]->toString() << "))) " << std::endl;

    return strstr.str().c_str();
}


void   StrongClassifier::fromString(const std::string &str)
{
    yarp::os::Bottle bLoader(str.c_str());

    for(int i=0; i<bLoader.size(); i++)
    {
        alphas.push_back(bLoader.get(i).asList()->find("alpha").asDouble());

        yarp::os::Bottle *bWL=bLoader.get(i).asList()->find("weak_classifier").asList();
        weak_classifiers.push_back(ClassifierFactory::instance().create(bWL->find("type").asString().c_str()));
        weak_classifiers.back()->fromString(bWL->toString().c_str());
    }
}


bool    StrongClassifier::save        (const std::string &path) const
{
    std::ofstream fout(path.c_str());

    if(!fout.is_open())
        return false;

    fout << this->toString();

    return true;
}


bool    StrongClassifier::load        (const std::string &path)
{
    this->clear();

    std::ifstream fin(path.c_str());

    if(!fin.is_open())
        return false;

    std::stringstream strstr;
    strstr<<fin.rdbuf();

    this->fromString(strstr.str().c_str());

    return true;
}




