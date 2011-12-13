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



#ifndef __CLASSIFIER_INPUT__
#define __CLASSIFIER_INPUT__


#include <string>
#include <map>
#include <cassert>
#include <stdexcept>
#include <sstream>


#include <stdio.h>

#include <yarp/sig/Vector.h>


namespace iCub
{

namespace boostMIL
{



class Inputs
{
private:
    std::map<std::string, const yarp::sig::Vector*>     map;
    std::map<std::string, const yarp::sig::Vector*>     pos;

    int                                                 label;

public:
    //Constructors
    Inputs(const int &_label)
        :label(_label)
    {}

    Inputs(const int &_label, const std::string &_type, const yarp::sig::Vector *_input, const yarp::sig::Vector *_positions=NULL)
        :label(_label)
    {
        setInput(_type,_input,_positions);
    }

    //Copy Constructor
    Inputs(const Inputs &input)
    {
        setLabel(input.getLabel());
        for(std::map<std::string,const yarp::sig::Vector*>::iterator itr = map.begin(); itr != map.end(); itr++)
            setInput(itr->first,itr->second);
    }


    ~Inputs()
    {
        for(std::map<std::string,const yarp::sig::Vector*>::iterator itr = map.begin(); itr != map.end(); itr++)
            delete itr->second;

        for(std::map<std::string,const yarp::sig::Vector*>::iterator itr = pos.begin(); itr != pos.end(); itr++)
            if(itr->second!=NULL)
                delete itr->second;
    }

    void setLabel(int _label){label=_label;}

    void setInput(const std::string &_type, const yarp::sig::Vector *_input, const yarp::sig::Vector *_positions=NULL)
    {
        if(map.count(_type) > 0) {
            std::ostringstream buffer;
            buffer << "Input of type '" << _type
                   << "' has already been added. Only one type of input per sample.";
            throw std::runtime_error(buffer.str());
        }

        //for safety reasons it is necessary to copy the entire Vector. This could cause sensible speed decrease
        yarp::sig::Vector *input = new yarp::sig::Vector(*_input);
        
        map[_type] = input;

        pos[_type] = _positions!=NULL?new yarp::sig::Vector(*_positions):NULL;
    }

    int getLabel() const {return label;}

    const yarp::sig::Vector *getInput(const std::string &type) const
    {
        if(map.count(type) > 0)
            return map.find(type)->second;
        else
            return NULL;
    }

    const yarp::sig::Vector *getPositions(const std::string &type) const
    {
        if(pos.count(type) > 0)
            return pos.find(type)->second;
        else
            return NULL;
    }

    yarp::sig::Vector getPosition(const std::string &type, const int &idx) const
    {
        yarp::sig::Vector position;

        if(pos.count(type)>0 && pos.find(type)->second!=NULL)
        {
            position.resize(2);
            position[0]=(*pos.find(type)->second)[2*idx];
            position[1]=(*pos.find(type)->second)[2*idx+1];
        }

        return position;
    }
};


class Output
{
private:
    int                                 label;
    yarp::sig::Vector                   positions;
    size_t                              cnt;

public:
    Output(const int &size)
        :positions(2*size),cnt(0)
    {}

    void setLabel(int _label)                    {label=_label;}
    void pushPosition(const yarp::sig::Vector &pos)
    {
        if(cnt>positions.size()) {
            std::ostringstream buffer;
            buffer << "Error! Position vector has inappropriate size.";
            fprintf(stdout,"[BoostMIL] Input Error! Position vector has inappropriate size!\n");
            throw std::runtime_error(buffer.str());
        }

        positions[cnt]=pos[0];
        positions[cnt+1]=pos[1];
        cnt+=2;
    }

    int                 getLabel()        const   {return label;}
    yarp::sig::Vector   getPositions()    const   
    {
        yarp::sig::Vector v;
        v.clear();
        
        if(cnt==0)
            return v;

        v.resize(cnt);
        for(size_t i=0; i<cnt; i++)
            v[i]=positions[i];

        return v;
    }
};

}

}

#endif



