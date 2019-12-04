/*
 * Copyright (C) 2019 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email:  Valentina.gaggero@iit.it
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



#include "../include/PeriodicEventsVerifier.h"
#include "embot_tools.h"
#include <yarp/os/LogStream.h>


using yarp::os::Log;



// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------

struct Tools::Emb_PeriodicEventVerifier::Impl
{

    embot::tools::PeriodValidator m_perVal;


    Impl()
    {

    }

    bool init(double period, double tolerance, double min, double max, double step, double reportPeriod)
    {
        // now transform each parameter expressed in seconds to microseconds
        uint64_t period_us=period*1000000;
        uint64_t tolerance_us=tolerance*1000000;
        uint64_t min_us=min*1000000;
        uint64_t max_us=max*1000000;
        uint32_t step_us=step*1000000;
        uint64_t reportPeriod_us=reportPeriod*1000000;
        
        
        return m_perVal.init({period_us, period_us+tolerance_us,  reportPeriod_us, 
                        {min_us, max_us, step_us}});

    }

    void tick(double currentTime)
    {
        
        uint64_t tnow = static_cast<std::uint64_t>(currentTime);
        uint64_t delta = 0;
        m_perVal.tick(tnow, delta);
        if(true == m_perVal.report())
        {
            std::vector<double> vect_prob;
            m_perVal.histogram()->probabilitydensityfunction(vect_prob);
            uint32_t min = m_perVal.histogram()->getconfig()->min;
            uint32_t step = m_perVal.histogram()->getconfig()->step;
            yWarning() << "---------- PRINT HISTO RECEIVER ---------------";
            for(int i=0; i<vect_prob.size(); i++)
            {
                if(vect_prob[i]==0)
                    continue;

                yWarning() << "-- histo RX [" << min+step*i << "]="<< vect_prob[i];
            }
            yWarning() << "---------- END PRINT HISTO RECEIVER ---------------";
            m_perVal.reset();
        }

    }

}; //end Tools::Emb_PeriodicEventVerifier::Impl

struct Tools::Emb_RensponseTimingVerifier::Impl
{
    embot::tools::RoundTripValidator m_roundTripVal;


    Impl()
    {
    }

    bool init(double desiredResponseTime, double tolerance, double min, double max, double step, double reportPeriod)
    {

        // now transform each parameter expressed in seconds to microseconds
        uint64_t desiredResponseTime_us=desiredResponseTime*1000000;
        uint64_t tolerance_us=tolerance*1000000;
        uint64_t min_us=min*1000000;
        uint64_t max_us=max*1000000;
        uint32_t step_us=step*1000000;
        uint64_t reportPeriod_us=reportPeriod*1000000;
        
        return m_roundTripVal.init({desiredResponseTime_us, desiredResponseTime_us+tolerance_us, reportPeriod_us,  
                                   {min_us, max_us, step_us}}); 
    }



    void tick(double currentResponseTime, double requestTime)
    {
        m_roundTripVal.tick(currentResponseTime, static_cast<std::uint64_t>(1000000.0*requestTime));

        if(true == m_roundTripVal.report())
        {
            const embot::tools::Histogram * histo = m_roundTripVal.histogram();
            const embot::tools::Histogram::Values* val=histo->getvalues();

            std::vector<double> vect_prob;
            m_roundTripVal.histogram()->probabilitydensityfunction(vect_prob);
            uint32_t min = m_roundTripVal.histogram()->getconfig()->min;
            uint32_t step = m_roundTripVal.histogram()->getconfig()->step;
            yInfo() << "---------- PRINT HISTO GETPID ---------------";
            for(int i=0; i<vect_prob.size(); i++)
            {
                if(vect_prob[i]==0)
                    continue;
                yInfo() << "-- histo PID [" << min+step*i << "]="<< vect_prob[i];
            }
            yInfo() << "---------- END PRINT HISTO GETPID ---------------";
            m_roundTripVal.reset();
        }
    }

};

// --------------------------------------------------------------------------------------------------------------------
// - all the rest
// --------------------------------------------------------------------------------------------------------------------



Tools::Emb_PeriodicEventVerifier::Emb_PeriodicEventVerifier():pImpl(new Impl)
{;}

Tools::Emb_PeriodicEventVerifier::~Emb_PeriodicEventVerifier()
{
    delete pImpl;
}


bool Tools::Emb_PeriodicEventVerifier::init(double period, double tolerance, double min, double max, double step, double reportPeriod)
{
    // now transform each parameter expressed in seconds to microseconds
    uint64_t period_us=period*1000000;
    uint64_t tolerance_us=tolerance*1000000;
    uint64_t min_us=min*1000000;
    uint64_t max_us=max*1000000;
    uint64_t step_us=step*1000000;
    uint64_t reportPeriod_us=reportPeriod*1000000;
    
    
    return pImpl->init(period_us, tolerance_us, min_us, max_us, step_us, reportPeriod_us);
}

void Tools::Emb_PeriodicEventVerifier::tick(double currentTime)
{
    pImpl->tick(currentTime);
}


Tools::Emb_RensponseTimingVerifier::Emb_RensponseTimingVerifier(): pImpl(new Impl)
{;}

Tools::Emb_RensponseTimingVerifier::~Emb_RensponseTimingVerifier()
{
    delete pImpl;
}


bool Tools::Emb_RensponseTimingVerifier::init(double desiredResponseTime, double tolerance, double min, double max, double step, double reportPeriod)
{
    return pImpl->init(desiredResponseTime, tolerance, min, max, step, reportPeriod);
}


void Tools::Emb_RensponseTimingVerifier::tick(double currentResponseTime, double requestTime)
{
    return pImpl->tick(currentResponseTime, requestTime);
}



// - end-of-file (leave a blank line after)----------------------------------------------------------------------------
