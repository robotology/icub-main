/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

 

/**
 * @file PeriodicEventsVerifier.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */


#ifndef _PERIODIC_EVENTS_VERIFIER_H_
#define _PERIODIC_EVENTS_VERIFIER_H_

//!  In the Tools namespace there are classes useful to check some kinds of performance on robot. 
/*!
  Currently, the available classes are Emb_PeriodicEventVerifier and Emb_RensponseTimingVerifier: 
  the first let you to verify the frequency of a periodic event, while the second let you to analyze the trend of response time 
  to a request.
  This class are based on classes belonging to emBEDDED RObot library of C++11, 
  so these classes are very suitable for working with high precision time, like 1 millisecond.
*/


namespace Tools 
{ 
    class Emb_PeriodicEventVerifier;
    class Emb_RensponseTimingVerifier;
}


//!  Tools::Emb_PeriodicEventVerifier 
/*!
  This class let you to verify if a periodic event is triggered with the desired frequency.
  After its initialization, you need to tick it every event occurrence.
  The class prints the histogram of effective event timing every @reportPeriod, by yarp logger mechanism and resets the data collected until now.
*/
class Tools::Emb_PeriodicEventVerifier
{

    public:
        
        /*!
         * Costructor.
        */
        Emb_PeriodicEventVerifier();
        
        /*!
         * Descructor.
        */
        ~Emb_PeriodicEventVerifier();
    
    /*!
     * Initializes the object with custom parameters
     * \param period is the desired period of the event [expressed in seconds].
     * \param tolerance is the acceptable tolerance of the desired period [expressed in seconds].
     * \param min is the minimum period you expected [expressed in seconds].
     * \param max is the maximum period you expected [expressed in seconds].
     * \param step is the step of the histogram. [expressed in seconds].
     * \param reportPeriod the class prints the histogram every @reportPeriod seconds. [expressed in seconds].
     * \return true/false on success/failure.
     *
     * \note the number of histogram columns are: ((@max-@min)/@step ) +2 
     */
        bool init(double period, double tolerance, double min, double max, double step, double reportPeriod);
        
        
    /*!
     * Signals occurrence of the event to the object
     * \param currentTime the current time [expressed in seconds].
     */
 
        void tick(double currentTime);
    private:
        struct Impl;
        Impl *pImpl;
    
};
    

//!  Tools::Emb_RensponseTimingVerifier 
/*!
  This class let you to analyze the needed time of a request to get a response during a given amount of time.
  This class has been developed with the idea in mind of monitoring the the response time of a request along a period
  in order to understand how many situations there are jitter, other than to calculate the medium value.
  This class needs to be initialize and then you need to add the current response time and its relative absolute request time.
  The class prints the histogram of real response timing every @reportPeriod, by yarp logger mechanism and resets the data collected until now.
*/
class Tools::Emb_RensponseTimingVerifier
{

    public:
        /*!
         * Costructor.
        */                
        Emb_RensponseTimingVerifier();
        
        /*!
         * Descructor.
        */
        ~Emb_RensponseTimingVerifier();
        
        /*!
        * Initializes the object with custom parameters
        * \param desiredResponseTime is the desired response period [expressed in seconds].
        * \param tolerance is the acceptable tolerance of the desired response period [expressed in seconds].
        * \param min is the minimum response period you expected [expressed in seconds].
        * \param max is the maximum response period you expected [expressed in seconds].
        * \param step is the step of the histogram. [expressed in seconds].
        * \param reportPeriod the class prints the histogram every @reportPeriod seconds. [expressed in seconds].
        * \return true/false on success/failure.
        *
        * \note the number of histogram columns are: ((@max-@min)/@step ) +2 
        */    
        bool init(double desiredResponseTime, double tolerance, double min, double max, double step, double reportPeriod);
        
        /*!
        * Adds the current response time to the collection of data of the object.
        * \param currentResponseTime is the time needed to the current request to get a response [expressed in seconds].
        * \param requestTime the time when request has been done [expressed in seconds].
        */        
        void tick(double currentResponseTime, double requestTime);
    private:
        struct Impl;
        Impl *pImpl;
    
};



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

