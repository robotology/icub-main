// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_01122009__
#define __ICUB_TEST_01122009__

// sostituire stringhe // OK
// nome classe file    // OK
// ritornare i report come return di run()
// risultato dei test su consolle
// numero di fallimenti nei valori di ritorno dei test
// criticità nei fallimenti
// nome: iCubDriver
// 

#include <vector>
#include <string>
#include <time.h>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "TestReport.h"

class iCubTest
{
public:
    iCubTest(yarp::os::Searchable& configuration)
    {
        m_bSuccess=false;
        m_bIsCritical=configuration.check("critical");
    }

    virtual ~iCubTest()
    {
    }
    
    virtual iCubTestReport run()=0;

    bool getSuccess()
    {
        return m_bSuccess;
    }

    bool isCritical()
    {
        return m_bIsCritical;
    }

protected:
    bool m_bSuccess;
    bool m_bIsCritical;
};

#endif
