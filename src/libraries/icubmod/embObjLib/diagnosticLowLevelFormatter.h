/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __diagnosticLowLevelFormatter_h__
#define __diagnosticLowLevelFormatter_h__

#include <string>


#include "EoManagement.h"
#include "ethManager.h"
#include "diagnosticInfo.h"



namespace Diagnostic {
    namespace LowLevel {
        class InfoFormatter;
    }  
}

//The info formatter is used to read the diagnostic embedded info on reception of a nv diagnostic realated.
class Diagnostic::LowLevel::InfoFormatter
{
public:
    InfoFormatter(eth::TheEthManager* ethManager, eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd);
    InfoFormatter() = delete;
    InfoFormatter(const Diagnostic::LowLevel::InfoFormatter &InfoFormatter){};
    ~InfoFormatter(){;};
    InfoFormatter(const Diagnostic::LowLevel::InfoFormatter &&InfoFormatter){};

    virtual bool getDiagnosticInfo(Diagnostic::EmbeddedInfo &info);


private:
    eOmn_info_basic_t* m_infobasic;
    uint8_t * m_extra;
    const EOnv* m_nv;
    const eOropdescriptor_t* m_rd;
    eth::TheEthManager* m_ethManager;
 
    void getTimeOfInfo(Diagnostic::TimeOfInfo &timeOfInfo);
    void getSourceOfMessage(Diagnostic::EmbeddedInfo &info);
    void ipv4ToString(Diagnostic::EmbeddedInfo &info);
    void getSeverityOfError(Diagnostic::EmbeddedInfo &info);
};

#endif //__diagnosticLowLevelFormatter_h__