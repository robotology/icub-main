// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * Copyright (C) 2014 Francesco Giovannini, iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Francesco Giovannini
 * email:   francesco.giovannini@iit.it
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

#ifndef SKIN_DIAGNOSTIC_DEFINITIONS
#define SKIN_DIAGNOSTIC_DEFINITIONS

#include <string>
#include <stdint.h>
#include <sstream>


namespace iCub {
    namespace skin {
        namespace diagnostics {

            /**
             * The detected error.
             * Fields are:
             *      - net: The CAN bus ID
             *      - board: The MTB board ID
             *      - sensor: The skin sensor ID (triangle/fingertip/...)
             *      - error: The full error message
             */
            struct DetectedError {
                short net;
                short board;
                short sensor;
                int error;
            };

            /**
             * Enum to provide intelligible error codes for the skin.
             */
            struct SkinErrorCode {
            public:
                enum ErrorCode  {
                    // Error flags
                    // Stuck taxels - (Value < 0) || (Value > 255)
                    TaxelStuck11 = 0x8000,
                    TaxelStuck10 = 0x4000,
                    TaxelStuck09 = 0x2000,
                    TaxelStuck08 = 0x1000,
                    TaxelStuck07 = 0x0800,
                    TaxelStuck06 = 0x0400,
                    TaxelStuck05 = 0x0200,
                    TaxelStuck04 = 0x0100,
                    TaxelStuck03 = 0x0080,
                    TaxelStuck02 = 0x0040,
                    TaxelStuck01 = 0x0020,
                    TaxelStuck00 = 0x0010,

                    // Error codes
                    StatusOK = 0x0000,
                    ErrorReading12C = 0x0001,
                    ErrorACK4C = 0x0002
                };

                ErrorCode t_;
                SkinErrorCode(ErrorCode t) : t_(t) {}
                operator ErrorCode () const {return t_;}

            private:
               // Prevent automatic conversion for any other built-in types such as bool, int, etc
               template<typename T>
               operator T () const;
            };

 /*           inline std::string printErrorCode(int code)
            {
                std::string s;

                if (code & (1<<0)) {s+="1";} else {s+="0";}
                if (code & (1<<1)) {s+="1";} else {s+="0";}
                s+="R";
                s+="R";
                s+=".";
                for (int i=4; i<16; i++)
                {
                    if (code & (1<<i)) {s+="1";} else {s+="0";}
                    if (i%4==3 && i!=15) s+=".";
                }
                return s;
            }
            */
            
            inline std::string printErrorCode(int code)
            {
                std::string errortype_str;

                uint8_t error_type = code & 0x000f;
                uint16_t outofrange_error = (code & 0xfff0)>>4;
                //printf("sono nella print error code %d", code);
                switch(error_type)
                {
                    case 0: //no_ack see mtb code
                    {
                        errortype_str+="";
                        break;
                    }
                    case 1: //no_ack see mtb code
                    {
                        errortype_str+="no ack";
                        break;
                    }
                    case 2: //no_ack see mtb code
                    {
                        errortype_str+="not connected";
                        break;
                    }
                    default:
                    {
                        errortype_str+="unknown error";
                        break;
                    }
                }
                
                std::stringstream ss;
                int numoftaxel_in_error = 0;
                for(int i=0; i<12; i++)
                {
                    if(outofrange_error & (1<<i))
                    {
                        numoftaxel_in_error++;
                        ss << i <<", ";
                    }
                }
                
                std::stringstream outs;
                if(numoftaxel_in_error>0)
                {
                    outs << numoftaxel_in_error <<" taxels are out of range:"<< ss.str(); 
                }
                else
                {
                    outs << "None taxels are out of range";
                }
                
                if(error_type!=0)
                    outs << ". Triangle error is "<< errortype_str;
                else
                    outs << ". Triangle has not error ";
                
                return outs.str();
            }
        }
    }
}

#endif  // SKIN_DIAGNOSTIC_DEFINITIONS
