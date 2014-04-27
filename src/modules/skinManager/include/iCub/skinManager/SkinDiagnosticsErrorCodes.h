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


namespace iCub {
    namespace skin {
        namespace diagnostics {
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
        }
    }
}
