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
             * The detected error.
             * Fields are:
             *      - board: The MTB board ID
             *      - sensor: The skin sensor ID (triangle/fingertip/...)
             *      - error: The full error message
             */
            struct DetectedError {
                short board;
                short sensor;
                int error;
            };
        }
    }
}
