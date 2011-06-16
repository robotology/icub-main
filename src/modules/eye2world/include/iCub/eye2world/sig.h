/*******************************************************************************
 * Copyright (C) 2009 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#ifndef __VISLAB_YARP_SIG_H_
#define __VISLAB_YARP_SIG_H_

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>

namespace vislab {
namespace yarp {

::yarp::sig::Vector operator/(const ::yarp::sig::Vector& left, const double right);
::yarp::sig::Vector operator/(const ::yarp::sig::Vector& left, const ::yarp::sig::Vector& right);
::yarp::sig::Vector operator*(const ::yarp::sig::Vector& left, const ::yarp::sig::Vector& right);

bool operator!=(const ::yarp::sig::Matrix& left, const ::yarp::sig::Matrix& right);

void resize(::yarp::sig::Vector& v, int s, double def = 0.0);
void resize(::yarp::sig::Matrix& m, int r, int c);

}
}

#endif /* __VISLAB_YARP_SIG_H_ */
