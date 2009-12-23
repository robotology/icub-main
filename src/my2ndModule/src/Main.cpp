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

#include "iCub/vislab/My2ndModule.h"

#include <yarp/os/all.h>

int main(int argc, char* argv[]) {
	/* initialize yarp network */
	yarp::os::Network yarp;

	/* create your module */
	vislab::demo::My2ndModule my2ndModule;

	/* prepare and configure the resource finder */

	yarp::os::ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("conf.ini"); //overridden by --from parameter
	rf.setDefaultContext("my2ndModule/conf"); //overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	/* run the module: runModule() calls configure first and, if successful, it then runs */
	my2ndModule.runModule(rf);

	return 0;
}
