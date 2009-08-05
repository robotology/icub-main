#!/bin/bash
# Copyright (C) 2007 Michele Tavella <michele@liralab.it>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

source ./config.sh
source ./mt_getenv.sh
MOD=$1

# Jonas: does YARP still export an env. var. that
#        describes the type of OS it is running on?
#        If so, we could add a ".exe" extension to the
#        the binaries.

# Jonas, this part is somehow experimental
# You said something like:
#  "Some modules require custom arguments"
# Now, this is a quick hack.
# If you call: 
#  ./startModuleGeneric.sh EGOSPHERE earg1=hello earg2=world 
# startModuleGeneric will simply send those arguments to the program
EXTRA_ARGS=$@;
EXTRA_ARGS=`echo $EXTRA_ARGS | sed -e s/$MOD//g`;

MOD_NAME=`mt_getenv $MOD NAME import check`
MOD_EXECUTABLE=`mt_getenv $MOD EXECUTABLE import check`
MOD_CONFIGFILE=`mt_getenv $MOD CONFIG_FILE import check`
MOD_CONFIGGROUP=`mt_getenv $MOD CONFIG_GROUP import check`
MOD_CMD="$EXECUTABLE_PATH/$MOD_EXECUTABLE --name $MOD_NAME --file $CONFIGURATION_PATH/$MOD_CONFIGFILE --group $MOD_CONFIGGROUP $EXTRA_ARGS"

echo "Starting generic YARP module:"
echo "  Name.............$MOD_NAME"
echo "  Executable.......$MOD_EXECUTABLE"
echo "  Config file......$MOD_CONFIGFILE"
echo "  Config group.....$MOD_CONFIGGROUP"
echo "  Executing........$MOD_CMD"

$MOD_CMD
