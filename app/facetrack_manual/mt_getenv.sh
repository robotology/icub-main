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

# Usage: mt_getenv \$NAME \$TYPE \$CONF \$ISNIX";
#         1. mt_getenv CAMCALIB_LEFT EXECUTABLE import check";
#         2. mt_getenv CAMCALIB_LEFT EXECUTABLE /path/to/config.sh check";
#         3. mt_getenv CAMCALIB_LEFT EXECUTABLE /path/to/config.sh";
#
# Notes:  1. We ask mt_getenv to import the env from its referrer."
#            Furthermore, mt_getenv will check if the config file and args are consistent";
#         2. We ask mt_getenv to import the env from some file.."
#            The same check as in (1) will be performed";
#         3. We ask mt_getenv to import the env from some file but"
#            not to check for args and file consistency";

function mt_getenv {
	NAME=$1
	TYPE=$2
	CONF=$3
	ISNIX=$4

	ENV_IMPORT="import";
	
	if [ "$ISNIX" == "check" ]; then
		mt_checkargs $NAME $TYPE $CONF
	fi
	
	if [[ -e $CONF  || "$CONF" == "$ENV_IMPORT" ]]; then
		if [ "$CONF" != "$ENV_IMPORT" ]; then
			source $CONF;
		fi
		TMP_VALUE="$"${NAME}"_"${TYPE}
		THE_VALUE=`eval echo $TMP_VALUE`
		echo $THE_VALUE
	else
		echo -e "[getenv.sh] \$CONF file ($CONF) does not exist"
		exit -2
	fi
}

function mt_checkargs {
	NAME=$1
	TYPE=$2
	CONF=$3
	if [ "$CONF" == "" ]; then
		echo -e "[getenv.sh] Missing \$CONF"
		if [ "$NAME" == "" ]; then
			echo -e "[getenv.sh] Missing \$NAME"
			if [ "$TYPE" == "" ]; then
				echo -e "[getenv.sh] Missing \$TYPE"
			fi
		fi
		exit -1;
	fi
}

