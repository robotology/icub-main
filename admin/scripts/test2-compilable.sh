#!/bin/bash

# run from your desired binary directory
# that is the iCub source directory for in-source builds
# or any directory you like for out-of-source builds

type=$1
module=$2

. ./admin/scripts/compile-config/$type/config.sh
. ./admin/scripts/compile-config/common.sh
. ./admin/scripts/compile-config/helpers.sh

# don't let gcc use UTF-8 encoding
unset LANG
unset LC_ALL
unset LC_CTYPE
export LANG=C
export LC_ALL=C

# main build
cd $ICUB_DIR
echo "Main build: in $ICUB_DIR"
rm -f $ICUB_ROOT/$module/CMakeCache.txt
rm -f CMakeCache.txt
  
make_config $ICUB_ROOT/$module --graphviz=graph.dot 


# This is a sneaky way to get the list of CMake build targets.
# For makefiles, "make help" works too, but we may not be using makefiles.

# This trick may need to be updated from one cmake version to the next.
# We exploit the fact that executables are tagged with shape "house"
# and libraries with shape "diamond" in graph.dot

targets=`cat graph.dot | egrep "shape=.((house)|(diamond))" | sed "s/shape=.*//" | sed "s/.*label=//" | sed 's/"//g'`

echo $targets

base=$ICUB_DIR
doc=$REPORT_DIR
rm -rf $doc
mkdir -p $doc

main=$doc/index.html
echo > $main
echo "<html><head><title>iCub module status</title></head><body>" >> $main
date >> $main
cat lastRevision.txt >> $main
echo "" >> $main
echo "<TABLE>" >> $main
col_0="eeffff"
col_1="ffffcc"
col=true
for m in $targets; do
    ok=true
    if $ok; then
	d=$m
	mkdir -p $doc/$d
	status="compiles"
	color="00ff00"
	echo " "
	if [ -e omit.txt ]; then
	    echo "ignored, omit.txt file present" > $doc/$d/compile.txt
	    status="ignored"
	    color="000000"
	else
	    ( ( ( 
			echo "----- BUILD ------------------------------"
			#make_clean
			make_build $m || echo FAILURE to build
			echo " "
			echo "----- DATE AND TIME ----------------------"
			date
			echo ""
			echo "----- ENVIRONMENT VARIABLES ON SERVER ----"
			echo YARP_ROOT $YARP_ROOT
			echo YARP_DIR $YARP_DIR
			echo ICUB_ROOT $ICUB_ROOT
			echo ICUB_DIR $ICUB_DIR
			echo ""
		    ) || echo "FAILURE to build" ) 2>&1 ) | tee $doc/$d/compile.txt
	    ( (
		    keys=`( find src/$m -iname "*.cpp" -exec grep -A 5 -i "@ingroup.*icub_module" {} \; | grep defgroup | sed "s/.*defgroup[ \t]*//" | sed "s/[ \t].*//" | sed "s/_/__/g" | sed "s/^/group__/" ) 2> /dev/null`
		    spec=`echo "$d" | sed "s|.*/||g"`
#		    spec_url="http://eris.liralab.it/wiki/$spec"
#		    rm -f junk.wget
#		    ( wget -O junk.wget $spec_url ) > /dev/null 2> /dev/null
#		echo hello > junk.wget
#		    if grep -q "currently no text" junk.wget; then
#			echo "<A HREF=http://eris.liralab.it/wiki/$spec>(add wiki)</a> "
#		    else 
#			echo "<A HREF=http://eris.liralab.it/wiki/$spec>(view wiki)</a> "
#		    fi
		    for k in $keys; do
			echo "<A HREF='../html/$k.html'>(doc)</A> "
		    done
	    ) 2>&1 ) | tee $doc/$d/document.html
	fi
	if grep FAILURE $doc/$d/compile.txt; then
	    status="doesn't compile"
	    color="ff0000"
	fi
	docs=`cat $doc/$d/document.html`
	bgcolor=$col_0
	if $col; then
	    bgcolor=$col_1
	    col=false
	else
	    col=true
	fi
	if [ ! -e omit.txt ]; then
	    echo "<tr><td bgcolor='#$bgcolor'><a href='$d/compile.txt'>$d</a></td><td bgcolor='#$bgcolor'><font color='#$color'>$status</font></td><td bgcolor='#$bgcolor'>$docs</td></tr>" >> $main
	fi
	cd $base
    fi
done
echo "</TABLE>" >> $main
(
    echo "<h2>Compile environment</h2>"
    echo "<pre>"
    echo -n "Operating system: "
    uname -a
    echo -n "Date: "
    date
    echo
    if [ -e "./admin/scripts/compile-config/$type/$SYSTEM_DESCRIPTION_FILE" ]
    then
	bash ./admin/scripts/compile-config/$type/$SYSTEM_DESCRIPTION_FILE
    else
	echo "No system description file found on server"
    fi

    echo
    echo "Environment variables present on server::"
    echo "ICUB_DIR=$ICUB_DIR"
    echo "YARP_DIR=$YARP_DIR"
    echo "ICUB_ROOT=$ICUB_ROOT"
    echo "YARP_ROOT=$YARP_ROOT"
    if [ "k$ACE_ROOT" = "k" ]; then
	echo "ACE_ROOT undefined"
    else
	echo "ACE_ROOT=$ACE_ROOT"
    fi
    if [ "k$ODE_DIR" = "k" ]; then
	echo "ODE_DIR undefined"
    else
	echo "ODE_DIR=$ODE_DIR"
    fi
    if [ "k$SDLDIR" = "k" ]; then
	echo "SDLDIR undefined"
    else
	echo "SLDDIR=$SDLDIR"
    fi
    if [ "k$OPENCV_DIR" = "k" ]; then
	echo "OPENCV_DIR undefined"
    else
	echo "OPENCV_DIR=$OPENCV_DIR"
    fi
    if [ "k$GSL_DIR" = "k" ]; then
	echo "GSL_DIR undefined"
    else
	echo "GSL_DIR=$GSL_DIR"
    fi

    if [ "k$GTK_BASEPATH" = "k" ]; then
	echo "GTK_BASEPATH undefined"
    else
	echo "GTK_BASEPATH=$GTK_BASEPATH"
    fi

    if [ "k$GTKMM_BASEPATH" = "k" ]; then
	echo "GTKMM_BASEPATH undefined"
    else
	echo "GTKMM_BASEPATH=$GTKMM_BASEPATH"
    fi
    
    if [ "k$IPOPT_DIR" = "k" ]; then
	echo "IPOPT_DIR undefined"
    else
	echo "IPOPT_DIR=$IPOPT_DIR"
    fi

    echo "# YARP_DIR = where YARP is compiled, should contain YARPConfig.cmake"
    echo "# ICUB_DIR = where iCub is compiled, should contain ICUBConfig.cmake"
    echo "# YARP_ROOT = source code for YARP"
    echo "# ICUB_ROOT = source code for iCub"
    echo "# ACE_ROOT = where ace was compiled (if not using precompiled debian package)"
    echo "# GSL_DIR = where GSL was compiled (if not using precompiled debian package)"
    echo "# OPENCV_DIR = where OPENCV was compiled (if not using precompiled debian package)"
    echo "# ODE_DIR = where ode was compiled (if not using precompiled debian package)"
    echo "# SDLDIR = where sdl was compiled (if not using precompiled debian package)"
    echo "# GTKMM_BASEPATH/GTK_BASEPATH = where gtk/gtkmm was compiled or installed (if not using precompiled debian package)"
    echo "# IPOPT_DIR = where ipopt was installed"

    echo "</pre>"
) >> $main

echo "</body></html>" >> $main

