#!/bin/bash

# run from your desired binary directory
# that is the iCub source directory for in-source builds
# or any directory you like for out-of-source builds

reldir=`dirname $0`
absdir=`realpath $reldir/..`
export ICUB_ROOT="$absdir"
export ICUB_DIR="$PWD"

. $ICUB_ROOT/scripts/config.sh

if [ "k$1" = "k" ]; then
  # get any packages needed
    if is_msvc8; then
	echo "Cannot use package management"
    else
#	yes | sudo $ICUB_ROOT/scripts/get-dependencies.sh
	echo "skipping package management"
    fi
fi

# don't let gcc use UTF-8 encoding
unset LANG
unset LC_ALL
unset LC_CTYPE
export LANG=C
export LC_ALL=C

# main build
cd $ICUB_DIR
echo "Main build: in $ICUB_DIR"
if [ "k$1" = "k" ]; then
  rm -f $ICUB_ROOT/CMakeCache.txt
  rm -f CMakeCache.txt
  make_config $ICUB_ROOT
fi

makers=`cd $ICUB_ROOT; find src -name "CMakeLists.txt" | grep -v vvv | grep -v yarp2 | grep -v CMakeFiles`

base=$PWD
doc=$PWD/doc/report
rm -rf $doc
mkdir -p $doc
main=$doc/index.html
echo > $main
echo "<html><head><title>iCub module status</title></head><body>" >> $main
date >> $main
echo "<TABLE>" >> $main
col_0="eeffff"
col_1="ffffcc"
col=true
for m in $makers; do
    ok=true
    if [ ! "k$1" = "k" ]; then
	ok=false
	echo "checking \"$m\" against \"$1\""
	result=`echo $m | grep "$1"`
	if [ ! "k$result" = "k" ]; then
	    ok=true
	fi
    fi
    if $ok; then
	d=`dirname $m`
	echo $d
	mkdir -p $d
	cd $d
	mkdir -p $doc/$d
	rm -f CMakeCache.txt
	status="compiles"
	color="00ff00"
	if [ -e omit.txt ]; then
	    echo "ignored, omit.txt file present" > $doc/$d/compile.txt
	    status="ignored"
	    color="000000"
	else
	    ( ( ( 
			echo "----- DATE AND TIME -----------------------"
			date
			echo "----- ENVIRONMENT VARIABLES ---------------"
			echo YARP_ROOT $YARP_ROOT
			echo YARP_DIR $YARP_DIR
			echo ICUB_ROOT $ICUB_ROOT
			echo ICUB_DIR $ICUB_DIR
			echo "----- CMAKE AND MAKE ----------------------"
			#cmake . && cmake . && make 
			#cmake . && cmake . && make 
			#SOURCE=`denormalize_path $PWD`
			SOURCE=`denormalize_path $ICUB_ROOT/$d`
			echo "Source is in $SOURCE"
			echo "Build is in $PWD"
			rm -f CMakeCache.txt
			make_config $SOURCE && make_config $SOURCE && make_build
		    ) || echo "FAILURE" ) 2>&1 ) | tee $doc/$d/compile.txt
	    ( (
		    keys=`find . -iname "*.cpp" -exec grep -A 5 -i "@ingroup.*icub_module" {} \; | grep defgroup | sed "s/.*defgroup[ \t]*//" | sed "s/[ \t].*//" | sed "s/_/__/g" | sed "s/^/group__/"`
		    spec=`echo "$d" | sed "s|.*/||g"`
		    spec_url="http://eris.liralab.it/wiki/$spec"
		    rm -f junk.wget
#		    ( wget -O junk.wget $spec_url ) > /dev/null 2> /dev/null
		echo hello > junk.wget
		    if grep -q "currently no text" junk.wget; then
			echo "<A HREF=http://eris.liralab.it/wiki/$spec>(add wiki)</a> "
		    else 
			echo "<A HREF=http://eris.liralab.it/wiki/$spec>(view wiki)</a> "
		    fi
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
    echo "<h2>Tips</h2>"
    echo "If you want a directory ignored, place a file called omit.txt in it."

    echo "<h2>Compile environment</h2>"
    echo "<pre>"
    echo -n "Operating system: "
    uname -a
    echo -n "Date: "
    date
    echo
    echo "Installed package information (see scripts/get-dependencies.sh):"
    grep "apt-get" $ICUB_ROOT/scripts/get-dependencies.sh
    echo
    echo "Environment variables present (see scripts/compilable.sh):"
    echo "ICUB_DIR=$ICUB_DIR"
    echo "YARP_DIR=$YARP_DIR"
    echo "ICUB_ROOT=$ICUB_ROOT"
    echo "YARP_ROOT=$YARP_ROOT"
    echo "# note - YARP_BUILD is deprecated, use YARP_DIR instead"
    echo "# YARP_DIR = where YARP is compiled, should contain YARPConfig.cmake"
    echo "# ICUB_DIR = where iCub is compiled, should contain ICUBConfig.cmake"
    echo "# YARP_ROOT = source code for YARP"
    echo "# ICUB_ROOT = source code for iCub"
    echo "</pre>"
) >> $main

echo "</body></html>" >> $main

