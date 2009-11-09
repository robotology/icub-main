#!/bin/bash

# run as ./admin/scripts/get-source-dependencies.sh

. ./admin//scripts/config.sh

echo This script installs random libraries needed by some modules
echo These libraries are not required for the main iCub build.
echo This script may or may not work today, but should at least
echo give an idea of what worked in the past.
echo Needs to be kept in sync with the exports in config.sh

mkdir -p $AUX_LIB_DIR
cd $AUX_LIB_DIR

get_gsl=true
get_ode=true
get_kdl=true

if [ ! "k$1" = "k" ]; then
	get_gsl=false
	get_ode=false
	get_kdl=false
	if [ $1 = "gsl" ]; then
		get_gsl=true
	fi
	if [ $1 = "ode" ]; then
		get_ode=true
	fi
	if [ $1 = "kdl" ]; then
		get_kdl=true
	fi
fi

if $get_gsl; then

  # Let us get GSL, needed for various modules.
  # Only need to use source when not on windows
  if [ ! -e gsl-1.8.tar ]; then
    if [ ! -e gsl-1.8.tar.gz ]; then
      wget http://mirrors.kernel.org/gnu/gsl/gsl-1.8.tar.gz
    fi

    if [ ! -e gsl-1.8.tar ]; then
      gunzip gsl-1.8.tar.gz
    fi
  fi

  if [ ! -e gsl-1.8 ]; then
    tar xvf gsl-1.8.tar
  fi
fi

if $get_ode; then
  # let us get ODE, needed for Vadim's iCubSimulation
  # need full source package, contains material not otherwise packaged.

  if [ ! -e ode-src-0.8.zip ]; then
      wget http://switch.dl.sourceforge.net/sourceforge/opende/ode-src-0.8.zip
  fi

  if [ ! -e ode-0.8 ]; then
      unzip ode-src-0.8.zip
  fi

  (
      cd ode-0.8
      if [ ! -e Makefile ]; then
  	./configure --enable-release --enable-double-precision
      fi
      make
  )
fi

# let us get roboop, needed for Alexis and co's work.
# -- no need anymore, they went and integrated the source in their code 
# directly.


if $get_kdl; then

  # Get KDL, for Torbjorn's arm_yarl

  if [ ! -e kdl-0.4.tar.bz2 ]; then
      wget http://people.mech.kuleuven.be/~rsmits/kdl-0.4.tar.bz2
  fi

  if [ ! -e kdl-0.4 ]; then
      tar xjvf - < kdl-0.4.tar.bz2
  fi

  (
      cd kdl-0.4
      cmake .
      make
  )

fi

