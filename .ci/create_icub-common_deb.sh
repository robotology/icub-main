#!/bin/bash -e
#set -x
# #####################################################
# SCRIPT NAME: create_icub-common_deb.sh
#
# DESCRIPTION: this script creates the icub-common
# metapackage
#
# AUTHOR : Matteo Brunettini <matteo.brunettini@iit.it>
#
# LATEST MODIFICATION DATE (YYYY-MM-DD): 2020-06-10
#
SCRIPT_VERSION="1.0"          # Sets version variable
SCRIPT_TEMPLATE_VERSION="1.2.1" #
SCRIPT_NAME=$(realpath -s $0)
SCRIPT_PATH=$(dirname $SCRIPT_NAME)
#
# #####################################################
# COLORS
COL_NORMAL="\e[0m"
COL_ERROR=" \e[31m"
COL_OK="\e[32m"
COL_DONE="\e[96m"
COL_ACTION="\033[1;90m"
COL_WARNING="\e[33m"

# Defaults
LOG_FILE=""
VARS_FILE="${SCRIPT_PATH}/packages_vars.sh"
PLATFORM_HARDWARE=""
PLATFORM_RELEASE=""

# locals
_WHO_AM_I=$(whoami)
_EQUIVS_BIN=$(which equivs-build || true)
_LSB_BIN=$(which lsb_release || true)
_CONTROL_FILE=""
# #####################################################

print_defs ()
{
  echo "Default parameters are"
  echo " SCRIPT_TEMPLATE_VERSION is $SCRIPT_TEMPLATE_VERSION"
  echo " SCRIPT_VERSION is $SCRIPT_VERSION"
  if [ "$LOG_FILE" != "" ]
  then
    echo "  log file is $LOG_FILE"
  fi
  echo "Local parameters are"
  echo "  VARS_FILE is $VARS_FILE"
  echo "  _WHO_AM_I is $_WHO_AM_I"
  echo "  _EQUIVS_BIN is $_EQUIVS_BIN"
  echo "  _LSB_BIN is $_LSB_BIN"
  echo "  _CONTROL_FILE is $_CONTROL_FILE "
}

usage ()
{
  echo "SCRIPT DESCRIPTION"

  echo "Usage: $0 [options]"
  echo "options are :"
  echo "  -f VARS_FILE : use file VARS_FILE as variables container file"
  echo "  -V PACKAGE_VERSION : use PACKAGE_VERSION as version for metapackage"
  echo "  -R DEBIAN_REVISION_NUMBER : use DEBIAN_REVISION_NUMBER ad revision for metapackage"
  echo "  -l LOG_FILE : write logs to file LOG_FILE"
  echo "  -d : print defaults"
  echo "  -v : print version"
  echo "  -h : print this help"
}

log() {
 if [ "$LOG_FILE" != "" ]
  then
    echo -e "$(date +%d-%m-%Y) - $(date +%H:%M:%S) : ${1}${COL_NORMAL}" >> $LOG_FILE
  else
    echo -e "$(date +%d-%m-%Y) - $(date +%H:%M:%S) : ${1}${COL_NORMAL}"
  fi
}

warn() {
  log "${COL_WARNING}WARNING - $1"
 }

error() {
  log "${COL_ERROR}ERROR - $1"
}

exit_err () {
  error "$1"
  exit 1
}

print_version() {
    echo "Script version is $SCRIPT_VERSION based of Template version $SCRIPT_TEMPLATE_VERSION"
}

parse_opt() {
  while getopts hdvl:V:R:f: opt
  do
    case "$opt" in
    "f")
      VARS_FILE="$OPTARG"
      ;;
    "V")
      ICUB_PACKAGE_VERSION="$OPTARG"
      ;;
    "R")
      ICUB_DEBIAN_REVISION_NUMBER="$OPTARG"
      ;;
    "l")
      LOG_FILE="$OPTARG"
      ;;
    h)
      usage
      exit 0
      ;;
    d)
      print_defs
      exit 0
      ;;
    v)
      print_version
      exit 0
      ;;
    \?) # unknown flag
      usage
      exit 1
      ;;
    esac
  done
}

init()
{
  if [ ! -f "$VARS_FILE" ]; then
    exit_err "variables container file $VARS_FILE not found"
  fi
  source "$VARS_FILE"

  check_and_install_deps

  PLATFORM_RELEASE=$(lsb_release -sc)
  if [ "$PLATFORM_RELEASE" == "" ]; then
    exit_err "unable to read release key"
  fi

  _MACHINE_TYPE=`uname -m`
  case "$_MACHINE_TYPE" in
    x86_64)
      PLATFORM_HARDWARE="amd64"
      ;;
    i?86)
      PLATFORM_HARDWARE="i386"
      ;;
    *)
      exit_err "invalid platform hardware $_MACHINE_TYPE"
      ;;
  esac

  _CONTROL_FILE="icub-common.${PLATFORM_RELEASE}-${PLATFORM_HARDWARE}.control"

  if [ "$ICUB_PACKAGE_VERSION" == "" ]; then
    exit_err "Package version string is empty"
  fi

  if [ "$ICUB_DEBIAN_REVISION_NUMBER" == "" ]; then
    exit_err "Package revision string is empty"
  fi

  if [[ ! "$SUPPORTED_DISTRO_LIST" =~ "$PLATFORM_RELEASE" ]]; then
    exit_err "Distro $PLATFORM_RELEASE is not supported, we support only $SUPPORTED_DISTRO_LIST"
  fi

  if [[ ! "$SUPPORTED_TARGET_LIST" =~ "$PLATFORM_HARDWARE" ]]; then
    exit_err "Distro $PLATFORM_HARDWARE is not supported, we support only $SUPPORTED_TARGET_LIST"
  fi

  log "$0 ${COL_OK}Creating package.."
}

fini()
{
  if [ -f "$_CONTROL_FILE" ]; then
    rm "$_CONTROL_FILE"
  fi

  export ICUB_COMMON_PACKAGE_NAME="icub-common_${ICUB_PACKAGE_VERSION}-${ICUB_DEBIAN_REVISION_NUMBER}~${PLATFORM_RELEASE}_${PLATFORM_HARDWARE}.deb"
  echo $ICUB_COMMON_PACKAGE_NAME > ICUB_COMMON_PACKAGE_NAME.txt
  ls
  log "${COL_OK}${ICUB_COMMON_PACKAGE_NAME} CREATED"
}

check_and_install_deps()
{
  export DEBIAN_FRONTEND=noninteractive

  if [ "$_WHO_AM_I" != "root" ]; then
    sudo apt-get update
  else
    apt-get update
  fi
  if [ "$_LSB_BIN" == "" ]; then
    if [ "$_WHO_AM_I" != "root" ]; then
      log "installing lsb_release, so we may need your password"
      sudo apt-get install -y lsb-release
    else
      log "installing lsb_release"
      apt-get install -y lsb-release
    fi
    _LSB_BIN=$(which lsb_release)
  fi
  if [ "$_EQUIVS_BIN" == "" ]; then
    if [ "$_WHO_AM_I" != "root" ]; then
      log "installing equivs, so we may need your password"
      sudo apt-get install -y equivs
    else
      log "installing equivs"
      apt-get install -y equivs
    fi
    _EQUIVS_BIN=$(which equivs-build)
  fi
}

create_control_file()
{
  _ICUB_COMMON_DEPENDENCIES=""
  for dep in $ICUB_DEPS_COMMON ; do
    if [ "$_ICUB_COMMON_DEPENDENCIES" == "" ]; then
      _ICUB_COMMON_DEPENDENCIES="$dep"
    else
      _ICUB_COMMON_DEPENDENCIES="${_ICUB_COMMON_DEPENDENCIES}, $dep"
    fi
  done
  _PLAT_DEPS_TAG="ICUB_DEPS_${PLATFORM_RELEASE}"
  for pdep in ${!_PLAT_DEPS_TAG} ; do
    if [ "$_ICUB_COMMON_DEPENDENCIES" == "" ]; then
      _ICUB_COMMON_DEPENDENCIES="$pdep"
    else
      _ICUB_COMMON_DEPENDENCIES="${_ICUB_COMMON_DEPENDENCIES}, $pdep"
    fi
  done
  echo "Package: icub-common
Version: ${ICUB_PACKAGE_VERSION}-${ICUB_DEBIAN_REVISION_NUMBER}~${PLATFORM_RELEASE}
Section: contrib/science
Priority: optional
Architecture: $PLATFORM_HARDWARE
Depends: $_ICUB_COMMON_DEPENDENCIES, cmake (>=${CMAKE_MIN_REQ_VER})
Homepage: http://www.icub.org
Maintainer: ${ICUB_PACKAGE_MAINTAINER}
Description: List of dependencies for iCub software (metapackage)
 This metapackage lists all the dependencies needed to install the icub platform software or to download the source code and compile it directly onto your machine." | tee $_CONTROL_FILE

}

create_deb()
{
  $_EQUIVS_BIN $_CONTROL_FILE
}

main()
{
  create_control_file
  create_deb
}

parse_opt "$@"
init
main
fini
exit 0
