#!/bin/bash -e
#set -x
# #####################################################
# SCRIPT NAME: create_icub-main_deb.sh
#
# DESCRIPTION: this script creates the icub-main
# package
#
# AUTHOR : Matteo Brunettini <matteo.brunettini@iit.it>
#
# LATEST MODIFICATION DATE (YYYY-MM-DD): 2020-06-15
#
SCRIPT_VERSION="1.3"          # Sets version variable
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

# locals
_WHO_AM_I=$(whoami)
_EQUIVS_BIN=$(which equivs-build || true)
_LSB_BIN=$(which lsb_release || true)
__PLATFORM_RELEASE=""
_PLATFORM_HARDWARE=""
_CONTROL_FILE=""
ICUB_SCRIPT_DIR=$(pwd)


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
  echo "  ICUB_DEBIAN_REVISION_NUMBER is $ICUB_DEBIAN_REVISION_NUMBER"
  echo "  VARS_FILE is $VARS_FILE"
  echo "  _WHO_AM_I is $_WHO_AM_I"
  echo "  _EQUIVS_BIN is $_EQUIVS_BIN"
  echo "  _LSB_BIN is $_LSB_BIN"
  echo "  __PLATFORM_RELEASE is $__PLATFORM_RELEASE"
  echo "  _PLATFORM_HARDWARE is $_PLATFORM_HARDWARE"
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
      export ICUB_PACKAGE_VERSION="$OPTARG"
      ;;
    "R")
      export ICUB_DEBIAN_REVISION_NUMBER="$OPTARG"
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

  _PLATFORM_RELEASE=$(lsb_release -sc)
  if [ "$_PLATFORM_RELEASE" == "" ]; then
    exit_err "unable to read release key"
  fi

  _MACHINE_TYPE=`uname -m`
  case "$_MACHINE_TYPE" in
    x86_64)
      _PLATFORM_HARDWARE="amd64"
      ;;
    i?86)
      _PLATFORM_HARDWARE="i386"
      ;;
    *)
      exit_err "invalid platform hardware $_MACHINE_TYPE"
      ;;
  esac

  _CONTROL_FILE="icub.${_PLATFORM_RELEASE}-${PLATFORM_HARDWARE}.control"

  if [ "$ICUB_PACKAGE_VERSION" == "" ]; then
    exit_err "Package version string is empty"
  fi

  if [ "$ICUB_DEBIAN_REVISION_NUMBER" == "" ]; then
    exit_err "Package revision string is empty"
  fi

  if [[ ! "$SUPPORTED_DISTRO_LIST" =~ "$_PLATFORM_RELEASE" ]]; then
    exit_err "Distro $_PLATFORM_RELEASE is not supported, we support only $SUPPORTED_DISTRO_LIST"
  fi

  if [[ ! "$SUPPORTED_TARGET_LIST" =~ "$_PLATFORM_HARDWARE" ]]; then
    exit_err "Distro $_PLATFORM_HARDWARE is not supported, we support only $SUPPORTED_TARGET_LIST"
  fi

  if [ "$_WHO_AM_I" != "root" ]; then
    sudo apt-get update
  else
    apt-get update
  fi

  log "$0 ${COL_OK}STARTED"
}

fini()
{
  echo $ICUB_MAIN_PACKAGE_NAME > ${ICUB_SCRIPT_DIR}/ICUB_MAIN_PACKAGE_NAME.txt
  mv $ICUB_MAIN_PACKAGE_NAME ${ICUB_SCRIPT_DIR}
  ls
  log "${COL_OK}${ICUB_MAIN_PACKAGE_NAME} CREATED"
}

check_and_install_deps()
{
  export DEBIAN_FRONTEND=noninteractive

  if [ "$_WHO_AM_I" != "root" ]; then
    export _SUDO=sudo
  fi
  if [ "$_LSB_BIN" == "" ]; then
    if [ "$_WHO_AM_I" != "root" ]; then
      log "installing lsb_release, so we may need your password"
      $_SUDO apt-get install -y lsb-release
    else
      log "installing lsb_release"
      apt-get install -y lsb-release
    fi
    _LSB_BIN=$(which lsb_release)
  fi
  if [ "$_EQUIVS_BIN" == "" ]; then
    if [ "$_WHO_AM_I" != "root" ]; then
      log "installing equivs, so we may need your password"
      $_SUDO apt-get install -y equivs
    else
      log "installing equivs"
      apt-get install -y equivs
    fi
    _EQUIVS_BIN=$(which equivs-build)
  fi
}


create_deb()
{
  # Generate dpkg DEBIAN folder
  mkdir -p ${D_ICUB_INSTALL_DIR}/DEBIAN
  # Generate 'conffiles' file
  touch ${D_ICUB_INSTALL_DIR}/DEBIAN/conffiles

  # Generate dpkg DEBIAN/control file
  touch ${D_ICUB_INSTALL_DIR}/DEBIAN/control
  mkdir -p ${D_ICUB_INSTALL_DIR}/usr/share/doc/icub
  cp ${D_ICUB_ROOT}/COPYING ${D_ICUB_INSTALL_DIR}/usr/share/doc/icub/copyright
  cp ${D_ICUB_ROOT}/AUTHORS ${D_ICUB_INSTALL_DIR}/usr/share/doc/icub/AUTHORS
  touch ${D_ICUB_INSTALL_DIR}/DEBIAN/md5sums

  _CONTROL_FILE=${D_ICUB_INSTALL_DIR}/DEBIAN/control

  SIZE=$(du -s $D_ICUB_INSTALL_DIR/)
  SIZE=$(echo $SIZE | awk '{ split($0, array, "/" ); print array[1] }')
  echo "Size: $SIZE"
  echo "Package: icub
Version: ${ICUB_PACKAGE_VERSION}-${ICUB_DEBIAN_REVISION_NUMBER}~${_PLATFORM_RELEASE}
Section: contrib/science
Priority: optional
Architecture: $_PLATFORM_HARDWARE
Depends: icub-common (= ${ICUB_PACKAGE_VERSION}~${_PLATFORM_RELEASE}), yarp (>= ${YARP_REQUIRED_VERSION})
Installed-Size:  $SIZE
Homepage: http://www.icub.org
Maintainer: ${ICUB_PACKAGE_MAINTAINER}
Description: Software platform for iCub humanoid robot with simulator.
 The iCub is the humanoid robot developed as part of the European project
 RobotCub and subsequently adopted by more than 20 laboratories worldwide.
 It has 53 motors that move the head, arms & hands, waist, and legs. It can
 see and hear, it has the sense of proprioception and movement.
 .
 This package provides the standard iCub software platform and apps to
 interact with the real iCub robot, or with the included simulator." | tee $_CONTROL_FILE

  # Build package
  export ICUB_MAIN_PACKAGE_NAME="iCub${ICUB_PACKAGE_VERSION}-${ICUB_DEBIAN_REVISION_NUMBER}~${_PLATFORM_RELEASE}.deb"
  echo $ICUB_MAIN_PACKAGE_NAME
  cd ${D_ICUB_INSTALL_DIR} && dpkg -b ${D_ICUB_INSTALL_DIR} $ICUB_MAIN_PACKAGE_NAME
  if [ "$?" != "0" ]; then
    echo "Error: unable to build the package"
    exit 1
  fi

}

install_deps()
{
  ###------------------- Handle cmake ----------------------###
  echo "Installing CMAKE in the environment"

  if [ "$_PLATFORM_RELEASE" == "bionic" ]; then
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | $_SUDO apt-key add -
    $_SUDO apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
    DEBIAN_FRONTEND=noninteractive; $_SUDO apt-get install $APT_OPTIONS cmake
  else
    DEBIAN_FRONTEND=noninteractive; $_SUDO apt-get install $APT_OPTIONS cmake
  fi

  if [ "$?" != "0" ]; then
    echo "Error: unable to install cmake"
    exit 1
  fi

###------------------- Handle YCM ----------------------###
  echo "Installing YCM package"
  YCM_URL_TAG="YCM_PACKAGE_URL_${_PLATFORM_RELEASE}"
  wget ${!YCM_URL_TAG} -O /tmp/ycm.deb
  DEBIAN_FRONTEND=noninteractive; $_SUDO dpkg --ignore-depends=libjs-sphinxdoc -i /tmp/ycm.deb; $_SUDO apt-get install $APT_OPTIONS -f

  if [ "$?" != "0" ]; then
    echo "Error: unable to install ycm"
    exit 1
  fi

###----------- Handle iCub deps from icub-common -------###
  echo "Installing icub-common dependencies in the environment"
  DEP_TAG="ICUB_DEPS_${_PLATFORM_RELEASE}"
  _DEPENDENCIES="$ICUB_DEPS_COMMON ${!DEP_TAG}"
  DEBIAN_FRONTEND=noninteractive; $_SUDO apt-get install $APT_OPTIONS $_DEPENDENCIES

  if [ "$?" != "0" ]; then
    echo "Error: unable to install dependencies"
    exit 1
  fi

###------------------- Handle IpOpt --------------------###
## It seems already handeled by $_SUDO apt install coinor-libipopt-dev

###------------------- Handle YARP --------------------###
  echo "Installing YARP package"
  YARP_URL_TAG="YARP_PACKAGE_URL_${_PLATFORM_RELEASE}"
  wget ${!YARP_URL_TAG} -O /tmp/yarp.deb
  DEBIAN_FRONTEND=noninteractive; $_SUDO dpkg -i /tmp/yarp.deb

  if [ "$?" != "0" ]; then
    echo "Error: unable to install yarp"
    exit 1
  fi
}

build_icub() {

  #echo "Cloning icub sources from ${ICUB_REPO_URL}"
  #git clone $ICUB_REPO_URL
  #if [ "$?" != "0" ]; then
  #  echo "Error: unable to clone icub repositoy from ${ICUB_REPO_URL}"
  #  exit 1
  #fi
  cd ${ICUB_SCRIPT_DIR}
  export D_ICUB_ROOT=$(pwd)
  git checkout v$ICUB_PACKAGE_VERSION
  if [ "$?" != "0" ]; then
    echo "Error: unable to checkout to v$ICUB_PACKAGE_VERSION"
    exit 1
  fi

  mkdir build && cd build

  CMAKE_OPTIONS_TAG="CMAKE_OPTIONS_${PLATFORM_KEY}"
  _SPECIAL_DIST_CMAKE_OPTIONS="${!CMAKE_OPTIONS_TAG}"
  export D_ICUB_INSTALL_DIR=$(pwd)/install
  cmake $ICUB_CMAKE_OPTIONS -DCMAKE_INSTALL_PREFIX=$D_ICUB_INSTALL_DIR/usr ..

  make -j && make install
  if [ "$?" != "0" ]; then
    echo "Error: unable to checkout to build and/or install icub-main"
    exit 1
  fi
}

fix_relocatable_files(){
  ICUB_INI_PATH="$D_ICUB_INSTALL_DIR/usr/share/yarp/config/path.d"
  ICUB_INI_FILE="iCub.ini"

  echo "Fixing iCub.ini in path.d"
  # this fixes missing iCub.ini file
  if [ ! -e "${ICUB_INI_PATH}/${ICUB_INI_FILE}" ]
  then
    mkdir -p $ICUB_INI_PATH
    ls  $ICUB_INI_PATH
    touch ${ICUB_INI_PATH}/${ICUB_INI_FILE}
    echo ###### This file is automatically generated by CMake. >> ${ICUB_INI_PATH}/${ICUB_INI_FILE}
    echo [search iCub] >> ${ICUB_INI_PATH}/${ICUB_INI_FILE}
    echo path "/usr/share/iCub">> ${ICUB_INI_PATH}/${ICUB_INI_FILE}
  fi
  echo "Fix path inside cmake files"
  _cmake_files=$(find ${D_ICUB_INSTALL_DIR} -name "*.cmake")
  for f in $_cmake_files ; do
    sed -i "s|$D_ICUB_INSTALL_DIR||g" $f
  done

  echo "Fix path inside  ini files"
  _ini_files=$(find ${D_ICUB_INSTALL_DIR} -name "*.ini")
  for f in $_ini_files ; do
    sed -i "s|$D_ICUB_INSTALL_DIR||g" $f
  done

}

main()
{
  install_deps
  build_icub
  # Probably this step is not needed anymore after cmake components/modernization
  fix_relocatable_files
  create_deb
}

parse_opt "$@"
init
main
fini
exit 0
