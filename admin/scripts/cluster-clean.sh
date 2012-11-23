#!/bin/bash

# Copyright: (C) 2011 RobotCub Consortium
# Author: Vadim Tikhanoff & Marco Randazzo
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

function killFunc(){

ssh -t icub@$1<<"END"
    cd ${ICUB_DIR}/bin
    pwd
    for f in `ls -l --ignore=*.sh --ignore=*.py | grep '^-' | awk '{print $9}'`; do       
            kill `ps -ef | grep -i $f | grep -v grep | awk '{print $2}'` &>/dev/null
    done
    yarp clean timeout 0.1 &>/dev/null
END

echo ""
echo "You are now clean on $1"
echo ""

}

function actionFunc(){
ssh -t icub@$1<<"END"
    cd ${ICUB_DIR}/bin
    pwd
    echo ""
    echo "I have found the follwing modules:"
    for f in `ls -l --ignore=*.sh --ignore=*.py | grep '^-' | awk '{print $9}'`; do 
        ps -ef | grep -i $f | grep -v grep
    done
    echo ""
END

echo ""
echo "WARNING!!! Should I go ahead and kill these processes?"
echo ""
select yn in "Yes" "No"; do
    case $yn in
        Yes ) killFunc $1; break;;
        No ) exit;;
    esac
done
}



if [[ -n "$1" ]] ; then
    echo ""
    echo "Are you sure you want to run this script on node "[$1]"?"
    select yn in "Yes" "No"; do
        case $yn in
            Yes ) actionFunc $1; break;;
            No ) exit;;
        esac
    done
else
    echo 'node is empty, please select which node you want to clean-up'
fi
    
