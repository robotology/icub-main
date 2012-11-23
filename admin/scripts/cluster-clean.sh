#!/bin/bash

# Copyright: (C) 2011 RobotCub Consortium
# Author: Vadim Tikhanoff & Marco Randazzo
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

count=0

function killFunc(){

ssh -T icub@$1<<END
    cd `echo "${2}"`/bin
    echo ""
    pwd
    ls -l --ignore=*.sh --ignore=*.py | grep '^-' | awk '{print `echo '$9'`}' | while read line
    do      
       ps -ef | grep -i `echo '$line'` | grep -v grep | awk '{print `echo '$2'`}' | xargs kill -9 &>/dev/null
    done
    yarp clean timeout 0.1 &>/dev/null
END

echo ""
echo "You are now clean on $1"
echo ""

}

function actionFunc(){
ssh -T icub@$1<<END
    cd `echo "${2}"`/bin
    echo ""
    echo "I have found the follwing modules:"
    ls -l --ignore=*.sh --ignore=*.py | grep '^-' | awk '{print `echo '$9'`}' | while read line 
    do
        ps -ef | grep -i `echo '$line'` | grep -v grep 
    done

    exit
    echo ""
END
}

function askForYarp(){
    count=1
    echo "WARNING!!! Should I go ahead run this script for yarp?"
    askForListFunc $1 '${YARP_DIR}'
}

function askForKillFunc(){

    echo ""
    echo "WARNING!!! Should I go ahead and kill these processes?"
    echo ""
    select yn in "Yes" "No"; do
        case $yn in
            Yes ) killFunc $1 $2; break;;
            No ) exit;;
        esac
    done
}

function askForListFunc(){

    echo "Are you sure you want to run this script on node "[$1]"?"
    select yn in "Yes" "No"; do
        case $yn in
            Yes )   actionFunc $1 $2; 
                    askForKillFunc $1 $2; 
                    if [ $count -eq 0 ] 
                    then 
                        askForYarp $1 
                    fi; 
                    break;;
            No ) exit;;
        esac
    done
}

if [[ -n "$1" ]] ; then
    echo ""
    askForListFunc $1 '${ICUB_DIR}'
else
    echo 'node is empty, please select which node you want to clean-up'
fi
    
