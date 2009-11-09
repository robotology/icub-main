#!/bin/bash

. ./cluster-config.sh

for node in $CLUSTERS
do
  cmd="scp -r $1 $USER@$node:$2"
  echo $cmd
  bash -c "$cmd"
done
