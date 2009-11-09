#!/bin/bash

. ./cluster-config.sh

for node in $CLUSTERS
do
  cmd="ssh $USER@$node $1"
  echo $cmd
  bash -c "$cmd"
done
