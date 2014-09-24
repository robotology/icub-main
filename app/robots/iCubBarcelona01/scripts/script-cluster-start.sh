#!/bin/sh
echo “Starting the cluster...”
cd $ICUB_ROOT/app/iCubCluster/scripts
./icub-cluster.py cluster-config.xml
