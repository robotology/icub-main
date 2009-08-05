

(
cat<<EOF
  set pos 0 0
  set pos 1 0
  set pos 2 0
  set pos 3 0
  set pos 4 0
  set pos 5 0
EOF
) | yarp rpc /icubSim/left_arm/rpc:i

