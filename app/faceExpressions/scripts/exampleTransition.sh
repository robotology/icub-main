#!/bin/bash
happy() {
    echo "set $1 hap" | yarp rpc /icub/face/emotions/in
    sleep 0.001
    echo "set brig green" | yarp rpc /icub/face/emotions/in
}

neutral() {
    echo "set $1 neu" | yarp rpc /icub/face/emotions/in
    sleep 0.001
    echo "set brig $2" | yarp rpc /icub/face/emotions/in
}

sad() {
    echo "set $1 sad" | yarp rpc /icub/face/emotions/in
    sleep 0.001
    echo "set brig $2" | yarp rpc /icub/face/emotions/in

}

surprised() {
    echo "set $1 sur" | yarp rpc /icub/face/emotions/in
    sleep 0.001
    echo "set brig $2" | yarp rpc /icub/face/emotions/in

}

while true; do 
    # neutral fade out
    echo "set mask (blue BM_EB1 4) (blue BM_EB1 4) (blue BM_M0 4)" | yarp rpc /icub/face/emotions/in
    sleep 0.5
    echo "set mask (blue BM_EB1 3) (blue BM_EB1 3) (blue BM_M0 3)" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set mask (blue BM_EB1 2) (blue BM_EB1 2) (blue BM_M0 2)" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set mask (blue BM_EB1 1) (blue BM_EB1 1) (blue BM_M0 1)" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    # eyebrow and mouth getting surprised
    echo "set mask (blue BM_EB0 1) (blue BM_EB0 1) (blue BM_M3 1)" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set mask (blue BM_EB0 2) (blue BM_EB0 2) (blue BM_M3 2)" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set mask (blue BM_EB0 3) (blue BM_EB0 3) (blue BM_M3 3)" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set mask (blue BM_EB0 4) (blue BM_EB0 4) (blue BM_M3 4)" | yarp rpc /icub/face/emotions/in
    sleep 0.5

    echo "Transition finished"

done

