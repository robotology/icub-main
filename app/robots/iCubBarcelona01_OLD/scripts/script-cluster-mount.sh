#!/bin/sh
echo “Mounting the cluster...”
ssh -t icubsrv 'echo icub | sudo -S mount -a'
ssh -t icubclient1 'echo icub | sudo -S mount -a'
ssh -t icubclient2 'echo icub | sudo -S mount -a'
ssh -t icubclient3 'echo icub | sudo -S mount -a'
ssh -t pc104 'echo icub | sudo -S mount -a'

