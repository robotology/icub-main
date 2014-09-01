#!/bin/sh
echo “Shuting down the cluster...”
ssh -t icubclient1 'echo icub | sudo -S poweroff'
ssh -t icubclient2 'echo icub | sudo -S poweroff'
ssh -t icubclient3 'echo icub | sudo -S poweroff'
ssh -t pc104 'echo icub | sudo -S poweroff'
ssh -t ikart 'echo icubAdmin | sudo -S poweroff'
