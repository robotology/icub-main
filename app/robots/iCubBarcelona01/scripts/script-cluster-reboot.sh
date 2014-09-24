#!/bin/sh
echo “Rebooting the clients...”
ssh -t icubclient1 'echo icub | sudo -S reboot'
ssh -t icubclient2 'echo icub | sudo -S reboot'
ssh -t icubclient3 'echo icub | sudo -S reboot'
#ssh -t pc104 'echo icub | sudo -S poweroff'
#ssh -t ikart 'echo icubAdmin | sudo -S poweroff'
