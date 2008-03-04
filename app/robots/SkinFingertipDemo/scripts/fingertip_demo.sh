
#!/bin/sh

gnome-terminal -e "yarp server"
gnome-terminal -e "robotInterface"

gnome-terminal -e "iCubSkinGui --from /home/icub/workspace/icub-main/app/skinGui/conf/skinGui/right_hand.ini"

sleep 4

yarp connect /SkinFingertipDemo/skin/fingertip /skinGui/right_hand:i

gnome-terminal -e "cansend can0 -i 0x20e 0x4E 0x00 0x22 0xF0 0xFF 0xFF 0x00 0x0A"
