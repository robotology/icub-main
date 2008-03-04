
#!/bin/sh

#try to enlarge net layer max RX buf
 
#set to 1mbps. If the kernel is configured with bit calculation option it is easier
sudo canconfig can0 bittiming  prop-seg 2 phase-seg1 12 phase-seg2 5 sjw 1 brp 3 tq 50
sudo modprobe esd_usb2

#on ESD usb2 drv the BPR reg should be 0x804d0002

#iface up
sudo canconfig can0 start

sudo ifconfig can0 up