Based on the libdc1394-2.1.0 from
http://damien.douxchamps.net/ieee1394/libdc1394/

Download libdc1394-2.1.0.tar.gz from:
http://sourceforge.net/projects/libdc1394

something like:
wget http://downloads.sourceforge.net/libdc1394/libdc1394-2.1.0.tar.gz?modtime=1200479950&big_mirror=0

INSTALLATION:

Untar and unzip the package, 
then in the main folder do 
the following steps:

./configure --prefix=/usr
make
sudo make install

This will install the libraries in /usr/libs
and the include files in /usr/include/dc1394
