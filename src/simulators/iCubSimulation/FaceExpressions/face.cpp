#include <cstdlib>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>

using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace std;
using namespace yarp::os;

#define NUM_PORTS 24

int width = 512;
int height = 512;
unsigned char *maskData = new unsigned char [width*height*3];
unsigned char *data = new unsigned char [width*height*3];
int ledsActivation[24];
int eyeLids[8];
float eyeLidPos;

ImageOf<PixelRgb> mask;
ImageOf<PixelRgb> image;

//linear function
float linearMap( float x, float min, float max, float outMin, float outMax ) {
         float m = - ( outMax-outMin )/( max-min );
        float q = outMax - m*min;
        float ret = m*x+q;
        if (ret < outMin) return outMin;
        if (ret > outMax) return outMax;
        return ret;
}

//read mask which contains all information to create any facial expression
void readMask()
{
	read(mask,"Mask.ppm");
	maskData = mask.getRawImage();
}

//prepare data for processing
void prepareData(unsigned char *data, int width, int height) 
{
	image.setQuantum(1);
	image.setExternal(data,width,height);
}

void setSubSystem(const char *command)
{
//variable holding position in the array
int pos = 0;
//cout<<"command received: "<<command<<endl;
//get sub system id and set writing position in the array
char systemID = command[0];
//cout<<"sub system ID: "<<systemID<<endl;
if (systemID == 'L') pos = 0;
else if (systemID == 'R') pos = 8;
else if (systemID == 'M') pos = 16;
//cout<<"array position: "<<pos<<endl;

//truncate string
char hex[2];
hex[0]=command[1];
hex[1]=command[2];
//cout<<strtol(hex, (char **)NULL, 16)<<endl;

//convert hexdec to binary 
int num = strtol(hex, (char **)NULL, 16);

if (systemID == 'S')
{
	for (int i=0;i<8;i++) eyeLids[i] = (num >> i)&1;
	//print the array
	//for (int i = 0; i<8;i++) cout<<eyeLids[i];
	//cout<<endl;
	eyeLidPos = linearMap(num,36,72,0,30);
	//cout<<eyeLidPos<<endl;
}
else 
{
	for (int i=0;i<8;i++) ledsActivation[pos+i] = (num >> i)&1;
	//print the array
	//for (int i = 0; i<24;i++) cout<<ledsActivation[i];
	//cout<<endl;
}

}

//generate texture by checking leds that are switched on and drawing them 
void generateTexture()
{
	for (int i=0;i<512;i++)
		for (int j =0;j<512;j++)
		{
			PixelRgb& maskPixel = mask.pixel(i,j);
			PixelRgb& imagePixel = image.pixel(i,j);
			if (maskPixel.b <NUM_PORTS && ledsActivation[maskPixel.b]==1) 
			{
				imagePixel.r = 255;
				imagePixel.g = 120;
				imagePixel.b = 120;
			}
			else
			{
				imagePixel.r = 255;
				imagePixel.g = 255;
				imagePixel.b = 245;
			}
		}
		write(image,"Texture.ppm");
}

int main() {
	Network yarp;
	Port reader,eyeLidsPos;
	reader.open("/icubSim/face/raw/in");
	BufferedPort<ImageOf<PixelRgb> > writer;
	writer.open("/face/image/out");
	eyeLidsPos.open("/face/eyelids");

	readMask();
	prepareData(data,width,height);

	Bottle bot;
	while (1) {
		reader.read(bot);
		ConstString message = bot.toString();
		printf("Message received: %s\n",message.c_str());
		setSubSystem(message.c_str());
		generateTexture();
		writer.prepare() = image;
		writer.write();
		//send eyelids position
		if ((bot.toString().c_str())[0]=='S')
		{
			bot.clear();
			bot.add(eyeLidPos);
			eyeLidsPos.write(bot);
			printf("Eye lids position sent: %s\n",bot.toString().c_str());
		}
	}
	delete[] data;
	return 0;
}
