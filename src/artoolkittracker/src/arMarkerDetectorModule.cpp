/*
 * Copyright (C) 2008 Alexandre Bernardino, Manuel LopesVislab, IST/ISR.
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#include <iostream>
#include "arMarkerDetectorModule.h"

#include <highgui.h>

ARMarkerDetectorModule::ARMarkerDetectorModule()
{

}


ARMarkerDetectorModule::~ARMarkerDetectorModule()
{

}

bool ARMarkerDetectorModule::open(Searchable& config)
{
    if (config.check("help","if present, display usage message")) {
        printf("Call with --file configFile.ini\n");

        return false;
    }

    ConstString camera_file = config.check("cameraparameters", Value("camera_para.dat"), "Camera intrinsics definition file.").asString().c_str();

    ConstString objects_file = config.check("objects", Value("object_data.txt"), "Markers definition file.").asString().c_str();

	tracker.loadCameraParameters( camera_file.c_str(), 640, 480);

	//tracker.loadObjectData( objects_file.c_str() );

	int numberofobjects = config.check("numberofpatterns", 0, "number of patterns").asInt();
	tracker.initobjectdata( numberofobjects );

	ConstString directory = config.check("directory", "/", "directory where patterns are located").asString();

	for(int cnt=0;cnt<numberofobjects;cnt++)
	{
		char name[256];
		char filename[256];
		char pattern[16];

		sprintf( pattern,"pattern%d",cnt+1);

		Bottle& K = config.findGroup( pattern, "object");
		cout << K.toString() << endl;

		strcpy( name, (const char*)K.get(1).asString());
		
		strcpy( filename, (const char*)directory);
		strcat( filename, (const char*)K.get(2).asString()  );

		double size = K.get(3).asDouble();
		printf("reading conf bootle s0%s s1%s s2%s s3%g\n",
		       (const char*)K.get(0).asString(),
		       (const char*)K.get(1).asString(),
		       (const char*)K.get(2).asString(),
		       size);

		tracker.addobjectdata( cnt, name, filename, size );
	}

	//tracker.loadObjectData( config );

	//check visualization

	int visualize = config.check("Visualization", 0, "Show the tracker image").asInt();
	
	if(visualize)
		tracker.openVisualization();

    //load in the object data - trained markers and associated bitmap files 



    _imgPort.open(getName("debimage"));
    _configPort.open(getName("debconf"));
    _outPort.open(getName("debout"));

    attach(_configPort, true);

    return true;

}



bool ARMarkerDetectorModule::close()
{
    _imgPort.close();
    _configPort.close();
    _outPort.close();

    return true;
}



bool ARMarkerDetectorModule::interruptModule()
{

    _imgPort.interrupt();
    _configPort.interrupt();
    _outPort.interrupt();

    return true;
}



bool ARMarkerDetectorModule::updateModule()
{
    int i, j;

    ImageOf<PixelRgb> *yrpImgIn;
    yrpImgIn = _imgPort.read();

	if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;

	double auxtime, aux;

	auxtime = yarp::os::Time::now();

	aux = auxtime - timelastround;

	printf( "%g ms %g fps \n", aux*1000, 1/aux );

	timelastround = auxtime;



	IplImage *iplimg = (IplImage*)yrpImgIn->getIplImage();

	tracker.detectMarkers( iplimg );

	cvWaitKey( 1 );

	Bottle &b = _outPort.prepare();
	b.clear();

	for(int cnt=0; cnt<tracker.nobjects(); cnt++)
	{
		if( tracker.isObjectDetected(cnt) )
		{    
				double T[3][4];
				tracker.getMatrix(cnt,T);
				char name[256];
				tracker.getObjectName(cnt, name);
				printf("%s\nT\n",name);
				b.addString( name );
				b.addString( "T");
				for(i = 0; i < 3; i++ )
				{
					b.addDouble( T[i][0]);
					b.addDouble( T[i][1]);
					b.addDouble( T[i][2]);
					b.addDouble( T[i][3]);
					char str[64];
					printf("%g %g %g %g\n",T[i][0],T[i][1],T[i][2],T[i][3]);
				}

				int cx, cy;
				tracker.getCenter( cnt, &cx, &cy);
				b.addString("imgcoord");
				b.addDouble( cx);
				b.addDouble( cy);
				b.addString("size");
				b.addDouble( tracker.getSize(cnt));
				b.addString("imgcoordnorm");
				b.addDouble( 2.0 * cx / (double)iplimg->width  - 1.0 );
				b.addDouble( - (2.0 * cy / (double)iplimg->height - 1.0) );

				printf("img coord\n %d %d\t size %d\n", cx, cy,  tracker.getSize(cnt));
				printf("imgcoordnorm\n %1.3g %1.3g\n\n",
						2.0 * cx / (double)iplimg->width  - 1.0 ,
						- (2.0 * cy / (double)iplimg->height - 1.0));
		}
	}

	_outPort.write();

    return true;
}
