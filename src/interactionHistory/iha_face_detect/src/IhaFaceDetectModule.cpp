// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include <iCub/iha/IhaFaceDetectModule.h>

#include <iCub/iha/iha_utils.h>
#include <iCub/iha/mem_util.h>

/**
 * @addtogroup icub_iha_IhaFaceDetect
 *
\section lib_sec Libraries
- YARP libraries.
- OpenCV Library
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : config file - must contain a CASCADES group

--output_image [TRUE/false]  : output an image with faces outlined
\endverbatim

\section portsa_sec Ports Accessed
- /icub/cam/left - or any other image port

\section portsc_sec Ports Created
- /iha/fd/facedetect:in - for receiving the input image
- /iha/fd/facedetect:out - for sending a modified image with detected face in a blue box
- /iha/fd/facedetect:coords - for sending the coordinates of the detected face(s)

 
\section conf_file_sec Configuration Files
conf/ihaIhaFaceDetect.ini

Sample INI file:
\verbatim
name iha

[CASCADES]
cascade1 conf/haarcascade_frontalface_alt2.xml
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaIhaFaceDetect --name /iha/fd --file conf/ihaIhaFaceDetect.ini

or

ihaIhaFaceDetect --name /iha/fd --CASCADES \"(cascade1 conf/haarcascade_frontalface_alt2.xml)\"

See also the script $ICUB_ROOT/app/iha_manual/facedetector.sh

\see iCub::contrib::IhaFaceDetectModule
\see \ref icub_iha_Dynamics

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/iha_face_detect/src/IhaFaceDetectModule.cpp.
*/

IhaFaceDetectModule::IhaFaceDetectModule(){
}

IhaFaceDetectModule::~IhaFaceDetectModule(){ 
}


bool IhaFaceDetectModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --output_image [TRUE/false]  : output an image with faces outlined"<< "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  conf file : [CASCADES] - list of cascade names" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

    bool ok = true;

    // Read parameters
	output_image = boolStringTest(config.check("output_image",Value("TRUE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"output_image %s\n",output_image?"TRUE":"FALSE");

	// read Cascade names from config file
	if (!config.check("CASCADES")) {
		ACE_OS::fprintf(stderr,"require group CASCADES\n");
		return false;
	}

	Bottle cascBot = config.findGroup("CASCADES");

	numcascades = cascBot.size()-1;
	if (numcascades==0) {
		fprintf(stderr,"No cascades specified in config\n");
		return false;
	}
    IhaDebug::pmesg(DBGL_INFO,"%d cascades configured\n",numcascades);

    // allocate memory for cascades 
    cascades = allocAndCheck<CvHaarClassifierCascade*> (numcascades);
    

	for ( int c=0 ; c<numcascades ; c++ ) {
		// get the cascade name

		Bottle* bot = cascBot.get(c+1).asList();
		ConstString cascadeName = bot->get(1).asString();

		IhaDebug::pmesg(DBGL_INFO,"Cascade %d = %s\n",c,cascadeName.c_str());

		// load the cascade

		CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*) cvLoad( cascadeName.c_str(), 0, 0, 0 );
		if (!cascade) {
			ACE_OS::fprintf(stderr,"Could not load classifier %d %s\n",c,cascadeName.c_str());
			return false;
		}
		cascades[c]=cascade;
	}

	// create names of ports
    ConstString imageInPortName = getName("facedetect:in");
    ConstString imageOutPortName = getName("facedetect:out");
    ConstString coordsOutPortName = getName("facedetect:coords");
	
	// open the ports
	imageInPort.open(imageInPortName.c_str());
	if (output_image) imageOutPort.open(imageOutPortName.c_str());
	coordsOutPort.open(coordsOutPortName.c_str());

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool IhaFaceDetectModule::close(){
    // release allocated memory
    checkAndDestroy<CvHaarClassifierCascade*> (cascades);

	imageInPort.close();
	if (output_image) imageOutPort.close();
	coordsOutPort.close();
    return true;
}

bool IhaFaceDetectModule::interruptModule(){
	imageInPort.interrupt();
	if (output_image) imageOutPort.interrupt();
    coordsOutPort.interrupt();
    return true;
}


bool IhaFaceDetectModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	


/**
 * Detect face in image using OpenCV
 * and draw their locations onto the image
 */
bool IhaFaceDetectModule::detectFace( IplImage* img, CvHaarClassifierCascade* cascade, int& num_faces, Vector& coords )
{

	if (!cascade) {
		ACE_OS::fprintf(stderr,"No cascade\n");
		return false;
	}

    // allocate storage
    CvMemStorage* storage = cvCreateMemStorage(0); // 0 means use default block size
    
	// get faces from detector
	CvSeq* faces = cvHaarDetectObjects( img, 
                                        cascade, 
                                        storage, 
                                        1.1, 
                                        2, 
                                        CV_HAAR_DO_CANNY_PRUNING, 
                                        cvSize(40, 40) 
                                      );
	IhaDebug::pmesg(DBGL_DEBUG2,"%d faces detected\n",faces->total);

	num_faces=faces->total;

    
	for( int i = 0; i < (faces ? faces->total : 0); i++ )
	{
		CvRect* r = (CvRect*)cvGetSeqElem( faces, i );

        coords.push_back((double) r->x );           // Point 1 X
        coords.push_back((double) r->y );           // Point 1 Y
        coords.push_back((double) r->x+r->width );  // Point 2 X
        coords.push_back((double) r->y+r->height ); // Point 2 Y

	}

    // release storage
    cvReleaseMemStorage( &storage );

	if (num_faces > 0) return true;
	else return false;

}

bool IhaFaceDetectModule::detectFaces( IplImage* img, CvHaarClassifierCascade** cascades, int& num_faces, Vector& coords ) {
	if (!(cascades[0])) {
		ACE_OS::fprintf(stderr,"No cascades\n");
		return false;
	}
    //coords.clear();
    //num_faces=0;
    for (int c=0;c<numcascades;c++) {
        Vector facecoords;
        int nf;
        if (detectFace(img, cascades[c], nf, facecoords)) {
            IhaDebug::pmesg(DBGL_STATUS1,"%d faces detected by cascade %d\n",nf, c);
            for (int i=0;i<facecoords.size();i++) {
                coords.push_back(facecoords[i]);
            }
            num_faces += nf;;
        }
    }
    return true;
}

bool IhaFaceDetectModule::chooseFace(Vector &coords, int &faceindex) {
	if (coords.size() > 0) {
        int numfaces = (int) coords[0];
        IhaDebug::pmesg(DBGL_DEBUG1, "Choosing from %d faces\n",numfaces);

        if (coords.size() != numfaces * 4 + 1) {
            ACE_OS::fprintf(stderr,"Error: not correct number of coords (%d) for faces (%d)\n", coords.size(), numfaces);
            return false;
        }

		// choose largest face
		double maxsize=0;

		for( int f = 0; f < numfaces; f++)
		{
            double w = coords[f*4 + 2 +1] - coords[f*4 + 0 +1];
            double h = coords[f*4 + 3 +1] - coords[f*4 + 1 +1];

			double size = w*w + h*h;

			IhaDebug::pmesg(DBGL_DEBUG1,"face %d size %f\n",f,size);

			if (size > maxsize) {
				maxsize=size;
				faceindex=f;
			}
		}
		IhaDebug::pmesg(DBGL_STATUS1,"%d faces found - choosing face %d\n",numfaces, faceindex);

        return true;

	} else {
        return false;
    }

}

/** 
 * draw faces given a vector of coordinates
 * first item in coords is number of faces.
 * following ar groups of 4 numbers indicating x1,y1,x2,y2 of each face
 */
bool IhaFaceDetectModule::drawFaceBoxes(IplImage* img, Vector coords , int highlight) {
    
	CvPoint pt1, pt2;

    if (coords.size()>0) {
        int numfaces = (int) coords[0];
        IhaDebug::pmesg(DBGL_DEBUG1, "Drawing %d faces\n",numfaces);

        if (coords.size() != numfaces * 4 + 1) {
            ACE_OS::fprintf(stderr,"Error: not correct number of coords (%d) for faces (%d)\n", coords.size(), numfaces);
            return false;
        }

        for( int f = 0; f<numfaces; f++)
        {
            pt1.x = (int)coords[f*4 + 0 +1];
            pt1.y = (int)coords[f*4 + 1 +1];
            pt2.x = (int)coords[f*4 + 2 +1];
            pt2.y = (int)coords[f*4 + 3 +1];

            if (f==highlight) {
                cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 2, 4, 0);
            } else {
                cvRectangle( img, pt1, pt2, CV_RGB(0,0,255),1,4,0);
            }
        }
    }
    return true;
}

bool IhaFaceDetectModule::updateModule() {

    // read from the input port
    ImageOf<PixelRgb> *imgIn;
    imgIn = imageInPort.read(true);
    IplImage * iplImage;
    if (imgIn!=0)
    {
        double st=Time::now();
        if (output_image)  {
            // prepare an output image
            ImageOf<PixelRgb>& imgOut = imageOutPort.prepare();

            // copy the input image directly onto the output image
            imgOut.copy(*imgIn);

            // get an Ipl image - the memory should still be owned by YARP
            iplImage = (IplImage*) imgOut.getIplImage();
        } else {
            // get an Ipl image - the memory should still be owned by YARP
            iplImage = (IplImage*) imgIn->getIplImage();
        }


        // do face detect
        int num_faces=0;
        Vector coords;  // YARP Vector

        // first item is number of faces
        coords.push_back(0.0); 

        detectFaces( iplImage, cascades, num_faces, coords );

        coords[0] = num_faces;

        if (num_faces==0) 
        {
            coords.push_back(0.0);
            coords.push_back(0.0);
            coords.push_back(0.0);
            coords.push_back(0.0);
        } 
        else 
        {
            int chosen_face=0;
            chooseFace(coords, chosen_face);
            if (output_image) drawFaceBoxes(iplImage, coords, chosen_face);
        } 

        // write the coordinates
        coordsOutPort.write(coords);

        // write the image on the output port
        if (output_image) imageOutPort.write();

        IhaDebug::pmesg(DBGL_STATUS2,"Update Time: %dms\n",(int) ((Time::now()-st)*1000) );
    }

    return true;
}
