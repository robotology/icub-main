// own includes
#include <simplyAccessRobot.h>
#include <fstream>

//namespaces

using namespace halode::IO;
using namespace halode::app;

// ***************************************************************************

/**
 * Implementation of the handlocalization module
 *
 * This "module" is uses the input off images and somato-sensory information 
 * (encoder values) to localize the robots hand and then acquire visual 
 * information about the hand (basically an exact segmentation).
 *	
 * For the whole understanding I refer to my thesis: 
 *
 *        "Autonoumous hand localization and detection of a humanoid robot" 
 *
 * which you can find in the repository in 
 *
 *                "$ICUB_ROOT/doc/papers/HALODE07-04.pdf"
 *
 * How it works:
 * 
 * The module  acquires images and  motor sensory information online from  the 
 * robot in  "robot mode" - when being offline the module  works with recorded
 * data sets. But the module must be running in the "file mode".
 *
 * 1. Localization.		The localization is performed by correlation of visual 
 *                      information and body sensed information.
 *
 * 2. Segmentation.		After the localization  a region  of interest is known 
 *                      (or not  in  dissatisfactory circumstances)  and  over
 *						this region the background  is  recovered by  applying 
 *                      the "Background Initialization In Cluttered Sequences"
 *						algorithm by Fusiello and Colombari. 
 *                      For further information either see my thesis & consult 
 *
 *                          "http://profs.sci.univr.it/~fusiello/demo/bkg/".
 *
 * (May 07: In a next release there  might be a much more efficient way to re-
 * cover the background by reorganising the algorithm.)					
 *
 */


int main(int argc, char *argv[]) {

	string config_file_ptr = "config.txt";

	Parameters p (config_file_ptr.c_str());
	p.loadParameters();
	
	double		t1				= Time::now();

	// Create and establish the Network
	Connection	conn (&p);
	ImgInput	*imageInput1_ptr = NULL;
	ImgInput	*imageInput2_ptr = NULL;

	// Display
	ImgOutputRobot	img_result_out_ptr (p.cam1_out_port);
	ImgOutputRobot	img_alt_result_out_ptr (p.cam2_out_port);

	ImgOutput		*img_outs_ptr [2] = { &img_result_out_ptr, &img_alt_result_out_ptr };

	if (p.execution == "robot") {
		// robot mode
		imageInput1_ptr	= new ImgInputRobot	(&p, p.cam1_in_port);
		if (p.stereovision) {
			imageInput2_ptr	= new ImgInputRobot	(&p, p.cam2_in_port);
		}
		//dataInput	= new DataInputRobot(&p, p.enc_in_port);

	}
	else if (p.execution == "visiononly") {
		imageInput1_ptr	= new ImgInputRobot	(&p, p.cam1_in_port);
		if (p.stereovision) {
			imageInput2_ptr	= new ImgInputRobot	(&p, p.cam2_in_port);
		}
	}
	else {
		// file mode
		imageInput1_ptr	= new ImgInputFile (&p, p.cam1_in_port, 1);
		if (p.stereovision) {
			imageInput2_ptr	= new ImgInputFile (&p, p.cam2_in_port, 2);
		}
	}
	ImgInput		*img_ins_ptr [2] = { imageInput1_ptr, imageInput2_ptr };

	// connecting the source and destination ports
	conn.connect(p.cam1_src.c_str(), imageInput1_ptr->getPortname());
	if (p.stereovision) {
		conn.connect(p.cam2_src.c_str(), imageInput2_ptr->getPortname());
	}

	conn.connect(img_result_out_ptr.getPortname(), p.viewer1_dst.c_str());
	conn.connect(img_alt_result_out_ptr.getPortname(), p.viewer2_dst.c_str());

	string param = "";

	bool loop = true;
	ImageOf<PixelRgb> tempRGB[5];

	string app = "src/imgs/app.ppm";
	string quit = "src/imgs/quit.ppm";
	string go = "src/imgs/go.ppm";
	string loopy = "src/imgs/loop.ppm";

	if (!p.linuxFlag) {
		yarp::sig::file::read(tempRGB[0], (string("../")+app).c_str());
		yarp::sig::file::read(tempRGB[1], (string("../")+quit).c_str());
		yarp::sig::file::read(tempRGB[2], (string("../")+go).c_str());
		yarp::sig::file::read(tempRGB[4], (string("../")+loopy).c_str());
	}
	else {
		yarp::sig::file::read(tempRGB[0], app.c_str());
		yarp::sig::file::read(tempRGB[1], quit.c_str());
		yarp::sig::file::read(tempRGB[2], go.c_str());
		yarp::sig::file::read(tempRGB[4], loopy.c_str());
	}

	ImageOf<PixelRgb> img;
	img.copy(*imageInput1_ptr->readImg());
	if (p.stereovision) {
		imageInput2_ptr->readImg();
	}

	p.setIntParameter(&p.img_width, img.width());
	p.setIntParameter(&p.img_height, img.height());

	img_result_out_ptr.write(&tempRGB[0]);
	img_alt_result_out_ptr.write(&tempRGB[0]);

	while (loop) {		
		//cout << "Your choice can be: \n\t<go>, <loop>, <set>, <reset> or <quit>\n";
		cout << "Your choice can be: \n\t<go>, <loop> or <quit>\n";
		cin >> param;
		if (param == "quit") {
			img_result_out_ptr.write(&tempRGB[1]);
			img_alt_result_out_ptr.write(&tempRGB[1]);
			printf("Quit accepted\n");
			loop = false;
		} // end else if end
		else if (param == "go") {
			img_result_out_ptr.write(&tempRGB[2]);
			img_alt_result_out_ptr.write(&tempRGB[2]);
			LaunchApplication app (img_outs_ptr, img_ins_ptr, &p);
			app.launch();

		} // end else if go
		else if (param == "loop") {
			img_result_out_ptr.write(&tempRGB[4]);
			img_alt_result_out_ptr.write(&tempRGB[4]);
			cout << "Please give a number of required loops\n";
			int loops = 10;
			cout << "current value of loops to be performed = " << loops << endl;
			cin >> loops;
			cout << "new value = " << loops << endl;
			if (loops>100) {
				loops = 100;
			}

			LaunchApplication app (img_outs_ptr, img_ins_ptr, &p);
			while (p.exec_loop < loops) {
				printf("loop nr: %d\n", p.exec_loop);
				app.launch();
				p.exec_loop++;
			}
		} // end else if loop
		else {
			printf("enter sth again, please\n");
		}
	} // end while loop


	//printf("Info:\t[Application]\t{time needed to end all: %f}\n", Time::now()-t1);

	// disconnecting the source and destination ports
	conn.disconnect(p.cam1_src.c_str(), imageInput1_ptr->getPortname());
	if (p.stereovision) {
		conn.disconnect(p.cam2_src.c_str(), imageInput2_ptr->getPortname());
	}

	conn.disconnect(img_result_out_ptr.getPortname(), p.viewer1_dst.c_str());
	conn.disconnect(img_alt_result_out_ptr.getPortname(), p.viewer2_dst.c_str());

	delete imageInput1_ptr;
	delete imageInput2_ptr;

	return 0;
} // end main

