//includes
#include <IO.h>

#include <iostream>
#include <fstream>

// namespaces
using namespace halode::IO;

// ***************************************************************************

/**
 * Implementation of Connection class
 */

// Default constructor
Connection::Connection() {
	//printf("Start:\t[IO]\n");
	if (this->params_ptr->netCounter != 0) {
		printf("Error:\t[IO.Connection]\t{Constructor() netCounter != 0}\n");
	}
	else {
		printf("Error:\t[IO.Connection]\t{Constructor() else part}\n");
	}
}

// Init constructor
Connection::Connection(Parameters *_params_ptr) {
	//printf("Opening Network\n");
	this->params_ptr = _params_ptr;

	if (this->params_ptr->netCounter == 0) {
		this->params_ptr->netCounter++;
		Network::init();
	}
	this->params_ptr->netCounter++;
}

// Destructor
Connection::~Connection() {
	this->params_ptr->netCounter--;
	if (this->params_ptr->netCounter == 0) {
		Network::fini();
	}
	//printf("Quit:\t[IO.Connection]\n");
}

// method to connect ports
bool Connection::connect(const char *_inputPort, const char *_outputPort) {
	//printf("Connecting:\n\tport %s with \n\tport %s\n", _inputPort, _outputPort);
	bool b;
	b = Network::connect(_inputPort, _outputPort);
	if (b) {
		printf("Info:\t[IO.Connection]\t{%s port connected}\n",_outputPort);
	}
	else {
		printf("Info:\t[IO.Connection]\t{%s port NOT connected}\n",_outputPort);
	}

	return b;
}

bool Connection::disconnect(const char *_inputPort, const char *_outputPort) {
	//printf("Connecting:\n\tport %s with \n\tport %s\n", _inputPort, _outputPort);
	bool b;
	b = Network::disconnect(_inputPort, _outputPort);
	if (b) {
		printf("Info:\t[IO.Connection]\t{%s port disconnected}\n",_outputPort);
	}
	else {
		printf("Info:\t[IO.Connection]\t{%s port NOT disconnected}\n",_outputPort);
	}
	return b;
}

// ***************************************************************************

/**
 * Implementation of Input class
 */

// Init constructor
Input::Input() {
}

Input::Input(Parameters *_params_ptr, const char *_portname) {
	this->params_ptr	= _params_ptr;
	this->inputPortname = _portname;
}

// Destructor
Input::~Input() {
}

// method to receive the portname
const char* Input::getPortname() {
	return this->inputPortname;
}

// ***************************************************************************

/**
 * Implementation of ImgInput class
 */

// Init constructor
ImgInput::ImgInput() {
}

ImgInput::ImgInput(Parameters *_params_ptr, const char *_portname) : Input(_params_ptr, _portname) {
	//printf("Info:\t[IO.ImgInput]\t{image file input port %s to open\n", _portname);
	//this->inputPort.setStrict();
	this->inputPort.open(this->inputPortname);
	//printf("Info:\t[IO.ImgInput]\t{image file input port %s is opened}\n", _portname);
}

// Destructor
ImgInput::~ImgInput() {
	this->inputPort.close();
}

//virtual
ImageOf<PixelRgb>* ImgInput::readImg() {
	ImageOf<PixelRgb> *ptr = NULL;
	return ptr;
}
// ***************************************************************************

/**
 * Implementation of ImgInputFile class
 */

// Init constructor
/*ImgInputFile::ImgInputFile() {
}*/

ImgInputFile::ImgInputFile(Parameters *_params_ptr, const char *_portname, int _channel) : ImgInput(_params_ptr, _portname) {
	//printf("Info:\t[IO.ImgInputFile]\t{image file input port %s is opened}\n", _portname);
	this->imgHandler = ImgFileHandler (_params_ptr, _channel);
}

// Destructor
ImgInputFile::~ImgInputFile() {
	//delete this->img_ptr;
	//printf("Quit:\t[IO.ImgInputFile]\n");
}

void ImgInputFile::next() {
	this->imgHandler.next();
}

// method to receive the img in form of an address to read from
ImageOf<PixelRgb>* ImgInputFile::getImg() {
	return this->img_ptr;
}

// method to get the next image in the pipe. in form of an image address to read from
ImageOf<PixelRgb>* ImgInputFile::readImg() {
	//printf("Reading image: %s \n", this->imgHandler.getFilename());
	read(this->img, this->imgHandler.getFilename());
	this->img_ptr = &this->img;
	this->next();
	return this->img_ptr;
}

// return form the filehandler the currentfilenumber 
int ImgInputFile::getCurrentFilecounter() {
	int i;
	i = this->imgHandler.getFilecounter();
	return i;
}

// ***************************************************************************

/**
 * Implementation of ImgInputRobot class
 */

// Init constructor
ImgInputRobot::ImgInputRobot(Parameters *_params_ptr, const char *_portname) : ImgInput(_params_ptr, _portname) {
	//printf("Info:\t[IO.InputRobot]\t{robot image input port opened}\n");
	if (_params_ptr->rec_perception) {
		this->imgOut_ptr = new ImgOutputFile (this->params_ptr->result_dir_1);
		this->cnt = 0;
		this->filename = string(this->getPortname());
		int loc = this->filename.find( "/", 0 );
		while ( loc != -1 ) {
			this->filename.replace( loc, 1, "_" );;
			loc = this->filename.find("/", 0);
			//printf("%s, loc = %d \n", this->filename.c_str(), loc);
		}
	}
}

// Destructor
ImgInputRobot::~ImgInputRobot(){
	//printf("Info:\t[IO.InputRobot]\t{robot image input port closed}\n");
}

// get the next image from the port
ImageOf<PixelRgb>* ImgInputRobot::readImg() {
	this->img.copy(*this->inputPort.read());
	this->img_ptr = &this->img;
	if (this->params_ptr->rec_perception) {
		char tmp1 [100];
		sprintf(tmp1, "%s%d", this->filename.c_str(),  this->cnt);
		imgOut_ptr->write(this->img_ptr, tmp1);
		this->cnt++;
	}
	return this->img_ptr;
}

// ***************************************************************************

/**
 * Implementation of Output class
 */

// Init constructor
Output::Output(const char *_portName) {
	//printf("Start:\t[Output]\n");
	this->outputPortname = _portName;
}

// Destructor
Output::~Output() {
	//printf("Quit:\t[Output]\n");
}

// method to receive the port name
const char* Output::getPortname() {
	return this->outputPortname;
}

// ***************************************************************************

/**
 *  Implementation of DataOutput class
 */

// Init Constructor 
DataOutput::DataOutput(const char *_portName_ptr, bool _createPort) : Output(_portName_ptr) {
	if (_createPort) {
		this->outputPort.open(this->outputPortname);
	}
	else {
		this->outputFile.open(this->outputPortname);
	}
	this->createdPort = _createPort;
}

// Destructor 	
DataOutput::~DataOutput() {
	if (this->createdPort) {
		this->outputPort.close();
	}
	else {
		this->outputFile.close();
	}
}
// methods to write to the port (virtual)	
void DataOutput::write(Bottle *_data_ptr) {
	printf("Info:\t[DataOutput]\t\t{I should write some data? That is no good!}\n");
}

// ***************************************************************************

/**
 *  Implementation of DataOutputFile class
 */

// Constructor
DataOutputFile::DataOutputFile(const char *_fileName_ptr) : DataOutput(_fileName_ptr, 0) {
}

// Destructor
DataOutputFile::~DataOutputFile() {
}

// method to write over port into file (little cheating, 'cause writing directly to file, but
// pretending as having the normal structure as the other ports)
void DataOutputFile::write(Bottle *_data_ptr) {
	//printf("writing to %s file\n", this->outputPortname);
	int bfSize = _data_ptr->size();
	string temp = "";
	for (int i=0; i<bfSize; i++) {
		//temp.append(_data_ptr->get(i).asString());
		temp.append(_data_ptr->get(i).toString());
		//temp.append(" ");
		temp.append("\t");
	}
	temp.append("\n");
	this->outputFile << temp.c_str();
}

// ***************************************************************************

/**
 *  Implementation of DataOutputRobot class
 */

// Constructor
DataOutputRobot::DataOutputRobot(const char *_portName_ptr) : DataOutput(_portName_ptr, 1) {
}
// Destructor
DataOutputRobot::~DataOutputRobot(){}

// method to write over port 
void DataOutputRobot::write(Bottle *_data_ptr) {
	Bottle &out = this->outputPort.prepare();
	out.clear();
	out.copy(*_data_ptr);
	this->outputPort.write();
}

// ***************************************************************************

/**
 * Implementation of ImgOutput class
 */

// Init constructor that calls the Output constructor that stores the portname, here it opens the imageport to write out
ImgOutput::ImgOutput(const char *_portName, bool _createPort) : Output(_portName) {

	if (_createPort) {
		this->outputPort.open(this->getPortname());
		this->img_rgb_ptr	= new ImageOf<PixelRgb>; //&this->img_rgb;
	}
	this->createdPort = _createPort;
}

// Destructor
ImgOutput::~ImgOutput() {
	//printf("Info:\t[IO.ImgOutput]\t{image output port closed}\n");
	if (this->createdPort) {
		this->outputPort.close();
	}
}

void ImgOutput::write(ImageOf<PixelRgb>  *_img_ptr) {
	printf("ImgOutput::write\n");
}
void ImgOutput::write(ImageOf<PixelMono> *_img_ptr) {
	printf("ImgOutput::write\n");
}
void ImgOutput::write(ImageOf<PixelRgb>  *_img_ptr, string _fileName) {
	printf("ImgOutput::write\n");
}
void ImgOutput::write(ImageOf<PixelMono> *_img_ptr, string _fileName) {
	printf("ImgOutput::write\n");
}

// ***************************************************************************

/**
 * Implementation of ImgOutputFile class
 */

// Init constructor that calls the Output constructor that stores the portname, here it opens the imageport to write out
ImgOutputFile::ImgOutputFile(const char *_destDir_ptr) : ImgOutput("/imageport", 0) {
	this->writeDir_ptr = _destDir_ptr;
}

// destructor
ImgOutputFile::~ImgOutputFile() {
	
}

// method to write the content of <_img> out
void ImgOutputFile::write(ImageOf<PixelRgb> *_img_ptr) {
	printf("not possible to write img rgb out without a filename\n");
}

void ImgOutputFile::write(ImageOf<PixelMono> *_img_ptr) {
	printf("not possible to write img mono out without a filename\n");
}

void ImgOutputFile::write(ImageOf<PixelRgb> *_img_ptr, string _fileName_ptr) {

	string tmp = string(this->writeDir_ptr) + _fileName_ptr + ".ppm";
	yarp::sig::file::write(*_img_ptr, tmp.c_str());
}

void ImgOutputFile::write(ImageOf<PixelMono> *_img_ptr, string _fileName_ptr) {
	string tmp = string(this->writeDir_ptr) + _fileName_ptr + ".pgm";
	yarp::sig::file::write(*_img_ptr, tmp.c_str());
}


// ***************************************************************************

/**
 * Implementation of ImgOutputRobot class
 */

// Init constructor that calls the Output constructor that stores the portname, here it opens the imageport to write out
ImgOutputRobot::ImgOutputRobot(const char *_portName_ptr) : ImgOutput(_portName_ptr, 1) {

}

// destructor
ImgOutputRobot::~ImgOutputRobot() {
	
}

// method to write the content of <_img> out
void ImgOutputRobot::write(ImageOf<PixelRgb> *_img_ptr) {
	this->img_rgb_ptr = &this->outputPort.prepare();
	this->img_rgb_ptr->copy(*_img_ptr);
	this->outputPort.write();
}

void ImgOutputRobot::write(ImageOf<PixelMono> *_img_ptr) {
	this->img_rgb_ptr = &this->outputPort.prepare();
	this->img_rgb_ptr->copy(*_img_ptr);
	this->outputPort.write();
}


void ImgOutputRobot::write(ImageOf<PixelRgb> *_img_ptr, string _fileName_ptr) {
	//printf("write rgb img to port, but with receiving a filename\n");
	this->write(_img_ptr);
}

void ImgOutputRobot::write(ImageOf<PixelMono> *_img_ptr, string _fileName_ptr) {
	//printf("write mono img to port, but with receiving a filename\n");
	this->write(_img_ptr);
}

// ***************************************************************************

