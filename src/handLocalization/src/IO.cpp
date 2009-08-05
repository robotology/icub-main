//includes
#include <IO.h>

// namespaces
using namespace thesis::IO;

// ***************************************************************************

/**
 * Implementation of Connection class
 */

// Default constructor
Connection::Connection() {
	//printf("Start:\t[IO]\n");
	if (this->netCounter != 0) {
		printf("Error:\t[IO.Connection]\t{Constructor() netCounter != 0}\n");
	}
	else {
		printf("Error:\t[IO.Connection]\t{Constructor() else part}\n");
	}
}

// Init constructor
Connection::Connection(int _netCounter) {
	//printf("Opening Network\n");
	if (_netCounter == 0) {
		this->netCounter = _netCounter;
		Network::init();
	}
	this->netCounter++;
}

// Destructor
Connection::~Connection() {
	this->netCounter--;
	if (this->netCounter == 0) {
		Network::fini();
	}
	//printf("Quit:\t[IO.Connection]\n");
}

// method to connect ports
bool Connection::connect(const char *_inputPort, const char *_outputPort) {
	//printf("Connecting:\n\tport %s with \n\tport %s\n", _inputPort, _outputPort);
	bool b;
	b = Network::connect(_inputPort, _outputPort);
	printf("Info:\t[IO.Connection]\t{%s port connected}\n",_outputPort);
	return b;
}

bool Connection::disconnect(const char *_inputPort, const char *_outputPort) {
	//printf("Connecting:\n\tport %s with \n\tport %s\n", _inputPort, _outputPort);
	bool b;
	b = Network::disconnect(_inputPort, _outputPort);
	printf("Info:\t[IO.Connection]\t{%s port connected}\n",_outputPort);
	return b;
}

// ***************************************************************************

/**
 * Implementation of Input class
 */

// Init constructor
Input::Input(const char *_portname) {
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
 * Implementation of DataInput class
 */

// Init constructor that is calling the Input init constructor that sets the portname
DataInput::DataInput(const char *_portName) : Input(_portName) {
	this->inputPort.open(this->inputPortname);
}

// Destructor
DataInput::~DataInput() {
	this->inputPort.close();
}

//virtual
Bottle* DataInput::readData() {
	Bottle *b_ptr = NULL;
	return b_ptr;
}

// ***************************************************************************

/**
 * Implementation of DataInputFile class
 */

// Init constructor that is calling the DataInput init constructor that opening the port
DataInputFile::DataInputFile(const char *_portName) : DataInput(_portName) {
	//printf("Open input port from files on port \n");
	this->dataHandler = DataFileHandler ();
	this->currentLine = 0;

	this->fileContent.clear();
	this->encoderValues.clear();
	this->loadFile();
}

// Destructor
DataInputFile::~DataInputFile() {
	//printf("Closing input port from file\n");

}

/**
 * Load the motor command log file into a bottle. 
 * every line of the file is stored raw into an element of the bottle.
 **/
void DataInputFile::loadFile() {
	string fileLine = "";
	const char *filename = this->dataHandler.getFilename();
	//printf("load from file %s \n", filename);
	std::ifstream pullFileContent (filename);
	while (std::getline(pullFileContent, fileLine)) {
		//printf("add line [%s]\n", fileLine.c_str());
		this->fileContent.addString(fileLine.c_str());
	}
	this->maxLineNr = fileContent.size();
	pullFileContent.close();
}

/**
 * increasing in a standard way the line counter, in order to get the next line.
 **/
void DataInputFile::next() {
	this->currentLine++;
}

/**
 * Provide the information from the file bottle in a proper clean new bottle. line per line.
 **/
Bottle* DataInputFile::readData() {
	// get the encoder values from the file -- 
	Bottle values;
	string temp;
	string element;
	const string space = " ";
	const string tab = "\t";
	int pos1, pos2;
	int i = 0;

	if (this->currentLine < this->maxLineNr) {
		temp = ((this->fileContent.get(this->currentLine)).toString());

		//printf("currentline[%d] = %s\n", this->currentLine, temp.c_str());
		while (temp.length() > 0) {
		//while (i < 20 ) {
			//printf("{%s}\n", temp.c_str());
			//pos1 = temp.find_first_not_of(tab);
			pos1 = 0;
			pos2 = temp.find(tab);
			while (pos2 == 0) {
				temp = temp.replace(0,1,"");
				pos2 = temp.find(tab);
			}
			/*if (pos2 == -1) {
				pos2 = temp.find(space);
			}*/
			//printf("pos1: %d -> pos2: %d\n", pos1, pos2);
			element = temp.substr(pos1, pos2-pos1);
			if (element != tab) {
				values.addString(element.c_str());
			}
			//printf("element: [%s]\n", element.c_str());
			temp = temp.replace(pos1, pos2-pos1, "");
		}  		
		//printf("elements in value: %s\n", values.size());

		//values.addDouble((double)(x.c_str()));
	}
	else {
		//values.addString(NULL);
		this->currentLine = -1;
		return this->readData();
	}
	this->next();
	// returning the current values of the arm controls
	this->encoderValues = values;
	return &(this->encoderValues);
}


// ***************************************************************************

/**
 * Implementation of DataInputRobot class
 */

// Init constructor that calls the DataInput constructor to store the port name
DataInputRobot::DataInputRobot(const char *_portName) : DataInput(_portName) {
	//printf("Info:\t[IO.DataInputRobot]\t{robot input port opened}\n");
}

// Destructor
DataInputRobot::~DataInputRobot(){
	//printf("Info:\t[IO.DataInputRobot]\t{robot input port closed}\n");
}

Bottle* DataInputRobot::readData() {
	this->encoderValues.clear();
	this->encoderValues.copy(*this->inputPort.read());
	return &this->encoderValues;
}
// ***************************************************************************

/**
 * Implementation of ImgInput class
 */

// Init constructor
ImgInput::ImgInput(const char *_portName) : Input(_portName) {
	this->inputPort.open(this->inputPortname);
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
ImgInputFile::ImgInputFile(const char *_portname) : ImgInput(_portname) {
	//printf("Info:\t[IO.ImgInputFile]\t{image file input port opened}\n");
	this->imgHandler = ImgFileHandler ();
	//this->readImg();
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
ImgInputRobot::ImgInputRobot(const char *_portName) : ImgInput(_portName) {
	//printf("Info:\t[IO.InputRobot]\t{robot image input port opened}\n");
}

// Destructor
ImgInputRobot::~ImgInputRobot(){
	//printf("Info:\t[IO.InputRobot]\t{robot image input port closed}\n");
}

// get the next image from the port
ImageOf<PixelRgb>* ImgInputRobot::readImg() {
	this->img.copy(*this->inputPort.read());
	this->img_ptr = &this->img;
	return this->img_ptr;
}

// ***************************************************************************

/**
 * Implementation of Output class
 */

// Init constructor
Output::Output(const char *_portName) {
	this->outputPortname = _portName;
}

// Destructor
Output::~Output() {
}

// method to receive the port name
const char* Output::getPortname() {
	return this->outputPortname;
}

// ***************************************************************************

/**
 * Implementation of ImgOutput class
 */

// Init constructor that calls the Output constructor that stores the portname, here it opens the imageport to write out
ImgOutput::ImgOutput(const char *_portName) : Output(_portName) {
	//printf("Info:\t[IO.ImgOutput]\t{image output port opened}\n");
	this->outputPort.open(this->outputPortname);

}

// Destructor
ImgOutput::~ImgOutput() {
	//printf("Info:\t[IO.ImgOutput]\t{image output port closed}\n");
	this->outputPort.close();
}

// method to write the content of <_img> out
void ImgOutput::write(ImageOf<PixelRgb> *_img_ptr) {
	ImageOf<PixelRgb> &out = this->outputPort.prepare();
	out.zero();
	out.copy(*_img_ptr);
	this->outputPort.write();
}

void ImgOutput::write(ImageOf<PixelMono> *_img_ptr) {
	ImageOf<PixelRgb> out;
	out.zero();
	out.copy(*_img_ptr);
	this->write(&out);
}

// ***************************************************************************
