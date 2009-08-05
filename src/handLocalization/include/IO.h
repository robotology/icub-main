#ifndef _HL_CONNECTIONS_
#define _HL_CONNECTIONS_

//includes

// including own includes
#include <includes.h>
#include <FileHandler.h>

#include <fstream>

// namespaces 
using namespace thesis::files;

namespace thesis {
    /**
     * Networking.
     */
    namespace IO {
		// Connection
		class Connection;
		// Input classes
		class Input;
		class DataInput;
		class DataInputFile;
		class DataInputRobot;
		class ImgInput;
		class ImgInputFile;
		class ImgInputRobot;
		// Output classes
		class Output;
		class ImgOutput;
		class DataOutput;
	}
}

// ***************************************************************************

/**
 *  Class Connection: definition
 */
class thesis::IO::Connection {
protected:
	// Declaration of hidden class variables
	int netCounter;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Connection();
	Connection(int _netCounter);
	~Connection();
	bool connect(const char *_input_port, const char *_output_port);
	bool disconnect(const char *_input_port, const char *_output_port);
};

// ***************************************************************************

/**
 *  Class Input: definition
 */
class thesis::IO::Input {
protected:
	// Declaration of hidden class variables
	const char *inputPortname;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Input(const char *_portname);
	~Input();
	const char* getPortname();
};

// ***************************************************************************

/**
 *  Class DataInput: definition
 */
class thesis::IO::DataInput : public thesis::IO::Input {
protected:
	// Declaration of hidden class variables
	BufferedPort<Bottle> inputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataInput(const char *_portName);
	~DataInput();

	virtual Bottle* readData();
};

// ***************************************************************************

/**
 *  Class DataInputFile: definition
 */
class thesis::IO::DataInputFile : public thesis::IO::DataInput {
protected:
	// Declaration of hidden variables
	Bottle fileContent;
	Bottle encoderValues;
	int currentLine;
	int maxLineNr;
	// Declaration of hidden methods
	void loadFile();
	void next();
public:
	// Declaration of class variables
	DataFileHandler dataHandler;
	// Declaration of Constructor, Destructor and methods
	DataInputFile(const char *_portName);
	~DataInputFile();
	Bottle* readData();
};

// ***************************************************************************

/**
 *  Class DataInputRobot: definition
 */
class thesis::IO::DataInputRobot : public thesis::IO::DataInput {
protected:
	Bottle encoderValues;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataInputRobot(const char *_portName);
	~DataInputRobot();
	Bottle* readData();
};

// ***************************************************************************

/**
 *  Class ImgInput: definition
 */
class thesis::IO::ImgInput : public thesis::IO::Input {
protected:
	// Declaration of hidden class variables
	BufferedPort<ImageOf<PixelRgb> > inputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgInput(const char *_portName);
	~ImgInput();

	virtual ImageOf<PixelRgb>* readImg();
};

// ***************************************************************************

/**
 *  Class ImgInputFile: definition
 */
class thesis::IO::ImgInputFile : public thesis::IO::ImgInput {
protected:
	// Declaration of hidden class variables
	ImgFileHandler		imgHandler;
	ImageOf<PixelRgb>	img;
	ImageOf<PixelRgb>	*img_ptr;
	// Declaration of hidden methods
	void next();
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgInputFile(const char *_portName);
	~ImgInputFile();

	int	getCurrentFilecounter();
	ImageOf<PixelRgb>*	getImg();
	ImageOf<PixelRgb>*	readImg();
};

// ***************************************************************************

/**
 *  Class ImgInputRobot: definition
 */
class thesis::IO::ImgInputRobot : public thesis::IO::ImgInput {
public:
	// Declaration of class variables
	ImageOf<PixelRgb> img;
	ImageOf<PixelRgb> *img_ptr;

	// Declaration of Constructor, Destructor and methods
	ImgInputRobot(const char *_portname);
	~ImgInputRobot();
	ImageOf<PixelRgb>* readImg();	
};

// ***************************************************************************

/**
 *  Class Output: definition
 */
class thesis::IO::Output {
protected:
	// Declaration of hidden class variables
	const char *outputPortname;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Output(const char *_portname);
	~Output();
	const char* getPortname();
};

// ***************************************************************************

/**
 *  Class ImgOutput: definition
 */
class thesis::IO::ImgOutput : public thesis::IO::Output {
protected:
	// Declaration of hidden class variables
	BufferedPort<ImageOf<PixelRgb> > outputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgOutput(const char *_portName);
	~ImgOutput();
	void write(ImageOf<PixelRgb> *_img_ptr);
	void write(ImageOf<PixelMono> *_img_ptr);
};


#endif

