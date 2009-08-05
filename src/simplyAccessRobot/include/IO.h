#ifndef _HL_CONNECTIONS_
#define _HL_CONNECTIONS_

//includes

// including own includes
#include <includes.h>
#include <FileHandler.h>

#include <fstream>

// namespaces 
using namespace halode::files;

namespace halode {
    /**
     * Networking.
     */
    namespace IO {
		// Connection
		class Connection;
		// Input classes
		class Input;
		//class DataInput;
		//class DataInputFile;
		//class DataInputRobot;
		class ImgInput;
		class ImgInputFile;
		class ImgInputRobot;
		// Output classes
		class Output;
		class DataOutput;
		class DataOutputFile;
		class DataOutputRobot;
		class ImgOutput;
		class ImgOutputFile;
		class ImgOutputRobot;
	}
}

// ***************************************************************************

/**
 *  Class Connection: definition
 */
class halode::IO::Connection {
protected:
	// Declaration of hidden class variables
	Parameters *params_ptr;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Connection();
	Connection(Parameters *_params_ptr);
	~Connection();
	bool connect(const char *_input_port, const char *_output_port);
	bool disconnect(const char *_input_port, const char *_output_port);
};

// ***************************************************************************

/**
 *  Class Input: definition
 */
class halode::IO::Input {
protected:
	// Declaration of hidden class variables
	const char *inputPortname;
	Parameters *params_ptr;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Input();
	Input(Parameters *_params_ptr, const char *_portname);
	~Input();
	const char* getPortname();
};

// ***************************************************************************

/**
 *  Class DataInput: definition
 */
/*class halode::IO::DataInput : public halode::IO::Input {
protected:
	// Declaration of hidden class variables
	BufferedPort<Bottle> inputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataInput(Parameters *_params_ptr, const char *_portname);
	~DataInput();

	virtual Bottle* readData();
};*/

// ***************************************************************************

/**
 *  Class ImgInput: definition
 */
class halode::IO::ImgInput : public halode::IO::Input {
protected:
	// Declaration of hidden class variables
	BufferedPort<ImageOf<PixelRgb> > inputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgInput();
	ImgInput(Parameters *_params_ptr, const char *_portname);
	~ImgInput();

	virtual ImageOf<PixelRgb>* readImg();
};

// ***************************************************************************

/**
 *  Class ImgInputFile: definition
 */
class halode::IO::ImgInputFile : public halode::IO::ImgInput {
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
	//ImgInputFile();
	ImgInputFile(Parameters *_params_ptr, const char *_portname, int _channel);
	~ImgInputFile();

	int	getCurrentFilecounter();
	ImageOf<PixelRgb>*	getImg();
	ImageOf<PixelRgb>*	readImg();
};

// ***************************************************************************

/**
 *  Class ImgInputRobot: definition
 */
class halode::IO::ImgInputRobot : public halode::IO::ImgInput {
private:
	ImgOutputFile		*imgOut_ptr;
	int					cnt;
	string				filename;
public:
	// Declaration of class variables
	ImageOf<PixelRgb> img;
	ImageOf<PixelRgb> *img_ptr;

	// Declaration of Constructor, Destructor and methods
	ImgInputRobot(Parameters *_params_ptr, const char *_portname);
	~ImgInputRobot();
	ImageOf<PixelRgb>* readImg();	
};

// ***************************************************************************

/**
 *  Class Output: definition
 */
class halode::IO::Output {
protected:
	// Declaration of hidden class variables
	const char		*outputPortname;
	bool			createdPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Output(const char *_portname);
	~Output();
	const char* getPortname();
};

// ***************************************************************************

/**
 *  Class DataOutput: definition
 */
class halode::IO::DataOutput : public halode::IO::Output {
protected:
	// Declaration of hidden class variables
	BufferedPort<Bottle >	outputPort;
	std::ofstream			outputFile;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataOutput(const char *_portName_ptr, bool _createPort);
	~DataOutput();
	virtual void write(Bottle *_data_ptr);
};

// ***************************************************************************

/**
 *  Class DataOutputFile: definition
 */
class halode::IO::DataOutputFile : public halode::IO::DataOutput {
protected:
	// Declaration of hidden class variables
	Bottle				data_bottle;
	const char			*writeDir_ptr;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataOutputFile(const char *_portName_ptr);
	~DataOutputFile();
	void write(Bottle *_data_ptr);
};

// ***************************************************************************

/**
 *  Class DataOutputRobot: definition
 */
class halode::IO::DataOutputRobot : public halode::IO::DataOutput {
protected:
	// Declaration of hidden class variables
	Bottle data_bottle;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataOutputRobot(const char *_portName_ptr);
	~DataOutputRobot();
	void write(Bottle *_data_ptr);
};

// ***************************************************************************

/**
 *  Class ImgOutput: definition
 */
class halode::IO::ImgOutput : public halode::IO::Output {
protected:
	// Declaration of hidden class variables
	BufferedPort<ImageOf<PixelRgb> > outputPort;
	ImageOf<PixelRgb>				img_rgb;
	//ImageOf<PixelMono>				img_mono;
	ImageOf<PixelRgb>				*img_rgb_ptr;
	//ImageOf<PixelMono>				*img_mono_ptr;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgOutput(const char *_portName, bool _createPort);
	~ImgOutput();
	virtual void write(ImageOf<PixelRgb>  *_img_ptr);
	virtual void write(ImageOf<PixelMono> *_img_ptr);
	virtual void write(ImageOf<PixelRgb>  *_img_ptr, string _fileName);
	virtual void write(ImageOf<PixelMono> *_img_ptr, string _fileName);
};

// ***************************************************************************

/**
 *  Class ImgOutputRobot: definition
 */
class halode::IO::ImgOutputRobot : public halode::IO::ImgOutput {
protected:
	// Declaration of hidden class variables
	//BufferedPort<ImageOf<PixelRgb> > outputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgOutputRobot(const char *_portName);
	~ImgOutputRobot();
	void write(ImageOf<PixelRgb>  *_img_ptr);
	void write(ImageOf<PixelMono> *_img_ptr);

	void write(ImageOf<PixelRgb>  *_img_ptr, string _fileName);
	void write(ImageOf<PixelMono> *_img_ptr, string _fileName);
};

// ***************************************************************************

/**
 *  Class ImgOutputFile: definition
 */
class halode::IO::ImgOutputFile : public halode::IO::ImgOutput {
protected:
	// Declaration of hidden class variables
	const char *writeDir_ptr;
	//BufferedPort<ImageOf<PixelRgb> > outputPort;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgOutputFile(const char *_destDir_ptr);
	~ImgOutputFile();
	void write(ImageOf<PixelRgb>  *_img_ptr);
	void write(ImageOf<PixelMono> *_img_ptr);
	void write(ImageOf<PixelRgb>  *_img_ptr, string _fileName);
	void write(ImageOf<PixelMono> *_img_ptr, string _fileName);
};

#endif

