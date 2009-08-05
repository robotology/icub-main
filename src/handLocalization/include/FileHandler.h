#ifndef _FILEHANDLER_
#define _FILEHANDLER_

//includes
#include <includes.h>

// namespaces 
namespace thesis {
    /**
     * Files.
     */
	namespace files {
		class FileHandler;
		class DataFileHandler;
		class ImgFileHandler;
	}
}

// ***************************************************************************

/**
 *  Class FileHandler: definition
 */
class thesis::files::FileHandler {
protected:
	// Declaration of hidden class variables
	string filename;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	FileHandler();
	~FileHandler();
	const char* getFilename();
	//string getFilename();
};

// ***************************************************************************

/**
 *  Class DataFileHandler: definition
 */
class thesis::files::DataFileHandler  : public FileHandler {
protected:
	// Declaration of hidden class variables
	// Declaration of hidden methods
	string generateFilename();
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	DataFileHandler();
	~DataFileHandler();
};

// ***************************************************************************

/**
 *  Class ImgFileHandler: definition
 */
class thesis::files::ImgFileHandler : public FileHandler {
protected:
	// Declaration of hidden class variables
	int filecounter;

	// Declaration of hidden methods
	string generateFilename();
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgFileHandler();
	~ImgFileHandler();
	void next();
	int getFilecounter();
};


#endif
