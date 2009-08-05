#ifndef _FILEHANDLER_
#define _FILEHANDLER_

//includes
#include <includes.h>

// namespaces 
namespace halode {
    /**
     * Files.
     */
	namespace files {
		class FileHandler;
		//class DataFileHandler;
		class ImgFileHandler;
	}
}

// ***************************************************************************

/**
 *  Class FileHandler: definition
 */
class halode::files::FileHandler {
protected:
	// Declaration of hidden class variables
	string filename;
	int channel;
	Parameters *params_ptr;
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	FileHandler();
	FileHandler(Parameters *_params_ptr);
	~FileHandler();
	const char* getFilename();
	//string getFilename();
};

// ***************************************************************************

/**
 *  Class ImgFileHandler: definition
 */
class halode::files::ImgFileHandler : public FileHandler {
protected:
	// Declaration of hidden class variables
	int		filecounter, dump_cnt_len, dump_max_filenum, dump_min_filenum;
	string	dump_directory, dump_name, dump_end;
	bool	dump_zerofilling;

	// Declaration of hidden methods
	string generateFilename();
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	ImgFileHandler();
	ImgFileHandler(Parameters *_params_ptr, int _channel);
	~ImgFileHandler();
	void next();
	int getFilecounter();
};


#endif
