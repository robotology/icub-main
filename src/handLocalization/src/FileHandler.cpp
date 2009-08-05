#include <FileHandler.h>

using namespace thesis::files;

// paulfitz - portability hack
char *_itoa_(int val, char *str, int base) {
    if (base!=10) {
        printf("Please count your fingers.\n");
    }
    sprintf(str,"%d",val);
    return str;
}

// ***************************************************************************

/**
 * Implementation of FileHandler class
 *
 *
 */

// Default constructor
FileHandler::FileHandler() {
	//printf("Start:\t[FileHandler]\n");
	this->filename = "";
}

FileHandler::~FileHandler() {
	//printf("Quit:\t[FileHandler]\n");
}

//string FileHandler::getFilename() {
const char*  FileHandler::getFilename() {
	return this->filename.c_str();
}

// ***************************************************************************

/**
 * Implementation of ImgFileHandler class
 *
 *
 */

// Default constructor
ImgFileHandler::ImgFileHandler() {
	//printf("Image File Handler called...\n");
	this->filecounter = 0;
	this->filename = this->generateFilename();
}

ImgFileHandler::~ImgFileHandler() {
}

string ImgFileHandler::generateFilename() {
	string fn = "";
	char numb[NUMLEN];
	fn.append(FILE_DIR);
	fn.append(FILE_NAME);
	if (ZEROFILLING==1) {
		if (this->filecounter == 0) {
			for (int i=0; i<(NUMLEN-1); i++) {
				fn.append("0");
			}
		}
		else if (this->filecounter > 0 ){
			int res = this->filecounter*10;
			while (res < pow(10, NUMLEN)) {
				fn.append("0");
				res = res *10;
			}
		}
	}
    fn.append(string(_itoa_(this->filecounter, numb, 10)));
    fn.append(FILE_ENDING);
	//printf("** generated filename: %s **\n", fn.c_str());
	return fn; //.c_str();
    //return string("");
}

void ImgFileHandler::next() {
	this->filecounter++;
	if (this->filecounter > MAXFILENUM) {
		this->filecounter = MINFILENUM;
	}
	this->filename = generateFilename();
}

int ImgFileHandler::getFilecounter() {
	return this->filecounter;
}

// ***************************************************************************

/**
 * Implementation of DataFileHandler class
 *
 *
 */

// Default constructor
DataFileHandler::DataFileHandler() {
	this->filename = this->generateFilename();
}

DataFileHandler::~DataFileHandler() {

}

string DataFileHandler::generateFilename() {
	string fn = "";
	fn.append(FILE_DIR);
	fn.append(MOTORINFORMATION);
	return fn.c_str();
}
