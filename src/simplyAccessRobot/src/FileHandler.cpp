#include <FileHandler.h>

using namespace halode::files;

// ***************************************************************************

/**
 * Implementation of FileHandler class
 *
 *
 */

// Default constructor
FileHandler::FileHandler() {
	this->filename = "";
}

FileHandler::FileHandler(Parameters *_params_ptr) {
	//printf("Start:\t[FileHandler]\n");
	this->params_ptr = _params_ptr;
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
 *  handle stereovision by giving the camera channel. (e.g left = 1, right = 2)
 */

// Default constructor
ImgFileHandler::ImgFileHandler() {
}

ImgFileHandler::ImgFileHandler(Parameters *_params_ptr, int _channel) : FileHandler (_params_ptr) {
	//printf("Image File Handler called...\n");
	this->channel			= _channel;
	if (_channel == 1) {
		this->filecounter		= this->params_ptr->dump1_min_filenum;
		this->dump_directory	= this->params_ptr->dump1_dir;
		this->dump_name			= this->params_ptr->dump1_name;
		this->dump_cnt_len		= this->params_ptr->dump1_cnt_len;
		this->dump_zerofilling	= this->params_ptr->dump1_zerofilling;
		this->dump_min_filenum	= this->params_ptr->dump1_min_filenum;
		this->dump_max_filenum	= this->params_ptr->dump1_max_filenum;
		this->dump_end			= this->params_ptr->dump1_end;
		printf("channel 1 reads from directory : %s\n", this->dump_directory.c_str());
	
	}
	else if (_channel == 2) {
		this->filecounter		= this->params_ptr->dump2_min_filenum;
		this->dump_directory	= this->params_ptr->dump2_dir;
		this->dump_name			= this->params_ptr->dump2_name;
		this->dump_cnt_len		= this->params_ptr->dump2_cnt_len;
		this->dump_zerofilling	= this->params_ptr->dump2_zerofilling;
		this->dump_min_filenum	= this->params_ptr->dump2_min_filenum;
		this->dump_max_filenum	= this->params_ptr->dump2_max_filenum;
		this->dump_end			= this->params_ptr->dump2_end;
		printf("channel 2 reads from directory : %s\n", this->dump_directory.c_str());
	}
	else {
		printf("no no, channel must be 1 or 2. Or do you have more than 2 eyes?\n");
		exit(1);
	}
	this->filename			= this->generateFilename();
}

ImgFileHandler::~ImgFileHandler() {
}

string ImgFileHandler::generateFilename() {
	string fn = "";
	char *numb_ptr = new char [this->dump_cnt_len];
	fn.append(this->dump_directory);
	fn.append(this->dump_name);
	if (this->dump_zerofilling) {
		if (this->filecounter == 0) {
			for (int i=0; i<(this->dump_cnt_len-1); i++) {
				fn.append("0");
			}
		}
		else if (this->filecounter > 0 ){
			int res = this->filecounter*10;
			while (res < pow(10, this->dump_cnt_len)) {
				fn.append("0");
				res = res *10;
			}
		}
	}
	sprintf(numb_ptr,"%d",this->filecounter);
	fn.append(numb_ptr);
	fn.append(this->dump_end);
	delete numb_ptr;
	//printf("** generated filename: %s **\n", fn.c_str());
	return fn;
}

void ImgFileHandler::next() {
	this->filecounter++;
	if (this->filecounter > this->dump_max_filenum) {
		this->filecounter = this->dump_min_filenum;
	}
	this->filename = generateFilename();
}

int ImgFileHandler::getFilecounter() {
	return this->filecounter;
}

