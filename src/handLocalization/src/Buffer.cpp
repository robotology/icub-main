// include header
#include <Tools.h>

// namespaces
using namespace thesis::tools;
using namespace thesis::IO;

using std::vector;

// ***************************************************************************

/**
 * Implementation of Buffer class
 *
 * This class is a pool of methods /functionality that can serve every module. 
 *
 */

Buffer::Buffer(ImgInput *_input_img_port, DataInput *_input_data_port) {
	printf("Start:\t[Buffer]\n");
	int i;
	this->wc			= 0;
	this->rc			= 0;
	this->data_port_ptr			= _input_data_port;
	this->img_port_ptr			= _input_img_port;
	this->buffersize			= BUFFERSIZE;
	this->image_buffer_ptr		= new ImageOf<PixelMono>	[BUFFERSIZE];
	this->data_buffer_ptr		= new Bottle				[BUFFERSIZE];
	this->wB_ptr		= new bool					[BUFFERSIZE];
	this->rB_img_ptr	= new bool					[BUFFERSIZE];
	this->rB_data_ptr	= new bool					[BUFFERSIZE];

	for (i = 0; i<BUFFERSIZE; i++) {
		this->image_buffer_ptr[i].resize(320,240);
		this->image_buffer_ptr[i].zero();
		this->data_buffer_ptr[i].clear();
		this->wB_ptr[i]		= false;
		this->rB_img_ptr[i]		= true;
		this->rB_data_ptr[i]	= true;
		this->addToBuffer();
	}
}

Buffer::~Buffer() {
	delete [] this->image_buffer_ptr;
	delete [] this->data_buffer_ptr;
	delete [] this->wB_ptr;
	delete [] this->rB_img_ptr;
	delete [] this->rB_data_ptr;
	printf("Quit:\t[Buffer]\n");
}

void Buffer::increaseCounter(string _cntName) {
	if (_cntName == "write") {
		if ((this->wc+1) >= this->buffersize) {
			this->wc = 0;
		}
		else {
			this->wc++;
		}

	}
	else if (_cntName == "read") {
		if ((this->rc+1) >= this->buffersize) {
			this->rc = 0;
		}
		else {
			this->rc++;
		}
	}
}

bool Buffer::addToBuffer() {
	//printf("Info:\t[Buffer]\t{write[%d] if  w(0 ==[%d]) and ri(1==[%d]) and rd(1==[%d])}\n", this->wc, this->wB_ptr[this->wc], this->rB_img_ptr[this->wc], this->rB_data_ptr[this->wc] );
	if ((!this->wB_ptr[this->wc]) & (this->rB_img_ptr[this->wc]) & (this->rB_data_ptr[this->wc])) {
		this->image_buffer_ptr[this->wc].copy(*this->img_port_ptr->readImg());
		this->data_buffer_ptr[this->wc].copy(*this->data_port_ptr->readData());
		this->wB_ptr[this->wc]			= true;
		this->rB_img_ptr[this->wc]		= false;
		this->rB_data_ptr[this->wc]		= false;
		//printf("Info:\t[Buffer]\t{wrote[%d] now w(1 ==[%d]) and ri(0==[%d]) and rd(0==[%d])}\n", this->wc, this->wB_ptr[this->wc], this->rB_img_ptr[this->wc], this->rB_data_ptr[this->wc]);
		this->increaseCounter("write");
		return true;
	}
	else {
		return false;
	}
}

ImageOf<PixelMono>* Buffer::readImg() {
	ImageOf<PixelMono> *ptr = NULL;
	//printf("Info:\t[Buffer]\t{i:read[%d] if ri(0==[%d]) and w (0==[%d])}\n", this->rc, this->rB_img_ptr[this->rc], this->wB_ptr[this->rc]);
	if ((!this->rB_img_ptr[this->rc]) & (this->wB_ptr[this->rc])) {
		this->rB_img_ptr[this->rc] = true;
		ptr = &this->image_buffer_ptr[this->rc];
		//printf("Info:\t[Buffer]\t{i:read'[%d] now ri(1==[%d]) and w (0==[%d])}\n", this->rc, this->rB_img_ptr[this->rc], this->wB_ptr[this->rc]);
		if (this->rB_data_ptr[this->rc]) {
			this->increaseCounter("read");
		}
		return ptr;
	}
	else {
		printf("riesenfehler\n");
		exit(-99);
	}
}

Bottle* Buffer::readData() {
	Bottle *ptr = NULL;
	//printf("Info:\t[Buffer]\t{d:read[%d] if rd(0==[%d]) and ri(1==[%d]) and w (1==[%d])}\n", this->rc, this->rB_data_ptr[this->rc], this->rB_img_ptr[this->rc], this->wB_ptr[this->rc]);
	if ((!this->rB_data_ptr[this->rc]) & (this->wB_ptr[this->rc])) {
		this->rB_data_ptr[this->rc] = true;
		ptr = &this->data_buffer_ptr[this->rc];
		//printf("Info:\t[Buffer]\t{d:read'[%d] now rd(1==[%d]) and ri(1==[%d]) and w (0==[%d])}\n", this->rc, this->rB_data_ptr[this->rc], this->rB_img_ptr[this->rc], this->wB_ptr[this->rc]);
		if (this->rB_img_ptr[this->rc]) {
			this->wB_ptr[this->rc] = false;
			this->increaseCounter("read");
			this->addToBuffer();
		}
		return ptr;
	}
	else {
		printf("error: sorry no more elements in buffer\n");
		exit(-99);
	}
}
