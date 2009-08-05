#ifndef _TOOLS_
#define _TOOLS_

#include <includes.h>
#include <IO.h>
#include <vector>

// namespaces 
namespace thesis {
    /**
     * tools to help different modules to "interact" without overhead.
     */
    namespace tools {
		class Decider;
		class Buffer;
    }
}


// ***************************************************************************
// ********                  namespace thesis::tools                  ********
// ***************************************************************************

/**
 *  Class DeciderAndConnector: definition 
 *
 */
class thesis::tools::Decider {
private:
	// Declaration of hidden class variables
	bool	motionTriggered;
	bool	patchClassificationTriggered;

	// Declaration of hidden  methods

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Decider();
	~Decider();
	
	bool reset();
	bool decide(string _onWhat);
	void setTrigger(string _onWhat);

};

// ***************************************************************************

/**
 *  Class Buffer: definition 
 *
 */
class thesis::tools::Buffer {
private:
	// Declaration of hidden class variables
	int						buffersize, wc, rc;
	bool					full;
	ImageOf<PixelMono>		*image_buffer_ptr;
	Bottle					*data_buffer_ptr;
	bool					*wB_ptr;
	bool					*rB_img_ptr;
	bool					*rB_data_ptr;
	IO::DataInput			*data_port_ptr;
	IO::ImgInput			*img_port_ptr;

	// Declaration of hidden  methods
	void increaseCounter(string _cntName);
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	//Buffer();
	Buffer(IO::ImgInput *_input_img_port, IO::DataInput *_input_data_port);
	~Buffer();

	bool				addToBuffer();
	ImageOf<PixelMono>* readImg();
	Bottle*				readData();
	//void				readFromBuffer(ImageOf<PixelMono> *_img_ptr, Bottle *_data_bottle_ptr);
};

#endif

