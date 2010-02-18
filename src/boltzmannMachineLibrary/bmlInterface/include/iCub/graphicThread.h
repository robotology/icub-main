// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _GRAPHICTHREAD_H_
#define _GRAPHICTHREAD_H_



//=============================================================================
// GTK Includes 
//=============================================================================
#include <gtk/gtk.h>

//within Project Include
#include <iCub/YARPImgRecv.h>
#include <iCub/YarpImage2Pixbuf.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;


class graphicThread : public RateThread{
private:
    /**
	* maximum value for the gtk sliding control
	*/
	double maxAdj;
	/**
	* maximum value for the gtk sliding control
	*/
	double minAdj;
	/**
	* step adjustment for the gtk sliding control
	*/
	double stepAdj;

    /**
	* Output Port for commands 
	*/
	yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
	/**
	* Output Port for commands 
	*/
	yarp::os::BufferedPort<ImageOf<PixelRgb> > *_pOutPort2;

public:
    /**
    * default constructor
    */
    graphicThread();
    /**
    * destructor
    */
    ~graphicThread();
    /**
    *	initialization of the thread 
    */
    bool threadInit();
    /**
    * active loop of the thread
    */
    void run();
    /**
    *	releases the thread
    */
    void threadRelease();
    /**
    * function that safely close this thread
    */
    void close();
    /**
	* open all the ports and all the YARP image receivers 
	* on the dedicated ports
	*/
	bool openPorts();
	/**
	* close all the ports and all the YARP image receivers 
	*/
	bool closePorts();
	/**
	* creates all the objects in the window
	*/
	void createObjects();
    /**
	* sets the module up
	*/
	void setUp();
    /**
	* sets the adjustments in the window
	*/
	void setAdjs();
    /**
    * set the reference to the module that has istantiated this thread
    * @param module reference to the module
    */
    void setImageProcessModule(void *module);
    /** 
	* create the main Window
	*/
	GtkWidget* createMainWindow(); //
	/**
	* creates the menu bar
	*/
	GtkWidget* createMenubar(void); //
	/**
    * function that sets the Interface reference which called this thread
    */
    void setModule(void *);
    /**
	* function that set the dimension of the layer (rows)
	*/
	void setRowDim(int number);
	/**
	* function that sets the dimension of the layer (column)
	*/
	void setColDim(int number);
	//----------- attributes ---------------
    
    /**
	* pointer to the input image o
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImage; //
	/**
	* pointer to the input image of layer0
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer0; //
	/**
	* pointer to the input image of layer1
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer1; //
	/**
	* pointer to the input image of layer2
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer2; //
	/**
	* pointer to the input image of layer3
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer3;  //
	/**
	* pointer to the input image of layer4
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer4;  //
	/**
	* pointer to the input image of layer5
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer5; //
	/**
	* pointer to the input image of layer6
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer6; //
	/**
	* pointer to the input image of layer7
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer7; //
	/**
	* pointer to the input image of layer8
	*/
	yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputLayer8; //

    //---------- FLAGS -------------------------------
	/**
	* flag that indicates the control box inputImage is active
	*/
	bool inputImage_flag;
	
	/**
	* flag that indicates the control box Layer0 is active
	*/
	bool inLayer0_flag;
	/**
	* flag that indicates the control box Layer1 is active
	*/
	bool inLayer1_flag;
	/**
	* flag that indicates the control box Layer2 is active
	*/
	bool inLayer2_flag;
	/**
	* flag that indicates the control box Layer3 is active
	*/
	bool inLayer3_flag;
	/**
	* flag that indicates the control box Layer4 is active
	*/
	bool inLayer4_flag;
	/**
	* flag that indicates the control box Layer5 is active
	*/
	bool inLayer5_flag;
	/**
	* flag that indicates the control box Layer6 is active
	*/
	bool inLayer6_flag;
	/**
	* flag that indicates the control box Layer7 is active
	*/
	bool inLayer7_flag;
	/**
	* flag that indicates the control box Layer8 is active
	*/
	bool inLayer8_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer0_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer1_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer2_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer3_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer4_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer5_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer6_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer7_flag;
	/**
	* flag that indicates the control box SelectLayer0 is active
	*/
	bool SelectLayer8_flag;
	/**
	* flag that indicates when the freely run mode is active
	*/
	bool runFreely_flag;
	/**
	* flag that indicates when the clamped run mode is active
	*/
	bool runClamped_flag;
	/**
	* flag that indicates when the ClampLayer Button has been pushed
	*/
	bool clampLayer_flag;
    

    /**
	*Row Dimension of the layer currently set
	*/
	int rowDim;
	/**
	*Column Dimension of the layer currently set
	*/
	int colDim;

	//----------- checkButtons ---------------------

	/**
	* button present on the graphical interface
	*/
	GtkWidget *buttonCheckGreen,*buttonCheckRed,*buttonCheckBlue;


};

#endif //__GRAPHICTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
