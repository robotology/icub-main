// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/fingerInterfaceModule.h>
#include <gtk/gtk.h>

static GtkWidget *mainWindow = NULL;

//drawing area
static GtkWidget *da;
static fingerInterfaceModule *_module;
// Current frame
static GdkPixbuf *frame = NULL;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;
// Semaphore
static yarp::os::Semaphore *ptr_semaphore;

static yarp::sig::ImageOf<yarp::sig::PixelRgb>* _outputImage;
static fingerInterfaceModule *_fingerInterface;


#define _imgRecv (*(ptr_imgRecv))
#define _semaphore (*(ptr_semaphore))


void cleanExit(){
	
	//closePorts();
	gtk_main_quit ();
	//_module->close();
	_module->stop_flag=true;
    //deleteObjects();
}

//timer that runs every x milliseconds, I will use it to redraw the pic;
void time_handler()
{
	gtk_widget_queue_draw (da);
}

bool fingerInterfaceModule::open(Searchable& config) {
	this->stop_flag=false;
	imageThread=new fingerImageThread(20); //rateThread: set refresh rate in ms
	imageThread->start();

	_module=this;
	
    //port.open(getName());
    	
	// create a new window
	ptr_imgRecv = new YARPImgRecv;
    ptr_semaphore = new yarp::os::Semaphore;
	if (openPorts() == false)  return false;

    mainWindow = this->createMainWindow();
	//X is pressed -> cleanExit is called
	gtk_signal_connect(GTK_OBJECT(mainWindow),		
                       "destroy",
                       GTK_SIGNAL_FUNC(cleanExit),
                       NULL);
	//timer that runs every x milliseconds, I will use it to redraw the pic;
	g_timeout_add(5, (GSourceFunc) time_handler, (gpointer) mainWindow); 

	// Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
	gtk_window_move(GTK_WINDOW(mainWindow), 10,10);
	
	// All GTK applications must have a gtk_main(). Control ends here
	// and waits for an event to occur (like a key press or
	// mouse event).
	gtk_main ();

	gtk_widget_destroy(mainWindow);

    return true;
}


// try to interrupt any communications or resource usage
bool fingerInterfaceModule::interruptModule() {
	//port.interrupt();
	g_print("Image Thread stopping.....");
	imageThread->stop();
	return true;
}

bool fingerInterfaceModule::close() {
	//port.close();
	imageThread->stop();

	gdk_threads_leave(); //see before: g_thread_init(NULL); gdk_threads_init(); gdk_threads_enter();

	return false;
	}

bool fingerInterfaceModule::updateModule() {
	if(stop_flag){
		this->close();
		return false;
	}
	return true;
}

//-------------------------------------------------
// Service Fuctions
//-------------------------------------------------


bool fingerInterfaceModule::openPorts(){
	bool ret = true;
	
	//g_print("Registering port %s on network %s...\n", "/rea/ImageProcessor/in","default");
	//ret = _imgRecv.Connect("/rea/ImageProcessor/in","default");

	//if (ret == true)       
	//	g_print("Port registration succeed!\n");
    //else
     //   g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
    
	return true;
}

//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------


static gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
	//printf("entering expose_CB \n");
	if(frame){
		//printf("frame not null");
		if ( mainWindow){
			//printf("frame and mainWindow present \n");
			guchar *pixels;
			unsigned int rowstride;
			
			if(_fingerInterface->imageThread->redraw)
			{
				_outputImage=_fingerInterface->imageThread->image2;
				if(_outputImage==NULL){
					printf("expose_CB:_inputImg NULL");
					return false;
				}
							
				_semaphore.wait();
				bool result=yarpImage2Pixbuf(_outputImage, frame);
				_semaphore.post();
			}
			
		//from here till I say this is necessary if window is resized
			unsigned int imageWidth = 320;
			unsigned int imageHeight = 240;

			unsigned int areaWidth = event->area.width;
			unsigned int areaHeight = event->area.height;

			unsigned int pixbufWidth=gdk_pixbuf_get_width(frame);
			unsigned int pixbufHeight=gdk_pixbuf_get_height(frame);

			if ((imageWidth!=pixbufWidth) || (imageHeight!=pixbufHeight))
				{
					g_object_unref(frame);
					//printf("unreferencing frame \n");
					frame=gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, imageWidth, imageHeight);
				}

			if ( (areaWidth != imageWidth) || (areaHeight != imageHeight) )
				{
					GdkPixbuf *scaledFrame;
					//printf("scaling image... \n");
					scaledFrame = gdk_pixbuf_scale_simple(	frame,
															areaWidth,
															areaHeight,
															GDK_INTERP_BILINEAR); // Best quality
					//GDK_INTERP_NEAREST); // Best speed

					pixels = gdk_pixbuf_get_pixels (scaledFrame);
					rowstride = gdk_pixbuf_get_rowstride(scaledFrame);
					gdk_draw_rgb_image (widget->window,
										widget->style->black_gc,
										event->area.x, event->area.y,
										event->area.width, event->area.height,
										GDK_RGB_DITHER_NORMAL,
										pixels,
										rowstride);
					g_object_unref(scaledFrame);
			
				}
		//here is the end of the handler in case the window is resized
			else
				{
					pixels = gdk_pixbuf_get_pixels (frame);
					rowstride = gdk_pixbuf_get_rowstride(frame);
					//printf("drawing image... \n");
					gdk_draw_rgb_image (widget->window,
										widget->style->black_gc,
										event->area.x, event->area.y,
										event->area.width, event->area.height,
										GDK_RGB_DITHER_NORMAL,
										pixels,
										rowstride);
				}
		}
	}
		
	return TRUE;
}


//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* fingerInterfaceModule::createMainWindow(void)
{
	_fingerInterface=this; //it is necessary to synchronise the static function with this class
	
	GtkWidget* window;
	
    //gtk_init (&argc, &argv);
	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "Image Processing Module");
	gtk_window_set_default_size(GTK_WINDOW (window), 320, 240); //320, 700
	gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    
	// Box for main window
	GtkWidget *box;
	box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
	
	// Drawing Area : here the image will be drawn
	da = gtk_drawing_area_new ();
	
    g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
	
	gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);
	

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240); 

	mainWindow=window;

	return window;
}
