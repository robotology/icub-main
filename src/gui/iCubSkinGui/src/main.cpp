// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <memory.h>

#include <yarp/os/all.h>
#include <iCub/SkinMeshThread.h>

#include <gtk/gtk.h>

#ifdef USE_ICUB_MOD
#include "iCub/drivers.h"
#endif

using namespace yarp::os;

GtkWidget *da=NULL;

SkinMeshThread *skinMeshThread=NULL;

void time_handler()
{
	gtk_widget_queue_draw(da);
}

int gRunning=TRUE;

static yarp::os::Semaphore mutex(1);

static gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
    static int width=640,height=640;
    static int rowStride=3*width;
    static int imageSize=rowStride*height;
    static int imageArea=width*height;
    static guchar *imageBuff=new guchar[imageSize];
    static double *values=new double[width*height];

    static bool drawing=false;
 
    if (drawing) return TRUE;

    mutex.wait();
    
    if (!gRunning) return FALSE;

    drawing=true;

    if (skinMeshThread)
    {
        memset(values,0,imageArea*sizeof(double));
        memset(imageBuff,0,imageSize);

        skinMeshThread->eval(values);

        for (int i=0; i<imageArea; ++i)
        {
            imageBuff[i*3]=values[i]<=255.9?guchar(values[i]):255;
        }

        skinMeshThread->draw(imageBuff);

        gdk_draw_rgb_image(widget->window,
                        widget->style->black_gc,
					    event->area.x, event->area.y,
						event->area.width, event->area.height,
						GDK_RGB_DITHER_NORMAL,
						imageBuff,
						rowStride);
    }

    mutex.post();

    drawing=false;

    return TRUE;
}

guint gTimer;

void cleanExit()
{
    mutex.wait();
    gRunning=FALSE;
    g_source_remove(gTimer);
    gtk_main_quit();
    mutex.post();
}

int main(int argc, char *argv[]) 
{
    //Network yarp;
 
    #ifdef USE_ICUB_MOD
	yarp::dev::DriverCollection dev;
	#endif

    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("tutorials/iCubSkinDemo");
    rf.setDefaultConfigFile("skin.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    int width =rf.find("width").asInt();
    int height=rf.find("width").asInt();
	
    gtk_init (&argc, &argv);
	
    GtkWidget *mainWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    
    gtk_window_set_title(GTK_WINDOW(mainWindow),"iCub Skin");

    gtk_window_set_resizable(GTK_WINDOW(mainWindow),TRUE);

	gtk_window_set_default_size(GTK_WINDOW(mainWindow),width,height); //320, 700
    
    gtk_window_resize(GTK_WINDOW(mainWindow),width,height); //320, 700

	// Box for main window
	GtkWidget *box=gtk_vbox_new(FALSE,0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add(GTK_CONTAINER(mainWindow),box);
	
	// Drawing Area : here the image will be drawn
	da=gtk_drawing_area_new();
	g_signal_connect(da,"expose_event",G_CALLBACK(expose_CB),NULL);
	gtk_box_pack_start(GTK_BOX(box),da,TRUE,TRUE,0);

	gtk_signal_connect(GTK_OBJECT(mainWindow),"destroy",GTK_SIGNAL_FUNC(cleanExit),NULL);

    gTimer=g_timeout_add(50,(GSourceFunc)time_handler,(gpointer)mainWindow);

	gtk_widget_show_all(mainWindow);
	gtk_window_move(GTK_WINDOW(mainWindow),32,32);
    //gtk_window_set_resizable(GTK_WINDOW(mainWindow),FALSE);


    //gtk_widget_queue_draw(da);

    
	
	// All GTK applications must have a gtk_main(). Control ends here
	// and waits for an event to occur (like a key press or
	// mouse event).

    skinMeshThread=new SkinMeshThread(rf);
    skinMeshThread->start();

	gtk_main();

	//gtk_widget_destroy(mainWindow);

    skinMeshThread->stop();
    delete skinMeshThread;
}
