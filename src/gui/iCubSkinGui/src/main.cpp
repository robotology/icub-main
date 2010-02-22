// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <memory.h>

#include <yarp/os/all.h>
#include <iCub/SkinMeshThread.h>

#include <gtk/gtk.h>

#ifdef USE_ICUB_MOD
#include "iCub/drivers.h"
#endif

using namespace yarp::os;

static GtkWidget *gpDrawingArea=NULL;
static SkinMeshThread *gpSkinMeshThread=NULL;
static yarp::os::Semaphore gMutex(1);
static int gbRunning=TRUE;
static int gWidth=0,gHeight=0,gRowStride=0,gImageSize=0,gMapSize=0,gImageArea=0;
static double *gpActivationMap=NULL;
static guchar *gpImageBuff=NULL;
static guint gTimer;

static void timer_handler()
{
	gtk_widget_queue_draw(gpDrawingArea);
}

static gint paint(GtkWidget *pWidget,GdkEventExpose *pEvent,gpointer pData)
{
    gint width=0,height=0;
    gdk_drawable_get_size(pWidget->window,&width,&height);

    //gint width =gpDrawingArea->allocation.width;
    //gint height=gpDrawingArea->allocation.height;

    static bool bDrawing=false;
 
    if (!gbRunning) return FALSE;

    if (bDrawing) return TRUE;

    gMutex.wait();
    
    bDrawing=true;

    if (width!=gWidth || height!=gHeight)
    {
        gWidth=width;
        gHeight=height;
        
        gRowStride=3*gWidth;
        gImageSize=gRowStride*gHeight;
        gImageArea=gWidth*gHeight;
        gMapSize=gWidth*gHeight*sizeof(double);

        if (gpActivationMap) delete [] gpActivationMap;
        if (gpImageBuff) delete [] gpImageBuff;
        gpActivationMap=new double[gImageArea];
        gpImageBuff=new guchar[gImageSize];

        if (gpSkinMeshThread && gWidth>=180 && gHeight>=180) gpSkinMeshThread->resize(gWidth,gHeight);
    }
    
    if (gpSkinMeshThread)
    {
        memset(gpActivationMap,0,gMapSize);
        memset(gpImageBuff,0,gImageSize);
        
        if (gWidth>=180 && gHeight>=180)
        {
            /*
            gpSkinMeshThread->eval(gpActivationMap);

            for (int i=0; i<gImageArea; ++i)
            {
                gpImageBuff[i*3]=gpActivationMap[i]<255.0?guchar(gpActivationMap[i]):255;
            }
            */
            gpSkinMeshThread->draw(gpImageBuff);
        }
        
        gdk_draw_rgb_image(pWidget->window,
                           pWidget->style->black_gc,
					       pEvent->area.x,pEvent->area.y,
						   pEvent->area.width,pEvent->area.height,
						   GDK_RGB_DITHER_NONE,
						   gpImageBuff,
						   gRowStride);
    }
 
    gMutex.post();

    bDrawing=false;

    return TRUE;
}

void clean_exit()
{
    gMutex.wait();
    gbRunning=FALSE;
    g_source_remove(gTimer);
    gtk_main_quit();
    gMutex.post();
}

int main(int argc, char *argv[]) 
{
    //Network yarp;
 
    #ifdef USE_ICUB_MOD
	yarp::dev::DriverCollection dev;
	#endif

    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("iCubSkinDemo");
    rf.setDefaultConfigFile("hand.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    gWidth =rf.find("width" ).asInt();
    gHeight=rf.find("height").asInt();

    gRowStride=3*gWidth;
    gImageSize=gRowStride*gHeight;
    gMapSize=gWidth*gHeight*sizeof(double);
    gImageArea=gWidth*gHeight;

    gpActivationMap=new double[gImageArea];
    gpImageBuff=new guchar[gImageSize];

    gtk_init (&argc, &argv);
	
    GtkWidget *pMainWindow=gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(pMainWindow),"iCub Skin");
    gtk_window_set_resizable(GTK_WINDOW(pMainWindow),TRUE);
	gtk_window_set_default_size(GTK_WINDOW(pMainWindow),gWidth,gHeight);
    gtk_window_resize(GTK_WINDOW(pMainWindow),gWidth,gHeight);

	GtkWidget *pBox=gtk_vbox_new(FALSE,0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add(GTK_CONTAINER(pMainWindow),pBox);

	gpDrawingArea=gtk_drawing_area_new();
	gtk_box_pack_start(GTK_BOX(pBox),gpDrawingArea,TRUE,TRUE,0);

	gtk_signal_connect(GTK_OBJECT(pMainWindow),"destroy",GTK_SIGNAL_FUNC(clean_exit),NULL);
    g_signal_connect(gpDrawingArea,"expose_event",G_CALLBACK(paint),NULL);

    gTimer=g_timeout_add(50,(GSourceFunc)timer_handler,(gpointer)pMainWindow);

	gtk_widget_show_all(pMainWindow);
	gtk_window_move(GTK_WINDOW(pMainWindow),32,32);

    gpSkinMeshThread=new SkinMeshThread(rf);
    gpSkinMeshThread->start();

	gtk_main();

	//gtk_widget_destroy(mainWindow);

    gpSkinMeshThread->stop();
    delete gpSkinMeshThread;

    if (gpActivationMap) delete [] gpActivationMap;
    if (gpImageBuff) delete [] gpImageBuff;
}
