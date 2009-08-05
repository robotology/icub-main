#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>

#include "support.h"
#include "callbacks.h"
#include "interface.h"
#include "functions.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <windows.h>

#include <iCub/RC_DIST_FB_logpolar_mapper.h>

extern GList * windowsList;
extern GList * pixbufList;
extern gboolean changeZorder;
extern imagedata imgPar;
extern GtkWidget *mainMenuWindow;

void on_menu_window_activation (GtkWidget *widget, gpointer user_data)
{
}

void on_file_open_activate (GtkMenuItem * menuitem,
							gpointer user_data)
{
	GtkWidget *filechooserdialog;
	GtkFileFilter *fileFilter;

	fileFilter = gtk_file_filter_new();
	gtk_file_filter_set_name (fileFilter,	_("bmp"));

	gtk_file_filter_add_pattern(fileFilter,"*.bmp");

	filechooserdialog = create_filechooserdialog ();

	gtk_widget_show (filechooserdialog);

	gtk_file_chooser_add_filter(GTK_FILE_CHOOSER (filechooserdialog),fileFilter);

	fileFilter = gtk_file_filter_new();
	gtk_file_filter_set_name (fileFilter,	_("jpg"));

	gtk_file_filter_add_pattern(fileFilter,"*.jpg");

	gtk_file_chooser_add_filter(GTK_FILE_CHOOSER (filechooserdialog),fileFilter);

	g_signal_connect ((gpointer) filechooserdialog, "destroy", G_CALLBACK(gtk_widget_destroy),
						NULL);
	GtkImage *preview;
	preview = GTK_IMAGE(gtk_image_new ());
	gtk_file_chooser_set_preview_widget (GTK_FILE_CHOOSER (filechooserdialog), GTK_WIDGET(preview));

	g_signal_connect (filechooserdialog, "update-preview",
 			G_CALLBACK (update_preview_cb), preview);

	if (gtk_dialog_run (GTK_DIALOG (filechooserdialog)) == GTK_RESPONSE_OK)
		drawImagefromFile(filechooserdialog);

	gtk_widget_destroy (filechooserdialog);
}


void update_preview_cb (GtkFileChooser *file_chooser, gpointer data)
{
	GtkWidget *preview;
	char *filename;
	GdkPixbuf *pixbuf;
	gboolean have_preview;

	filename = NULL;

	preview = GTK_WIDGET (data);
	filename = gtk_file_chooser_get_preview_filename (file_chooser);
	
	if (filename != NULL)
	{
		pixbuf = gdk_pixbuf_new_from_file_at_size (filename, 128, 128, NULL);
		have_preview = (pixbuf != NULL);
		g_free (filename);

		gtk_image_set_from_pixbuf (GTK_IMAGE (preview), pixbuf);
		if (pixbuf)
		gdk_pixbuf_unref (pixbuf);

		gtk_file_chooser_set_preview_widget_active (file_chooser, have_preview);
	}
}


void on_image_window_activation (	GtkWidget *widget,
									gpointer user_data)
{
	if (changeZorder)
	{
		guint listPos;

		//Browse the list for NULL pointers
		listPos = g_list_index(windowsList,(gconstpointer) widget); // Gets the index of current window

		gpointer pBuf = g_list_nth_data(pixbufList,listPos); //Gets a pointer to current pixbuf

		windowsList = g_list_remove(windowsList,(gconstpointer) widget);
		pixbufList  = g_list_remove(pixbufList ,(gconstpointer) pBuf);

		windowsList = g_list_prepend(windowsList,(gpointer) widget);
		pixbufList  = g_list_prepend(pixbufList ,(gpointer) pBuf);

		gint listLength = g_list_length (windowsList);
	}

}

gboolean delete_event( GtkWidget *widget,
                              GdkEvent  *event,
                              gpointer   data )
{
    /* If you return FALSE in the "delete_event" signal handler,
     * GTK will emit the "destroy" signal. Returning TRUE means
     * you don't want the window to be destroyed.
     * This is useful for popping up 'are you sure you want to quit?'
     * type dialogs. */

	guint listPos;

	//Browse the list for NULL pointers

	listPos = g_list_index(windowsList,(gconstpointer) widget); // Gets the index of current window
	gpointer pBuf = g_list_nth_data(pixbufList,listPos); //Gets a pointer to current pixbuf

	windowsList = g_list_remove(windowsList,(gconstpointer) widget);
	pixbufList  = g_list_remove(pixbufList ,(gconstpointer) pBuf);

	// TODO :dovrei prendermi il puntatore ai dati e deallocarlo

	changeZorder = TRUE;

    /* Change TRUE to FALSE and the main window will be destroyed with
     * a "delete_event". */

    return FALSE;
}

gboolean on_darea_expose(	GtkWidget *widget,
							GdkEventExpose *event,
							gpointer user_data)
{
	gdk_draw_rgb_image (widget->window,
						widget->style->fg_gc[GTK_STATE_NORMAL],
						0, 0, 
						gdk_pixbuf_get_width ((GdkPixbuf *)user_data), 
						gdk_pixbuf_get_height((GdkPixbuf *)user_data),
						GDK_RGB_DITHER_MAX, 
						gdk_pixbuf_get_pixels((GdkPixbuf *)user_data), 
						gdk_pixbuf_get_rowstride((GdkPixbuf *)user_data));
	return TRUE;
}


void on_file_save_activate (GtkMenuItem * menuitem,gpointer user_data)
{
	if(g_list_length (windowsList)!=0)
	{
		GtkWidget *filechooserdialog;

		filechooserdialog = create_filesavedialog ();

		gtk_widget_show (filechooserdialog);

		g_signal_connect ((gpointer) filechooserdialog, "destroy", G_CALLBACK(gtk_widget_destroy),
							NULL);

		if (gtk_dialog_run (GTK_DIALOG (filechooserdialog)) == GTK_RESPONSE_OK)
			saveImagetoFile(filechooserdialog);

		gtk_widget_destroy (filechooserdialog);
	}

}


gboolean on_ext_changed (GtkWidget *widget,gpointer   data)
{
	GtkFileFilter * currFilter = gtk_file_chooser_get_filter  (GTK_FILE_CHOOSER (widget));

	const gchar* filter = gtk_file_filter_get_name (currFilter);
	const gchar* bmpFilter = "Bitmaps";

	gchar fullname [256];

	if (!strcmp (filter,bmpFilter))
	{
		sprintf(fullname,"%s.bmp",gtk_window_get_title(GTK_WINDOW(g_list_nth_data(windowsList,0))));
		gtk_file_chooser_set_current_name (GTK_FILE_CHOOSER (widget), fullname);
	}
	else 
	{
		sprintf(fullname,"%s.jpg",gtk_window_get_title(GTK_WINDOW(g_list_nth_data(windowsList,0))));
		gtk_file_chooser_set_current_name (GTK_FILE_CHOOSER (widget), fullname);
	}

	return FALSE;
}

void on_file_quit_activate (GtkMenuItem * menuitem, gpointer user_data)
{
	//TODO : Implementation
}


void on_tools_c2lp_activate(GtkMenuItem * menuitem, 
					   gpointer user_data)
{
	if (g_list_length (windowsList)!=0)
	{
		GtkWidget *dialog;
		dialog = create_cart2lp_dialog(); 
		gtk_window_set_modal(GTK_WINDOW(dialog),TRUE);
		changeZorder = FALSE;
		gtk_widget_show (dialog);
	}
}

void on_tools_lp2c_activate(GtkMenuItem * menuitem,
					   gpointer user_data)
{
	if (g_list_length (windowsList)!=0)
	{
		GtkWidget *dialog;
		dialog = create_lp2cart_dialog();
		gtk_window_set_modal(GTK_WINDOW(dialog),TRUE);
		changeZorder = FALSE;
		gtk_widget_show (dialog);
	}
}

void on_tools_toolbox_activate(	GtkMenuItem * menuitem, 
								gpointer user_data)
{
	if (gtk_check_menu_item_get_active  (GTK_CHECK_MENU_ITEM(menuitem)))
		create_toolbox ();
	else
	{
		GtkWidget* toolbox = GTK_WIDGET(g_object_get_data (G_OBJECT (mainMenuWindow),"toolbox"));
		g_signal_emit_by_name (	(gpointer) toolbox, "destroy");
	}
}

void c2lp_OK_clicked(GtkButton *button,
					 gpointer user_data)
{
	gint width, height;
	guchar * pixData;
	gint listLength = g_list_length (windowsList);

	guchar * polarImg = new guchar [imgPar.rho_Size*imgPar.theta_Size*3];

	if (listLength>0)
	{
		gtk_widget_destroy (gtk_widget_get_toplevel(GTK_WIDGET(button)));

		GList* children = gtk_container_get_children (GTK_CONTAINER(g_list_nth_data(windowsList,0)));

		GtkWidget *dialog1;
		GtkWidget *dialog_vbox1;
		GtkWidget *progBar;

		dialog1 = gtk_dialog_new ();
		gtk_window_set_title (GTK_WINDOW (dialog1), _("Progress"));
		gtk_window_set_type_hint (GTK_WINDOW (dialog1), GDK_WINDOW_TYPE_HINT_DIALOG);

		dialog_vbox1 = GTK_DIALOG (dialog1)->vbox;
		gtk_widget_show (dialog_vbox1);

		progBar = gtk_progress_bar_new ();
		gtk_box_pack_start (GTK_BOX (dialog_vbox1), progBar, FALSE, FALSE, 0);
		gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);
		gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Building Table"));
		gtk_widget_show (progBar);

		gtk_widget_show (dialog1);

		while (gtk_events_pending()) gtk_main_iteration_do(FALSE);

		GdkPixbuf * pixBuf =  gdk_pixbuf_copy((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));

		width   = gdk_pixbuf_get_width (pixBuf);
		height  = gdk_pixbuf_get_height(pixBuf);
		pixData = gdk_pixbuf_get_pixels(pixBuf);

		//Load Table, if existing
		//else generate it
		
		cart2LpPixel * c2LpMap = new cart2LpPixel[imgPar.theta_Size*imgPar.rho_Size];

//		if (loadMap(c2LpMap,progBar)==NULL)
//		if (RCallocateC2LTable(c2LpMap, imgPar.rho_Size, imgPar.theta_Size, false, ".\\"))
//		{
		imagedata storedData;
		memset(&storedData, 0, sizeof(imagedata));

//		memcpy (&storedData,&imgPar, sizeof(imagedata) );


		char filename [256];
		FILE * fin;
		sprintf(filename,"%sC2LP.data",".\\");

		if ((fin = fopen(filename,"rb")) != NULL)
		{
			fread (&storedData,sizeof(imagedata),1,fin);
			fclose (fin);
		}

		if( (imgPar.rho_Size==storedData.rho_Size) && (imgPar.theta_Size==storedData.theta_Size) && ((int)(2*imgPar.mapRadius)==(int)(2*storedData.mapRadius)))
		{
			RCallocateC2LTable(c2LpMap, imgPar.rho_Size, imgPar.theta_Size, false, ".\\");
		}
		else
		{
			FILE * fout;
			sprintf(filename,"%sC2LP.data",".\\");
			fout = fopen(filename,"wb");
			fwrite(&imgPar,sizeof(imagedata),1,fout);
			fclose(fout);
			RCbuildC2LMap(imgPar.rho_Size, imgPar.theta_Size,  (int)(2*imgPar.mapRadius),imgPar.overlap,imgPar.scaleFactor,ELLIPTICAL,".\\");
			RCallocateC2LTable(c2LpMap, imgPar.rho_Size, imgPar.theta_Size, false, ".\\");
		}

//		if (loadData()==NULL))

//			buildMap(progBar);
//			loadMap(c2LpMap,progBar);
//		}

		gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Processing Table"));
		gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);
		gtk_widget_show (GTK_WIDGET(progBar));

		int i,j;
		int x,y;
		int x1,y1;

		int sum;

		for (i=0; i<imgPar.theta_Size*imgPar.rho_Size; i++)
		{
			if (!(i%imgPar.theta_Size))
			{
				gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), (double)i/double(imgPar.theta_Size*imgPar.rho_Size-1));
				gtk_widget_show (GTK_WIDGET(progBar));
			}

			while (gtk_events_pending()) gtk_main_iteration_do(FALSE);

			sum = 0;
			for (j=0; j<c2LpMap[i].divisor; j++)
			{
				if (c2LpMap[i].position[j]!=0)
				{
					x = (c2LpMap[i].position[j]/3)%((int)(2*imgPar.mapRadius));
					y = (c2LpMap[i].position[j]/3)/((int)(2*imgPar.mapRadius));
					x1 = x+imgPar.xCenter-(int)(imgPar.mapRadius);
					y1 = y+imgPar.yCenter-(int)(imgPar.mapRadius);

					if ((x1>=0)&&(x1<imgPar.X_Size)&&(y1>=0)&&(y1<imgPar.Y_Size))
						c2LpMap[i].position[j] = 3*(y1*imgPar.X_Size+x1);
					else
					{
						c2LpMap[i].position[j] = 0;
						c2LpMap[i].iweight[j] = 0;
					}
				}
				sum += c2LpMap[i].iweight[j];
			}
			if (sum == 0)
				c2LpMap[i].iweight[0] = c2LpMap[i].divisor * 255;
		}

		gtk_widget_destroy (gtk_widget_get_toplevel(GTK_WIDGET(progBar)));
//		generate_Log_Polar_Image_with_Table(pixBuf,imgPar.theta_Size,imgPar.rho_Size,polarImg,c2LpMap);
	
		pixData[0] = 192;
		pixData[1] = 192;
		pixData[2] = 160;

		RCgetLpImg(polarImg,pixData,c2LpMap,imgPar.theta_Size*imgPar.rho_Size,0);	

		drawImagefromMemory(polarImg,imgPar.theta_Size,imgPar.rho_Size);

		//set the new name for the image:
		gpointer wind = g_list_nth_data(windowsList,0); //Gets a pointer to current window
		gpointer parentwind = g_list_nth_data(windowsList,1); //Gets a pointer to current window

		const gchar * imagenameStart = gtk_window_get_title (GTK_WINDOW (parentwind));
		
		gchar imagename [256];

		strcpy(imagename,imagenameStart);
		
		//1. Remove extension, if any
		gchar * fileExt = strrchr( imagename, '.' );

		if (fileExt!=NULL)
			*fileExt = '\0';

		//2. Remove Size properties, if any
		char last = imagename[strlen(imagename)-1];

		if (last == ']')
		{
			fileExt = strrchr( imagename, '[' );
			
			if (fileExt!=NULL)
			{
				int len = strlen(fileExt);

				if ((len<=11)&&(len>=5))
					if(strrchr( imagename, 'x' )!=NULL)
					{
						if (fileExt!=NULL)
						{
							*fileExt = '\0';
							fileExt = (char*) &imagename[strlen(imagename)-1];
							if (fileExt!=NULL)
								*fileExt = '\0';
						}
					}
			}
		}

		//3. Remove Type properties, if any
		const gchar * compString = " LP";
		fileExt = strrchr( imagename, ' ' );
		if (fileExt!=NULL)
		{
			if (!strcmp (compString,fileExt))
			{
				*fileExt = '\0';
			}
		}
		compString = " Rem";
		fileExt = strrchr( imagename, ' ' );
		if (fileExt!=NULL)
		{
		}


		//Add type and Size
		char winTitle [256];

		sprintf (winTitle, "%s LP [%dx%d]", imagename, imgPar.theta_Size, imgPar.rho_Size);

		gtk_window_set_title (GTK_WINDOW (wind), winTitle);
	}

	changeZorder = TRUE;
}


gboolean mouse_button_press_event (	GtkWidget * widget,
									GdkEventButton * event,
									gpointer data)
{
	double scale;

	if (event->button == 1)
	{
		gint width  = gdk_pixbuf_get_width ((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
		gint height = gdk_pixbuf_get_height((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));

		gint maxSize = __max(width, height);

		gdouble topLim, botLim;

		if (maxSize == width)
		{
			if (imgPar.xCenter>(imgPar.X_Size-imgPar.mapRadius))
				topLim = imgPar.xCenter+imgPar.mapRadius;
			else
				topLim = imgPar.X_Size;

			if (imgPar.xCenter<imgPar.mapRadius)
				botLim = imgPar.xCenter-imgPar.mapRadius;
			else
				botLim = 0;
		}
		else
		{
			if (imgPar.yCenter>(imgPar.Y_Size-imgPar.mapRadius))
				topLim = imgPar.yCenter+imgPar.mapRadius;
			else
				topLim = imgPar.Y_Size;

			if (imgPar.yCenter<imgPar.mapRadius)
				botLim = imgPar.yCenter-imgPar.mapRadius;
			else
				botLim = 0;
		}

		if(imgPar.mapInside!=1)
			scale = (topLim-botLim)/maxSize;
		else scale = 1.0;


		gint dWidth, dHeight;

		if (maxSize == width)
		{
			dHeight  = 100-(100*height/width);
			dHeight /= 2;

			imgPar.xCenter = (int)(width * (50-((50-event->x)*scale))/100);
			imgPar.yCenter = (int)((height/2)+(event->y-50)*scale*width/(100));
		}
		else
		{
			dWidth  = 100-(100*width/height);
			dWidth /= 2;

			imgPar.xCenter = (int)((width/2)+(event->x-50)*scale*height/(100));
			imgPar.yCenter = (int)(height * (50-((50-event->y)*scale))/100);
		}

		if(imgPar.mapInside==1)
		if ((imgPar.xCenter<0)||(imgPar.xCenter>=width)||(imgPar.yCenter<0)||(imgPar.yCenter>=height))
		{
			imgPar.xCenter = imgPar.X_Size/2;
			imgPar.yCenter = imgPar.Y_Size/2;
		}

		GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
		GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
		GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
		GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));
		gint childNumb = g_list_length (children3);
	
		gpointer tempWidget;
		gint i;

		GtkWidget * xCenterEntry;
		GtkWidget * yCenterEntry;
		GtkWidget * scaleFactorEntryEdit;
		GtkWidget * scaleFactorEntryNoEdit;
		GtkWidget * radiusBox;
		GtkWidget * radiusEntry;

		for (i = 0; i<childNumb; i++)
		{
			tempWidget = g_list_nth_data(children3,i);
			const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

			if (!strcmp(widgetName,"Scale Factor Entry No Edit"))
				scaleFactorEntryNoEdit = GTK_WIDGET(tempWidget);
			else if (!strcmp(widgetName,"Scale Factor Entry Edit"))
				scaleFactorEntryEdit = GTK_WIDGET(tempWidget);
			else if (!strcmp(widgetName,"X Center Entry"))
				xCenterEntry = GTK_WIDGET(tempWidget);
			else if (!strcmp(widgetName,"Y Center Entry"))
				yCenterEntry = GTK_WIDGET(tempWidget);
			else if (!strcmp(widgetName,"Radius Box"))
				radiusBox = GTK_WIDGET(tempWidget);
		}

		GList* children4 = gtk_container_get_children (GTK_CONTAINER(radiusBox));
		childNumb = g_list_length (children4);

		for (i = 0; i<childNumb; i++)
		{
			tempWidget = g_list_nth_data(children4,i);
			const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

			if (!strcmp(widgetName,"Radius Entry"))
				radiusEntry = GTK_WIDGET(tempWidget);
		}

		updateRadius();

		gchar radiusString[11];
		sprintf(radiusString,"%3.2f", imgPar.mapRadius);
		gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

		imgPar.scaleFactor  = 2*imgPar.mapRadius;
		imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
		imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

		gchar xCenterString[5];
		sprintf(xCenterString,"%d", imgPar.xCenter);
		gtk_entry_set_text(GTK_ENTRY(xCenterEntry),xCenterString);

		gchar yCenterString[5];
		sprintf(yCenterString,"%d", imgPar.yCenter);
		gtk_entry_set_text(GTK_ENTRY(yCenterEntry),yCenterString);

		gchar scaleFactorString[11];
		sprintf(scaleFactorString,"%1.8f", imgPar.scaleFactor);

		if (imgPar.mapInside!=2)
			gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorString);
		else
			gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorString);

		cartesian_expose(widget,NULL,NULL);
		logpolar_expose(widget,NULL,NULL);
	}
  
	return TRUE;
}


gboolean logpolar_expose (	GtkWidget * da,
							GdkEventExpose * event,
							gpointer data)
{
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (da));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));

	gint childNumb = g_list_length (children3);
 	gpointer tempWidget;
	GtkWidget * LPAlign;

	int i;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"LP Align"))
			LPAlign = GTK_WIDGET(tempWidget);
	}

	GList* children4 = gtk_container_get_children (GTK_CONTAINER(LPAlign));
	GList* children5 = gtk_container_get_children (GTK_CONTAINER(children4->data));
	
	GtkWidget * dArea	= GTK_WIDGET(children5->data);//get a pointer to LP drawing area

	GdkGC *gc1;
	GdkColor color;

	if (da->window !=NULL)
	{
		gc1 = gdk_gc_new (dArea->window);
		color.red = 32767;
		color.green = 32767;
		color.blue = 32767;
		gdk_gc_set_rgb_fg_color (gc1, &color);

		gdk_draw_rectangle (dArea->window,
				  gc1,
				  TRUE,
				  0, 0,
				  100,
				  100);

		g_object_unref (gc1);

		GdkPixbuf * pixbuf =  gdk_pixbuf_copy((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
			
		guchar * pixData;
		guchar * polarImg = new guchar [imgPar.rho_Size*imgPar.theta_Size*3];

		gint width   = gdk_pixbuf_get_width (pixbuf);
		gint height  = gdk_pixbuf_get_height(pixbuf);
		pixData = gdk_pixbuf_get_pixels(pixbuf);

		generate_Log_Polar_Image(pixbuf,imgPar.theta_Size,imgPar.rho_Size,polarImg);

		GdkPixbuf * pixbuf1 = gdk_pixbuf_new_from_data(	polarImg,
											GDK_COLORSPACE_RGB,
											FALSE,
											8,
											imgPar.theta_Size,
											imgPar.rho_Size,
											imgPar.theta_Size*3,
											NULL,
											NULL);

		gint maxSize = __max(imgPar.theta_Size, imgPar.rho_Size);

		gint dWidth, dHeight;

		if (maxSize == imgPar.theta_Size)
		{
			dHeight = 100*imgPar.rho_Size/imgPar.theta_Size;
			dWidth  = 100;
		}
		else
		{
			dWidth  = 100*imgPar.theta_Size/imgPar.rho_Size;
			dHeight = 100;
		}

		GdkPixbuf * pixbufLP = gdk_pixbuf_scale_simple (pixbuf1,
														 dWidth,
														 dHeight,
														 GDK_INTERP_NEAREST);

  		gdk_draw_pixbuf((GdkDrawable *) dArea->window,
						NULL,
						pixbufLP,
						0,0,(100-dWidth)/2,(100-dHeight)/2,dWidth,dHeight,GDK_RGB_DITHER_NONE,0,0);

		delete [] polarImg;
		g_object_unref (pixbuf);
		g_object_unref (pixbuf1);
		g_object_unref (pixbufLP);
	}

  return TRUE;
}


gboolean
cartesian_expose (GtkWidget	    *da,
		     GdkEventExpose *event,
		     gpointer	     data)
{

	//Draw Background
  gint i, j, xcount, ycount;
  GdkGC *gc1, *gc2, *gc3;
  GdkColor color;
  
#define CHECK_SIZE 4
#define SPACING 0

	if (da->window !=NULL)
	{

	  gc1 = gdk_gc_new (da->window);
	  color.red = 32767;
	  color.green = 32767;
	  color.blue = 32767;
	  gdk_gc_set_rgb_fg_color (gc1, &color);

	  gc2 = gdk_gc_new (da->window);
	  color.red = 65535;
	  color.green = 65535;
	  color.blue = 65535;
	  gdk_gc_set_rgb_fg_color (gc2, &color);
  
	  xcount = 0;
	  i = SPACING;
	  while (i < da->allocation.width)
		{
		  j = SPACING;
		  ycount = xcount % 2; /* start with even/odd depending on row */
		  while (j < da->allocation.height)
			{
			  GdkGC *gc;
			  
			  if (ycount % 2)
				gc = gc1;
			  else
				gc = gc2;

			  gdk_draw_rectangle (da->window,
						  gc,
						  TRUE,
						  i, j,
						  CHECK_SIZE,
						  CHECK_SIZE);

			  j += CHECK_SIZE + SPACING;
			  ++ycount;
			}

		  i += CHECK_SIZE + SPACING;
		  ++xcount;
		}

	/////////////////////////////End Draw Background

		gint height = gdk_pixbuf_get_height((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
		gint width  = gdk_pixbuf_get_width ((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
		gint maxSize = __max(width, height);

		gint dWidth, dHeight;

		gdouble rad = imgPar.mapRadius;

	//	gdouble ratio = width/height;

		gdouble scale;

		gdouble topLim, botLim;


		if (maxSize == width)
	//	if (ratio >= 1.0)
		{
			dHeight = 100*height/width;
	//		dHeight = 100/ratio;
			dWidth  = 100;
			rad = 100*rad/width;

			if (imgPar.xCenter>(imgPar.X_Size-imgPar.mapRadius))
				topLim = imgPar.xCenter+imgPar.mapRadius;
			else
				topLim = imgPar.X_Size;

			if (imgPar.xCenter<imgPar.mapRadius)
				botLim = imgPar.xCenter-imgPar.mapRadius;
			else
				botLim = 0;
		}
		else
		{
			dWidth  = 100*width/height;
	//		dWidth  = 100*ratio;
			dHeight = 100;
			rad = 100*rad/height;

			if (imgPar.yCenter>(imgPar.Y_Size-imgPar.mapRadius))
				topLim = imgPar.yCenter+imgPar.mapRadius;
			else
				topLim = imgPar.Y_Size;

			if (imgPar.yCenter<imgPar.mapRadius)
				botLim = imgPar.yCenter-imgPar.mapRadius;
			else
				botLim = 0;
		}




		if(imgPar.mapInside!=1)
			scale = (topLim-botLim)/maxSize;
		else 
			scale = 1.0;

		double sXC = 100*(imgPar.xCenter)/(maxSize*scale);
		double sYC = 100*(imgPar.yCenter)/(maxSize*scale);
		double sRad = rad/scale;
		double sdW = dWidth/scale;
		double sdH = dHeight/scale;

		GdkPixbuf * pixbuf = gdk_pixbuf_scale_simple ((GdkPixbuf *)(g_list_nth_data(pixbufList,0)),
								 (gint)(0.5+sdW),
								 (gint)(0.5+sdH),
								 GDK_INTERP_NEAREST);

  		gdk_draw_pixbuf((GdkDrawable *) da->window,
						NULL,
						pixbuf,
						0,0,(gint)(0.5 + (100-sdW)/2),(gint)(0.5 + (100-sdH)/2),(gint)(0.5+sdW),(gint)(0.5+sdH),GDK_RGB_DITHER_NONE,0,0);

		gc3 = gdk_gc_new (da->window);
		color.red = 65535;
		color.green = 0;
		color.blue = 0;
		gdk_gc_set_rgb_fg_color (gc3, &color);

		gdk_draw_arc(da->window,
					gc3,
					FALSE,
					(gint)(0.5+sXC-sRad+(100 - sdW)/2),
					(gint)(0.5+sYC-sRad+(100 - sdH)/2),
					(gint)(0.5+2*sRad),
					(gint)(0.5+2*sRad),
					0,
					64*360);	

//		printf("Radius:%f\n",rad);
//		printf("Wid:%d\n",imgPar.X_Size);

		gdk_draw_line (	da->window,
						gc3,
						(gint)(0.5+(sXC)-2+(100 - sdW)/2),
						(gint)(0.5+(sYC)  +(100 - sdH)/2),
						(gint)(0.5+(sXC)+2+(100 - sdW)/2),
						(gint)(0.5+(sYC)  +(100 - sdH)/2));

		gdk_draw_line (	da->window,
						gc3,
						(gint)(0.5+(sXC)  +(100 - sdW)/2),
						(gint)(0.5+(sYC)-2+(100 - sdH)/2),
						(gint)(0.5+(sXC)  +(100 - sdW)/2),
						(gint)(0.5+(sYC)+2+(100 - sdH)/2));


		g_object_unref (gc1);
		g_object_unref (gc2);
		g_object_unref (gc3);
		g_object_unref (pixbuf);
	}
  

  /* return TRUE because we've handled this event, so no
   * further processing is required.
   */
  return TRUE;
}


gboolean delete_dialog( GtkWidget *widget,
                              GdkEvent  *event,
                              gpointer   data )
{
    /* If you return FALSE in the "delete_event" signal handler,
     * GTK will emit the "destroy" signal. Returning TRUE means
     * you don't want the window to be destroyed.
     * This is useful for popping up 'are you sure you want to quit?'
     * type dialogs. */

//    g_print ("delete event occurred\n");

	// TODO :dovrei prendermi il puntatore ai dati e deallocarlo

	changeZorder = TRUE;

    /* Change TRUE to FALSE and the main window will be destroyed with
     * a "delete_event". */

    return TRUE;
}


gboolean
logpolar2_expose (GtkWidget	    *da,
		     GdkEventExpose *event,
		     gpointer	     data)
{

	//Draw Background
  gint i, j, xcount, ycount;
  GdkGC *gc1, *gc2;
  GdkColor color;
  
#define CHECK_SIZE 4
#define SPACING 0
  
  gc1 = gdk_gc_new (da->window);
  color.red = 32767;
  color.green = 32767;
  color.blue = 32767;
  gdk_gc_set_rgb_fg_color (gc1, &color);

  gc2 = gdk_gc_new (da->window);
  color.red = 65535;
  color.green = 65535;
  color.blue = 65535;
  gdk_gc_set_rgb_fg_color (gc2, &color);
  
  xcount = 0;
  i = SPACING;
  while (i < da->allocation.width)
    {
      j = SPACING;
      ycount = xcount % 2; /* start with even/odd depending on row */
      while (j < da->allocation.height)
	{
	  GdkGC *gc;
	  
	  if (ycount % 2)
	    gc = gc1;
	  else
	    gc = gc2;

	  gdk_draw_rectangle (da->window,
			      gc,
			      TRUE,
			      i, j,
			      CHECK_SIZE,
			      CHECK_SIZE);

	  j += CHECK_SIZE + SPACING;
	  ++ycount;
	}

      i += CHECK_SIZE + SPACING;
      ++xcount;
    }
/////////////////////////////End Draw Background

	gint height = gdk_pixbuf_get_height((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	gint width  = gdk_pixbuf_get_width ((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	gint maxSize = __max(width, height);

	gint dWidth, dHeight;

	if (maxSize == width)
	{
		dHeight = 100*height/width;
		dWidth  = 100;
	}
	else
	{
		dWidth  = 100*width/height;
		dHeight = 100;
	}

	double sXC = 50;
	double sYC = 50;
	double sdW = dWidth;
	double sdH = dHeight;

	GdkPixbuf * pixbuf = gdk_pixbuf_scale_simple ((GdkPixbuf *)(g_list_nth_data(pixbufList,0)),
							 (gint)(0.5+sdW),
							 (gint)(0.5+sdH),
							 GDK_INTERP_NEAREST);

  	gdk_draw_pixbuf((GdkDrawable *) da->window,
					NULL,
					pixbuf,
					0,0,(gint)(0.5 + (100-sdW)/2),(gint)(0.5 + (100-sdH)/2),(gint)(0.5+sdW),(gint)(0.5+sdH),GDK_RGB_DITHER_NONE,0,0);

	g_object_unref (gc1);
	g_object_unref (gc2);
	g_object_unref (pixbuf);

//	g_signal_emit_by_name (	(gpointer) da, "focus-in-event");

  /* return TRUE because we've handled this event, so no
   * further processing is required.
   */
  return TRUE;
}


gboolean
remapped_expose (GtkWidget	    *da,
				 GdkEventExpose *event,
				 gpointer	     data)
{
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (da));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));

	gint childNumb = g_list_length (children3);
 	gpointer tempWidget;
	GtkWidget * RemAlign;

	int i;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"Cartesian Align"))
			RemAlign = GTK_WIDGET(tempWidget);
	}

	GList* children4 = gtk_container_get_children (GTK_CONTAINER(RemAlign));
	GList* children5 = gtk_container_get_children (GTK_CONTAINER(children4->data));
	
	GtkWidget * dArea	= GTK_WIDGET(children5->data);//gets a pointer to Rem drawing area

	if (dArea->window !=NULL)
	{
		
		GdkGC *gc1;
		GdkColor color;

		gc1 = gdk_gc_new (dArea->window);
		color.red = 32767;
		color.green = 32767;
		color.blue = 32767;
		color.green = 0;
		color.blue = 0;
		gdk_gc_set_rgb_fg_color (gc1, &color);

		gdk_draw_rectangle (dArea->window,
				  gc1,
				  TRUE,
				  0, 0,
				  100,
				  100);

		g_object_unref (gc1);

		GdkPixbuf * pixbuf =  gdk_pixbuf_copy((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
			
		guchar * pixData;
		guchar * remImg = new guchar [imgPar.X_Size*imgPar.Y_Size*3];

		gint width   = gdk_pixbuf_get_width (pixbuf);
		gint height  = gdk_pixbuf_get_height(pixbuf);
		pixData = gdk_pixbuf_get_pixels(pixbuf);

		generate_Remapped_Image(pixbuf,imgPar.X_Size,imgPar.Y_Size,remImg);
//		printf ("XSize = %d, YSize = %d\n", imgPar.X_Size, imgPar.Y_Size);
//		printf ("ScaleFactor = %f \n\n", imgPar.scaleFactor);

		GdkPixbuf * pixbuf1 = gdk_pixbuf_new_from_data(	remImg,
											GDK_COLORSPACE_RGB,
											FALSE,
											8,
											imgPar.X_Size,
											imgPar.Y_Size,
											imgPar.X_Size*3,
											NULL,
											NULL);


		gint maxSize = __max(imgPar.X_Size, imgPar.Y_Size);

		gint dWidth, dHeight;

		if (maxSize == imgPar.X_Size)
		{
			dHeight = 100*imgPar.Y_Size/imgPar.X_Size;
			dWidth  = 100;
		}
		else
		{
			dWidth  = 100*imgPar.X_Size/imgPar.Y_Size;
			dHeight = 100;
		}

		GdkPixbuf * pixbufRem = gdk_pixbuf_scale_simple (pixbuf1,
														 dWidth,
														 dHeight,
														 GDK_INTERP_NEAREST);

  		gdk_draw_pixbuf((GdkDrawable *) dArea->window,
						NULL,
						pixbufRem,
						0,0,(100-dWidth)/2,(100-dHeight)/2,dWidth,dHeight,GDK_RGB_DITHER_NONE,0,0);

		delete [] remImg;
		g_object_unref (pixbuf);
		g_object_unref (pixbuf1);
		g_object_unref (pixbufRem);
	}

  return TRUE;
}


void lp2c_OK_clicked(GtkButton *button,
					 gpointer user_data)
{
	gint width, height;
	guchar * pixData;
	gint listLength = g_list_length (windowsList);

//	allowedAcq = FALSE;
//	while (!g_mutex_trylock(acq_mutex))
//	{
//	}
//
//	remapped = TRUE;

	guchar * cartImg = new guchar [imgPar.X_Size*imgPar.Y_Size*3];

	if (listLength>0)
	{
		GdkPixbuf * pixBuf =  gdk_pixbuf_copy((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));

		GtkWindow * wind = GTK_WINDOW(g_list_nth_data(windowsList,0));


		
		width   = gdk_pixbuf_get_width (pixBuf);
		height  = gdk_pixbuf_get_height(pixBuf);
		pixData = gdk_pixbuf_get_pixels(pixBuf);

		pixData[0] = 192;
		pixData[1] = 192;
		pixData[2] = 160;

		GtkWidget *dialog1;
		GtkWidget *dialog_vbox1;
		GtkWidget *progBar;

		dialog1 = gtk_dialog_new ();
		gtk_window_set_title (GTK_WINDOW (dialog1), _("Progress"));
		gtk_window_set_type_hint (GTK_WINDOW (dialog1), GDK_WINDOW_TYPE_HINT_DIALOG);

		dialog_vbox1 = GTK_DIALOG (dialog1)->vbox;
		gtk_widget_show (dialog_vbox1);

		progBar = gtk_progress_bar_new ();
		gtk_box_pack_start (GTK_BOX (dialog_vbox1), progBar, FALSE, FALSE, 0);
		gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);
		gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Building Table"));
		gtk_widget_show (progBar);

		gtk_widget_show (dialog1);

		while (gtk_events_pending()) gtk_main_iteration_do(FALSE);


		imagedata storedData;
		memset(&storedData, 0, sizeof(imagedata));
		gint i,j;

		gint cartSize = imgPar.X_Size*imgPar.Y_Size;

		lp2CartPixel * l2cTable     = new lp2CartPixel [(int)(2*imgPar.mapRadius)*(int)(2*imgPar.mapRadius)];
		lp2CartPixel * l2cTableProc = new lp2CartPixel [cartSize];

		for (i=0; i<cartSize; i++)
		{
			l2cTableProc[i].iweight  = 0;
//			l2cTableProc[i].position = 0;
		}

		char filename [256];
		FILE * fin;
		sprintf(filename,"%sLP2C.data",".\\");

		if ((fin = fopen(filename,"rb")) != NULL)
		{
			fread (&storedData,sizeof(imagedata),1,fin);
			fclose (fin);
		}

		if( (imgPar.rho_Size==storedData.rho_Size) && (imgPar.theta_Size==storedData.theta_Size) && ((int)(2*imgPar.mapRadius)==(int)(2*storedData.mapRadius)))
//		if( (imgPar.X_Size==storedData.X_Size) && (imgPar.Y_Size==storedData.Y_Size) && (imgPar.rho_Size==storedData.rho_Size) && (imgPar.theta_Size==storedData.theta_Size) )
		{
			RCallocateL2CTable(l2cTable,(int)(2*imgPar.mapRadius),(int)(2*imgPar.mapRadius),".\\");
		}
		else
		{
			FILE * fout;
			sprintf(filename,"%sLP2C.data",".\\");
			fout = fopen(filename,"wb");
			fwrite(&imgPar,sizeof(imagedata),1,fout);
			fclose(fout);
			RCbuildL2CMap(imgPar.rho_Size,imgPar.theta_Size,(int)(2*imgPar.mapRadius),imgPar.overlap,imgPar.scaleFactor,0,0,ELLIPTICAL,".\\");
			RCallocateL2CTable(l2cTable,(int)(2*imgPar.mapRadius),(int)(2*imgPar.mapRadius),".\\");
		}

//		generate_Remapped_Image_with_LinInt(pixBuf,imgPar.X_Size,imgPar.Y_Size,cartImg);

		gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Processing Table"));
		gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);
		gtk_widget_show (GTK_WIDGET(progBar));

		gint x,y,x1,y1;
		for (i=0; i<(int)(2*imgPar.mapRadius)*(int)(2*imgPar.mapRadius); i++)
		{
			if (!(i%(int)(2*imgPar.mapRadius)))
			{
				gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), (double)i/double((int)(2*imgPar.mapRadius)*(int)(2*imgPar.mapRadius)-1));
				gtk_widget_show (GTK_WIDGET(progBar));
			}

			while (gtk_events_pending()) gtk_main_iteration_do(FALSE);
			x = (i)%((int)(2*imgPar.mapRadius));
			y = (i)/((int)(2*imgPar.mapRadius));
			x1 = x+imgPar.xCenter-(int)(imgPar.mapRadius);
			y1 = y+imgPar.yCenter-(int)(imgPar.mapRadius);

			if ((x1>=0)&&(x1<imgPar.X_Size)&&(y1>=0)&&(y1<imgPar.Y_Size))
			{
				l2cTableProc[y1*imgPar.X_Size+x1].iweight = l2cTable[i].iweight;
				l2cTableProc[y1*imgPar.X_Size+x1].position = new gint [l2cTableProc[y1*imgPar.X_Size+x1].iweight];
				for (j=0; j<l2cTable[i].iweight; j++)
					l2cTableProc[y1*imgPar.X_Size+x1].position[j] = l2cTable[i].position[j];
			}
		}

		for (i=0; i<cartSize; i++)
		{
			if (l2cTableProc[i].iweight == 0)
			{
				l2cTableProc[i].iweight	= 1;
				l2cTableProc[i].position = new gint [1];
				l2cTableProc[i].position[0] = 0;
			}
		}

		gtk_widget_destroy (gtk_widget_get_toplevel(GTK_WIDGET(progBar)));
		RCgetCartImg(cartImg,pixData,l2cTableProc,imgPar.X_Size*imgPar.Y_Size);	

		drawImagefromMemory(cartImg,imgPar.X_Size,imgPar.Y_Size);

		//TODO delete maps;
		for (i=0; i<cartSize; i++)
			delete [] l2cTableProc[i].position;	

		delete [] l2cTableProc;				

//		for (i=0; i<(int)(2*imgPar.mapRadius)*(int)(2*imgPar.mapRadius); i++)
//			delete [] l2cTable[i].position;	

//		delete [] l2cTable;				


	}

	gtk_widget_destroy (gtk_widget_get_toplevel(GTK_WIDGET(button)));
	changeZorder = TRUE;

}

gboolean delete_toolbox	(	GtkWidget *widget,
							GdkEvent  *event,
							gpointer   data )
{
    /* If you return FALSE in the "delete_event" signal handler,
     * GTK will emit the "destroy" signal. Returning TRUE means
     * you don't want the window to be destroyed.
     * This is useful for popping up 'are you sure you want to quit?'
     * type dialogs. */


	GtkWidget * toolsToolbox = lookup_widget(GTK_WIDGET (mainMenuWindow), "toolsToolbox");
	gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(toolsToolbox),FALSE);
    /* Change TRUE to FALSE and the main window will be destroyed with
     * a "delete_event". */

    return TRUE;
}



gdouble on_zoomIn_button_clicked(GtkWidget *widget, gpointer user_data)
{
	gdouble dRatio = 1.0;
	
	if(g_list_length (windowsList)!=0)
	{
		
		GdkPixbuf * pixBuf =  gdk_pixbuf_copy((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));

		GtkWindow * wind = GTK_WINDOW(g_list_nth_data(windowsList,0));

		GtkWidget *darea;
		GtkWidget *newdarea;

		GdkPixbuf *resizedpixbuf;

		resizedpixbuf = GDK_PIXBUF(g_object_get_data (G_OBJECT (wind),"pixbuf"));

		if (resizedpixbuf != NULL)
		{
			dRatio = (gdouble)gdk_pixbuf_get_width (resizedpixbuf)/gdk_pixbuf_get_width (pixBuf);
			g_object_unref (resizedpixbuf);
		}

		if (dRatio < 1.0)
			dRatio = 1.0/((gint)(0.5+1.0/dRatio)-1.0);
		else
			dRatio = (gint)(dRatio+1.5);

		gint newWid = (gint)(gdk_pixbuf_get_width (pixBuf)*dRatio);
		gint newHei = (gint)(gdk_pixbuf_get_height (pixBuf)*dRatio);
		newdarea = gtk_drawing_area_new();

		resizedpixbuf = gdk_pixbuf_scale_simple(pixBuf,
												newWid,
												newHei,
												GDK_INTERP_NEAREST);

		gtk_window_resize (wind,newWid,newHei);		
		
		darea = GTK_WIDGET(g_object_get_data (G_OBJECT (wind),"darea"));

		gtk_container_remove (GTK_CONTAINER (wind), darea);

		gtk_drawing_area_size (GTK_DRAWING_AREA (newdarea), gdk_pixbuf_get_width(resizedpixbuf),
								gdk_pixbuf_get_height(resizedpixbuf));

		gtk_container_add (GTK_CONTAINER (wind), newdarea);

		gtk_signal_connect (GTK_OBJECT (newdarea), "expose-event",
						   GTK_SIGNAL_FUNC (on_darea_expose), resizedpixbuf);

		gtk_widget_show_all (GTK_WIDGET(wind));

		g_object_set_data (G_OBJECT (wind),"darea",newdarea);
		g_object_set_data (G_OBJECT (wind),"pixbuf",resizedpixbuf);
		g_object_unref (pixBuf);		
	}

	return dRatio;
}


gdouble on_zoomOut_button_clicked(GtkWidget *widget, gpointer user_data)
{
	gdouble dRatio = 1.0;
	
	if(g_list_length (windowsList)!=0)
	{
		GdkPixbuf * pixBuf =  gdk_pixbuf_copy((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));

		GtkWindow * wind = GTK_WINDOW(g_list_nth_data(windowsList,0));

		GtkWidget *darea;
		GtkWidget *newdarea;

		GdkPixbuf *resizedpixbuf;

		resizedpixbuf = GDK_PIXBUF(g_object_get_data (G_OBJECT (wind),"pixbuf"));

		if (resizedpixbuf != NULL)
		{
			dRatio = (gdouble)gdk_pixbuf_get_width (resizedpixbuf)/gdk_pixbuf_get_width (pixBuf);
			g_object_unref (resizedpixbuf);
		}

		if (dRatio <= 1.0)
			dRatio = 1.0/((gint)(0.5+1.0/dRatio)+1.0);
		else
			dRatio = (gint)(dRatio-0.5);

		gint newWid = (gint)(gdk_pixbuf_get_width (pixBuf)*dRatio);
		gint newHei = (gint)(gdk_pixbuf_get_height (pixBuf)*dRatio);
		newdarea = gtk_drawing_area_new();

		resizedpixbuf = gdk_pixbuf_scale_simple(pixBuf,
												newWid,
												newHei,
												GDK_INTERP_NEAREST);


		gtk_window_resize (wind,newWid,newHei);		
		
		darea = GTK_WIDGET(g_object_get_data (G_OBJECT (wind),"darea"));

		gtk_container_remove (GTK_CONTAINER (wind), darea);

		gtk_drawing_area_size (GTK_DRAWING_AREA (newdarea), gdk_pixbuf_get_width(resizedpixbuf),
								gdk_pixbuf_get_height(resizedpixbuf));

		gtk_container_add (GTK_CONTAINER (wind), newdarea);

		gtk_signal_connect (GTK_OBJECT (newdarea), "expose-event",
						   GTK_SIGNAL_FUNC (on_darea_expose), resizedpixbuf);

		gtk_widget_show_all (GTK_WIDGET(wind));

		g_object_set_data (G_OBJECT (wind),"darea",newdarea);
		g_object_set_data (G_OBJECT (wind),"pixbuf",resizedpixbuf);
		g_object_unref (pixBuf);
	}

	return dRatio;
}


void on_mirror_button_clicked(GtkWidget *widget, gpointer user_data)
{
	gdouble dRatio = 1.0;

	if(g_list_length (windowsList)!=0)
	{
		GtkWindow * wind = GTK_WINDOW(g_list_nth_data(windowsList,0));

		GtkWidget *darea;
		GtkWidget *newdarea;

		GdkPixbuf *mirroredpixbuf;
		GdkPixbuf *resizedpixbuf;

		mirroredpixbuf = GDK_PIXBUF(g_object_get_data (G_OBJECT (wind),"pixbuf"));

		GdkPixbuf *pixBuf = (GdkPixbuf *)(g_list_nth_data(pixbufList,0));

		if (mirroredpixbuf != NULL)
		{
			dRatio = (gdouble)gdk_pixbuf_get_width (mirroredpixbuf)/gdk_pixbuf_get_width (pixBuf);
			g_object_unref (mirroredpixbuf);
		}

		mirroredpixbuf = gdk_pixbuf_flip(pixBuf, TRUE);

		pixbufList  = g_list_remove(pixbufList ,(gconstpointer) pixBuf);
		pixbufList  = g_list_prepend(pixbufList ,(gpointer) mirroredpixbuf);

		gint newWid = (gint)(gdk_pixbuf_get_width (mirroredpixbuf)*dRatio);
		gint newHei = (gint)(gdk_pixbuf_get_height (mirroredpixbuf)*dRatio);

		resizedpixbuf = gdk_pixbuf_scale_simple(mirroredpixbuf,
												newWid,
												newHei,
												GDK_INTERP_NEAREST);

		gtk_window_resize (wind,newWid,newHei);		

		newdarea = gtk_drawing_area_new();
		darea = GTK_WIDGET(g_object_get_data (G_OBJECT (wind),"darea"));
		gtk_container_remove (GTK_CONTAINER (wind), darea);
		gtk_drawing_area_size (GTK_DRAWING_AREA (newdarea), gdk_pixbuf_get_width(resizedpixbuf),
								gdk_pixbuf_get_height(resizedpixbuf));
		gtk_container_add (GTK_CONTAINER (wind), newdarea);

		gtk_signal_connect (GTK_OBJECT (newdarea), "expose-event",
						   GTK_SIGNAL_FUNC (on_darea_expose), resizedpixbuf);
		gtk_widget_show_all (GTK_WIDGET(wind));
		g_object_set_data (G_OBJECT (wind),"darea",newdarea);
		g_object_set_data (G_OBJECT (wind),"pixbuf",resizedpixbuf);
	}
}


void on_flip_button_clicked(GtkWidget *widget, gpointer user_data)
{
	gdouble dRatio = 1.0;

	if(g_list_length (windowsList)!=0)
	{
		GtkWindow * wind = GTK_WINDOW(g_list_nth_data(windowsList,0));

		GtkWidget *darea;
		GtkWidget *newdarea;

		GdkPixbuf *mirroredpixbuf;
		GdkPixbuf *resizedpixbuf;

		mirroredpixbuf = GDK_PIXBUF(g_object_get_data (G_OBJECT (wind),"pixbuf"));

		GdkPixbuf *pixBuf = (GdkPixbuf *)(g_list_nth_data(pixbufList,0));

		if (mirroredpixbuf != NULL)
		{
			dRatio = (gdouble)gdk_pixbuf_get_width (mirroredpixbuf)/gdk_pixbuf_get_width (pixBuf);
			g_object_unref (mirroredpixbuf);
		}

		mirroredpixbuf = gdk_pixbuf_flip(pixBuf, FALSE);

		pixbufList  = g_list_remove(pixbufList ,(gconstpointer) pixBuf);
		pixbufList  = g_list_prepend(pixbufList ,(gpointer) mirroredpixbuf);

		gint newWid = (gint)(gdk_pixbuf_get_width (mirroredpixbuf)*dRatio);
		gint newHei = (gint)(gdk_pixbuf_get_height (mirroredpixbuf)*dRatio);

		resizedpixbuf = gdk_pixbuf_scale_simple(mirroredpixbuf,
												newWid,
												newHei,
												GDK_INTERP_NEAREST);


		gtk_window_resize (wind,newWid,newHei);		

		newdarea = gtk_drawing_area_new();
		darea = GTK_WIDGET(g_object_get_data (G_OBJECT (wind),"darea"));
		gtk_container_remove (GTK_CONTAINER (wind), darea);
		gtk_drawing_area_size (GTK_DRAWING_AREA (newdarea), gdk_pixbuf_get_width(resizedpixbuf),
								gdk_pixbuf_get_height(resizedpixbuf));
		gtk_container_add (GTK_CONTAINER (wind), newdarea);

		gtk_signal_connect (GTK_OBJECT (newdarea), "expose-event",
						   GTK_SIGNAL_FUNC (on_darea_expose), resizedpixbuf);
		gtk_widget_show_all (GTK_WIDGET(wind));
		g_object_set_data (G_OBJECT (wind),"darea",newdarea);
		g_object_set_data (G_OBJECT (wind),"pixbuf",resizedpixbuf);
	}
}