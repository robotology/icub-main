#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <string.h>
#include <stdio.h>

#include <gdk/gdkkeysyms.h>
#include <gtk/gtk.h>

#include <stdlib.h>
#include <math.h>

#include "support.h"
#include "callbacks.h"
#include "callbacksC2LP.h"
#include "callbacksLP2C.h"
#include "interface.h"
#include "functions.h"


#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)

extern GList * windowsList;
extern GList * pixbufList;
extern imagedata imgPar;
extern GtkWidget *mainMenuWindow;

void initPar()
{
	imgPar.rho_Size = 152;
	imgPar.theta_Size = 252;
	imgPar.X_Size = 256;
	imgPar.Y_Size = 256;
	imgPar.log_index = (1.0+sin(PI/imgPar.theta_Size))/(1.0-sin(PI/imgPar.theta_Size));
	imgPar.fovea = (int)(imgPar.log_index/(imgPar.log_index-1));
	imgPar.r_zero = 1.0/(pow(imgPar.log_index,imgPar.fovea)*(imgPar.log_index-1));
	imgPar.scaleFactor  = __min(imgPar.X_Size,imgPar.Y_Size);
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));
	imgPar.xCenter = 128;
	imgPar.yCenter = 128;
	imgPar.mapRadius = 128.0;
	imgPar.mapInside = 1;
	imgPar.overlap = 1.0;

}

GtkWidget* create_main_menu_window ()
{
	GtkWidget *window;
	GtkWidget *menuBar;

	GtkWidget *fileMenuItem;
	GtkWidget *fileMenu;
	GtkWidget *fileOpen;
	GtkWidget *fileSave;
	GtkWidget *fileMenuSeparator;
	GtkWidget *fileQuit;

	GtkWidget *toolsMenuItem;
	GtkWidget *toolsMenu;
	GtkWidget *toolsC2Lp;
	GtkWidget *toolsLp2C;
	GtkWidget *toolsToolbox;

	GtkAccelGroup *AccelGroup;

	AccelGroup = gtk_accel_group_new ();

	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title (GTK_WINDOW (window), _("Log Polar Studio"));
	gtk_widget_set_name(window,"Menu Window");

	menuBar = gtk_menu_bar_new ();
	gtk_widget_show (menuBar);
	gtk_container_add (GTK_CONTAINER (window), menuBar);
	gtk_widget_set_name(menuBar,"Menu Bar");

	fileMenuItem = gtk_menu_item_new_with_mnemonic (_("_File"));
	gtk_widget_show (fileMenuItem);
	gtk_container_add (GTK_CONTAINER (menuBar), fileMenuItem);

	fileMenu = gtk_menu_new ();
	gtk_menu_item_set_submenu (GTK_MENU_ITEM (fileMenuItem), fileMenu);

	fileOpen = gtk_image_menu_item_new_from_stock ("gtk-open", AccelGroup);
	gtk_widget_show (fileOpen);
	gtk_container_add (GTK_CONTAINER (fileMenu), fileOpen);

	fileSave = gtk_image_menu_item_new_from_stock ("gtk-save", AccelGroup);
	gtk_widget_show (fileSave);
	gtk_container_add (GTK_CONTAINER (fileMenu), fileSave);

	fileMenuSeparator = gtk_separator_menu_item_new ();
	gtk_widget_show (fileMenuSeparator);
	gtk_container_add (GTK_CONTAINER (fileMenu), fileMenuSeparator);
	gtk_widget_set_sensitive (fileMenuSeparator, FALSE);

	fileQuit = gtk_image_menu_item_new_from_stock ("gtk-quit", AccelGroup);
	gtk_widget_show (fileQuit);
	gtk_container_add (GTK_CONTAINER (fileMenu), fileQuit);

	toolsMenuItem = gtk_menu_item_new_with_mnemonic (_("_Tools"));
	gtk_widget_show (toolsMenuItem);
	gtk_container_add (GTK_CONTAINER (menuBar), toolsMenuItem);

	toolsMenu = gtk_menu_new ();
	gtk_menu_item_set_submenu (GTK_MENU_ITEM (toolsMenuItem), toolsMenu);

	toolsC2Lp = gtk_image_menu_item_new_with_mnemonic ("Cartesian to Log Polar ...");
	gtk_widget_show (toolsC2Lp);
	gtk_container_add (GTK_CONTAINER (toolsMenu), toolsC2Lp);

	GtkWidget * c2LpImage = gtk_image_new_from_file ("C2Lp3.gif");
	gtk_widget_show (c2LpImage);
	gtk_image_menu_item_set_image   (GTK_IMAGE_MENU_ITEM(toolsC2Lp), c2LpImage);

	
	toolsLp2C = gtk_image_menu_item_new_with_mnemonic ("Log Polar to Cartesian ...");
	gtk_widget_show (toolsLp2C);
	gtk_container_add (GTK_CONTAINER (toolsMenu), toolsLp2C);

	GtkWidget * lp2CImage = gtk_image_new_from_file ("Lp2C3.gif");
	gtk_widget_show (lp2CImage);
	gtk_image_menu_item_set_image   (GTK_IMAGE_MENU_ITEM(toolsLp2C), lp2CImage);

	toolsToolbox = gtk_check_menu_item_new_with_label  ("Toolbox ...");//, AccelGroup);
	gtk_widget_show (toolsToolbox);
	gtk_container_add (GTK_CONTAINER (toolsMenu), toolsToolbox);

	///////////////
	// Callbacks //
	///////////////

	g_signal_connect ((gpointer) window, "focus_in_event",
					G_CALLBACK (on_menu_window_activation),
					NULL);

	g_signal_connect ((gpointer) fileOpen, "activate",
					G_CALLBACK (on_file_open_activate),
					NULL);

	g_signal_connect ((gpointer) fileSave, "activate",
					G_CALLBACK (on_file_save_activate),
					NULL);

	g_signal_connect ((gpointer) fileQuit, "activate",
					G_CALLBACK (on_file_quit_activate),
					NULL);

	g_signal_connect ((gpointer) toolsC2Lp, "activate",
					G_CALLBACK (on_tools_c2lp_activate),
					NULL);

	g_signal_connect ((gpointer) toolsLp2C, "activate",
					G_CALLBACK (on_tools_lp2c_activate),
					NULL);

	g_signal_connect ((gpointer) toolsToolbox, "activate",
					G_CALLBACK (on_tools_toolbox_activate),
					NULL);
	
	// Store pointers to all widgets, for use by lookup_widget()
	GLADE_HOOKUP_OBJECT_NO_REF (window, window, "window");
	GLADE_HOOKUP_OBJECT (window, menuBar, "menuBar");
	GLADE_HOOKUP_OBJECT (window, fileMenuItem, "fileMenuItem");
	GLADE_HOOKUP_OBJECT (window, fileMenu, "fileMenu");
	GLADE_HOOKUP_OBJECT (window, fileOpen, "fileOpen");
	GLADE_HOOKUP_OBJECT (window, fileOpen, "fileOpenCamera");
	GLADE_HOOKUP_OBJECT (window, fileSave, "fileSave");
	GLADE_HOOKUP_OBJECT (window, fileMenuSeparator, "fileMenuSeparator");
	GLADE_HOOKUP_OBJECT (window, fileQuit, "fileQuit");
	GLADE_HOOKUP_OBJECT (window, toolsMenuItem, "toolsMenuItem");
	GLADE_HOOKUP_OBJECT (window, toolsMenu, "toolsMenu");
	GLADE_HOOKUP_OBJECT (window, toolsC2Lp, "toolsC2Lp");
	GLADE_HOOKUP_OBJECT (window, toolsLp2C, "toolsLp2C");
	GLADE_HOOKUP_OBJECT (window, toolsToolbox, "toolsToolbox");

	gtk_window_add_accel_group (GTK_WINDOW (window), AccelGroup);
	GtkRequisition winSize;

	gtk_widget_size_request (window,&winSize);

	gtk_widget_set_size_request (window, 176, winSize.height );

	return window;
}

GtkWidget* create_filechooserdialog ()
{
	GtkWidget *filechooserdialog;
	GtkWidget *dialog_vbox1;
	GtkWidget *dialog_action_area1;
	GtkWidget *cancelButton;
	GtkWidget *openButton;

	filechooserdialog = gtk_file_chooser_dialog_new (	"File Open", 
														NULL, 
														GTK_FILE_CHOOSER_ACTION_OPEN,
														NULL);

	gtk_window_set_modal (GTK_WINDOW (filechooserdialog), TRUE);
	gtk_window_set_destroy_with_parent (GTK_WINDOW (filechooserdialog), TRUE);

	gtk_window_set_type_hint (GTK_WINDOW (filechooserdialog), GDK_WINDOW_TYPE_HINT_DIALOG);
	gtk_window_set_gravity (GTK_WINDOW (filechooserdialog), GDK_GRAVITY_CENTER);

	dialog_vbox1 = GTK_DIALOG (filechooserdialog)->vbox;
	gtk_widget_show (dialog_vbox1);

	dialog_action_area1 = GTK_DIALOG (filechooserdialog)->action_area;
	gtk_widget_show (dialog_action_area1);
	gtk_button_box_set_layout (GTK_BUTTON_BOX (dialog_action_area1), GTK_BUTTONBOX_END);

	cancelButton = gtk_button_new_from_stock ("gtk-cancel");
	gtk_widget_show (cancelButton);
	gtk_dialog_add_action_widget (GTK_DIALOG (filechooserdialog), cancelButton, GTK_RESPONSE_CANCEL);
	GTK_WIDGET_SET_FLAGS (cancelButton, GTK_CAN_DEFAULT);

	openButton = gtk_button_new_from_stock ("gtk-open");
	gtk_widget_show (openButton);
	gtk_dialog_add_action_widget (GTK_DIALOG (filechooserdialog), openButton, GTK_RESPONSE_OK);
	GTK_WIDGET_SET_FLAGS (openButton, GTK_CAN_DEFAULT);

	/* Store pointers to all widgets, for use by lookup_widget(). */
	GLADE_HOOKUP_OBJECT_NO_REF (filechooserdialog, filechooserdialog, "filechooserdialog");
	GLADE_HOOKUP_OBJECT_NO_REF (filechooserdialog, dialog_vbox1, "dialog_vbox1");
	GLADE_HOOKUP_OBJECT_NO_REF (filechooserdialog, dialog_action_area1, "dialog_action_area1");
	GLADE_HOOKUP_OBJECT (filechooserdialog, cancelButton, "cancelButton");
	GLADE_HOOKUP_OBJECT (filechooserdialog, openButton, "openButton");

	gtk_widget_grab_default (openButton);
	return filechooserdialog;
}


GtkWidget* create_filesavedialog ()
{
	GtkWidget *filechooserdialog;
	GtkWidget *dialog_vbox;
	GtkWidget *dialog_action_area;
	GtkWidget *cancelButton;
	GtkWidget *saveButton;
	GtkFileFilter *fileFilter;

	filechooserdialog = gtk_file_chooser_dialog_new (	"File Save", 
														NULL, 
														GTK_FILE_CHOOSER_ACTION_SAVE,
														NULL);

	gtk_file_chooser_set_do_overwrite_confirmation (GTK_FILE_CHOOSER (filechooserdialog), TRUE);


	fileFilter = gtk_file_filter_new();
	gtk_file_filter_set_name (fileFilter,	_("Bitmaps"));

	gtk_file_filter_add_pattern(fileFilter,"*.bmp");
	gtk_file_chooser_add_filter(GTK_FILE_CHOOSER (filechooserdialog),fileFilter);

	fileFilter = gtk_file_filter_new();
	gtk_file_filter_set_name (fileFilter,	_("Jpeg"));
	gtk_file_filter_add_pattern(fileFilter,"*.jpg");
	gtk_file_chooser_add_filter(GTK_FILE_CHOOSER (filechooserdialog),fileFilter);

//	gtk_file_chooser_set_current_folder (GTK_FILE_CHOOSER (filechooserdialog), "C:/Users/Fabio/Projects/Samples/gtk test1");

	GtkFileFilter * currFilter = gtk_file_chooser_get_filter  (GTK_FILE_CHOOSER (filechooserdialog));

	const gchar* filtername = gtk_file_filter_get_name (currFilter);
	const gchar* bmpFilter = "Bitmaps";

	gchar fullname [256];
	sprintf(fullname,"%s.bmp",gtk_window_get_title(GTK_WINDOW(g_list_nth_data(windowsList,0))));

	if (!strcmp (filtername,bmpFilter))
	{
		sprintf(fullname,"%s.bmp",gtk_window_get_title(GTK_WINDOW(g_list_nth_data(windowsList,0))));
		gtk_file_chooser_set_current_name (GTK_FILE_CHOOSER (filechooserdialog), fullname);
	}
	else 
	{
		sprintf(fullname,"%s.jpg",gtk_window_get_title(GTK_WINDOW(g_list_nth_data(windowsList,0))));
		gtk_file_chooser_set_current_name (GTK_FILE_CHOOSER (filechooserdialog), fullname);
	}

	gtk_window_set_modal (GTK_WINDOW (filechooserdialog), TRUE);
	gtk_window_set_destroy_with_parent (GTK_WINDOW (filechooserdialog), TRUE);

	gtk_window_set_type_hint (GTK_WINDOW (filechooserdialog), GDK_WINDOW_TYPE_HINT_DIALOG);
	gtk_window_set_gravity (GTK_WINDOW (filechooserdialog), GDK_GRAVITY_CENTER);

	dialog_vbox = GTK_DIALOG (filechooserdialog)->vbox;
	gtk_widget_show (dialog_vbox);

	GList* childList = gtk_container_get_children (GTK_CONTAINER(dialog_vbox));//children[0]=>DialogVBox
	GtkFileChooserWidget * fileChooWid = (GtkFileChooserWidget *)(childList->data);
	GList* grandChildList = gtk_container_get_children (GTK_CONTAINER(fileChooWid));//children[0]=>DialogVBox

	dialog_action_area = GTK_DIALOG (filechooserdialog)->action_area;
	gtk_widget_show (dialog_action_area);
	gtk_button_box_set_layout (GTK_BUTTON_BOX (dialog_action_area), GTK_BUTTONBOX_END);

	cancelButton = gtk_button_new_from_stock ("gtk-cancel");
	gtk_widget_show (cancelButton);
	gtk_dialog_add_action_widget (GTK_DIALOG (filechooserdialog), cancelButton, GTK_RESPONSE_CANCEL);
	GTK_WIDGET_SET_FLAGS (cancelButton, GTK_CAN_DEFAULT);

	saveButton = gtk_button_new_from_stock ("gtk-save");
	gtk_widget_show (saveButton);
	gtk_dialog_add_action_widget (GTK_DIALOG (filechooserdialog), saveButton, GTK_RESPONSE_OK);
	GTK_WIDGET_SET_FLAGS (saveButton, GTK_CAN_DEFAULT);

	g_signal_connect ((gpointer) grandChildList->data, "notify",
					G_CALLBACK (on_ext_changed),
					gpointer(filechooserdialog));

//	Store pointers to all widgets, for use by lookup_widget(). 
	GLADE_HOOKUP_OBJECT_NO_REF (filechooserdialog, filechooserdialog, "filechooserdialog");
	GLADE_HOOKUP_OBJECT_NO_REF (filechooserdialog, dialog_vbox, "dialog_vbox");
	GLADE_HOOKUP_OBJECT_NO_REF (filechooserdialog, dialog_action_area, "dialog_action_area");
	GLADE_HOOKUP_OBJECT (filechooserdialog, cancelButton, "cancelButton");
	GLADE_HOOKUP_OBJECT (filechooserdialog, saveButton, "saveButton");

	gtk_widget_grab_default (saveButton);
	return filechooserdialog;
}


GtkWidget*
create_cart2lp_dialog ()
{
	GtkWidget *dialog;
	GtkWidget *dialog_vbox;
	GtkWidget *table;

	GtkWidget *da;
	GtkWidget *daLP;

	GtkWidget *rhoSizeEntry;
	GtkWidget *thetaSizeEntry;
	GtkWidget *xSizeEntry;
	GtkWidget *ySizeEntry;
	GtkWidget *lambdaEntry;
	GtkWidget *foveaSizeEntry;
	GtkWidget *rhoZeroEntry;
	GtkWidget *scaleFactorEntryEdit;
	GtkWidget *scaleFactorEntryNoEdit;
	GtkWidget *xCenterEntry;
	GtkWidget *yCenterEntry;

	GtkWidget *rhoSizeLabel;
	GtkWidget *thetaSizeLabel;
	GtkWidget *xSizeLabel;
	GtkWidget *ySizeLabel;
	GtkWidget *lambdaLabel;
	GtkWidget *foveaSizeLabel;
	GtkWidget *rhoZeroLabel;
	GtkWidget *scaleFactorLabel;
	GtkWidget *xCenterLabel;
	GtkWidget *yCenterLabel;

	GtkWidget *CartLabel;
	GtkWidget *LogPLabel;

	GtkWidget *radLabel;
	GtkWidget *radEntry;

	GtkWidget *resetButton;
//	GtkWidget *mapCircleToggleButton;

	GtkWidget *dialog_action_area;
	GtkWidget *cancelbutton;
	GtkWidget *okbutton;

	dialog = gtk_dialog_new ();

	gtk_window_set_title (GTK_WINDOW (dialog), _("Cartesian to Log Polar Transform"));
	gtk_window_set_type_hint (GTK_WINDOW (dialog), GDK_WINDOW_TYPE_HINT_DIALOG);

	dialog_vbox = GTK_DIALOG (dialog)->vbox;
	gtk_widget_show (dialog_vbox);

	table = gtk_table_new (7, 4, FALSE);
	gtk_widget_show (table);
	gtk_box_pack_start (GTK_BOX (dialog_vbox), table, TRUE, TRUE, 0);

	//Cartesian View
	GtkWidget * frame = gtk_frame_new (NULL);
	gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);

	GtkWidget * align = gtk_alignment_new (0.5, 0.5, 0, 0);
	gtk_widget_set_name(align,"Cartesian Align");	
	gtk_container_add (GTK_CONTAINER (align), frame);
	gtk_widget_set_size_request (frame, 104, 104);
	gtk_table_attach (GTK_TABLE (table), align, 1, 2, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	da = gtk_drawing_area_new ();
	gtk_widget_set_name(da,"Cartesian Drawing Area");	

	gtk_container_add (GTK_CONTAINER (frame), da);
	gtk_widget_show (da);

	gtk_widget_show (frame);
	gtk_widget_show (align);

	// "Cartesian" Label
	CartLabel = gtk_label_new (_("Cartesian Image"));
	gtk_widget_show (CartLabel);
	gtk_table_attach (GTK_TABLE (table), CartLabel, 1, 2, 6, 7,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	//Log Polar View
	GtkWidget * l2cFrameLp = gtk_frame_new (NULL);
	gtk_frame_set_shadow_type (GTK_FRAME (l2cFrameLp), GTK_SHADOW_IN);

	GtkWidget * alignLP = gtk_alignment_new (0.5, 0.5, 0, 0);
	gtk_widget_set_name(alignLP,"LP Align");	
	gtk_container_add (GTK_CONTAINER (alignLP), l2cFrameLp);
	gtk_widget_set_size_request (l2cFrameLp, 104, 104);
	gtk_table_attach (GTK_TABLE (table), alignLP, 2, 3, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	daLP = gtk_drawing_area_new ();
	gtk_widget_set_name(daLP,"LP Drawing Area");	

	gtk_container_add (GTK_CONTAINER (l2cFrameLp), daLP);
	gtk_widget_show (daLP);

	gtk_widget_show (l2cFrameLp);
	gtk_widget_show (alignLP);

	// "LogPolar" Label
	LogPLabel = gtk_label_new (_("Log Polar Image"));
	gtk_widget_show (LogPLabel);
	gtk_table_attach (GTK_TABLE (table), LogPLabel, 2, 3, 6, 7,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Reset Button
	resetButton = gtk_button_new_with_label(_("Reset Center"));
	gtk_widget_set_name(resetButton,"Reset Button");	
	gtk_widget_show (resetButton);
	gtk_table_attach (GTK_TABLE (table), resetButton, 3, 4, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);
	
	// X Size Entry (gets its value from current image)
	gint xSize;
	xSize = gdk_pixbuf_get_width((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	imgPar.X_Size = xSize;
	imgPar.xCenter = xSize/2;

	gchar xSizeString[5];
	sprintf(xSizeString,"%d", xSize);
	xSizeEntry = gtk_label_new (_(xSizeString));
	gtk_widget_set_name(xSizeEntry,"X Size Entry");	
	gtk_widget_show (xSizeEntry);
	gtk_table_attach (GTK_TABLE (table), xSizeEntry, 0, 1, 0, 1,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);


	// X Size Label
	xSizeLabel = gtk_label_new (_("Width"));
	gtk_widget_show (xSizeLabel);
	gtk_table_attach (GTK_TABLE (table), xSizeLabel, 1, 2, 0, 1,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Y Size Entry
	gint ySize;
	ySize = gdk_pixbuf_get_height((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	imgPar.Y_Size = ySize;
	imgPar.yCenter = ySize/2;

	gchar ySizeString[5];
	sprintf(ySizeString,"%d", ySize);
	ySizeEntry = gtk_label_new (_(ySizeString));
	gtk_widget_set_name(ySizeEntry,"Y Size Entry");	
	gtk_widget_show (ySizeEntry);
	gtk_table_attach (GTK_TABLE (table), ySizeEntry, 0, 1, 1, 2,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Y Size Label
	ySizeLabel = gtk_label_new (_("Height"));
	gtk_widget_show (ySizeLabel);
	gtk_table_attach (GTK_TABLE (table), ySizeLabel, 1, 2, 1, 2,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Size Entry
	rhoSizeEntry = gtk_entry_new ();
	gtk_widget_set_name(rhoSizeEntry,"Rho Size Entry");	
	gchar rings[5];
	sprintf(rings,"%d", imgPar.rho_Size);
	gtk_entry_set_width_chars(GTK_ENTRY(rhoSizeEntry),4);
	gtk_entry_set_text(GTK_ENTRY(rhoSizeEntry),rings);
	gtk_widget_show (rhoSizeEntry);
	gtk_table_attach (GTK_TABLE (table), rhoSizeEntry, 0, 1, 2, 3,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Size Label
	rhoSizeLabel = gtk_label_new (_("Rings"));
	gtk_widget_show (rhoSizeLabel);
	gtk_table_attach (GTK_TABLE (table), rhoSizeLabel, 1, 2, 2, 3,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Theta Size Entry
	thetaSizeEntry = gtk_entry_new ();
	gtk_widget_set_name(thetaSizeEntry,"Theta Size Entry");	
	gchar ppring[5];
	sprintf(ppring,"%d", imgPar.theta_Size);
	gtk_entry_set_width_chars(GTK_ENTRY(thetaSizeEntry),4);
	gtk_entry_set_text(GTK_ENTRY(thetaSizeEntry),ppring);
	gtk_widget_show (thetaSizeEntry);
	gtk_table_attach (GTK_TABLE (table), thetaSizeEntry, 0, 1, 3, 4,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Theta Size Label
	thetaSizeLabel = gtk_label_new (_("Pixels per Ring"));
	gtk_widget_show (thetaSizeLabel);
	gtk_table_attach(GTK_TABLE (table), thetaSizeLabel, 1, 2, 3, 4,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Lambda Entry
//	lambdaEntry = gtk_entry_new ();
	gchar lambdaString[8];
	sprintf(lambdaString,"%1.4f", imgPar.log_index);
	lambdaEntry = gtk_label_new (_(lambdaString));
	gtk_widget_set_name(lambdaEntry,"Lambda Entry");	
	gtk_widget_show (lambdaEntry);
	gtk_table_attach (GTK_TABLE (table), lambdaEntry, 2, 3, 0, 1,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Lambda Label
	lambdaLabel = gtk_label_new (_("Log Base"));
	gtk_widget_show (lambdaLabel);
	gtk_table_attach (GTK_TABLE (table), lambdaLabel, 3, 4, 0, 1,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Fovea Size Entry
//	foveaSizeEntry = gtk_entry_new ();
	gchar foveaSizeString[5];
	sprintf(foveaSizeString,"%d", imgPar.fovea);
	foveaSizeEntry = gtk_label_new (_(foveaSizeString));
	gtk_widget_set_name(foveaSizeEntry,"Fovea Size Entry");	
	gtk_widget_show (foveaSizeEntry);
	gtk_table_attach (GTK_TABLE (table), foveaSizeEntry, 2, 3, 1, 2,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Fovea Size Label
	foveaSizeLabel = gtk_label_new (_("Fovea Rings"));
	gtk_widget_show (foveaSizeLabel);
	gtk_table_attach (GTK_TABLE (table), foveaSizeLabel, 3, 4, 1, 2,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Zero Entry
//	rhoZeroEntry = gtk_entry_new ();

	gchar rhoZeroString[8];
	sprintf(rhoZeroString,"%1.4f", imgPar.r_zero);
	rhoZeroEntry = gtk_label_new (_(rhoZeroString));
	gtk_widget_set_name(rhoZeroEntry,"Rho Zero Entry");	
	gtk_widget_show (rhoZeroEntry);
	gtk_table_attach (GTK_TABLE (table), rhoZeroEntry, 2, 3, 2, 3,
					(GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Zero Label
	rhoZeroLabel = gtk_label_new (_("Rho Zero"));
	gtk_widget_show (rhoZeroLabel);
	gtk_table_attach (GTK_TABLE (table), rhoZeroLabel, 3, 4, 2, 3,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Scale Factor Entry
	imgPar.scaleFactor  = __min(imgPar.X_Size,imgPar.Y_Size);
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

	scaleFactorEntryEdit   = gtk_entry_new ();
	gtk_widget_set_name(scaleFactorEntryEdit,"Scale Factor Entry Edit");	

	gchar scalefactorString[11];
	sprintf(scalefactorString,"%1.8f", imgPar.scaleFactor);
	gtk_entry_set_width_chars(GTK_ENTRY(scaleFactorEntryEdit),8);
	gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scalefactorString);
	scaleFactorEntryNoEdit = gtk_label_new (_(scalefactorString));
	gtk_widget_set_name(scaleFactorEntryNoEdit,"Scale Factor Entry No Edit");	

	gtk_widget_hide (scaleFactorEntryEdit);
	gtk_widget_show (scaleFactorEntryNoEdit);

	gtk_table_attach (GTK_TABLE (table), scaleFactorEntryEdit, 2, 3, 3, 4,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	gtk_table_attach (GTK_TABLE (table), scaleFactorEntryNoEdit, 2, 3, 3, 4,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Scale Factor Label
	scaleFactorLabel = gtk_label_new (_("Scale Factor"));
	gtk_widget_show (scaleFactorLabel);
	gtk_table_attach (GTK_TABLE (table), scaleFactorLabel, 3, 4, 3, 4,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// X Center Entry
	xCenterEntry = gtk_entry_new ();
	gtk_widget_set_name(xCenterEntry,"X Center Entry");	
	gchar xCenter[5];
	sprintf(xCenter,"%d", imgPar.xCenter);
	gtk_entry_set_width_chars(GTK_ENTRY(xCenterEntry),4);
	gtk_entry_set_text(GTK_ENTRY(xCenterEntry),xCenter);
	gtk_widget_show (xCenterEntry);
	gtk_table_attach (GTK_TABLE (table), xCenterEntry, 0, 1, 4, 5,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// X Center Label
	xCenterLabel = gtk_label_new (_("Horizontal Center"));
	gtk_widget_show (xCenterLabel);
	gtk_table_attach (GTK_TABLE (table), xCenterLabel, 1, 2, 4, 5,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);
	
	// Y Center Entry
	yCenterEntry = gtk_entry_new ();
	gtk_widget_set_name(yCenterEntry,"Y Center Entry");	
	gchar yCenter[5];
	sprintf(yCenter,"%d", imgPar.yCenter);
	gtk_entry_set_width_chars(GTK_ENTRY(yCenterEntry),4);
	gtk_entry_set_text(GTK_ENTRY(yCenterEntry),yCenter);
	gtk_widget_show (yCenterEntry);
	gtk_table_attach (GTK_TABLE (table), yCenterEntry, 2, 3, 4, 5,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Y Center Label
	yCenterLabel = gtk_label_new (_("Vertical Center"));
	gtk_widget_show (yCenterLabel);
	gtk_table_attach (GTK_TABLE (table), yCenterLabel, 3, 4, 4, 5,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);
	

	// Radius Entry and Label

	GtkWidget * horbox = gtk_hbox_new (TRUE, 2);

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	radEntry = gtk_label_new (_(radiusString));
	gtk_widget_set_name(horbox,"Radius Box");	
	gtk_widget_set_name(radEntry,"Radius Entry");	
	gtk_widget_show (radEntry);

//	printf("Rad entry %x\n", radEntry);

	radLabel = gtk_label_new (_("Map Radius"));
	gtk_widget_show (radLabel);

	gtk_box_pack_start (GTK_BOX (horbox), radEntry, TRUE, TRUE, 2);
	gtk_box_pack_start (GTK_BOX (horbox), radLabel, TRUE, TRUE, 2);
	gtk_table_attach (GTK_TABLE (table), horbox, 0, 1, 6, 7,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	gtk_widget_show_all (horbox);

	//Radio Buttons
	
	GtkWidget *radio1, *radio2, *radio3, *box;
	
	box = gtk_vbox_new (TRUE, 4);
   
	radio1 = gtk_radio_button_new_with_label (NULL,"Circle inside of the box");
	radio2 = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radio1),
							"Circle outside of the box");
	radio3 = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radio1),
							"Custom");

	gtk_box_pack_start (GTK_BOX (box), radio1, TRUE, TRUE, 2);
	gtk_box_pack_start (GTK_BOX (box), radio2, TRUE, TRUE, 2);
	gtk_box_pack_start (GTK_BOX (box), radio3, TRUE, TRUE, 2);
	gtk_table_attach (GTK_TABLE (table), box, 0, 1, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	gtk_widget_show_all (box);
	
	dialog_action_area = GTK_DIALOG (dialog)->action_area;
	gtk_widget_show (dialog_action_area);
	gtk_button_box_set_layout (GTK_BUTTON_BOX (dialog_action_area), GTK_BUTTONBOX_END);

	cancelbutton = gtk_button_new_from_stock ("gtk-cancel");
	gtk_widget_show (cancelbutton);
	gtk_dialog_add_action_widget (GTK_DIALOG (dialog), cancelbutton, GTK_RESPONSE_CANCEL);
	GTK_WIDGET_SET_FLAGS (cancelbutton, GTK_CAN_DEFAULT);

	okbutton = gtk_button_new_from_stock ("gtk-ok");
	gtk_widget_show (okbutton);
	gtk_dialog_add_action_widget (GTK_DIALOG (dialog), okbutton, GTK_RESPONSE_OK);
	GTK_WIDGET_SET_FLAGS (okbutton, GTK_CAN_DEFAULT);
  
	g_signal_connect (	okbutton, "clicked",
						G_CALLBACK (c2lp_OK_clicked), NULL);

	g_signal_connect (	dialog, "destroy",
						G_CALLBACK (delete_dialog), NULL);

//	g_signal_connect (	cancelbutton, "clicked",
//						G_CALLBACK (dialog_cancel_clicked), dialog);

	g_signal_connect (	(gpointer) rhoSizeEntry, "focus_out_event",
						G_CALLBACK (on_rho_size_updated),
						NULL);

	g_signal_connect (	(gpointer) thetaSizeEntry, "focus_out_event",
						G_CALLBACK (on_theta_size_updated),
						NULL);

	g_signal_connect (	(gpointer) xCenterEntry, "focus_out_event",
						G_CALLBACK (on_c2lp_XY_center_updated),
						NULL);

	g_signal_connect (	(gpointer) yCenterEntry, "focus_out_event",
						G_CALLBACK (on_c2lp_XY_center_updated),
						NULL);

	g_signal_connect (	da, "expose_event",
						G_CALLBACK (cartesian_expose), NULL);

	g_signal_connect (	daLP, "expose_event",
						G_CALLBACK (logpolar_expose), NULL);

	gtk_widget_set_events (da, GDK_ALL_EVENTS_MASK);

	g_signal_connect (	(gpointer) (da), "button_press_event",
						G_CALLBACK (mouse_button_press_event), NULL);

	g_signal_connect (	resetButton, "clicked",
						G_CALLBACK (on_c2lp_reset_button_clicked), NULL);

	g_signal_connect (	(gpointer) radio1, "toggled",
						G_CALLBACK (on_c2lp_circle_inside_toggled),
						NULL);

	g_signal_connect (	(gpointer) radio2, "toggled",
						G_CALLBACK (on_c2lp_circle_outside_toggled),
						NULL);

	g_signal_connect (	(gpointer) radio3, "toggled",
						G_CALLBACK (on_c2lp_circle_custom_toggled),
						NULL);

	g_signal_connect (	(gpointer) scaleFactorEntryEdit, "focus_out_event",
						G_CALLBACK (on_c2lp_scale_changed),
						NULL);

//	g_signal_connect (mapCircleToggleButton, "toggled",
//		    G_CALLBACK (map_mode_toggled), NULL);

	updateRadius();

	if (imgPar.mapInside == 1)
		gtk_toggle_button_set_active ((GtkToggleButton *) radio1,TRUE);
	else if (imgPar.mapInside == 0)
		gtk_toggle_button_set_active ((GtkToggleButton *) radio2,TRUE);
	else
		gtk_toggle_button_set_active ((GtkToggleButton *) radio3,TRUE);

	/* Store pointers to all widgets, for use by lookup_widget(). 
	GLADE_HOOKUP_OBJECT_NO_REF (dialog, dialog, "dialog");
	GLADE_HOOKUP_OBJECT_NO_REF (dialog, dialog_vbox, "dialog_vbox");
	GLADE_HOOKUP_OBJECT (dialog, table, "table");
	GLADE_HOOKUP_OBJECT (dialog, rhoSizeEntry, "rhoSizeEntry");
	GLADE_HOOKUP_OBJECT (dialog, thetaSizeEntry, "thetaSizeEntry");
	GLADE_HOOKUP_OBJECT (dialog, rhoSizeLabel, "rhoSizeLabel");
	GLADE_HOOKUP_OBJECT (dialog, thetaSizeLabel, "thetaSizeLabel");
	GLADE_HOOKUP_OBJECT (dialog, scaleFactorEntry, "scaleFactorEntry");
	GLADE_HOOKUP_OBJECT (dialog, scaleFactorLabel, "scaleFactorLabel");
	GLADE_HOOKUP_OBJECT_NO_REF (dialog, dialog_action_area, "dialog_action_area");
	GLADE_HOOKUP_OBJECT (dialog, cancelbutton, "cancelbutton");
	GLADE_HOOKUP_OBJECT (dialog, okbutton, "okbutton");*/

  return dialog;
}


GtkWidget* create_lp2cart_dialog ()
{
	GtkWidget *l2cDialog;
	GtkWidget *l2cDialogVbox;
	GtkWidget *l2cTable;

	GtkWidget *l2cRemDa;
	GtkWidget *l2cLpDa;

	GtkWidget *l2cRhoSizeEntry;
	GtkWidget *l2cThetaSizeEntry;
	GtkWidget *l2cXSizeEntry;
	GtkWidget *l2cYSizeEntry;
	GtkWidget *l2cLambdaEntry;
	GtkWidget *l2cFoveaSizeEntry;
	GtkWidget *l2cRhoZeroEntry;
	GtkWidget *l2cScaleFactorEntryEdit;
	GtkWidget *l2cScaleFactorEntryNoEdit;
	GtkWidget *l2cXCenterEntry;
	GtkWidget *l2cYCenterEntry;

	GtkWidget *l2cRhoSizeLabel;
	GtkWidget *l2cThetaSizeLabel;
	GtkWidget *l2cXSizeLabel;
	GtkWidget *l2cYSizeLabel;
	GtkWidget *l2cLambdaLabel;
	GtkWidget *l2cFoveaSizeLabel;
	GtkWidget *l2cRhoZeroLabel;
	GtkWidget *l2cScaleFactorLabel;
	GtkWidget *l2cXCenterLabel;
	GtkWidget *l2cYCenterLabel;

	GtkWidget *l2cRadiusLabel;
	GtkWidget *l2cRadiusEntry;
	GtkWidget *l2cRadiusBox;

	GtkWidget *l2cRemLabel;
	GtkWidget *l2cLpLabel;

	GtkWidget *l2cResetButton;

	GtkWidget *l2cDialogActionArea;
	GtkWidget *l2cCancelButton;
	GtkWidget *l2cOkButton;

	GtkWidget * l2cFrameLp;
	GtkWidget * l2cLpAlign;

	l2cDialog = gtk_dialog_new ();

	gtk_window_set_title (GTK_WINDOW (l2cDialog), _("Log Polar to Cartesian Transform"));
	gtk_window_set_type_hint (GTK_WINDOW (l2cDialog), GDK_WINDOW_TYPE_HINT_DIALOG);

	l2cDialogVbox = GTK_DIALOG (l2cDialog)->vbox;
	gtk_widget_show (l2cDialogVbox);

	l2cTable = gtk_table_new (7, 4, FALSE);
	gtk_widget_show (l2cTable);
	gtk_box_pack_start (GTK_BOX (l2cDialogVbox), l2cTable, TRUE, TRUE, 0);


	//Log Polar View
	l2cFrameLp = gtk_frame_new (NULL);
	gtk_frame_set_shadow_type (GTK_FRAME (l2cFrameLp), GTK_SHADOW_IN);

	l2cLpAlign = gtk_alignment_new (0.5, 0.5, 0, 0);
	gtk_widget_set_name(l2cLpAlign,"LP Align");	
	gtk_container_add (GTK_CONTAINER (l2cLpAlign), l2cFrameLp);
	gtk_widget_set_size_request (l2cFrameLp, 104, 104);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cLpAlign, 1, 2, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	l2cLpDa = gtk_drawing_area_new ();
	gtk_widget_set_name(l2cLpDa,"LP Drawing Area");	

	gtk_container_add (GTK_CONTAINER (l2cFrameLp), l2cLpDa);
	gtk_widget_show (l2cLpDa);

//	testda = l2cLpDa;

	gtk_widget_show (l2cFrameLp);
	gtk_widget_show (l2cLpAlign);

	// "LogPolar" Label
	l2cLpLabel = gtk_label_new (_("Log Polar Image"));
	gtk_widget_show (l2cLpLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cLpLabel, 1, 2, 6, 7,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	//Cartesian View
	GtkWidget * frame = gtk_frame_new (NULL);
	gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);

	GtkWidget * align = gtk_alignment_new (0.5, 0.5, 0, 0);
	gtk_widget_set_name(align,"Cartesian Align");	
	gtk_container_add (GTK_CONTAINER (align), frame);
	gtk_widget_set_size_request (frame, 104, 104);
	gtk_table_attach (GTK_TABLE (l2cTable), align, 2, 3, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	l2cRemDa = gtk_drawing_area_new ();
	gtk_widget_set_name(l2cRemDa,"Cartesian Drawing Area");	

	gtk_container_add (GTK_CONTAINER (frame), l2cRemDa);
	gtk_widget_show (l2cRemDa);
//	printf("Cartesian DA: 0x%x\n",l2cRemDa);
//	dbgWidg = l2cRemDa;

	gtk_widget_show (frame);
	gtk_widget_show (align);

	// "Cartesian" Label
	l2cRemLabel = gtk_label_new (_("Remapped Image"));
	gtk_widget_show (l2cRemLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cRemLabel, 2, 3, 6, 7,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);


	// Rho Size Entry
	gint rhoSize;
	rhoSize = gdk_pixbuf_get_height((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	imgPar.rho_Size = rhoSize;

	gchar rhoSizeString[5];
	sprintf(rhoSizeString,"%d", rhoSize);
	l2cRhoSizeEntry = gtk_label_new (_(rhoSizeString));
	gtk_widget_set_name(l2cRhoSizeEntry,"Rho Size Entry");	
	gtk_widget_show (l2cRhoSizeEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cRhoSizeEntry, 0, 1, 0, 1,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Size Label
	l2cRhoSizeLabel = gtk_label_new (_("Rings"));
//	printf("Rings Label: 0x%x\n",l2cRhoSizeLabel);
	gtk_widget_show (l2cRhoSizeLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cRhoSizeLabel, 1, 2, 0, 1,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Theta Size Entry
	gint thetaSize;
	thetaSize = gdk_pixbuf_get_width((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	imgPar.theta_Size = thetaSize;

	gchar thetaSizeString[5];
	sprintf(thetaSizeString,"%d", thetaSize);
	l2cThetaSizeEntry = gtk_label_new (_(thetaSizeString));
	gtk_widget_set_name(l2cThetaSizeEntry,"Theta Size Entry");	
	gtk_widget_show (l2cThetaSizeEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cThetaSizeEntry, 0, 1, 1, 2,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Theta Size Label
	l2cThetaSizeLabel = gtk_label_new (_("Pixels per Ring"));
	gtk_widget_show (l2cThetaSizeLabel);
	gtk_table_attach(GTK_TABLE (l2cTable), l2cThetaSizeLabel, 1, 2, 1, 2,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);


	//Compute lambda, rhozero and fovea
	imgPar.log_index = (1.0+sin(PI/imgPar.theta_Size))/(1.0-sin(PI/imgPar.theta_Size));
	imgPar.fovea = (int)(imgPar.log_index/(imgPar.log_index-1));
	imgPar.r_zero = 1.0/(pow(imgPar.log_index,imgPar.fovea)*(imgPar.log_index-1));

	// X Size Entry
	l2cXSizeEntry = gtk_entry_new ();
//	gint xSize;
//	xSize = gdk_pixbuf_get_width((GdkPixbuf *)(g_list_nth_data(pixbufList,0)));
	gtk_widget_set_name(l2cXSizeEntry,"X Size Entry");	
	gchar xSizeString[5];
	sprintf(xSizeString,"%d", imgPar.X_Size);
	gtk_entry_set_width_chars(GTK_ENTRY(l2cXSizeEntry),4);
	gtk_entry_set_text(GTK_ENTRY(l2cXSizeEntry),xSizeString);
	gtk_widget_show (l2cXSizeEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cXSizeEntry, 0, 1, 2, 3,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// X Size Label
	l2cXSizeLabel = gtk_label_new (_("Width"));
	gtk_widget_show (l2cXSizeLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cXSizeLabel, 1, 2, 2, 3,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Y Size Entry
	l2cYSizeEntry = gtk_entry_new ();
	gtk_widget_set_name(l2cYSizeEntry,"Y Size Entry");	
	gchar ySizeString[5];
	sprintf(ySizeString,"%d", imgPar.Y_Size);
	gtk_entry_set_width_chars(GTK_ENTRY(l2cYSizeEntry),4);
	gtk_entry_set_text(GTK_ENTRY(l2cYSizeEntry),ySizeString);
	gtk_widget_show (l2cYSizeEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cYSizeEntry, 0, 1, 3, 4,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Y Size Label
	l2cYSizeLabel = gtk_label_new (_("Height"));
	gtk_widget_show (l2cYSizeLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cYSizeLabel, 1, 2, 3, 4,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Lambda Entry
//	lambdaEntry = gtk_entry_new ();
	gchar lambdaString[8];
	sprintf(lambdaString,"%1.4f", imgPar.log_index);
	l2cLambdaEntry = gtk_label_new (_(lambdaString));
	gtk_widget_set_name(l2cLambdaEntry,"Lambda Entry");	
	gtk_widget_show (l2cLambdaEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cLambdaEntry, 2, 3, 0, 1,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Lambda Label
	l2cLambdaLabel = gtk_label_new (_("Log Base"));
	gtk_widget_show (l2cLambdaLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cLambdaLabel, 3, 4, 0, 1,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Fovea Size Entry
	gchar foveaSizeString[5];
	sprintf(foveaSizeString,"%d", imgPar.fovea);
	l2cFoveaSizeEntry = gtk_label_new (_(foveaSizeString));
	gtk_widget_set_name(l2cFoveaSizeEntry,"Fovea Size Entry");	
	gtk_widget_show (l2cFoveaSizeEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cFoveaSizeEntry, 2, 3, 1, 2,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Fovea Size Label
	l2cFoveaSizeLabel = gtk_label_new (_("Fovea Rings"));
	gtk_widget_show (l2cFoveaSizeLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cFoveaSizeLabel, 3, 4, 1, 2,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Zero Entry
	gchar rhoZeroString[8];
	sprintf(rhoZeroString,"%1.4f", imgPar.r_zero);
	l2cRhoZeroEntry = gtk_label_new (_(rhoZeroString));
	gtk_widget_set_name(l2cRhoZeroEntry,"Rho Zero Entry");	
	gtk_widget_show (l2cRhoZeroEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cRhoZeroEntry, 2, 3, 2, 3,
					(GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
					(GtkAttachOptions) (0), 20, 10);

	// Rho Zero Label
	l2cRhoZeroLabel = gtk_label_new (_("Rho Zero"));
	gtk_widget_show (l2cRhoZeroLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cRhoZeroLabel, 3, 4, 2, 3,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Scale Factor Entry
	imgPar.scaleFactor  = __min(imgPar.X_Size,imgPar.Y_Size);
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

	l2cScaleFactorEntryEdit   = gtk_entry_new ();
//	GLADE_HOOKUP_OBJECT (l2cDialog, l2cScaleFactorEntryEdit, "l2cScaleFactorEntryEdit");
	gtk_widget_set_name(l2cScaleFactorEntryEdit,"Scale Factor Entry Edit");	

	gchar scalefactorString[11];
	sprintf(scalefactorString,"%1.8f", imgPar.scaleFactor);
	gtk_entry_set_width_chars(GTK_ENTRY(l2cScaleFactorEntryEdit),8);
	gtk_entry_set_text(GTK_ENTRY(l2cScaleFactorEntryEdit),scalefactorString);

	l2cScaleFactorEntryNoEdit = gtk_label_new (_(scalefactorString));
//	GLADE_HOOKUP_OBJECT (l2cDialog, l2cScaleFactorEntryNoEdit, "l2cScaleFactorEntryNoEdit");
	gtk_widget_set_name(l2cScaleFactorEntryNoEdit,"Scale Factor Entry No Edit");	

	gtk_widget_hide (l2cScaleFactorEntryEdit);
	gtk_widget_show (l2cScaleFactorEntryNoEdit);

	gtk_table_attach (GTK_TABLE (l2cTable), l2cScaleFactorEntryEdit, 2, 3, 3, 4,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	gtk_table_attach (GTK_TABLE (l2cTable), l2cScaleFactorEntryNoEdit, 2, 3, 3, 4,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Scale Factor Label
	l2cScaleFactorLabel = gtk_label_new (_("Scale Factor"));
	gtk_widget_show (l2cScaleFactorLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cScaleFactorLabel, 3, 4, 3, 4,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);


	// X Center Entry
	l2cXCenterEntry = gtk_entry_new ();
	gtk_widget_set_name(l2cXCenterEntry,"X Center Entry");	
	gchar xCenterString[5];
	sprintf(xCenterString,"%d", imgPar.xCenter);
	gtk_entry_set_width_chars(GTK_ENTRY(l2cXCenterEntry),4);
	gtk_entry_set_text(GTK_ENTRY(l2cXCenterEntry),xCenterString);
	gtk_widget_show (l2cXCenterEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cXCenterEntry, 0, 1, 4, 5,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// X Center Label
	l2cXCenterLabel = gtk_label_new (_("Horizontal Center"));
	gtk_widget_show (l2cXCenterLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cXCenterLabel, 1, 2, 4, 5,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);
	
	// Y Center Entry
	l2cYCenterEntry = gtk_entry_new ();
	gtk_widget_set_name(l2cYCenterEntry,"Y Center Entry");	
	gchar yCenterString[5];
	sprintf(yCenterString,"%d", imgPar.yCenter);
	gtk_entry_set_width_chars(GTK_ENTRY(l2cYCenterEntry),4);
	gtk_entry_set_text(GTK_ENTRY(l2cYCenterEntry),yCenterString);
	gtk_widget_show (l2cYCenterEntry);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cYCenterEntry, 2, 3, 4, 5,
					(GtkAttachOptions) (GTK_EXPAND),
					(GtkAttachOptions) (0), 20, 10);

	// Y Center Label
	l2cYCenterLabel = gtk_label_new (_("Vertical Center"));
	gtk_widget_show (l2cYCenterLabel);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cYCenterLabel, 3, 4, 4, 5,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Reset Button
	l2cResetButton = gtk_button_new_with_label(_("Reset Center"));
	gtk_widget_set_name(l2cResetButton,"Reset Button");	
	gtk_widget_show (l2cResetButton);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cResetButton, 3, 4, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	// Radius Entry and Label

	l2cRadiusBox = gtk_hbox_new (TRUE, 2);

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	l2cRadiusEntry = gtk_label_new (_(radiusString));
	gtk_widget_set_name(l2cRadiusBox,"Radius Box");	
	gtk_widget_set_name(l2cRadiusEntry,"Radius Entry");	
	gtk_widget_show (l2cRadiusEntry);

//	printf("Rad entry %x\n", radEntry);

	l2cRadiusLabel = gtk_label_new (_("Map Radius"));
	gtk_widget_show (l2cRadiusLabel);

	gtk_box_pack_start (GTK_BOX (l2cRadiusBox), l2cRadiusEntry, TRUE, TRUE, 2);
	gtk_box_pack_start (GTK_BOX (l2cRadiusBox), l2cRadiusLabel, TRUE, TRUE, 2);
	gtk_table_attach (GTK_TABLE (l2cTable), l2cRadiusBox, 0, 1, 6, 7,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	gtk_widget_show_all (l2cRadiusBox);

	//Radio Buttons
	
	GtkWidget *radio1, *radio2, *radio3, *box;
	
	box = gtk_vbox_new (TRUE, 4);
   
	radio1 = gtk_radio_button_new_with_label (NULL,"Circle inside of the box");
	radio2 = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radio1),
							"Circle outside of the box");
	radio3 = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radio1),
							"Custom");

	gtk_box_pack_start (GTK_BOX (box), radio1, TRUE, TRUE, 2);
	gtk_box_pack_start (GTK_BOX (box), radio2, TRUE, TRUE, 2);
	gtk_box_pack_start (GTK_BOX (box), radio3, TRUE, TRUE, 2);
	gtk_table_attach (GTK_TABLE (l2cTable), box, 0, 1, 5, 6,
					(GtkAttachOptions) (0),
					(GtkAttachOptions) (0), 20, 10);

	gtk_widget_show_all (box);

	l2cDialogActionArea = GTK_DIALOG (l2cDialog)->action_area;
	gtk_widget_show (l2cDialogActionArea);
	gtk_button_box_set_layout (GTK_BUTTON_BOX (l2cDialogActionArea), GTK_BUTTONBOX_END);

	l2cCancelButton = gtk_button_new_from_stock ("gtk-cancel");
	gtk_widget_show (l2cCancelButton);
	gtk_dialog_add_action_widget (GTK_DIALOG (l2cDialog), l2cCancelButton, GTK_RESPONSE_CANCEL);
	GTK_WIDGET_SET_FLAGS (l2cCancelButton, GTK_CAN_DEFAULT);

	l2cOkButton = gtk_button_new_from_stock ("gtk-ok");
	gtk_widget_show (l2cOkButton);
	gtk_dialog_add_action_widget (GTK_DIALOG (l2cDialog), l2cOkButton, GTK_RESPONSE_OK);
	GTK_WIDGET_SET_FLAGS (l2cOkButton, GTK_CAN_DEFAULT);
  
	g_signal_connect (	l2cOkButton, "clicked",
						G_CALLBACK (lp2c_OK_clicked), NULL);

	g_signal_connect (	l2cDialog, "destroy",
						G_CALLBACK (delete_dialog), NULL);

	g_signal_connect (	(gpointer) l2cXSizeEntry, "focus_out_event",
						G_CALLBACK (on_lp2c_XY_size_updated),
						NULL);

	g_signal_connect (	(gpointer) l2cYSizeEntry, "focus_out_event",
						G_CALLBACK (on_lp2c_XY_size_updated),
						NULL);

	g_signal_connect (	l2cRemDa, "expose_event",
						G_CALLBACK (remapped_expose), NULL);

	g_signal_connect (	l2cLpDa, "expose_event",
						G_CALLBACK (logpolar2_expose), NULL);

//	g_signal_connect (	dialog, "destroy",
//						G_CALLBACK (delete_dialog), NULL);

	g_signal_connect (	(gpointer) l2cXCenterEntry, "focus_out_event",
						G_CALLBACK (on_lp2c_XY_center_updated),
						NULL);

	g_signal_connect (	(gpointer) l2cYCenterEntry, "focus_out_event",
						G_CALLBACK (on_lp2c_XY_center_updated),
						NULL);

	g_signal_connect (	(gpointer) radio1, "toggled",
						G_CALLBACK (on_lp2c_circle_inside_toggled),
						NULL);

	g_signal_connect (	(gpointer) radio2, "toggled",
						G_CALLBACK (on_lp2c_circle_outside_toggled),
						NULL);

	g_signal_connect (	(gpointer) radio3, "toggled",
						G_CALLBACK (on_lp2c_circle_custom_toggled),
						NULL);

	g_signal_connect (	(gpointer) l2cScaleFactorEntryEdit, "focus_out_event",
						G_CALLBACK (on_lp2c_scale_changed),
						NULL);

	g_signal_connect (l2cResetButton, "clicked",
			G_CALLBACK (on_lp2c_reset_button_clicked), NULL);

	// Store pointers to all widgets, for use by lookup_widget(). 
	GLADE_HOOKUP_OBJECT_NO_REF (l2cDialog, l2cDialog, "l2cDialog");
	GLADE_HOOKUP_OBJECT_NO_REF (l2cDialog, l2cDialogVbox, "l2cDialogVbox");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cTable, "l2cTable");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cXSizeEntry, "l2cXSizeEntry");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cYSizeEntry, "l2cYSizeEntry");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cXCenterEntry, "l2cXCenterEntry");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cYCenterEntry, "l2cYCenterEntry");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cScaleFactorEntryEdit, "l2cScaleFactorEntryEdit");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cScaleFactorEntryNoEdit, "l2cScaleFactorEntryNoEdit");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cRadiusBox, "l2cRadiusBox");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cRadiusEntry, "l2cRadiusEntry");
	GLADE_HOOKUP_OBJECT_NO_REF (l2cDialog, l2cDialogActionArea, "l2cDialogActionArea");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cCancelButton, "l2cCancelButton");
	GLADE_HOOKUP_OBJECT (l2cDialog, l2cOkButton, "l2cOkButton");

	if (imgPar.mapInside == 1)
		gtk_toggle_button_set_active ((GtkToggleButton *) radio1,TRUE);
	else if (imgPar.mapInside == 0)
		gtk_toggle_button_set_active ((GtkToggleButton *) radio2,TRUE);
	else
		gtk_toggle_button_set_active ((GtkToggleButton *) radio3,TRUE);


	return l2cDialog;
}


GtkWidget* create_toolbox ()
{
	GtkWidget *toolbox;
	GtkWidget *toolbar;
	GtkWidget *tbMainVBox;
	GtkWidget *zoomInToolItem;
	GtkWidget *zoomInButton;
	GtkWidget *zoomInImage;
	GtkWidget *zoomOutToolItem;
	GtkWidget *zoomOutButton;
	GtkWidget *zoomOutImage;
	GtkWidget *flipToolItem;
	GtkWidget *flipImage;
	GtkWidget *flipButton;
	GtkWidget *mirrorToolItem;
	GtkWidget *mirrorImage;
	GtkWidget *mirrorButton;
	GtkIconSize tmp_toolbar_icon_size;

	toolbox = gtk_dialog_new ();
	gtk_window_set_title (GTK_WINDOW (toolbox), _("Toolbox"));
	gtk_window_set_type_hint (GTK_WINDOW (toolbox), GDK_WINDOW_TYPE_HINT_DIALOG);

	tbMainVBox = GTK_DIALOG (toolbox)->vbox;
	gtk_widget_show (tbMainVBox);

	toolbar = gtk_toolbar_new ();
	gtk_widget_show (toolbar);
	gtk_box_pack_start (GTK_BOX (tbMainVBox), toolbar, FALSE, FALSE, 0);
	gtk_toolbar_set_style (GTK_TOOLBAR (toolbar), GTK_TOOLBAR_BOTH);
	tmp_toolbar_icon_size = gtk_toolbar_get_icon_size (GTK_TOOLBAR (toolbar));
	gtk_widget_set_size_request (toolbar, 214, 34);

	zoomInToolItem = (GtkWidget*) gtk_tool_item_new ();
	gtk_widget_show (zoomInToolItem);
	gtk_container_add (GTK_CONTAINER (toolbar), zoomInToolItem);
	gtk_widget_set_size_request (zoomInToolItem, 30, 30);

	zoomInButton = gtk_button_new ();
	gtk_widget_show (zoomInButton);
	gtk_container_add (GTK_CONTAINER (zoomInToolItem), zoomInButton);

	zoomInImage = gtk_image_new_from_stock ("gtk-zoom-in", GTK_ICON_SIZE_BUTTON);
	gtk_widget_show (zoomInImage);
	gtk_container_add (GTK_CONTAINER (zoomInButton), zoomInImage);

	zoomOutToolItem = (GtkWidget*) gtk_tool_item_new ();
	gtk_widget_show (zoomOutToolItem);
	gtk_container_add (GTK_CONTAINER (toolbar), zoomOutToolItem);
	gtk_widget_set_size_request (zoomOutToolItem, 30, 30);

	zoomOutButton = gtk_button_new ();
	gtk_widget_show (zoomOutButton);
	gtk_container_add (GTK_CONTAINER (zoomOutToolItem), zoomOutButton);

	zoomOutImage = gtk_image_new_from_stock ("gtk-zoom-out", GTK_ICON_SIZE_BUTTON);
	gtk_widget_show (zoomOutImage);
	gtk_container_add (GTK_CONTAINER (zoomOutButton), zoomOutImage);

	flipToolItem = (GtkWidget*) gtk_tool_item_new ();
	gtk_widget_show (flipToolItem);
	gtk_container_add (GTK_CONTAINER (toolbar), flipToolItem);
	gtk_widget_set_size_request (flipToolItem, 30, 30);

	flipButton = gtk_button_new ();
	gtk_widget_show (flipButton);
	gtk_container_add (GTK_CONTAINER (flipToolItem), flipButton);

	flipImage = gtk_image_new_from_file ("Flip.gif");
	gtk_widget_show (flipImage);
	gtk_container_add (GTK_CONTAINER (flipButton), flipImage);

	mirrorToolItem = (GtkWidget*) gtk_tool_item_new ();
	gtk_widget_show (mirrorToolItem);
	gtk_container_add (GTK_CONTAINER (toolbar), mirrorToolItem);
	gtk_widget_set_size_request (mirrorToolItem, 30, 30);

	mirrorButton = gtk_button_new ();
	gtk_widget_show (mirrorButton);
	gtk_container_add (GTK_CONTAINER (mirrorToolItem), mirrorButton);

	mirrorImage = gtk_image_new_from_file ("Mirror.gif");
	gtk_widget_show (mirrorImage);
	gtk_container_add (GTK_CONTAINER (mirrorButton), mirrorImage);


	gtk_widget_show_all (toolbox);

	g_signal_connect ((gpointer) toolbox, "destroy", G_CALLBACK(delete_toolbox),
						NULL);

	g_signal_connect (zoomInButton, "clicked",
						G_CALLBACK (on_zoomIn_button_clicked), NULL);

	g_signal_connect (zoomOutButton, "clicked",
						G_CALLBACK (on_zoomOut_button_clicked), NULL);

	g_signal_connect (mirrorButton, "clicked",
						G_CALLBACK (on_mirror_button_clicked), NULL);

	g_signal_connect (flipButton, "clicked",
						G_CALLBACK (on_flip_button_clicked), NULL);

	/* Store pointers to all widgets, for use by lookup_widget(). */
//	GLADE_HOOKUP_OBJECT_NO_REF (toolbox, toolbox, "toolbox");
//	GLADE_HOOKUP_OBJECT_NO_REF (toolbox, tbMainVBox, "tbMainVBox");
//	GLADE_HOOKUP_OBJECT (toolbox, tbVButtonBox, "tbVButtonBox");
//	GLADE_HOOKUP_OBJECT (toolbox, tbZoomIn, "tbZoomIn");
//	GLADE_HOOKUP_OBJECT (toolbox, tbZoomInImage, "tbZoomInImage");
//	GLADE_HOOKUP_OBJECT (toolbox, tbZoomOut, "tbZoomOut");
//	GLADE_HOOKUP_OBJECT (toolbox, tbZoomOutImage, "tbZoomOutImage");
//	GLADE_HOOKUP_OBJECT (toolbox, tbFlip, "tbFlip");
//	GLADE_HOOKUP_OBJECT (toolbox, tbFlipImage, "tbFlipImage");
//	GLADE_HOOKUP_OBJECT (toolbox, tbMirror, "tbMirror");
//	GLADE_HOOKUP_OBJECT (toolbox, tbMirrorImage, "tbMirrorImage");
//	GLADE_HOOKUP_OBJECT (toolbox, tbFilter, "tbFilter");
//	GLADE_HOOKUP_OBJECT (toolbox, tbFilterImage, "tbFilterImage");
//	GLADE_HOOKUP_OBJECT_NO_REF (toolbox, tbActionArea, "tbActionArea");
	g_object_set_data (G_OBJECT (mainMenuWindow),"toolbox",toolbox);

	return toolbox;
}
