#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>
#include <stdlib.h>
#include <math.h>

#include "support.h"
#include "callbacks.h"
#include "callbacksc2lp.h"
#include "interface.h"
#include "functions.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

extern imagedata imgPar;

gboolean on_rho_size_updated (GtkWidget *widget,gpointer user_data)
{
	gint i;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));
	gint childNumb = g_list_length (children3);

	GtkWidget * scaleFactorEntryEdit;
	GtkWidget * scaleFactorEntryNoEdit;
	GtkWidget * radiusBox;
	GtkWidget * radiusEntry;
	GtkWidget * rhoSizeEntry;
	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"Rho Size Entry"))
			rhoSizeEntry = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry No Edit"))
			scaleFactorEntryNoEdit = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry Edit"))
			scaleFactorEntryEdit = GTK_WIDGET(tempWidget);
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

	const gchar* rhoSizeTxt = gtk_entry_get_text(GTK_ENTRY(rhoSizeEntry));
	imgPar.rho_Size = atoi (rhoSizeTxt);

	updateRadius();

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

	imgPar.scaleFactor = 2*imgPar.mapRadius;
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

	gchar scaleFactorString[11];
	sprintf(scaleFactorString,"%1.8f", imgPar.scaleFactor);

	if (imgPar.mapInside!=2)
		gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorString);
	else
		gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorString);

	logpolar_expose(widget,NULL,NULL);

	return FALSE;
}

gboolean on_theta_size_updated (GtkWidget *widget,gpointer user_data)
{
	gint i;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));
	gint childNumb = g_list_length (children3);
	
	GtkWidget * scaleFactorEntryEdit;
	GtkWidget * scaleFactorEntryNoEdit;
	GtkWidget * rhoZeroEntry;
	GtkWidget * foveaSizeEntry;
	GtkWidget * lambdaEntry;
	GtkWidget * thetaSizeEntry;
	GtkWidget * radiusBox;
	GtkWidget * radiusEntry;

	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"Rho Zero Entry"))
			rhoZeroEntry = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry No Edit"))
			scaleFactorEntryNoEdit = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry Edit"))
			scaleFactorEntryEdit = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Fovea Size Entry"))
			foveaSizeEntry = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Theta Size Entry"))
			thetaSizeEntry = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Lambda Entry"))
			lambdaEntry = GTK_WIDGET(tempWidget);
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

	gint foveaSize;
	gint thetaSize;
	gdouble lambda;
	gdouble rhoZero;

	const gchar* thetaSizeTxt = gtk_entry_get_text(GTK_ENTRY(thetaSizeEntry));
	thetaSize = atoi (thetaSizeTxt);

	lambda    = (1.0+sin(PI/thetaSize))/(1.0-sin(PI/thetaSize));
	foveaSize = (int)(lambda/(lambda-1));
	rhoZero   = 1.0/(pow(lambda,foveaSize)*(lambda-1));

	imgPar.theta_Size = thetaSize;
	imgPar.log_index = lambda;
	imgPar.fovea = foveaSize;
	imgPar.r_zero = rhoZero;

	updateRadius();

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

	imgPar.scaleFactor  = 2*imgPar.mapRadius;
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

	gchar lambdaString[7];
	sprintf(lambdaString,"%1.4f", lambda);
	gtk_label_set_label(GTK_LABEL(lambdaEntry), lambdaString);

	gchar foveaSizeString[7];
	sprintf(foveaSizeString,"%d", foveaSize);
	gtk_label_set_label(GTK_LABEL(foveaSizeEntry), foveaSizeString);

	gchar rhoZeroString[7];
	sprintf(rhoZeroString,"%1.4f", rhoZero);
	gtk_label_set_label(GTK_LABEL(rhoZeroEntry), rhoZeroString);

	gchar scaleFactorString[11];
	sprintf(scaleFactorString,"%1.8f", imgPar.scaleFactor);

	if (imgPar.mapInside!=2)
		gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorString);
	else
		gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorString);

	logpolar_expose(widget,NULL,NULL);

	return FALSE;
}

gboolean on_c2lp_XY_center_updated (GtkWidget *widget,gpointer user_data)
{
	gint i;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));//children[0]=>DialogVBox
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));//children2[0]=>Table
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));//children3=>List of Entries and Labels
	gint childNumb = g_list_length (children3);
	
 	gpointer tempWidget;

	GtkWidget * align;
	GtkWidget * scaleFactorEntryEdit;
	GtkWidget * scaleFactorEntryNoEdit;
	GtkWidget * xCenterEntry;
	GtkWidget * yCenterEntry;
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
		else if (!strcmp(widgetName,"Cartesian Align"))
			align = GTK_WIDGET(tempWidget);
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

	children4 = gtk_container_get_children (GTK_CONTAINER(align));
	GList* children5 = gtk_container_get_children (GTK_CONTAINER(children4->data));
	
	GtkWidget * dArea	= GTK_WIDGET(children5->data);//get a pointer to Cart drawing area

	const gchar* xCenterTxt = gtk_entry_get_text(GTK_ENTRY(xCenterEntry));
	gint xCenter = atoi (xCenterTxt);

	const gchar* yCenterTxt = gtk_entry_get_text(GTK_ENTRY(yCenterEntry));
	gint yCenter = atoi (yCenterTxt);

	imgPar.xCenter = xCenter;
	imgPar.yCenter = yCenter;

	if (imgPar.mapInside==1)
	{
		if ((imgPar.yCenter>=imgPar.Y_Size)||(imgPar.yCenter<0))
			imgPar.yCenter = imgPar.Y_Size/2;

		if ((imgPar.xCenter>=imgPar.X_Size)||(imgPar.xCenter<0))
			imgPar.xCenter = imgPar.X_Size/2;
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

	cartesian_expose(GTK_WIDGET(dArea),NULL,NULL);
	logpolar_expose(widget,NULL,NULL);

	return FALSE;

}

void on_c2lp_reset_button_clicked(GtkWidget *widget,gpointer user_data)
{
	gint i;

	imgPar.yCenter = imgPar.Y_Size/2;
	imgPar.xCenter = imgPar.X_Size/2;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));//children[0]=>DialogVBox
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));//children3=>List of Entries and Labels

	GtkWidget * xCenterEntry;
	GtkWidget * yCenterEntry;

	gint childNumb = g_list_length (children3);
	
 	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"X Center Entry"))
			xCenterEntry = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Y Center Entry"))
			yCenterEntry = GTK_WIDGET(tempWidget);
	}

	gchar xCenterString[5];
	sprintf(xCenterString,"%d", imgPar.xCenter);
	gtk_entry_set_text(GTK_ENTRY(xCenterEntry),xCenterString);

	gchar yCenterString[5];
	sprintf(yCenterString,"%d", imgPar.yCenter);
	gtk_entry_set_text(GTK_ENTRY(yCenterEntry),yCenterString);

	on_c2lp_XY_center_updated(widget,NULL);
}
/*
void map_mode_toggled(GtkWidget *widget,gpointer user_data)
{
	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
	{
		imgPar.mapInside = TRUE;
	}
	else
	{
		imgPar.mapInside = FALSE;
	}

	updateRadius();
	on_c2lp_XY_center_updated(widget,NULL);
/*
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));//children[0]=>DialogVBox
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));//children2[0]=>Table
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));//children3=>List of Entries and Labels
	gint childNumb = g_list_length (children3);
	
 	GtkWidget * align	= GTK_WIDGET(g_list_nth_data(children3,25));
	GList* children4 = gtk_container_get_children (GTK_CONTAINER(align));
	GList* children5 = gtk_container_get_children (GTK_CONTAINER(children4->data));
	
	GtkWidget * dArea	= GTK_WIDGET(children5->data);//get a pointer to Cart drawing area

	cartesian_expose(GTK_WIDGET(dArea),NULL,NULL);
	logpolar_expose(widget,NULL,NULL);
*//*
}
*/
void on_c2lp_circle_inside_toggled(GtkWidget *widget,gpointer user_data)
{
	int i;

	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
		imgPar.mapInside = 1;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));

	GtkWidget * scaleFactorEdit;
	GtkWidget * scaleFactorNoEdit;

	gint childNumb = g_list_length (children3);
	
 	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"Scale Factor Entry Edit"))
			scaleFactorEdit = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry No Edit"))
			scaleFactorNoEdit = GTK_WIDGET(tempWidget);
	}

	gtk_widget_hide (scaleFactorEdit);
	gtk_widget_show (scaleFactorNoEdit);


	updateRadius();
	on_c2lp_XY_center_updated(widget,NULL);

	gchar scaleFactorTxt[11];
	sprintf(scaleFactorTxt,"%1.8f", imgPar.scaleFactor);
	gtk_entry_set_text(GTK_ENTRY(scaleFactorEdit),scaleFactorTxt);
	gtk_label_set_label(GTK_LABEL(scaleFactorNoEdit), scaleFactorTxt);

}

void on_c2lp_circle_outside_toggled(GtkWidget *widget,gpointer user_data)
{
	int i;

	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
		imgPar.mapInside = 0;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));

	GtkWidget * scaleFactorEdit;
	GtkWidget * scaleFactorNoEdit;

	gint childNumb = g_list_length (children3);
	
 	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"Scale Factor Entry Edit"))
			scaleFactorEdit = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry No Edit"))
			scaleFactorNoEdit = GTK_WIDGET(tempWidget);
	}

	gtk_widget_hide (scaleFactorEdit);
	gtk_widget_show (scaleFactorNoEdit);

	updateRadius();
	on_c2lp_XY_center_updated(widget,NULL);

	gchar scaleFactorTxt[11];
	sprintf(scaleFactorTxt,"%1.8f", imgPar.scaleFactor);
	gtk_entry_set_text(GTK_ENTRY(scaleFactorEdit),scaleFactorTxt);
	gtk_label_set_label(GTK_LABEL(scaleFactorNoEdit), scaleFactorTxt);
}


void on_c2lp_circle_custom_toggled(GtkWidget *widget,gpointer user_data)
{
	int i; 

	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
		imgPar.mapInside = 2;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));

	GtkWidget * scaleFactorEdit;
	GtkWidget * scaleFactorNoEdit;

	gint childNumb = g_list_length (children3);
	
 	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"Scale Factor Entry Edit"))
			scaleFactorEdit = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Scale Factor Entry No Edit"))
			scaleFactorNoEdit = GTK_WIDGET(tempWidget);
	}

	gtk_widget_hide (scaleFactorNoEdit);
	gtk_widget_show (scaleFactorEdit);

	const gchar* scaleFactorTxt = gtk_entry_get_text(GTK_ENTRY(scaleFactorEdit));
	gdouble sF = atof (scaleFactorTxt);

	imgPar.scaleFactor = sF;
}

gboolean on_c2lp_scale_changed (GtkWidget *widget,gpointer user_data)
{
	gint i;
	
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	GList* children = gtk_container_get_children (GTK_CONTAINER(tl));//children[0]=>DialogVBox
	GList* children2 = gtk_container_get_children (GTK_CONTAINER(children->data));//children2[0]=>Table
	GList* children3 = gtk_container_get_children (GTK_CONTAINER(children2->data));//children3=>List of Entries and Labels
	gint childNumb = g_list_length (children3);
	
	GtkWidget * xSizeEntry;
	GtkWidget * ySizeEntry;
	GtkWidget * xCenterEntry;
	GtkWidget * yCenterEntry;
	GtkWidget * scaleFactorEntryEdit;
	GtkWidget * radiusBox;
	GtkWidget * radiusEntry;

	gpointer tempWidget;

	for (i = 0; i<childNumb; i++)
	{
		tempWidget = g_list_nth_data(children3,i);
		const gchar * widgetName = gtk_widget_get_name (GTK_WIDGET(tempWidget));

		if (!strcmp(widgetName,"X Size Entry"))
			xSizeEntry = GTK_WIDGET(tempWidget);
		else if (!strcmp(widgetName,"Y Size Entry"))
			ySizeEntry = GTK_WIDGET(tempWidget);
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

	const gchar* scaleFactorTxt = gtk_entry_get_text(GTK_ENTRY(scaleFactorEntryEdit));
	imgPar.scaleFactor = atof (scaleFactorTxt);

	imgPar.mapRadius = imgPar.scaleFactor * (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));
	imgPar.mapRadius *= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.mapRadius /= 2.0;


	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

	on_c2lp_XY_center_updated(widget,NULL);

	return FALSE;
}
