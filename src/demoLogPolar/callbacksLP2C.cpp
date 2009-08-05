#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>
#include <stdlib.h>
#include <math.h>

#include "support.h"
#include "callbacks.h"
#include "callbackslp2c.h"
#include "interface.h"
#include "functions.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>




extern imagedata imgPar;

gboolean on_lp2c_XY_size_updated (GtkWidget *widget,gpointer user_data)
{
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	
	GtkWidget * xSizeEntry				= lookup_widget(GTK_WIDGET (tl), "l2cXSizeEntry");
	GtkWidget * ySizeEntry				= lookup_widget(GTK_WIDGET (tl), "l2cYSizeEntry");
	GtkWidget * xCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cXCenterEntry");
	GtkWidget * yCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cYCenterEntry");
	GtkWidget * scaleFactorEntryEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryEdit");
	GtkWidget * scaleFactorEntryNoEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryNoEdit");
	GtkWidget * radiusEntry				= lookup_widget(GTK_WIDGET (tl), "l2cRadiusEntry");

	gint oldXSz = imgPar.X_Size;
	gint oldYSz = imgPar.Y_Size;


	const gchar* xSizeTxt = gtk_entry_get_text(GTK_ENTRY(xSizeEntry));
	imgPar.X_Size = atoi (xSizeTxt);

	if (imgPar.X_Size>8192)
		imgPar.X_Size = 8192;

	gchar xSizeString[5];
	sprintf(xSizeString,"%d", imgPar.X_Size);
	gtk_entry_set_text(GTK_ENTRY(xSizeEntry),xSizeString);

	const gchar* ySizeTxt = gtk_entry_get_text(GTK_ENTRY(ySizeEntry));
	imgPar.Y_Size = atoi (ySizeTxt);

	if (imgPar.Y_Size>8192)
		imgPar.Y_Size = 8192;

	gchar ySizeString[5];
	sprintf(ySizeString,"%d", imgPar.Y_Size);
	gtk_entry_set_text(GTK_ENTRY(ySizeEntry),ySizeString);

	imgPar.xCenter = (int)(0.5+imgPar.X_Size*imgPar.xCenter/oldXSz);
	imgPar.yCenter = (int)(0.5+imgPar.Y_Size*imgPar.yCenter/oldYSz);

	//Update Center
	gchar xCenterString[5];
	sprintf(xCenterString,"%d", imgPar.xCenter);
	gtk_entry_set_text(GTK_ENTRY(xCenterEntry),xCenterString);

	gchar yCenterString[5];
	sprintf(yCenterString,"%d", imgPar.yCenter);
	gtk_entry_set_text(GTK_ENTRY(yCenterEntry),yCenterString);

	updateRadius();

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

	imgPar.scaleFactor  = 2*imgPar.mapRadius;
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

	gchar scaleFactorString[11];
	sprintf(scaleFactorString,"%1.8f", imgPar.scaleFactor);

	if (imgPar.mapInside!=2)
		gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorString);
	else
		gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorString);

	remapped_expose(widget,NULL,NULL);

	return FALSE;
}

gboolean on_lp2c_XY_center_updated (GtkWidget *widget,gpointer user_data)
{
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	
	GtkWidget * xCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cXCenterEntry");
	GtkWidget * yCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cYCenterEntry");
	GtkWidget * scaleFactorEntryEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryEdit");
	GtkWidget * scaleFactorEntryNoEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryNoEdit");
	GtkWidget * radiusEntry				= lookup_widget(GTK_WIDGET (tl), "l2cRadiusEntry");

	//Update Center
	const gchar* xCenterString = gtk_entry_get_text(GTK_ENTRY(xCenterEntry));
	imgPar.xCenter = atoi (xCenterString);

	const gchar* yCenterString = gtk_entry_get_text(GTK_ENTRY(yCenterEntry));
	imgPar.yCenter = atoi (yCenterString);

	updateRadius();

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

	imgPar.scaleFactor  = 2*imgPar.mapRadius;
	imgPar.scaleFactor /= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.scaleFactor /= (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));

	gchar scaleFactorString[11];
	sprintf(scaleFactorString,"%1.8f", imgPar.scaleFactor);

	if (imgPar.mapInside!=2)
		gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorString);
	else
		gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorString);

	remapped_expose(widget,NULL,NULL);

	return FALSE;
}

void on_lp2c_circle_inside_toggled(GtkWidget *widget,gpointer user_data)
{
	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
		imgPar.mapInside = 1;

	if (imgPar.mapInside == 1)
	{
		GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));

		GtkWidget * scaleFactorEntryEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryEdit");
		GtkWidget * scaleFactorEntryNoEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryNoEdit");

		gtk_widget_hide (scaleFactorEntryEdit);
		gtk_widget_show (scaleFactorEntryNoEdit);

		updateRadius();
		on_lp2c_XY_center_updated(widget,NULL);

		gchar scaleFactorTxt[11];
		sprintf(scaleFactorTxt,"%1.8f", imgPar.scaleFactor);
		gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorTxt);
		gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorTxt);
	}
}

void on_lp2c_circle_outside_toggled(GtkWidget *widget,gpointer user_data)
{
	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
		imgPar.mapInside = 0;

	if (imgPar.mapInside == 0)
	{
		GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));

		GtkWidget * scaleFactorEntryEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryEdit");
		GtkWidget * scaleFactorEntryNoEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryNoEdit");

		gtk_widget_hide (scaleFactorEntryEdit);
		gtk_widget_show (scaleFactorEntryNoEdit);

		updateRadius();
		on_lp2c_XY_center_updated(widget,NULL);

		gchar scaleFactorTxt[11];
		sprintf(scaleFactorTxt,"%1.8f", imgPar.scaleFactor);
		gtk_entry_set_text(GTK_ENTRY(scaleFactorEntryEdit),scaleFactorTxt);
		gtk_label_set_label(GTK_LABEL(scaleFactorEntryNoEdit), scaleFactorTxt);
	}

}


void on_lp2c_circle_custom_toggled(GtkWidget *widget,gpointer user_data)
{
	if (gtk_toggle_button_get_active ((GtkToggleButton *)widget))
		imgPar.mapInside = 2;

	if (imgPar.mapInside == 2)
	{
		GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));

		GtkWidget * scaleFactorEntryEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryEdit");
		GtkWidget * scaleFactorEntryNoEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryNoEdit");

		gtk_widget_hide (scaleFactorEntryNoEdit);
		gtk_widget_show (scaleFactorEntryEdit);

		const gchar* scaleFactorTxt = gtk_entry_get_text(GTK_ENTRY(scaleFactorEntryEdit));
		imgPar.scaleFactor = atof (scaleFactorTxt);
	}

}

gboolean on_lp2c_scale_changed (GtkWidget *widget,gpointer user_data)
{
	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));
	
	GtkWidget * xSizeEntry				= lookup_widget(GTK_WIDGET (tl), "l2cXSizeEntry");
	GtkWidget * ySizeEntry				= lookup_widget(GTK_WIDGET (tl), "l2cYSizeEntry");
	GtkWidget * xCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cXCenterEntry");
	GtkWidget * yCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cYCenterEntry");
	GtkWidget * scaleFactorEntryEdit	= lookup_widget(GTK_WIDGET (tl), "l2cScaleFactorEntryEdit");
	GtkWidget * radiusEntry				= lookup_widget(GTK_WIDGET (tl), "l2cRadiusEntry");

	const gchar* scaleFactorTxt = gtk_entry_get_text(GTK_ENTRY(scaleFactorEntryEdit));
	imgPar.scaleFactor = atof (scaleFactorTxt);

	imgPar.mapRadius = imgPar.scaleFactor * (imgPar.r_zero*pow(imgPar.log_index,imgPar.rho_Size-1));
	imgPar.mapRadius *= ((imgPar.log_index+1)*cos(PI/imgPar.theta_Size)+(imgPar.log_index-1));
	imgPar.mapRadius /= 2.0;

	gchar radiusString[11];
	sprintf(radiusString,"%3.2f", imgPar.mapRadius);
	gtk_label_set_label(GTK_LABEL(radiusEntry), radiusString);

	remapped_expose(widget,NULL,NULL);

	return FALSE;
}

void on_lp2c_reset_button_clicked(GtkWidget *widget,gpointer user_data)
{
	imgPar.yCenter = imgPar.Y_Size/2;
	imgPar.xCenter = imgPar.X_Size/2;

	GtkWidget*  tl = gtk_widget_get_toplevel(GTK_WIDGET (widget));

	GtkWidget * xCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cXCenterEntry");
	GtkWidget * yCenterEntry			= lookup_widget(GTK_WIDGET (tl), "l2cYCenterEntry");

	gchar xCenterString[5];
	sprintf(xCenterString,"%d", imgPar.xCenter);
	gtk_entry_set_text(GTK_ENTRY(xCenterEntry),xCenterString);

	gchar yCenterString[5];
	sprintf(yCenterString,"%d", imgPar.yCenter);
	gtk_entry_set_text(GTK_ENTRY(yCenterEntry),yCenterString);

	on_lp2c_XY_center_updated(widget,NULL);
}