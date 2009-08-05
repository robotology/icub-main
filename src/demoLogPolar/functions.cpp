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
#include <time.h>
#include <string.h>
#include <conio.h> 

extern GList * windowsList;
extern GList * pixbufList;
//extern gboolean changeZorder;
extern imagedata imgPar;

void drawImagefromFile(GtkWidget *filechooserdialog)
{
	char *filename;
	char *filenoPath;
	char *filenoExt;
	GError *error = NULL;
	GdkPixbuf *pixbuf;
	GtkWidget *window, *darea;

	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

	filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (filechooserdialog));

	filenoPath = strrchr( filename, '\\' )+1;

	gtk_widget_set_name (window,filenoPath);

	g_signal_connect (	(gpointer) window, "focus_in_event",
						G_CALLBACK (on_image_window_activation),
						NULL);

    g_signal_connect (G_OBJECT (window), "delete_event",
		      G_CALLBACK (delete_event), NULL);

	windowsList = g_list_prepend(windowsList,window);

	pixbuf = gdk_pixbuf_new_from_file (filename, &error);

	filenoExt  = strrchr( filenoPath, '.' );
	*filenoExt = '\0';
	gtk_window_set_title (GTK_WINDOW (window),filenoPath);

	pixbufList = g_list_prepend(pixbufList,pixbuf);

	if ((gdk_pixbuf_get_width(pixbuf)>gdk_screen_width())||(gdk_pixbuf_get_height(pixbuf)>gdk_screen_height()))
	{

		double ratioX = (gdk_pixbuf_get_width(pixbuf))/(double)(1.0*gdk_screen_width());
		double ratioY = (gdk_pixbuf_get_height(pixbuf))/(double)(1.0*gdk_screen_height());

		ratioX = __max(ratioX,ratioY);
		gint iRatio = (int)ratioX +1;

		const char *winTitle;
		winTitle = gtk_window_get_title (GTK_WINDOW (window));


        sprintf(filename,"%s [1:%d]",winTitle,iRatio);
		gtk_window_set_title (GTK_WINDOW (window),filename);


		GdkPixbuf *resizedpixbuf;

		gint newWid = (gint)(gdk_pixbuf_get_width (pixbuf)/iRatio);
		gint newHei = (gint)(gdk_pixbuf_get_height (pixbuf)/iRatio);

		resizedpixbuf = gdk_pixbuf_scale_simple(pixbuf,
												newWid,
												newHei,
												GDK_INTERP_NEAREST);



		darea = gtk_drawing_area_new();

		gtk_drawing_area_size (GTK_DRAWING_AREA (darea), gdk_pixbuf_get_width(resizedpixbuf),
								gdk_pixbuf_get_height(resizedpixbuf));

		gtk_container_add (GTK_CONTAINER (window), darea);

		gtk_signal_connect (GTK_OBJECT (darea), "expose-event",
						   GTK_SIGNAL_FUNC (on_darea_expose), resizedpixbuf);

	}
	else
	{
		darea = gtk_drawing_area_new();

		gtk_drawing_area_size (GTK_DRAWING_AREA (darea), gdk_pixbuf_get_width(pixbuf),
								gdk_pixbuf_get_height(pixbuf));

		gtk_container_add (GTK_CONTAINER (window), darea);

		gtk_signal_connect (GTK_OBJECT (darea), "expose-event",
						   GTK_SIGNAL_FUNC (on_darea_expose), pixbuf);

	}

	g_object_set_data (G_OBJECT (window),"darea",darea);

		g_free (filename);

		gtk_widget_show_all (window);
}


void saveImagetoFile(GtkWidget *filechooserdialog)
{
	//TODO: Save to Bitmap

	char *filename;
	GError *error = NULL;
	GdkPixbuf *pixbuf;

	filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (filechooserdialog));

	pixbuf = (GdkPixbuf *) g_list_nth_data(pixbufList,0); //Gets a pointer to current pixbuf

	gdk_pixbuf_save (pixbuf,filename,"jpeg", &error,
                 "quality", "100", NULL);

	g_free (filename);
}

void updateRadius()
{
	gint xDist, yDist;

	if (imgPar.mapInside==1)
	{
		xDist = __min(imgPar.xCenter,imgPar.X_Size-imgPar.xCenter);
		yDist = __min(imgPar.yCenter,imgPar.Y_Size-imgPar.yCenter);

		imgPar.mapRadius  = __min(xDist,yDist);
	}
	else if (imgPar.mapInside==0)

	{
		xDist = __max(imgPar.xCenter,imgPar.X_Size-imgPar.xCenter);
		yDist = __max(imgPar.yCenter,imgPar.Y_Size-imgPar.yCenter);
		imgPar.mapRadius  = sqrt(double(xDist*xDist+yDist*yDist));
	}

}


void generate_Log_Polar_Image(GdkPixbuf * pixBuf,gint sTheta, gint sRho, guchar * polarImg)
{
	gint xSize = gdk_pixbuf_get_width(pixBuf);
	gint ySize = gdk_pixbuf_get_height(pixBuf);
	gint sqSize = __min(xSize,ySize);
	gint rho,theta;

	gdouble radius;
	gdouble angle;

	guchar * inputImg = gdk_pixbuf_get_pixels(pixBuf);

	gint x;
	gint y;

	double co,si;

	double firstRing = 0.5 * ((imgPar.log_index/(imgPar.log_index-1)) - imgPar.fovea);

	for (rho=0; rho<__min(imgPar.fovea,sRho); rho++)
	{
		radius = imgPar.scaleFactor *(rho+2*firstRing-0.5);

		for (theta=0; theta<sTheta; theta++)
		{
			angle = -2*PI*(theta+0.5)/sTheta;

			co = radius * cos(angle);
			si = radius * sin(angle);

			if(co>0)
				x = (gint)(radius*cos(angle)+0.0)+imgPar.xCenter;
			else
				x = ((gint)(radius*cos(angle)+0.0)+imgPar.xCenter)-1;

			if (si>0)
				y = (gint)(radius*sin(angle)+0.0)+imgPar.yCenter;
			else
				y = ((gint)(radius*sin(angle)+0.0)+imgPar.yCenter)-1;


			if ((x<xSize)&&(y<ySize))
				if ((x>=0)&&(y>=0))
				{
					polarImg[3*(rho*sTheta+theta)] = inputImg[3*(y*xSize+x)];
					polarImg[3*(rho*sTheta+theta)+1] = inputImg[3*(y*xSize+x)+1];
					polarImg[3*(rho*sTheta+theta)+2] = inputImg[3*(y*xSize+x)+2];
				}
		}
	}

	for (rho=imgPar.fovea; rho<sRho; rho++)
	{
		radius = imgPar.scaleFactor * pow(imgPar.log_index, rho)*(imgPar.r_zero * 0.5*(imgPar.log_index+1));

		for (theta=0; theta<sTheta; theta++)
		{
			angle = -2*PI*(theta+0.5)/sTheta;

			co = radius * cos(angle);
			si = radius * sin(angle);

			if(co>0)
				x = (gint)(radius*cos(angle)+0.0)+imgPar.xCenter;
			else
				x = ((gint)(radius*cos(angle)+0.0)+imgPar.xCenter)-1;

			if (si>0)
				y = (gint)(radius*sin(angle)+0.0)+imgPar.yCenter;
			else
				y = ((gint)(radius*sin(angle)+0.0)+imgPar.yCenter)-1;

			if ((x<xSize)&&(y<ySize))
				if ((x>=0)&&(y>=0))
				{
					polarImg[3*(rho*sTheta+theta)] = inputImg[3*(y*xSize+x)];
					polarImg[3*(rho*sTheta+theta)+1] = inputImg[3*(y*xSize+x)+1];
					polarImg[3*(rho*sTheta+theta)+2] = inputImg[3*(y*xSize+x)+2];
				}
		}
	}
}


void generate_Remapped_Image(GdkPixbuf * pixBuf,gint sX, gint sY, guchar * remImg)
{
	gint sTheta = gdk_pixbuf_get_width(pixBuf);
	gint sRho = gdk_pixbuf_get_height(pixBuf);
	gint sqSize = __min(sTheta,sRho);
	gint rho,theta;

	gdouble dTheta;

	gdouble radius;

	guchar * inputImg = gdk_pixbuf_get_pixels(pixBuf);

	gint x;
	gint y;

	gint sqX,sqY;

	double firstRing = 0.5 * ((imgPar.log_index/(imgPar.log_index-1)) - imgPar.fovea);

	gdouble powLambda = pow(imgPar.log_index,imgPar.fovea);
	gdouble logLambda = log(imgPar.log_index);
	gdouble div = (imgPar.r_zero + 0.5/powLambda);

	for (y=0; y<sY; y++)
	{
		sqY = (y-imgPar.yCenter)*(y-imgPar.yCenter);

		for (x=0; x<sX; x++)
		{
			sqX = (x-imgPar.xCenter)*(x-imgPar.xCenter);

			radius = sqrt((double)(sqX+sqY))/ imgPar.scaleFactor;

			if (radius >imgPar.fovea)
			{
				rho = (gint)(log(radius/div)/logLambda);
				dTheta = (sTheta*atan2((double)((y-imgPar.yCenter)),(double)((x-imgPar.xCenter))/(2*PI)));
				if (dTheta<0)
					dTheta+=sTheta;
				theta = (gint)(dTheta);

				if ((rho>0)&&(theta>=0))
					if ((rho<sRho)&&(theta<sTheta))
					{
						remImg[3*(y*sX+x)]   = inputImg[3*(rho*sTheta+theta)];
						remImg[3*(y*sX+x)+1] = inputImg[3*(rho*sTheta+theta)+1];
						remImg[3*(y*sX+x)+2] = inputImg[3*(rho*sTheta+theta)+2];

					}
			}
			else
			{
				rho = (gint)((radius)-firstRing+0.5);
				if (radius == 0)
					rho = 0;
				dTheta = (sTheta*atan2((double)(imgPar.yCenter-y),(double)((x-imgPar.xCenter))/(2*PI)));
				if (dTheta<0)
					dTheta+=sTheta;
				theta = (gint)(dTheta);
				if ((rho>0)&&(theta>=0))
					if ((rho<sRho)&&(theta<sTheta))
					{
						remImg[3*(y*sX+x)]   = inputImg[3*(rho*sTheta+theta)];
						remImg[3*(y*sX+x)+1] = inputImg[3*(rho*sTheta+theta)+1];
						remImg[3*(y*sX+x)+2] = inputImg[3*(rho*sTheta+theta)+2];

					}
			}
		}
	}
}

void generate_Remapped_Image_with_LinInt(GdkPixbuf * pixBuf,gint sX, gint sY, guchar * remImg)
{
	gint sTheta = gdk_pixbuf_get_width(pixBuf);
	gint sRho = gdk_pixbuf_get_height(pixBuf);
	gint sqSize = __min(sTheta,sRho);
//	gint rho,theta;

//	gdouble dTheta;

	gdouble radius;

	guchar * inputImg = gdk_pixbuf_get_pixels(pixBuf);

	gint x;
	gint y;

	double firstRing = 0.5 * ((imgPar.log_index/(imgPar.log_index-1)) - imgPar.fovea);
////////////////////////////////////////////
	double jrho, itheta;
	int rhom, rhop;
	double drho;
	int thetam, thetap;
	double dtheta;
	double redval;
	double greval;
	double bluval;
	double w1,w2,w3,w4;
	int pos1,pos2,pos3,pos4;

	for (y=0; y<sY; y++)
		for (x=0; x<sX; x++)
		{
			radius = sqrt((double)((x-imgPar.xCenter)*(x-imgPar.xCenter)+(y-imgPar.yCenter)*(y-imgPar.yCenter)));

			if (radius/ imgPar.scaleFactor >imgPar.fovea)
			{
				jrho = log((radius/imgPar.scaleFactor)/(imgPar.r_zero + 0.5/pow(imgPar.log_index,imgPar.fovea)))/log(imgPar.log_index);
			}
			else
			{
				jrho = (radius / imgPar.scaleFactor)-firstRing+0.5;

				if (radius == 0)
					jrho = 0;
			}

			itheta = (sTheta/2)*(1.0+(atan2((double)(y-imgPar.yCenter),(double)(-(x-imgPar.xCenter))/PI)))-0.5;

			// here we have rho and theta coordinates of the point (both double)

			if (itheta>=sTheta)
				itheta -=sTheta;

			if (itheta<0)
				itheta +=sTheta;

			rhom = (int)(jrho);
			rhop = rhom+1;
			drho = jrho - rhom;


			thetam = (int)(itheta);

			if (thetam>=sTheta)
				thetam -=sTheta;
			if (thetam<0)
				thetam +=sTheta;
			
			thetap = thetam+1;

			if (thetap>=sTheta)
				thetap-=sTheta;
			if (thetap<0)
				thetap+=sTheta;

			dtheta = itheta - thetam;

			w1 = (1.0-drho)*(1.0-dtheta);
			w2 = (1.0-drho)*(    dtheta);
			w3 = (    drho)*(1.0-dtheta);
			w4 = (    drho)*(    dtheta);

			pos1 = 3*(rhom*sTheta+thetam);
			pos2 = 3*(rhom*sTheta+thetap);
			pos3 = 3*(rhop*sTheta+thetam);
			pos4 = 3*(rhop*sTheta+thetap);


			if ((jrho<sRho-1)&&(jrho>-10.0))
			{
				redval =  w1 * inputImg[pos1];
				redval += w2 * inputImg[pos2];
				redval += w3 * inputImg[pos3];
				redval += w4 * inputImg[pos4];

				greval =  w1 * inputImg[pos1+1];
				greval += w2 * inputImg[pos2+1];
				greval += w3 * inputImg[pos3+1];
				greval += w4 * inputImg[pos4+1];

				bluval =  w1 * inputImg[pos1+2];
				bluval += w2 * inputImg[pos2+2];
				bluval += w3 * inputImg[pos3+2];
				bluval += w4 * inputImg[pos4+2];

				remImg[3*((y)*sX+x)+0] = (unsigned char)(redval+0.5);
				remImg[3*((y)*sX+x)+1] = (unsigned char)(greval+0.5);
				remImg[3*((y)*sX+x)+2] = (unsigned char)(bluval+0.5);
			}
			else
			{
				remImg[3*((y)*sX+x)+0] = 192;
				remImg[3*((y)*sX+x)+1] = 192;
				remImg[3*((y)*sX+x)+2] = 192;
			}
			
		}
}

void drawImagefromMemory(guchar * data, gint width, gint height)
{
	GError *error = NULL;
	GdkPixbuf *pixbuf;
	GtkWidget *window, *darea;

	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title (GTK_WINDOW (window),"Processed Image");

	g_signal_connect (	(gpointer) window, "focus_in_event",
						G_CALLBACK (on_image_window_activation),
						NULL);
    g_signal_connect (G_OBJECT (window), "delete_event",
		      G_CALLBACK (delete_event), NULL);

	windowsList = g_list_prepend(windowsList,window);

//	printf("Cart Img address : %xd\n", data);

	pixbuf = gdk_pixbuf_new_from_data(	data,
										GDK_COLORSPACE_RGB,
										FALSE,
										8,
										width,
										height,
										width*3,
										NULL,
										NULL);

	pixbufList = g_list_prepend(pixbufList,pixbuf);

	darea = gtk_drawing_area_new();

	gtk_drawing_area_size (	GTK_DRAWING_AREA (darea), gdk_pixbuf_get_width(pixbuf),
							gdk_pixbuf_get_height(pixbuf));

	gtk_container_add (GTK_CONTAINER (window), darea);

	gtk_signal_connect (GTK_OBJECT (darea), "expose-event",
						GTK_SIGNAL_FUNC (on_darea_expose), pixbuf);

	g_object_set_data (G_OBJECT (window),"darea",darea);


	gtk_widget_show_all (window);
}
/*
void generate_Log_Polar_Image_with_Table (GdkPixbuf * pixBuf,gint sTheta, gint sRho, guchar * polarImg,cart2LpPixel * c2LpMap)
{
	int i,j;
	guchar * InImg = gdk_pixbuf_get_pixels(pixBuf);
	double sumR,sumG,sumB;
	for (j=0; j<imgPar.rho_Size*imgPar.theta_Size; j++)
	{
		sumR = 0;
		sumG = 0;
		sumB = 0;

		for (i=0; i<c2LpMap[j].divisor; i++)
		{
			sumR += InImg [c2LpMap[j].position[i]+0]*c2LpMap[j].weight[i];
			sumG += InImg [c2LpMap[j].position[i]+1]*c2LpMap[j].weight[i];
			sumB += InImg [c2LpMap[j].position[i]+2]*c2LpMap[j].weight[i];
		}
		polarImg[3*j+0] = (unsigned char) sumR;
		polarImg[3*j+1] = (unsigned char) sumG;
		polarImg[3*j+2] = (unsigned char) sumB;
	}
}
*/
/*
gint buildMap(gpointer progBar)
{
	double rfXcent;

	double * currRad;
	double * nextRad;
	double * a;
	double * b;
	double coeff[6];
	double limits[4];
	double angle;
	double funct;

	double xCoord, yCoord;
	double xCoord2, yCoord2;

	double loYl, hiYl;
	double loXl, hiXl;

	int width = (int)(2*imgPar.mapRadius);

	double hprecision;
	double vprecision;
	int addval;
	int i,j, x,y;
	int iSz3 = 3 * width;
	double h,k;
	int l;


	a = new double [imgPar.rho_Size];
	b = new double [imgPar.rho_Size];
	currRad = new double [imgPar.rho_Size];
	nextRad = new double [imgPar.rho_Size];

	cart2LpPixel * c2LpMap;
	c2LpMap = new cart2LpPixel[imgPar.theta_Size*imgPar.rho_Size];


	int loYlim;
	int hiYlim;
	int loXlim;
	int hiXlim;

	int hsize;
	int vsize;
	int psize;
	int count;
	int pos;

	int minX = 82;
	int maxX = -1;;

	getMetrics(currRad,nextRad,a,b);

	double w0,w1,w2,w3;
	double sum = 0;

	gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Building Table"));
	gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);

	for (j=0; j< imgPar.rho_Size; j++)
	{
		rfXcent = imgPar.scaleFactor * currRad[j];

		gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), (double)j/double(imgPar.rho_Size-1));
		gtk_widget_show (GTK_WIDGET(progBar));

		while (gtk_events_pending()) gtk_main_iteration_do(FALSE);

		for (i=0; i<imgPar.theta_Size; i+=1)
		{
			angle = 2*PI*(i+0.5)/imgPar.theta_Size;

			getCoeff(coeff,a[j],b[j],angle,rfXcent);
			getLimits(coeff,limits);

			hprecision = 33;  //3 vuol dire centro + 2 bordi
			vprecision = 33;

			addval = (int)(vprecision*hprecision/4.0);

			loYlim = (int)((width/2)-limits[0]-0.5);
			hiYlim = (int)((width/2)-limits[1]+0.5);
			loXlim = (int)(limits[2]+(width/2)-0.5);
			hiXlim = (int)(limits[3]+(width/2)+0.5);

			if (hiYlim>=width)
				hiYlim = width-1;

			if (loYlim<0)
				loYlim = 0;

			if (hiXlim>=width)
				hiXlim = width-1;

			if (loXlim<0)
				loXlim = 0;

			hsize = (hiXlim-loXlim+1);
			vsize = (hiYlim-loYlim+1);
			psize = hsize*vsize;

			c2LpMap[j*imgPar.theta_Size+i].position = new int [psize];
			c2LpMap[j*imgPar.theta_Size+i].weight = new double [psize];
			c2LpMap[j*imgPar.theta_Size+i].divisor = psize;

			for (l=0; l<psize; l++)
			{
				c2LpMap[j*imgPar.theta_Size+i].weight[l] = 0.0;
				c2LpMap[j*imgPar.theta_Size+i].position[l] = -1;
			}

			for (y=hiYlim; y>=loYlim; y--)
			{
				yCoord = (width/2-y)-0.5;
//				yCoord = (width/2-y)-0.0;
				for (x=loXlim; x<=hiXlim; x++)
				{
					count = 0;

					xCoord = (x-width/2)+0.5;
//					xCoord = (x-width/2)+0.0;

					funct  = xCoord*xCoord*coeff[0];
					funct += xCoord*yCoord*coeff[1];
					funct += yCoord*yCoord*coeff[2];
					funct += xCoord		  *coeff[3];
					funct += yCoord		  *coeff[4];
					funct += coeff[5];

					pos = 3*(y*width+x);

					if (funct <= 0)
						count ++;

					if (funct <= -coeff[3]-coeff[1]*yCoord-2*coeff[0]*xCoord-coeff[0])//x+1
						count ++;

					if (funct <= coeff[4]+coeff[1]*xCoord+2*coeff[2]*yCoord-coeff[2])//y-1
						count ++;

					if (funct <= coeff[4]+coeff[1]*xCoord+2*coeff[2]*yCoord-coeff[2]-coeff[3]-coeff[1]*yCoord-2*coeff[0]*xCoord-coeff[0]+coeff[1])//x+1 & y-1
						count ++;

					if (count == 4)
					{
						if (((hiYlim-y+1)*hsize+(x-loXlim+1))>=psize)
						{
							y=y;
						}
						else
						{
							c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y)*hsize+(x-loXlim))]=pos;
							c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y+1)*hsize+(x-loXlim))]=pos+iSz3;
							c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y)*hsize+(x-loXlim+1))]=pos+3;
							c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y+1)*hsize+(x-loXlim+1))]=pos+iSz3+3;

							c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y)*hsize+(x-loXlim))]+=addval;
							c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y+1)*hsize+(x-loXlim))]+=addval;
							c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y)*hsize+(x-loXlim+1))]+=addval;
							c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y+1)*hsize+(x-loXlim+1))]+=addval;

							if (x<minX)
								minX = x;
							if (x>maxX)
								maxX = x;
						}
					}
					else 
					{						
						if ((xCoord>=limits[2]-1)&&(xCoord<=limits[3]))//ok
						{
							if ((yCoord>=limits[1]-1)&&(yCoord<=limits[0]))//////////////////
							{
								if (limits[1]<=yCoord)
								{
									if (limits[1]>=0)
										loYl = -yCoord+limits[1]+((limits[0]-limits[1])/(vprecision-1))*(1+(int)((yCoord-limits[1])*(vprecision-1)/(limits[0]-limits[1])));
									else
										loYl = -yCoord+limits[1]+((limits[0]-limits[1])/(vprecision-1))*(0+(int)((yCoord-limits[1])*(vprecision-1)/(limits[0]-limits[1])));
								}
								else
									loYl = -yCoord+limits[1];

								if (loYl<0.0)
									loYl = loYl + ((limits[0]-limits[1])/(vprecision-1));

								if (limits[0]<yCoord+1)
									hiYl = -yCoord+limits[0];
								else if (limits[0]>=0)
									hiYl = -yCoord+limits[1]+((limits[0]-limits[1])/(vprecision-1))*(1+(int)((yCoord+1-limits[1])*(vprecision-1)/(limits[0]-limits[1])));
								else
									hiYl = -yCoord+limits[1]+((limits[0]-limits[1])/(vprecision-1))*(0+(int)((yCoord+1-limits[1])*(vprecision-1)/(limits[0]-limits[1])));

								if (hiYl>1.0)
									hiYl = hiYl - ((limits[0]-limits[1])/(vprecision-1));
								for (k=loYl; k<=hiYl; k+=(limits[0]-limits[1])/(vprecision-1))
								
								{
									yCoord2 = yCoord+k;

									if (limits[2]<=xCoord)
									{
										if (limits[2]>=0)
											loXl = -xCoord+limits[2]+((limits[3]-limits[2])/(hprecision-1))*(1+(int)((xCoord-limits[2])*(hprecision-1)/(limits[3]-limits[2])));
										else
											loXl = -xCoord+limits[2]+((limits[3]-limits[2])/(hprecision-1))*(0+(int)((xCoord-limits[2])*(hprecision-1)/(limits[3]-limits[2])));
									}
									else
										loXl = -xCoord+limits[2];

									if (loXl<0.0)
										loXl = loXl + ((limits[3]-limits[2])/(hprecision-1));

									if (limits[3]<xCoord+1)
										hiXl = -xCoord+limits[3];
									else if (limits[3]>=0)
									
										hiXl = -xCoord+limits[2]+((limits[3]-limits[2])/(hprecision-1))*(1+(int)((xCoord+1-limits[2])*(hprecision-1)/(limits[3]-limits[2])));
									else
										hiXl = -xCoord+limits[2]+((limits[3]-limits[2])/(hprecision-1))*(0+(int)((xCoord+1-limits[2])*(hprecision-1)/(limits[3]-limits[2])));
									
									if (hiXl>1.0)
										hiXl = hiXl - ((limits[3]-limits[2])/(hprecision-1));

									for (h=loXl; h<=hiXl+1.0e-010; h+=(limits[3]-limits[2])/(hprecision-1))
									{
										xCoord2 = xCoord+h;

										funct  = xCoord2*xCoord2*coeff[0];
										funct += xCoord2*yCoord2*coeff[1];
										funct += yCoord2*yCoord2*coeff[2];
										funct += xCoord2	    *coeff[3];
										funct += yCoord2	    *coeff[4];
										funct +=				 coeff[5];

										if (funct <= 1.0e-010)
										{ 
											w0 = (1.0-k)*(1.0-h);
											w2 = (1.0-k)*(h);
								
											w1 =	 (k)*(1.0-h);
											w3 =	 (k)*	 (h);
											
											if (((hiYlim-y+1)*hsize+(x-loXlim+1))>=psize)
											{
												x=x;
											}
											else
											{
												c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y)*hsize+(x-loXlim))]=pos;
												c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y+1)*hsize+(x-loXlim))]=pos-iSz3;
												c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y)*hsize+(x-loXlim+1))]=pos+3;
												c2LpMap[j*imgPar.theta_Size+i].position[((hiYlim-y+1)*hsize+(x-loXlim+1))]=pos-iSz3+3;

												c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y)*hsize+(x-loXlim))]+=w0;///(vprecision*hprecision);
												c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y+1)*hsize+(x-loXlim))]+=w1;///(vprecision*hprecision);
												c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y)*hsize+(x-loXlim+1))]+=w2;///(vprecision*hprecision);
												c2LpMap[j*imgPar.theta_Size+i].weight[((hiYlim-y+1)*hsize+(x-loXlim+1))]+=w3;///(vprecision*hprecision);

												if (x<minX)
													minX = x;
												if (x>maxX)
													maxX = x;
											}
										}
									}
								}
							}
						}
					}
				}
			}


			for (l=0; l<hsize*vsize; l++)
				if (c2LpMap[j*imgPar.theta_Size+i].position[l]<0)
				{
					c2LpMap[j*imgPar.theta_Size+i].position[l] = 0;
					c2LpMap[j*imgPar.theta_Size+i].weight[l] = 0.0;
				}

			sum = 0;

			for (l=0; l<hsize*vsize; l++)
				sum += c2LpMap[j*imgPar.theta_Size+i].weight[l];

			if (sum > 0)
				for (l=0; l<hsize*vsize; l++)
					c2LpMap[j*imgPar.theta_Size+i].weight[l] = c2LpMap[j*imgPar.theta_Size+i].weight[l]/sum;

		}
	}

	gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Saving Table"));
	gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);
	char filename [256];

	sprintf(filename,"c2lp %dx%d - %d.lut",imgPar.theta_Size, imgPar.rho_Size, width);

	FILE * fout = fopen(filename,"wb");

	for (i=0; i<imgPar.theta_Size*imgPar.rho_Size; i++)
	{
		if (!(i%imgPar.theta_Size))
		{
			gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), (double)i/double(imgPar.theta_Size*imgPar.rho_Size-1));
			gtk_widget_show (GTK_WIDGET(progBar));
		}

		while (gtk_events_pending()) gtk_main_iteration_do(FALSE);

		fwrite(&c2LpMap[i].divisor ,sizeof(int)   ,1,fout);
		fwrite( c2LpMap[i].position,sizeof(int)   ,c2LpMap[i].divisor,fout);
		fwrite( c2LpMap[i].weight  ,sizeof(double),c2LpMap[i].divisor,fout);
	}
		
	fclose (fout);

	return 0;
}
*/
/*
gint * loadMap(cart2LpPixel * c2LpMap,gpointer progBar)
{
	int j;
	char filename [256];
	FILE * fin;

	gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progBar), _("Loading Table"));
	gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), 0.0);

	sprintf(filename,"c2lp %dx%d - %d.lut",imgPar.theta_Size, imgPar.rho_Size, (int)(2*imgPar.mapRadius));

	if ((fin = fopen(filename,"rb")) != NULL)
	{
		for (j=0; j<imgPar.theta_Size*imgPar.rho_Size; j++)
		{
			if (!(j%imgPar.theta_Size))
			{
				gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progBar), (double)j/double(imgPar.theta_Size*imgPar.rho_Size-1));
				gtk_widget_show (GTK_WIDGET(progBar));
			}

			while (gtk_events_pending()) gtk_main_iteration_do(FALSE);

			fread(&c2LpMap[j].divisor,sizeof(int),1,fin);
			c2LpMap[j].position = new int [c2LpMap[j].divisor];
			fread(c2LpMap[j].position,sizeof(int),c2LpMap[j].divisor,fin);
			c2LpMap[j].weight = new double [c2LpMap[j].divisor];
			fread(c2LpMap[j].weight,sizeof(double),c2LpMap[j].divisor,fin);
		}
		fclose (fin);
		return (int*)c2LpMap;
	}
	else
		return NULL;
}
*/
void getMetrics(double * currRad, double * nextRad, double * a, double * b/*int rho, int theta, int ImgSize, int * fov, , double scaleFact, double overlap*/)
{
	int j;

	double sinus = sin (PI/imgPar.theta_Size);	
	double tangent = sinus/cos(PI/imgPar.theta_Size);


	double firstRing = 0.5 * ((imgPar.log_index/(imgPar.log_index-1)) - imgPar.fovea);
//	double firstRing = 1.0 * ((imgPar.log_index/(imgPar.log_index-1)) - imgPar.fovea);

	double Q,L;

	imgPar.overlap = 0;

	for (j=0; j<imgPar.rho_Size; j++)
	{
		if (j<imgPar.fovea)
			if (j==0)
			{
				currRad[j] = firstRing;	
				nextRad[j] = 2*firstRing+0.5;
			}
			else
			{
				currRad[j] = j+2*firstRing-0.5;
				nextRad[j] = currRad[j]+1.0;		
			}
		else
		{
			currRad[j] = pow(imgPar.log_index,j) * (imgPar.r_zero * 0.5*(imgPar.log_index+1));
			nextRad[j] = imgPar.log_index * currRad[j];
		}

		b[j] = imgPar.scaleFactor * currRad[j] * sinus * (imgPar.overlap + 1.0);

		a[j] = imgPar.scaleFactor * (nextRad[j] - currRad[j]) * (imgPar.overlap + 1.0);

		if (j<imgPar.fovea)
			a[j] /= 2.0;
		else
			a[j] /= (1.0+imgPar.log_index);//se moltiplico per 1+lambda e overlap è 0, i RF sono tangenti lungo i raggi

		if (j<imgPar.fovea)
		{
			Q = a[j] * a[j];
			L = imgPar.scaleFactor * currRad[j] * (imgPar.overlap + 1.0);
			L = L*L;
			b[j] = tangent*sqrt(fabs(L-Q));
		}

		if (j==0)
			a[j] = 0.5 * imgPar.scaleFactor * (firstRing)*(imgPar.overlap+1.0);
	}

//	double sinn = currRad[151]*imgPar.scaleFactor*cos(PI/imgPar.theta_Size)+b[151];
}


void getCoeff(double * coeff, double a, double b, double angle, double xC)
{
	double co__ = cos(angle);
	double si__ = sin(angle);
	double cosi = co__*si__;
	double cos2 = co__*co__;
	double sin2 = si__*si__;
	double aa = a*a;
	double bb = b*b;

	coeff[0] = bb*cos2+aa*sin2;
	coeff[1] = 2*cosi* (bb-aa);
	coeff[2] = bb*sin2+aa*cos2;
	coeff[3] = 2*(- bb*xC*co__);
	coeff[4] =-2*(bb*xC*si__);
	coeff[5] = bb*xC*xC - aa*bb;
}

void getLimits(double * coeff, double * limits)
{
	double A = coeff[0];
	double B = coeff[1];
	double C = coeff[2];
	double D = coeff[3];
	double E = coeff[4];
	double F = coeff[5];

	double BB = B*B;
	double AC = A*C;
	double DD = D*D;
	double EE = E*E;

	double swap;

	//Top
	limits[0] = 4*A*E-2*B*D+sqrt(4*BB*DD+16*A*A*EE-16*A*B*D*E-4*(BB-4*AC)*(DD-4*A*F));
	limits[0] = limits[0] /(2*BB-8*AC);
	//Bottom
	limits[1] = 4*A*E-2*B*D-sqrt(4*BB*DD+16*A*A*EE-16*A*B*D*E-4*(BB-4*AC)*(DD-4*A*F));
	limits[1] = limits[1] /(2*BB-8*AC);
	//Left
	limits[2] = 4*C*D-2*B*E+sqrt(4*BB*EE+16*C*C*DD-16*C*B*D*E-4*(BB-4*AC)*(EE-4*C*F));
	limits[2] = limits[2] /(2*BB-8*AC);
	//Right
	limits[3] = 4*C*D-2*B*E-sqrt(4*BB*EE+16*C*C*DD-16*C*B*D*E-4*(BB-4*AC)*(EE-4*C*F));
	limits[3] = limits[3] /(2*BB-8*AC);

	if (limits[1]>=limits[0])
	{
		swap = limits[0];
		limits[0] = limits[1];
		limits[1] = swap;
	}
	if (limits[2]>=limits[3])
	{
		swap = limits[2];
		limits[2] = limits[3];
		limits[3] = swap;
	}
}
