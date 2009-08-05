
void drawImagefromMemory(guchar * data, gint width, gint height);
void drawImagefromFile(GtkWidget *filechooserdialog);
void saveImagetoFile(GtkWidget *filechooserdialog);
void initPar();
void updateRadius();
//gint * loadMap(cart2LpPixel * c2LpMap,gpointer progBar);
//gint buildMap(gpointer progBar);
//void generate_Log_Polar_Image_with_Table (GdkPixbuf * pixBuf,gint sTheta, gint sRho, guchar * polarImg,cart2LpPixel * c2LpMap);
void generate_Log_Polar_Image(GdkPixbuf * pixBuf,gint sTheta, gint sRho, guchar * polarImg);
void generate_Remapped_Image_for_dialog(GdkPixbuf * pixBuf,gint sX, gint sY, guchar * remImg);
void generate_Remapped_Image(GdkPixbuf * pixBuf,gint sX, gint sY, guchar * remImg);
void generate_Remapped_Image_with_LinInt(GdkPixbuf * pixBuf,gint sX, gint sY, guchar * remImg);
void getMetrics(double * currRad, double * nextRad, double * a, double * b);
void getCoeff(double * coeff, double a, double b, double angle, double xC);
void getLimits(double * coeff, double * limits);
