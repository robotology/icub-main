// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/LogPolarModule.h>


/**
* two possible definitions SIMULATION, NOSIMULATION
*/
#define NOSIMULATION

/**
*function that opens the module
*/
bool LogPolarModule::open(Searchable& config) {
    ct = 0;
    port.open(getName("in"));
	//ConstString portName2 = options.check("name",Value("/worker2")).asString();
	port2.open(getName("out"));
	port3.open(getName("outInverse"));
	port4.open(getName("outSimulation"));
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
	
    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool LogPolarModule::interruptModule() {
    port.interrupt();
	port2.interrupt();
	port3.interrupt();
	port4.interrupt();
	cmdPort.interrupt();
    return true;
}


bool LogPolarModule::close(){
	port.close();
	port2.close();
	port3.close();
	port4.close();
	cmdPort.close();
    return true;
}

void LogPolarModule::setOptions(yarp::os::Property opt){
	options	=opt;
	// definition of the mode
	ConstString optcheck=opt.find("mode").asString();
	printf("Working in modality:%s \n", optcheck.c_str());
	printf("\n");
	if(!strcmp(optcheck.c_str(),"SIMULATION"))
		mode=0;
	else if(!strcmp(optcheck.c_str(),"FORWARD")){
		mode=1;
		//define the sequence of images for the forward mode
		dstColor= cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3 );
		dstColor2= cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3 );
		cvImage= cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 );
		cvImage2= cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 );
		image2=new ImageOf<PixelRgb>;
		image2->resize(320,240);
	}
	else if(!strcmp(optcheck.c_str(),"INVERSE")){
		mode=2;
		//creates the sequence of images for the inverse mode
		dstColor= cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3 );
		dstColor2= cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3 );
		cvImage= cvCreateImage(cvSize(240,240), IPL_DEPTH_8U, 3 );
		cvImage1=cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 );
		cvImage2= cvCreateImage(cvSize(240,240), IPL_DEPTH_8U, 3 );
		image2=new ImageOf<PixelRgb>;
		image2->resize(320,240);
		
		rec=cvRect(40,0,240,240);
	}
	// definition of the name of the module
	ConstString name=opt.find("name").asString();
	printf("Module named as :%s \n", name.c_str());
	this->setName(name.c_str());
	printf("\n");
}

bool LogPolarModule::updateModule() {
    //initialisation
	int nEcc = 250;
	int nAng = 350;
	//cart2LpPixel *c2lTable;
	//lp2CartPixel *l2cTable;
	char path[] = "./";
	CvScalar s;

	
	//set in grays the image
	img = port.read(false);
	if(img==NULL){
		return true;	
	}
    printf(" %d \n",img->width());
	printf(" %d \n",img->height());
	if(mode==0){

		//-------------drawing of simulated blobs
		// add a blue circle
		ct = 320/2;
		PixelRgb blue(0,0,255);
		PixelRgb red(255,0,0);
		PixelRgb green(0,255,0);
		PixelRgb yellow(255,255,0);
		PixelRgb black(0,0,0);
		addRectangle(*image2,black,0,0,320,240);
		//addCircle(*image2,blue,ct,50,15);
		addCircle(*image2,red,ct-50,100,20);
		//addCircle(*image2,green,ct+50,150,25);
		//addCircle(*image2,yellow,ct,200,10);
		
		// if simulation is active the simulated image is prepared on the port
		//otherwise the input image is present on the port
		port4.prepare() = *image2;
		port4.write();
	}
	else if(mode==1){
		if (img==NULL) 
			return true;
		printf("FORWARD \n");
		
		cvCopy(img->getIplImage(),cvImage2);
		cvCvtColor(cvImage2,cvImage,CV_RGB2BGR);
		//cvCvtColor((IplImage*)img->getIplImage(), cvImage, CV_RGB2GRAY);
		

		//ImageOf<PixelMono> returnCvImage;
		//returnCvImage.wrapIplImage(cvImage);
		//ImageOf<PixelMono> *returnCvImagePointer=NULL;
		//yarpReturnImagePointer=bayerLPImage;
		//returnCvImagePointer=&returnCvImage;
		
		//unsigned char *bayerCartesian=new unsigned char[img->width() * img->height()];

		int xSize=img->width(), ySize=img->height(), planes=0;
		/*unsigned char *bayerCartesian =
			Load_Bitmap (&xSize, &ySize, &planes, img);*/
		//unsigned char *bayerCartesian =Load_Bitmap (&xSize, &ySize, &planes, "./bayercartesian.bmp");
		//Save_Bitmap (bayerCartesian, xSize, ySize, planes, "./bayercartesian.bmp");
		//unsigned char *bayerCartesianInit=new unsigned char[img->width() * img->height()];
		//bayerCartesianInit=bayerCartesian;
		//delete img;
		//printf("Showing OpenCV/IPL image cvImage->height %d img->height %d planes %d \n",xSize,ySize,planes);



		//--------conversion to logPolar----------------
		/*int k=0;
		for(int i=0;i<xSize;i++){
			for(int j=0;j<ySize;j++){
				k++;
				//s=cvGet2D(cvImage,j,i); // get the (i,j) pixel value
				
				//getch();
				//unsigned char value=(unsigned char)floor(s.val[0]);
				
				//*bayerCartesian=(unsigned char) value;
				//printf("intensity=%d",(unsigned char)*bayerCartesian);	
				//if((j==0)||(i==0))
				//	bayerCartesianInit=bayerCartesian;
				//bayerCartesian=((uchar *)(cvImage->imageData + i*cvImage->widthStep))[j];						
				//bayerCartesian++;
			}
		}*/
		
		
		/*unsigned char *bayerLP = new unsigned char[nEcc * nAng];
		unsigned char *bayerLPInit=new unsigned char[nEcc * nAng];
		bayerLPInit=bayerLP;
		if (bayerLP == NULL)
		{
			exit (-1);
		}
		c2lTable = new cart2LpPixel[nEcc * nAng];
		if (c2lTable == NULL)
		{
			exit (-1);
		}
		printf("Image width=%d height=%d  pixelSize=%d \n", xSize,ySize,planes);
		
		double overlap=0;
		double scaleFact = RCcomputeScaleFactor (nEcc, nAng, xSize,ySize, overlap);

		int returnValue=RCbuildC2LMapBayer(nEcc, nAng, xSize,ySize, overlap,scaleFact, ELLIPTICAL,path);
		printf("return from RCbuildC2LMap: %d \n",returnValue);
		returnValue=RCallocateC2LTable (c2lTable, nEcc, nAng, 1, path);
		printf("return from RCallocateC2LMap: %d \n",returnValue);
		RCgetLpImg (bayerLP, bayerCartesian, c2lTable, nAng * nEcc, 1);
		//Save_Bitmap (bayerLP, nAng, nEcc, 1, "./TestlpBayer.bmp");
		RCdeAllocateC2LTable (c2lTable);*/
		
		//convert the unsigned char pointer to an YARPImage
		//IplImage* dst = cvCreateImage( cvSize(xSize,ySize), 8, 1 );
		
		//ImageOf<PixelMono> *bayerLPImage=new ImageOf<PixelMono>;
		//unsigned char* pImage=bayerLPImage->getRawImage();
		//pImage=bayerCartesianInit;
		//--
		/*bayerCartesian=bayerCartesianInit;
		for(int i=0;i<xSize;i++){
			for(int j=0;j<ySize;j++){
				//s.val[0]=*bayerLP;
				unsigned char value=*bayerCartesian;
				s.val[0]=(float)value;
				cvSet2D(dst,j,i,s); // set the (i,j) pixel value
				bayerCartesian++; //red
				bayerCartesian++; //green
				bayerCartesian++; // blue
			}
		}*/

		/*bayerLP=bayerLPInit;
		for(int i=0;i<nAng;i++){
			for(int j=0;j<nEcc;j++){
				unsigned char value=*bayerLP;
				s.val[0]=(float)value;
				cvSet2D(dst,j,i,s); // set the (i,j) pixel value
				bayerLP++; //gray
			}
		}*/
		
		
		//IplImage* src2 = cvCreateImage( cvGetSize(cvImage), 8, 3 );
		
		cvLogPolar( cvImage, dstColor, cvPoint2D32f(cvImage->width/2,cvImage->height/2), 50, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
		cvLogPolar( dstColor, dstColor2, cvPoint2D32f(cvImage->width/2,cvImage->height/2), 50, CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS );
		//delete cvImage;

		//cvShowImage("test",dstColor2);
		
		//cvCvtColor(dst,dstColor, CV_GRAY2RGB);
		//delete dst;

		// add a blue circle
		ct = 320/2;
		PixelRgb blue(0,0,255);
		PixelRgb red(255,0,0);
		PixelRgb green(0,255,0);
		PixelRgb yellow(255,255,0);
		PixelRgb black(0,0,0);
		addRectangle(*image2,black,0,0,320,240);
		addCircle(*image2,blue,ct,50,15);
		addCircle(*image2,red,ct-50,100,20);
		addCircle(*image2,green,ct+50,150,25);
		addCircle(*image2,yellow,ct,200,10);
		
		
		//----------conversion to YARP COLOUR-----------------
		
		// output the images
		//otherwise the input image is present on the port
		port4.prepare() = *image2;
		port4.write();
		//-----------
		yarpReturnImage.wrapIplImage(dstColor);
		//unsigned char* pointer=yarpReturnImage.getRawImage();
		//pointer=(uchar *)dstColor->imageData;	
		//yarpReturnImagePointer=bayerLPImage;
		yarpReturnImagePointer=&yarpReturnImage;
		//port2.prepare() = *img;	
		port2.prepare() = *yarpReturnImagePointer;		
		port2.write();
		//-----------
		yarpReturnImage.wrapIplImage(dstColor2);
		//unsigned char* pointer=yarpReturnImage.getRawImage();
		//pointer=(uchar *)dstColor->imageData;	
		//yarpReturnImagePointer=bayerLPImage;
		yarpReturnImagePointer=&yarpReturnImage;
		//port2.prepare() = *img;	
		port3.prepare() = *yarpReturnImagePointer;		
		port3.write();
	}
	else if(mode==2){
		if (img==NULL) 
			return true;
		printf("INVERSE \n");
		int xSize=img->width(), ySize=img->height(), planes=0;
		if(xSize!=ySize){
			//image needs a crop or 
			cvImage1=cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 );
			cvCopy(img->getIplImage(),cvImage1);
			cvSetImageROI(cvImage1,rec);
			cvCopy(cvImage1,cvImage2);
		}
		else
		{
			cvCopy(img->getIplImage(),cvImage2);
		}
		cvCvtColor(cvImage2,cvImage,CV_RGB2BGR);
		//cvCvtColor((IplImage*)img->getIplImage(), cvImage, CV_RGB2GRAY);
		

		//ImageOf<PixelMono> returnCvImage;
		//returnCvImage.wrapIplImage(cvImage);
		//ImageOf<PixelMono> *returnCvImagePointer=NULL;
		//yarpReturnImagePointer=bayerLPImage;
		//returnCvImagePointer=&returnCvImage;
		
		//unsigned char *bayerCartesian=new unsigned char[img->width() * img->height()];

		
		/*unsigned char *bayerCartesian =
			Load_Bitmap (&xSize, &ySize, &planes, img);*/
		//unsigned char *bayerCartesian =Load_Bitmap (&xSize, &ySize, &planes, "./bayercartesian.bmp");
		//Save_Bitmap (bayerCartesian, xSize, ySize, planes, "./bayercartesian.bmp");
		//unsigned char *bayerCartesianInit=new unsigned char[img->width() * img->height()];
		//bayerCartesianInit=bayerCartesian;
		//delete img;
		//printf("Showing OpenCV/IPL image cvImage->height %d img->height %d planes %d \n",xSize,ySize,planes);



		//--------conversion to logPolar----------------
		/*int k=0;
		for(int i=0;i<xSize;i++){
			for(int j=0;j<ySize;j++){
				k++;
				//s=cvGet2D(cvImage,j,i); // get the (i,j) pixel value
				
				//getch();
				//unsigned char value=(unsigned char)floor(s.val[0]);
				
				//*bayerCartesian=(unsigned char) value;
				//printf("intensity=%d",(unsigned char)*bayerCartesian);	
				//if((j==0)||(i==0))
				//	bayerCartesianInit=bayerCartesian;
				//bayerCartesian=((uchar *)(cvImage->imageData + i*cvImage->widthStep))[j];						
				//bayerCartesian++;
			}
		}*/
		
		
		/*unsigned char *bayerLP = new unsigned char[nEcc * nAng];
		unsigned char *bayerLPInit=new unsigned char[nEcc * nAng];
		bayerLPInit=bayerLP;
		if (bayerLP == NULL)
		{
			exit (-1);
		}
		c2lTable = new cart2LpPixel[nEcc * nAng];
		if (c2lTable == NULL)
		{
			exit (-1);
		}
		printf("Image width=%d height=%d  pixelSize=%d \n", xSize,ySize,planes);
		
		double overlap=0;
		double scaleFact = RCcomputeScaleFactor (nEcc, nAng, xSize,ySize, overlap);

		int returnValue=RCbuildC2LMapBayer(nEcc, nAng, xSize,ySize, overlap,scaleFact, ELLIPTICAL,path);
		printf("return from RCbuildC2LMap: %d \n",returnValue);
		returnValue=RCallocateC2LTable (c2lTable, nEcc, nAng, 1, path);
		printf("return from RCallocateC2LMap: %d \n",returnValue);
		RCgetLpImg (bayerLP, bayerCartesian, c2lTable, nAng * nEcc, 1);
		//Save_Bitmap (bayerLP, nAng, nEcc, 1, "./TestlpBayer.bmp");
		RCdeAllocateC2LTable (c2lTable);*/
		
		//convert the unsigned char pointer to an YARPImage
		//IplImage* dst = cvCreateImage( cvSize(xSize,ySize), 8, 1 );
		
		//ImageOf<PixelMono> *bayerLPImage=new ImageOf<PixelMono>;
		//unsigned char* pImage=bayerLPImage->getRawImage();
		//pImage=bayerCartesianInit;
		//--
		/*bayerCartesian=bayerCartesianInit;
		for(int i=0;i<xSize;i++){
			for(int j=0;j<ySize;j++){
				//s.val[0]=*bayerLP;
				unsigned char value=*bayerCartesian;
				s.val[0]=(float)value;
				cvSet2D(dst,j,i,s); // set the (i,j) pixel value
				bayerCartesian++; //red
				bayerCartesian++; //green
				bayerCartesian++; // blue
			}
		}*/

		/*bayerLP=bayerLPInit;
		for(int i=0;i<nAng;i++){
			for(int j=0;j<nEcc;j++){
				unsigned char value=*bayerLP;
				s.val[0]=(float)value;
				cvSet2D(dst,j,i,s); // set the (i,j) pixel value
				bayerLP++; //gray
			}
		}*/
		
		
		//IplImage* src2 = cvCreateImage( cvGetSize(cvImage), 8, 3 );
		cvLogPolar( cvImage, dstColor, cvPoint2D32f(dstColor->width/2,dstColor->height/2), 42, CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS );
		//cvLogPolar( dstColor, dstColor2, cvPoint2D32f(cvImage->width/2,cvImage->height/2), 50, CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS );
		//delete cvImage;


		//cvShowImage("test",dstColor2);
		
		//cvCvtColor(dst,dstColor, CV_GRAY2RGB);
		//delete dst;

		
		
		//----------conversion to YARP COLOUR-----------------
		
		// output the images
		//otherwise the input image is present on the port
		//port4.prepare() = *image2;
		//port4.write();
		//-----------
		yarpReturnImage.wrapIplImage(dstColor);
		//unsigned char* pointer=yarpReturnImage.getRawImage();
		//pointer=(uchar *)dstColor->imageData;	
		//yarpReturnImagePointer=bayerLPImage;
		yarpReturnImagePointer=&yarpReturnImage;
		//port2.prepare() = *img;	
		port2.prepare() = *yarpReturnImagePointer;		
		port2.write();
		//-----------
		//yarpReturnImage.wrapIplImage(dstColor2);
		//unsigned char* pointer=yarpReturnImage.getRawImage();
		//pointer=(uchar *)dstColor->imageData;	
		//yarpReturnImagePointer=bayerLPImage;
		//yarpReturnImagePointer=&yarpReturnImage;
		//port2.prepare() = *img;	
		//port3.prepare() = *yarpReturnImagePointer;		
		//port3.write();
	}
	//delete yarpReturnImagePointer;

    return true;
}

/**
 * load a bmp from file.
 */
/**
unsigned char * LogPolarModule::Load_Bitmap (int *X_Size, int *Y_Size, int *planes, char *filename)
{
    unsigned char *image;
    unsigned char c = 0;
    int x, y, z;
    int Offset;
    FILE *fin;
    BITMAPFILEHEADER bmpfh;
    BITMAPINFOHEADER bmpih;
    RGBQUAD palette[256];

    if ((fin = fopen (filename, "rb")) != NULL)
    {
        printf("Image Found");
		fread (&bmpfh, sizeof (BITMAPFILEHEADER), 1, fin);
        fread (&bmpih, sizeof (BITMAPINFOHEADER), 1, fin);
        *X_Size = bmpih.biWidth;
        *Y_Size = bmpih.biHeight;
        *planes = bmpih.biBitCount / 8;
        image = new unsigned char[*X_Size * *Y_Size * *planes];
        if (image == NULL)
        {
            printf("Image Null \n");
			exit (-1);
        }
        Offset = (4 - ((*X_Size * *planes) % 4)) % 4;

        if (*planes == 1)
            fread (&palette, sizeof (RGBQUAD), 256, fin);

        for (y = *Y_Size - 1; y >= 0; y--)
        {
            for (x = 0; x < *X_Size; x++)
                for (z = *planes - 1; z >= 0; z--)
                {
                    fread (&c, 1, 1, fin);
                    image[y ** planes * *X_Size + *planes * x + z] =
                        (unsigned char) (c);
                }
            for (x = 0; x < Offset; x++)
                fread (&c, 1, 1, fin);
        }
        fclose (fin);
    }
    else
        image = NULL;

    return image;
}*/

/**
 * load a bmp from ImageOf<PixelRgb>
 */
/*unsigned char * LogPolarModule::Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelRgb> *src)
{
    unsigned char *image;
    unsigned char c = 0;
    int x, y, z;
    int Offset;
    FILE *fin;
    BITMAPFILEHEADER bmpfh;
    BITMAPINFOHEADER bmpih;
    RGBQUAD palette[256];

    
	//*X_Size = src->imgHeight;
	//*Y_Size = src->imgWidth;
    //*planes = 24 / 8;
	*planes = 8 / 8;
    image = new unsigned char[*X_Size * *Y_Size * *planes];
    if (image == NULL)
    {
        exit (-1);
    }
    Offset = (4 - ((*X_Size * *planes) % 4)) % 4;



    for (y = *Y_Size - 1; y >= 0; y--)
    {
        for (x = 0; x < *X_Size; x++)
            for (z = *planes - 1; z >= 0; z--)
            {
                //fread (&c, 1, 1, fin);
				PixelRgb pRgb=src->pixel(x,y);
				if(z==2)
					c=pRgb.b;
				if(z==1)
					c=pRgb.g;
				if(z==0)
					c=pRgb.b;
                //image[y ** planes * *X_Size + *planes * x + z] =(unsigned char) (c);
				image[x ** planes * *Y_Size + *planes * y + z] =(unsigned char) (c);
            }
			
    }
    //fclose (fin);
    
    return image;
}*/

/**
 * load a bmp from ImageOf<PixelMono>
 */
/*
unsigned char * LogPolarModule::Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelMono> *src)
{
    unsigned char *image;
    unsigned char c = 0;
    int x, y, z;
    int Offset;
    FILE *fin;
    BITMAPFILEHEADER bmpfh;
    BITMAPINFOHEADER bmpih;
    RGBQUAD palette[256];

    
	//*X_Size = src->imgHeight;
	//*Y_Size = src->imgWidth;
    *planes = 8 / 8; //definition of plane:number of bits subdivided by 8
    image = new unsigned char[*X_Size * *Y_Size * *planes];
    if (image == NULL)
    {
        exit (-1);
    }
    Offset = (4 - ((*X_Size * *planes) % 4)) % 4;

    

    for (y = *Y_Size - 1; y >= 0; y--)
    {
        for (x = 0; x < *X_Size; x++)
            for (z = *planes - 1; z >= 0; z--)
            {
                //fread (&c, 1, 1, fin);
				PixelMono pMono=src->pixel(x,y);
				c=pMono;
                image[x ** planes * *Y_Size + *planes * y + z] =
                    (unsigned char) (c);
            }
       
    }
    //fclose (fin);
    
    return image;
}*/

/**
 * save a bmp to file.
 */
/*
void LogPolarModule::Save_Bitmap (unsigned char *image, int X_Size, int Y_Size, int planes,char *filename)
{
    FILE *fout;
    int size = X_Size * Y_Size * planes;
    int x, y, z;
    int Offset = (4 - ((X_Size * planes) % 4)) % 4;
    BITMAPFILEHEADER bmpfh;
    BITMAPINFOHEADER bmpih;

    fout = fopen (filename, "wb");
    if (fout == NULL)
    {
        printf ("Can't save %s bitmap file\n", filename);
        return;
    }

    bmpfh.bfType = 19778;       // 'MB';
    bmpfh.bfOffBits = 54;
    if (planes == 1)
        bmpfh.bfOffBits = 54 + 1024;

    bmpfh.bfSize = size + bmpfh.bfOffBits;
    bmpfh.bfReserved1 = 0;
    bmpfh.bfReserved2 = 0;

    bmpih.biSize = 40;
    bmpih.biWidth = X_Size;
    bmpih.biHeight = Y_Size;
    bmpih.biPlanes = 1;
    bmpih.biBitCount = planes * 8;
    bmpih.biCompression = 0;
    bmpih.biSizeImage = Y_Size * (X_Size * planes + Offset);
    bmpih.biXPelsPerMeter = 2835;
    bmpih.biYPelsPerMeter = bmpih.biXPelsPerMeter;
    bmpih.biClrUsed = 0;
    bmpih.biClrImportant = 0;
    if (planes == 1)
    {
        bmpih.biClrUsed = 256;
        bmpih.biClrImportant = 256;
    }

    fwrite (&bmpfh, sizeof (BITMAPFILEHEADER), 1, fout);
    fwrite (&bmpih, sizeof (BITMAPINFOHEADER), 1, fout);

    if (planes == 1)
        for (x = 0; x < 256; x++)
        {
            y = 0;
            fwrite (&x, sizeof (unsigned char), 1, fout);
            fwrite (&x, sizeof (unsigned char), 1, fout);
            fwrite (&x, sizeof (unsigned char), 1, fout);
            fwrite (&y, sizeof (unsigned char), 1, fout);
        }

    for (y = Y_Size - 1; y >= 0; y--)
    {
        for (x = 0; x < X_Size; x++)
            for (z = planes - 1; z >= 0; z--)
                fwrite (image + (planes * (x * Y_Size + y) + z),
                        sizeof (unsigned char), 1, fout);
        for (x = 0; x < Offset; x++)
            fwrite (image, sizeof (unsigned char), 1, fout);
    }

    fclose (fout);
}*/
