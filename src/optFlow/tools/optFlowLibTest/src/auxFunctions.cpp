#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#if !defined(__QNX__) && !defined(__LINUX__)
#include <windows.h>
#endif	/// __WIN32__


unsigned char * Load_Bitmap(int *X_Size,
							int *Y_Size,
							int *planes,
							char * filename)
{
#if defined(__QNX__) || defined(__LINUX__)

#define WORD short
#define DWORD int
#define LONG long
#define BYTE char

#ifdef __LINUX__
#pragma align 1
#endif

#ifdef __QNX4__
#pragma  pack (push) ;
#pragma  pack (1) ;
#endif

#ifdef __QNX6__
#pragma pack(1)
#endif

	typedef struct tagBITMAPFILEHEADER { 
	  WORD    bfType; 
	  DWORD   bfSize; 
	  WORD    bfReserved1; 
	  WORD    bfReserved2; 
	  DWORD   bfOffBits; 
	} BITMAPFILEHEADER, *PBITMAPFILEHEADER;

	typedef struct tagBITMAPINFOHEADER{
	  DWORD  biSize; 
	  LONG   biWidth; 
	  LONG   biHeight; 
	  WORD   biPlanes; 
	  WORD   biBitCount; 
	  DWORD  biCompression; 
	  DWORD  biSizeImage; 
	  LONG   biXPelsPerMeter; 
	  LONG   biYPelsPerMeter; 
	  DWORD  biClrUsed; 
	  DWORD  biClrImportant; 
	} BITMAPINFOHEADER, *PBITMAPINFOHEADER; 

	typedef struct tagRGBQUAD {
	  BYTE    rgbBlue; 
	  BYTE    rgbGreen; 
	  BYTE    rgbRed; 
	  BYTE    rgbReserved; 
	} RGBQUAD; 

#ifdef __LINUX__
#pragma align 0
#endif

#ifdef __QNX4__
#pragma  pack (pop) ;
#endif

#ifdef __QNX6__
#pragma pack()
#endif

#endif

	unsigned char *image;	
	unsigned char c=0;
	int x,y,z;
	int Offset;
	FILE* fin;
	BITMAPFILEHEADER bmpfh;
	BITMAPINFOHEADER bmpih;
	RGBQUAD palette[256];

	if ((fin = fopen(filename,"rb")) != NULL)
	{
		fread (&bmpfh,sizeof(BITMAPFILEHEADER),1,fin);
		fread (&bmpih,sizeof(BITMAPINFOHEADER),1,fin);
		*X_Size = bmpih.biWidth;
		*Y_Size = bmpih.biHeight;
		*planes = bmpih.biBitCount/8;
		image = (unsigned char *)malloc(*X_Size * *Y_Size * *planes *sizeof(unsigned char));
		Offset = (4 - ((*X_Size * *planes) %4))%4;

		if (*planes == 1)
			fread (&palette,sizeof(RGBQUAD),256,fin);
		
		for (y=*Y_Size-1; y>=0; y--)
		{
			for (x=0; x<*X_Size; x++)
				for (z=*planes-1; z>=0; z--)
				{
						fread (&c,1,1,fin);
						image[y**planes* *X_Size + *planes *x+z] = (unsigned char)(c);
				}
			for (x=0;x<Offset;x++)
				fread(&c,1,1,fin);
		}
		fclose(fin);
	}
	else
		image = NULL;
	
	return image;
}


/************************************************************************
* Save_Bitmap															*
* Scrive un'immagine bitmap su file										*
************************************************************************/	

void Save_Bitmap(unsigned char *image,
				 int X_Size,
				 int Y_Size,
				 int planes,
				 char * filename)
{
#if !defined(__QNX__) && !defined(__LINUX__)

	FILE* fout;
	int size = X_Size * Y_Size * planes;
	int x,y,z;
	int Offset = (4 - ((X_Size * planes) %4))%4;
	BITMAPFILEHEADER bmpfh;
	BITMAPINFOHEADER bmpih;

	fout = fopen(filename,"wb");

	bmpfh.bfType = 'MB';
	bmpfh.bfOffBits = 54;
	if (planes==1)
		bmpfh.bfOffBits = 54+1024;

	bmpfh.bfSize = size + bmpfh.bfOffBits;
	bmpfh.bfReserved1 = 0;
	bmpfh.bfReserved2 = 0;

	bmpih.biSize = 40;
	bmpih.biWidth = X_Size;
	bmpih.biHeight = Y_Size;
	bmpih.biPlanes = 1;
	bmpih.biBitCount = planes *8;
	bmpih.biCompression = 0;
	bmpih.biSizeImage = Y_Size * (X_Size * planes + Offset);
	bmpih.biXPelsPerMeter = 2835;
	bmpih.biYPelsPerMeter = bmpih.biXPelsPerMeter;
	bmpih.biClrUsed = 0;
	bmpih.biClrImportant = 0;
	if (planes==1)
	{
		bmpih.biClrUsed = 256;
		bmpih.biClrImportant = 256;
	}

	fwrite(&bmpfh,sizeof(BITMAPFILEHEADER),1,fout);
	fwrite(&bmpih,sizeof(BITMAPINFOHEADER),1,fout);

	if (planes ==1)
		for (x=0; x<256;x++)
		{
			y=0;
			fwrite(&x,sizeof(unsigned char),1,fout);
			fwrite(&x,sizeof(unsigned char),1,fout);
			fwrite(&x,sizeof(unsigned char),1,fout);
			fwrite(&y,sizeof(unsigned char),1,fout);
		}

	for (y=Y_Size-1; y>=0; y--)
	{	
		for (x=0; x<X_Size; x++)
			for (z=planes-1;z>=0; z--)
				fwrite(image+(planes*(y*X_Size+x)+z),sizeof(unsigned char),1,fout);
		for (x=0;x<Offset;x++)
			fwrite(image,sizeof(unsigned char),1,fout);
	}

	fclose(fout);
#endif
}



/************************************************************************
* Get_Time				  												*
************************************************************************/

long Get_Time()
{
#if !defined(__QNX__) && !defined(__LINUX__)

	LARGE_INTEGER perfTime;
	LARGE_INTEGER perfFreq;
	long l_tempo;
	double d_tempo;
	time_t offset_ora;

        QueryPerformanceFrequency(&perfFreq);
		time(&offset_ora);//secs from Jan 1st, 1970
		QueryPerformanceCounter(&perfTime);
	    d_tempo = (double)perfTime.QuadPart/(double)perfFreq.QuadPart;//Elapsed Time (secs from boot)
		l_tempo = (long)(d_tempo * 1000.0);//(msecs from boot)

		return l_tempo;
#else
		return -1;
#endif    
}