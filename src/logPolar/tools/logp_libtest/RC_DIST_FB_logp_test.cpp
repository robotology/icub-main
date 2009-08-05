/*
 *  logpolar mapper library: example code.
 *
 *  Copyright (C) 2005 Fabio Berton, LIRA-Lab
 *  RobotCub Consortium, European Commission FP6 Project IST-004370
 *  email:   fberton@dist.unige.it
 *  website: www.robotcub.org
 *
 *  Permission is granted to copy, distribute, and/or modify this program 
 *  under the terms of the GNU General Public License, version 2 or any later
 *  version published by the Free Software Foundation. A copy of the license can be 
 *  found at http://www.robotcub.org/icub/license/gpl.txt
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 *  PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 *  $Id: RC_DIST_FB_logp_test.cpp,v 1.8 2009/08/04 12:59:57 babybot Exp $
 */

/*
 * A nasty piece of code, to be used as an example only
 * as long as Fabio doesn't make it compile properly on multiple OSs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

#ifdef WIN32 
#include <windows.h>
#else

/*
 * Defines the bmp structure types for non-win32 systems.
 */

// dangerous defines for varying architectures (e.g. size of types change).

#define WORD short
#define DWORD int
#define LONG int
#define BYTE char

#pragma pack(1)

typedef struct tagBITMAPFILEHEADER
{
    WORD bfType;
    DWORD bfSize;
    WORD bfReserved1;
    WORD bfReserved2;
    DWORD bfOffBits;
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
    DWORD biSize;
    LONG biWidth;
    LONG biHeight;
    WORD biPlanes;
    WORD biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    LONG biXPelsPerMeter;
    LONG biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;

typedef struct tagRGBQUAD
{
    BYTE rgbBlue;
    BYTE rgbGreen;
    BYTE rgbRed;
    BYTE rgbReserved;
} RGBQUAD;

#pragma pack()

#endif

/*
 * load a bmp from file.
 */
unsigned char *
Load_Bitmap (int *X_Size, int *Y_Size, int *planes, char *filename)
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
        fread (&bmpfh, sizeof (BITMAPFILEHEADER), 1, fin);
        fread (&bmpih, sizeof (BITMAPINFOHEADER), 1, fin);
        *X_Size = bmpih.biWidth;
        *Y_Size = bmpih.biHeight;
        *planes = bmpih.biBitCount / 8;
        image = new unsigned char[*X_Size * *Y_Size * *planes];
        if (image == NULL)
        {
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
}


/*
 * save a bmp to file.
 */
void
Save_Bitmap (unsigned char *image, int X_Size, int Y_Size, int planes,
             char *filename)
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
                fwrite (image + (planes * (y * X_Size + x) + z),
                        sizeof (unsigned char), 1, fout);
        for (x = 0; x < Offset; x++)
            fwrite (image, sizeof (unsigned char), 1, fout);
    }

    fclose (fout);
}

/*
 * main. usage: prog_name image_filename
 *
 */
int
main (int argc, char *argv[])
{
    if (argc < 2)
    {
        printf ("%s: usage: %s image_filename\n", argv[0], argv[0]);
        exit (-1);
    }

    char path[] = "./";

    cart2LpPixel *c2lTable;
    lp2CartPixel *l2cTable;

    int nEcc = 152;
    int nAng = 252;

    double overlap = 2.00;

    int xSize, ySize, planes;

    double scaleFact;
    double logIndex = RCgetLogIndex (nAng);

    printf ("%s: loading bitmap\n", argv[0]);
    unsigned char *bayerCartesian =
        Load_Bitmap (&xSize, &ySize, &planes, argv[1]);
    if (bayerCartesian == NULL)
    {
        printf ("%s: Can't load image %s for processing\n", argv[0], argv[1]);
        exit (-1);
    }

    int cartSize = (xSize < ySize)?xSize:ySize;

    /*************************
     * Color Reconstruction 1 *
     *************************/

    printf ("%s: reconstructing color from bayer pattern\n", argv[0]);
    unsigned char *colorCartesian = new unsigned char[xSize * ySize * 3];
    if (colorCartesian == NULL)
    {
        exit (-1);
    }

    RCreconstructColor (colorCartesian, bayerCartesian, xSize, ySize);

    Save_Bitmap (colorCartesian, xSize, ySize, 3, "./TestcartColRec.bmp");

    /*******************
     * Bayer LP Mapping *
     *******************/

    printf
        ("%s: building a logpolar image, bayer pattern, no color reconstruction\n",
         argv[0]);
    scaleFact = RCcomputeScaleFactor (nEcc, nAng, xSize,ySize, overlap);

    unsigned char *bayerLP = new unsigned char[nEcc * nAng];
    if (bayerLP == NULL)
    {
        exit (-1);
    }

    c2lTable = new cart2LpPixel[nEcc * nAng];
    if (c2lTable == NULL)
    {
        exit (-1);
    }
    RCbuildC2LMapBayer (nEcc, nAng, xSize,ySize, overlap, scaleFact, ELLIPTICAL,
                        path);

    RCallocateC2LTable (c2lTable, nEcc, nAng, 1, path);

    RCgetLpImg (bayerLP, bayerCartesian, c2lTable, nAng * nEcc, 1);

    Save_Bitmap (bayerLP, nAng, nEcc, 1, "./TestlpBayer.bmp");

    RCdeAllocateC2LTable (c2lTable);

    delete[]bayerCartesian;

    /*************************
     * Color Reconstruction 2 *
     *************************/

    printf ("%s: reconstructing logpolar color\n", argv[0]);
    unsigned char *colorBLP = new unsigned char[nEcc * nAng * 3];
    if (colorBLP == NULL)
    {
        exit (-1);
    }

    RCreconstructColor (colorBLP, bayerLP, nAng, nEcc);

    Save_Bitmap (colorBLP, nAng, nEcc, 3, "./TestlpColRecB.bmp");

    /*******************
     * Color LP Mapping *
     *******************/

    printf ("%s: building color logpolar image\n", argv[0]);
    overlap = 2.00;

    scaleFact = RCcomputeScaleFactor (nEcc, nAng, xSize,ySize, overlap);

    unsigned char *colorLP = new unsigned char[nEcc * nAng * 3];
    if (colorLP == NULL)
    {
        exit (-1);
    }

    c2lTable = new cart2LpPixel[nEcc * nAng];
    if (c2lTable == NULL)
    {
        exit (-1);
    }

    RCbuildC2LMap (nEcc, nAng, xSize,ySize, overlap, scaleFact, ELLIPTICAL,
                   path);

    RCallocateC2LTable (c2lTable, nEcc, nAng, 0, path);

    RCgetLpImg (colorLP, colorCartesian, c2lTable, nEcc * nAng, 0);

    Save_Bitmap (colorLP, nAng, nEcc, 3, "./TestlpColor.bmp");

    /***********************
     * Cartesian Remapping *
     ***********************/

    printf ("%s: remapping from logpolar to rectangular\n", argv[0]);
    unsigned char *colorBRem = new unsigned char[xSize * ySize * 3];
    if (colorBRem == NULL)
    {
        exit (-1);
    }

    l2cTable = new lp2CartPixel[xSize * ySize];
    if (l2cTable == NULL)
    {
        exit (-1);
    }

    RCbuildL2CMap (nEcc, nAng, xSize,ySize, overlap, scaleFact, 0, 0, ELLIPTICAL,
                   path);

    RCallocateL2CTable (l2cTable, xSize, ySize, path);

    RCgetCartImg (colorBRem, colorBLP, l2cTable, xSize * ySize);

    Save_Bitmap (colorBRem, xSize, ySize, 3, "./TestRemColorfromB.bmp");

    /* missing all delete[] */

    return 0;
}
