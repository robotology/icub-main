/*
 *  logpolar mapper library. subsamples rectangular images into logpolar.
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
 *  $Id: RC_DIST_FB_logpolar_mapper.cpp,v 1.11 2009/06/18 08:30:13 nat Exp $
 */

#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE
#endif

#include <iCub/RC_DIST_FB_logpolar_mapper.h>

#include <stdio.h>
#include <math.h>
#include <memory.h>

#define MODE1

inline double
__max (double x, double y)
{
    return (x > y) ? x : y;
}

int
RCallocateC2LTable (cart2LpPixel * Table, int nEcc, int nAng, bool bayerImg,
                    char *path)
{
    char filename[256];
    int tablesize;
    int sizeLP = nEcc * nAng;
    int j;
    int *temppos;
    int *tempiw;
    int tempctr;

    if (bayerImg)
        sprintf (filename, "%sC2LPBayer.gio", path);
    else
        sprintf (filename, "%sC2LP.gio", path);

    FILE *fin;

    if ((fin = fopen (filename, "rb")) == NULL)
        return 1;
    else
    {
        fread (&tablesize, sizeof (int), 1, fin);

        temppos = new int[tablesize];
        tempiw = new int[tablesize];

        if ((temppos == NULL) || (tempiw == NULL))
        {
            fclose (fin);
            return 2;
        }
        else
        {
            tempctr = 0;

            for (j = 0; j < sizeLP; j++)
            {
                fread (&Table[j].divisor, sizeof (int), 1, fin);
                tempctr += Table[j].divisor;
                if (tempctr > tablesize)
                {
                    Table[j].divisor = 0;
                    break;
                }
                fread (temppos, sizeof (int), Table[j].divisor, fin);
                fread (tempiw, sizeof (int), Table[j].divisor, fin);
                Table[j].position = temppos;
                Table[j].iweight = tempiw;
                temppos += Table[j].divisor;
                tempiw += Table[j].divisor;
            }
            fclose (fin);
        }
    }
    return 0;
}

int
RCallocateL2CTable (lp2CartPixel * Table, int xSize, int ySize, char *path)
{
    char filename[256];
    int tablesize;
    int cartSize = xSize * ySize;
    int j;
    int *temppos;
    int tempctr;

    sprintf (filename, "%sLP2C.gio", path);

    FILE *fin;

    if ((fin = fopen (filename, "rb")) == NULL)
        return 1;
    else
    {
        fread (&tablesize, sizeof (int), 1, fin);

        temppos = new int[tablesize];

        if (temppos == NULL)
        {
            fclose (fin);
            return 2;
        }
        else
        {
            tempctr = 0;

            for (j = 0; j < cartSize; j++)
            {
                fread (&Table[j].iweight, sizeof (int), 1, fin);
                tempctr += Table[j].iweight;
                if (tempctr > tablesize)
                {
                    Table[j].iweight = 0;
                    break;
                }
                fread (temppos, sizeof (int), Table[j].iweight, fin);
                Table[j].position = temppos;
                temppos += Table[j].iweight;
            }

            fclose (fin);
        }
    }
    return 0;
}

void
RCdeAllocateC2LTable (cart2LpPixel * Table)
{
    delete[]Table[0].position;
    delete[]Table[0].iweight;
    delete[]Table;
}

void
RCdeAllocateL2CTable (lp2CartPixel * Table)
{
    delete[]Table[0].position;
    delete[]Table;
}

double
RCgetLogIndex (int nAng)
{
    double logIndex;

    logIndex = (1.0 + sin (PI / nAng)) / (1.0 - sin (PI / nAng));

    return logIndex;
}

double
RCcomputeScaleFactor (int nEcc, int nAng, int xSize, int ySize, double overlap)
{
    double maxRad;
    double receptFieldRadius;
    double r0;
    double lambda;
    int fov;
	int cSize = xSize;
	
	if(xSize>ySize)
		cSize = ySize;

    double totalRadius;

    double angle = (2.0 * PI / nAng);   //Angular size of one pixel
    double sinus = sin (angle / 2.0);

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    maxRad = pow (lambda, nEcc - 1) * (r0 + 0.5 / pow (lambda, fov));

    receptFieldRadius = maxRad * 2.0 * sinus * (overlap + 1.0) / (2.0);

    totalRadius = maxRad + receptFieldRadius;

    totalRadius = (cSize / 2) / totalRadius;

    return totalRadius;
}

int
RCbuildC2LMap (int nEcc, int nAng, int xSize, int ySize, double overlap,
               double scaleFact, int mode, char *path)
{
    if (overlap <= -1.0)
        return 1;

	int cSize = xSize;
	
	if(xSize>ySize)
		cSize = ySize;

    double angle = (2.0 * PI / nAng);   //Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);

    int fov;
    int lim;

    double lambda;              //Log Index
    double firstRing;           //Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            //Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            //Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  //lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j;

    cart2LpPixel c2LpMap;

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    tangaxis = new double[nEcc];
    radialaxis = new double[nEcc];
    focus = new double[nEcc];
    radii2 = new double[nEcc];

    currRad = new double[nEcc];
    nextRad = new double[nEcc];

    if ((tangaxis == NULL) || (radialaxis == NULL) || (focus == NULL)
        || (radii2 == NULL))
        return 2;

    if ((currRad == NULL) || (nextRad == NULL))
        return 2;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < nEcc; rho++)
    {
        if (rho < fov)
            if (rho == 0)
            {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else
            {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else
        {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL))
        {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }

    }
    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < nEcc; rho++)
    {
        if (mode != ELLIPTICAL)
            focus[rho] = 0;
        else
	{
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
	}

        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }

    int totalcounter = 0;

    FILE *fout;
    char filename[256];

    sprintf (filename, "%sC2LP.gio", path);

    fout = fopen (filename, "wb");

    fwrite (&totalcounter, sizeof (int), 1, fout);

    double *sintable;
    double *costable;

    int intx, inty;

    sintable = new double[nAng];
    costable = new double[nAng];

    if ((sintable == NULL) || (costable == NULL))
        return 2;

    for (j = 0; j < nAng; j++)
    {
        sintable[j] = sin (angle * (j + 0.5));
        costable[j] = cos (angle * (j + 0.5));
    }

    bool found;
    int mapsize;
    float *weight;
    double precision = 10.0;

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);

        c2LpMap.position = new int[mapsize];
//      c2LpMap.weight = new float [mapsize];
        weight = new float[mapsize];
        c2LpMap.iweight = new int[mapsize];

        if ((c2LpMap.position == NULL) || (weight == NULL)
            || (c2LpMap.iweight == NULL))
            return 2;

        for (theta = 0; theta < nAng; theta++)
        {
            memset (c2LpMap.position, 0, mapsize);
            memset (weight, 0, mapsize);

            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0)
            {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else
            {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }

            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];
		

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step)
                {
                    intx = (int) (locX + xSize / 2);
                    inty = (int) (locY + ySize / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));
                    if (locRad < 2 * maxaxis)
                    {
                        if ((inty < ySize) && (inty >= 0))
                        {
                            if ((intx < xSize) && (intx >= 0))
                            {
                                found = false;
                                j = 0;
                                while ((j < mapsize)
                                       && (c2LpMap.position[j] != 0))
                                {
                                    if (c2LpMap.position[j] ==
                                        3 * (inty * xSize + intx))
                                    {
                                        weight[j]++;
                                        found = true;
                                        j = mapsize;
                                    }
                                    j++;
                                }
                                if (!found)
                                    for (j = 0; j < mapsize; j++)
                                    {
                                        if (c2LpMap.position[j] == 0)
                                        {
                                            c2LpMap.position[j] =
                                                3 * (inty * xSize + intx);
                                            weight[j]++;
                                            break;
                                        }
                                    }
                            }
                        }
                    }
                }
            for (j = 0; j < mapsize; j++)
                if (weight[j] == 0)
                    break;

            c2LpMap.divisor = j;

            totalcounter += c2LpMap.divisor;

            float sum = 0.0;
            int k;
            for (k = 0; k < j; k++)
                sum += weight[k];
            for (k = 0; k < j; k++)
            {
                weight[k] = weight[k] / sum;
                c2LpMap.iweight[k] = (int) (weight[k] * 65536.0);
       }     
	
	fwrite (&c2LpMap.divisor, sizeof (int), 1, fout);
            fwrite (c2LpMap.position, sizeof (int), c2LpMap.divisor, fout);
            fwrite (c2LpMap.iweight, sizeof (int), c2LpMap.divisor, fout);

        }
        delete[]c2LpMap.position;
        delete[]weight;
        delete[]c2LpMap.iweight;
    }

    delete[]tangaxis;
    delete[]radialaxis;
    delete[]focus;
    delete[]radii2;
    delete[]currRad;
    delete[]nextRad;
    delete[]sintable;
    delete[]costable;

    fseek (fout, 0, SEEK_SET);
    fwrite (&totalcounter, sizeof (int), 1, fout);
    fclose (fout);

    return 0;

}


void
RCreconstructColor (unsigned char *color, unsigned char *grey, int width,
                    int height)
{
    int x, y;

#ifdef MODE1

    for (y = 1; y < height - 1; y += 2) //green1
        for (x = 2; x < width; x += 2)
        {
            color[3 * (y * width + x) + 0] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x]) / 2;
            color[3 * (y * width + x) + 1] = grey[y * width + x];
            color[3 * (y * width + x) + 2] =
                (grey[(y) * width + x - 1] + grey[(y) * width + x + 1]) / 2;
        }
    for (y = 2; y < height; y += 2) //green2
        for (x = 1; x < width - 1; x += 2)
        {
            color[3 * (y * width + x) + 0] =
                (grey[(y) * width + x - 1] + grey[(y) * width + x + 1]) / 2;
            color[3 * (y * width + x) + 1] = grey[y * width + x];
            color[3 * (y * width + x) + 2] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x]) / 2;
        }

    for (y = 2; y < height; y += 2) //red
        for (x = 2; x < width; x += 2)
        {
            color[3 * (y * width + x) + 0] = grey[y * width + x];
            color[3 * (y * width + x) + 1] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4;
            color[3 * (y * width + x) + 2] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4;
        }
    for (y = 1; y < height - 1; y += 2) //blue
        for (x = 1; x < width - 1; x += 2)
        {
            color[3 * (y * width + x) + 0] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4;
            color[3 * (y * width + x) + 1] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4;
            color[3 * (y * width + x) + 2] = grey[y * width + x];
        }

    //Borders
    for (x = 1; x < width - 1; x += 2)
    {
        color[3 * (x) + 0] = (grey[x - 1] + grey[x + 1]) / 2;
        color[3 * (x) + 1] = grey[x];
        color[3 * (x) + 2] = grey[width + x];

        color[3 * ((height - 1) * width + x) + 0] =
            (grey[(height - 2) * width + x - 1] +
             grey[(height - 2) * width + x + 1]) / 2;
        color[3 * ((height - 1) * width + x) + 1] =
            (grey[(height - 2) * width + x] +
             grey[(height - 1) * width + x - 1] + grey[(height - 1) * width +
                                                       x + 1]) / 3;
        color[3 * ((height - 1) * width + x) + 2] =
            grey[(height - 1) * width + x];
    }
    for (x = 2; x < width; x += 2)
    {
        color[3 * (x) + 0] = grey[x];
        color[3 * (x) + 1] =
            (grey[width + x] + grey[x - 1] + grey[x + 1]) / 3;
        color[3 * (x) + 2] = (grey[width + x - 1] + grey[width + x + 1]) / 2;

        color[3 * ((height - 1) * width + x) + 0] =
            grey[(height - 2) * width + x];
        color[3 * ((height - 1) * width + x) + 1] =
            grey[(height - 1) * width + x];
        color[3 * ((height - 1) * width + x) + 2] =
            (grey[(height - 1) * width + x - 1] +
             grey[(height - 1) * width + x + 1]) / 2;
    }
    for (y = 1; y < height - 1; y += 2)
    {
        color[3 * (y * width) + 0] =
            (grey[(y - 1) * width] + grey[(y + 1) * width]) / 2;
        color[3 * (y * width) + 1] = grey[y * width];
        color[3 * (y * width) + 2] =
            (grey[(y) * width + width - 1] + grey[(y) * width + 1]) / 2;

        color[3 * (y * width + width - 1) + 0] =
            (grey[(y - 1) * width + width - 2] + grey[(y - 1) * width] +
             grey[(y + 1) * width + width - 2] + grey[(y + 1) * width]) / 4;
        color[3 * (y * width + width - 1) + 1] =
            (grey[(y - 1) * width + width - 1] +
             grey[(y + 1) * width + width - 1] + grey[y * width + width - 2] +
             grey[y * width]) / 4;
        color[3 * (y * width + width - 1) + 2] = grey[y * width + width - 1];
    }
    for (y = 2; y < height; y += 2)
    {
        color[3 * (y * width) + 0] = grey[y * width];
        color[3 * (y * width) + 1] =
            (grey[(y - 1) * width] + grey[(y + 1) * width] +
             grey[y * width + width - 1] + grey[y * width + 1]) / 4;
        color[3 * (y * width) + 2] =
            (grey[(y - 1) * width + width - 1] + grey[(y - 1) * width + 1] +
             grey[(y + 1) * width + width - 1] + grey[(y + 1) * width +
                                                      1]) / 4;

        color[3 * (y * width + width - 1) + 0] =
            (grey[(y) * width + width - 2] + grey[(y) * width]) / 2;
        color[3 * (y * width + width - 1) + 1] = grey[y * width + width - 1];
        color[3 * (y * width + width - 1) + 2] =
            (grey[(y - 1) * width + width - 1] +
             grey[(y + 1) * width + width - 1]) / 2;
    }

    color[3 * (width * height - 1) + 0] =
        (grey[(height - 2) * width + width - 2] +
         grey[(height - 2) * width]) / 2;
    color[3 * (width * height - 1) + 1] =
        (grey[(height - 2) * width + width - 1] +
         grey[(height - 1) * width + width - 2] +
         grey[(height - 1) * width]) / 3;
    color[3 * (width * height - 1) + 2] = grey[width * height - 1];

    color[3 * ((height - 1) * width) + 0] = grey[(height - 2) * width];
    color[3 * ((height - 1) * width) + 1] = grey[(height - 1) * width];
    color[3 * ((height - 1) * width) + 2] =
        (grey[(height - 1) * width + width - 1] +
         grey[(height - 1) * width + 1]) / 2;

    color[0] = grey[0];
    color[1] = (grey[width] + grey[width - 1] + grey[1]) / 3;
    color[2] = (grey[width + width - 1] + grey[width + 1]) / 2;

    color[3 * (width - 1) + 0] = (grey[width - 2] + grey[0]) / 2;
    color[3 * (width - 1) + 1] = grey[width - 1];
    color[3 * (width - 1) + 2] = (grey[width + width - 1]);

#endif

#ifdef MODE2

    double *luminance = (double *) malloc (width * height * sizeof (double));
    double *Cb = (double *) malloc (width * height * sizeof (double));
    double *Cr = (double *) malloc (width * height * sizeof (double));

    double *avgRed = (double *) malloc (width * height * sizeof (double));
    double *avgGre = (double *) malloc (width * height * sizeof (double));
    double *avgBlu = (double *) malloc (width * height * sizeof (double));

    double *avgCr = (double *) malloc (width * height * sizeof (double));
    double *avgCg = (double *) malloc (width * height * sizeof (double));
    double *avgCb = (double *) malloc (width * height * sizeof (double));
    double *avgLum = (double *) malloc (width * height * sizeof (double));

    for (y = 3; y < height - 3; y += 2) //red
        for (x = 2; x < width - 3; x += 2)
        {
            avgBlu[y * width + x] =
                (grey[(y) * width + x - 1] + grey[(y) * width + x + 1]) / 2.0;
            avgRed[y * width + x] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x]) / 2.0;
            avgGre[y * width + x] =
                (grey[y * width + x] + grey[(y - 2) * width + x] +
                 grey[(y + 2) * width + x] + grey[y * width + x - 2] +
                 grey[y * width + x + 2]);
            avgGre[y * width + x] +=
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]);
            avgGre[y * width + x] = avgGre[y * width + x] / 9.0;
            avgLum[y * width + x] =
                0.2 * avgRed[y * width + x] + 0.7 * avgGre[y * width + x] +
                0.1 * avgBlu[y * width + x];
            avgCr[y * width + x] =
                avgRed[y * width + x] - avgLum[y * width + x];
            avgCg[y * width + x] =
                avgGre[y * width + x] - avgLum[y * width + x];
            avgCb[y * width + x] =
                avgBlu[y * width + x] - avgLum[y * width + x];
            luminance[y * width + x] =
                grey[y * width + x] - avgCg[y * width + x];
            if (luminance[y * width + x] + avgCr[y * width + x] < 256)
                color[3 * (y * width + x) + 0] =
                    (unsigned char) (luminance[y * width + x] +
                                     avgCr[y * width + x]);
            else
                color[3 * (y * width + x) + 0] = 255;
            color[3 * (y * width + x) + 1] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCg[y * width + x]);
            color[3 * (y * width + x) + 2] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCb[y * width + x]);

        }

    for (y = 2; y < height - 3; y += 2) //red
        for (x = 3; x < width - 3; x += 2)
        {
            avgRed[y * width + x] =
                (grey[(y) * width + x - 1] + grey[(y) * width + x + 1]) / 2.0;
            avgBlu[y * width + x] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x]) / 2.0;
            avgGre[y * width + x] =
                (grey[y * width + x] + grey[(y - 2) * width + x] +
                 grey[(y + 2) * width + x] + grey[y * width + x - 2] +
                 grey[y * width + x + 2]);
            avgGre[y * width + x] +=
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]);
            avgGre[y * width + x] = avgGre[y * width + x] / 9.0;
            avgLum[y * width + x] =
                0.2 * avgRed[y * width + x] + 0.7 * avgGre[y * width + x] +
                0.1 * avgBlu[y * width + x];
            avgCr[y * width + x] =
                avgRed[y * width + x] - avgLum[y * width + x];
            avgCg[y * width + x] =
                avgGre[y * width + x] - avgLum[y * width + x];
            avgCb[y * width + x] =
                avgBlu[y * width + x] - avgLum[y * width + x];
            luminance[y * width + x] =
                grey[y * width + x] - avgCg[y * width + x];
            if (luminance[y * width + x] + avgCr[y * width + x] < 256)
                color[3 * (y * width + x) + 0] =
                    (unsigned char) (luminance[y * width + x] +
                                     avgCr[y * width + x]);
            else
                color[3 * (y * width + x) + 0] = 255;
            color[3 * (y * width + x) + 1] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCg[y * width + x]);
            color[3 * (y * width + x) + 2] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCb[y * width + x]);

        }

    for (y = 2; y < height - 3; y += 2) //red
        for (x = 2; x < width - 3; x += 2)
        {
            avgBlu[y * width + x] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4;
            avgGre[y * width + x] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4;
            avgRed[y * width + x] =
                (grey[y * width + x] + grey[(y - 2) * width + x] +
                 grey[(y + 2) * width + x] + grey[y * width + x - 2] +
                 grey[y * width + x + 2]) / 5;
            avgLum[y * width + x] =
                0.2 * avgRed[y * width + x] + 0.7 * avgGre[y * width + x] +
                0.1 * avgBlu[y * width + x];
            avgCr[y * width + x] =
                avgRed[y * width + x] - avgLum[y * width + x];
            avgCg[y * width + x] =
                avgGre[y * width + x] - avgLum[y * width + x];
            avgCb[y * width + x] =
                avgBlu[y * width + x] - avgLum[y * width + x];
            luminance[y * width + x] =
                grey[y * width + x] - avgCr[y * width + x];
            if (luminance[y * width + x] + avgCr[y * width + x] < 256)
                color[3 * (y * width + x) + 0] =
                    (unsigned char) (luminance[y * width + x] +
                                     avgCr[y * width + x]);
            else
                color[3 * (y * width + x) + 0] = 255;

            color[3 * (y * width + x) + 1] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCg[y * width + x]);
            color[3 * (y * width + x) + 2] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCb[y * width + x]);

        }

    for (y = 3; y < height - 4; y += 2) //blue
        for (x = 3; x < width - 4; x += 2)
        {
            avgRed[y * width + x] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4.0;
            avgGre[y * width + x] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4.0;
            avgBlu[y * width + x] =
                (grey[y * width + x] + grey[(y - 2) * width + x] +
                 grey[(y + 2) * width + x] + grey[y * width + x - 2] +
                 grey[y * width + x + 2]) / 5.0;
            avgLum[y * width + x] =
                0.2 * avgRed[y * width + x] + 0.7 * avgGre[y * width + x] +
                0.1 * avgBlu[y * width + x];
            avgCr[y * width + x] =
                avgRed[y * width + x] - avgLum[y * width + x];
            avgCg[y * width + x] =
                avgGre[y * width + x] - avgLum[y * width + x];
            avgCb[y * width + x] =
                avgBlu[y * width + x] - avgLum[y * width + x];
            luminance[y * width + x] =
                grey[y * width + x] - avgCb[y * width + x];
            if (luminance[y * width + x] + avgCr[y * width + x] < 256)
                color[3 * (y * width + x) + 0] =
                    (unsigned char) (luminance[y * width + x] +
                                     avgCr[y * width + x]);
            else
                color[3 * (y * width + x) + 0] = 255;
            if (luminance[y * width + x] + avgCg[y * width + x] < 256)
                color[3 * (y * width + x) + 1] =
                    (unsigned char) (luminance[y * width + x] +
                                     avgCg[y * width + x]);
            else
                color[3 * (y * width + x) + 1] = 255;
            color[3 * (y * width + x) + 2] =
                (unsigned char) (luminance[y * width + x] +
                                 avgCb[y * width + x]);

        }
#endif

#ifdef MODE3

    double *red = (double *) malloc (width * height * sizeof (double));
    double *gre = (double *) malloc (width * height * sizeof (double));
    double *blu = (double *) malloc (width * height * sizeof (double));

    for (y = 2; y < height; y += 2) //red
        for (x = 2; x < width - 2; x += 2)
        {
            red[y * width + x] = grey[y * width + x];
            blu[y * width + x] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4.0;
            gre[y * width + x] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4.0;
        }

    for (y = 1; y < height - 2; y += 2) //blue
        for (x = 1; x < width; x += 2)
        {
            red[y * width + x] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4.0;
            blu[y * width + x] = grey[y * width + x];
            gre[y * width + x] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4.0;
        }

    for (y = 1; y < height - 2; y += 2) //green1
        for (x = 2; x < width - 2; x += 2)
        {
            red[y * width + x] =
                (red[(y - 1) * width + x] + red[(y + 1) * width + x] +
                 red[y * width + x - 1] + red[y * width + x + 1]) / 4.0;
            gre[y * width + x] = grey[y * width + x];
            blu[y * width + x] =
                (blu[(y - 1) * width + x] + blu[(y + 1) * width + x] +
                 blu[y * width + x - 1] + blu[y * width + x + 1]) / 4.0;
        }

    for (y = 2; y < height - 2; y += 2) //green2
        for (x = 1; x < width - 2; x += 2)
        {
            red[y * width + x] =
                (red[(y - 1) * width + x] + red[(y + 1) * width + x] +
                 red[y * width + x - 1] + red[y * width + x + 1]) / 4.0;
            gre[y * width + x] = grey[y * width + x];
            blu[y * width + x] =
                (blu[(y - 1) * width + x] + blu[(y + 1) * width + x] +
                 blu[y * width + x - 1] + blu[y * width + x + 1]) / 4.0;
        }

    for (y = 1; y < height - 1; y++)
        for (x = 2; x < width - 1; x++)
        {
            color[3 * (y * width + x) + 0] = red[y * width + x];
            color[3 * (y * width + x) + 1] = gre[y * width + x];
            color[3 * (y * width + x) + 2] = blu[y * width + x];
        }
#endif
#ifdef MODE4

    unsigned char G1, G2, G3, G4, G5, G6, G7, G8, G9;
    unsigned char E1, E2, E3, E4, E5, E6, E7, E8, E9;
    unsigned char B1, B2, B3, B4, B5, B6, B7, B8, B9;
    double T = 1.5;

    //Step1: Interpolation of Green

    for (y = 1; y < height - 1; y++)
        for (x = 1; x < width - 1; x++)
        {
            if ((x + y) % 2 == 0)   // interpolation on red and blue locations
            {
                G1 = grey[(y - 1) * width + (x - 1)];
                G2 = grey[(y - 1) * width + (x + 0)];
                G3 = grey[(y - 1) * width + (x + 1)];
                G4 = grey[(y + 0) * width + (x - 1)];
                G6 = grey[(y + 0) * width + (x + 1)];
                G7 = grey[(y + 1) * width + (x - 1)];
                G8 = grey[(y + 1) * width + (x + 0)];
                G9 = grey[(y + 1) * width + (x + 1)];

                if ((fabs (G4 - G6) < T) && (fabs (G2 - G8) > T))
                {
                    E2 = 0;
                    E8 = 0;
                    E4 = 1;
                    E6 = 1;
                }
                else if ((fabs (G4 - G6) > T) && (fabs (G2 - G8) < T))
                {
                    E2 = 1;
                    E8 = 1;
                    E4 = 0;
                    E6 = 0;
                }
                else
                {
                    E2 = 1;
                    E8 = 1;
                    E4 = 1;
                    E6 = 1;
                }

                E1 = 1;
                E3 = 1;
                E7 = 1;
                E9 = 1;

                color[3 * (y * width + x) + 1] =
                    (E2 * G2 + E4 * G4 + E6 * G6 + E8 * G8) / (E2 + E4 + E6 +
                                                               E8);
            }
            else
                color[3 * (y * width + x) + 1] = grey[y * width + x];

        }

    //Step2: Interpolation of Blue
    for (y = 2; y < height - 2; y += 2) //red locations
        for (x = 2; x < width - 2; x += 2)
        {
            G5 = color[3 * (y * width + x) + 1];
            B1 = grey[(y - 1) * width + (x - 1)];
            G1 = color[3 * ((y - 1) * width + (x - 1)) + 1];
            B3 = grey[(y - 1) * width + (x + 1)];
            G3 = color[3 * ((y - 1) * width + (x + 1)) + 1];
            B7 = grey[(y + 1) * width + (x - 1)];
            G7 = color[3 * ((y + 1) * width + (x - 1)) + 1];
            B9 = grey[(y + 1) * width + (x + 1)];
            G9 = color[3 * ((y + 1) * width + (x + 1)) + 1];
            color[3 * (y * width + x) + 2] =
                (unsigned char) (G5 *
                                 (((float) B1 / G1) + ((float) B3 / G3) +
                                  ((float) B7 / G7) +
                                  ((float) B9 / G9)) / 4.0f);

        }

    for (y = 1; y < height - 1; y += 2) //blue locations
        for (x = 1; x < width - 1; x += 2)
            color[3 * (y * width + x) + 2] = grey[y * width + x];


    for (y = 1; y < height - 1; y++)
        for (x = 1; x < width - 1; x++)
        {
            if ((x + y) % 2 == 1)   // interpolation on green locations
            {
                G5 = color[3 * (y * width + x) + 1];
                G2 = color[3 * ((y - 1) * width + (x + 0)) + 1];
                B2 = color[3 * ((y - 1) * width + (x + 0)) + 2];
                G4 = color[3 * ((y + 0) * width + (x - 1)) + 1];
                B4 = color[3 * ((y + 0) * width + (x - 1)) + 2];
                G6 = color[3 * ((y + 0) * width + (x + 1)) + 1];
                B6 = color[3 * ((y + 0) * width + (x + 1)) + 2];
                G8 = color[3 * ((y + 1) * width + (x + 0)) + 1];
                B8 = color[3 * ((y + 1) * width + (x + 0)) + 2];

                if ((x % 2 == 1) && (y % 2 == 0))
                {
                    E2 = 1;
                    E8 = 1;
                    E4 = 0;
                    E6 = 0;
                }
                else
                {
                    E2 = 0;
                    E8 = 0;
                    E4 = 1;
                    E6 = 1;
                }

                color[3 * (y * width + x) + 2] =
                    (unsigned char) (G5 *
                                     (E2 * ((float) B2 / G2) +
                                      E4 * ((float) B4 / G4) +
                                      E6 * ((float) B6 / G6) +
                                      E8 * ((float) B8 / G8)) / (E2 + E4 +
                                                                 E6 + E8));
            }
        }
    double d;

    //Step2: Interpolation of Red
    for (y = 1; y < height - 2; y += 2) //blue locations
        for (x = 1; x < width - 2; x += 2)
        {
            G5 = color[3 * (y * width + x) + 1];
            B1 = grey[(y - 1) * width + (x - 1)];
            G1 = color[3 * ((y - 1) * width + (x - 1)) + 1];
            B3 = grey[(y - 1) * width + (x + 1)];
            G3 = color[3 * ((y - 1) * width + (x + 1)) + 1];
            B7 = grey[(y + 1) * width + (x - 1)];
            G7 = color[3 * ((y + 1) * width + (x - 1)) + 1];
            B9 = grey[(y + 1) * width + (x + 1)];
            G9 = color[3 * ((y + 1) * width + (x + 1)) + 1];
            d = (G5 *
                 (((float) B1 / G1) + ((float) B3 / G3) + ((float) B7 / G7) +
                  ((float) B9 / G9)) / 4.0f);
            if (d < 256)
                color[3 * (y * width + x) + 0] = (unsigned char) (d);
            else
                color[3 * (y * width + x) + 0] = 255;

        }

    for (y = 2; y < height - 1; y += 2) //red locations
        for (x = 2; x < width - 1; x += 2)
            color[3 * (y * width + x) + 0] = grey[y * width + x];

    for (y = 1; y < height - 1; y++)
        for (x = 1; x < width - 1; x++)
        {
            if ((x + y) % 2 == 1)   // interpolation on green locations
            {
                G5 = color[3 * (y * width + x) + 1];
                G2 = color[3 * ((y - 1) * width + (x + 0)) + 1];
                B2 = color[3 * ((y - 1) * width + (x + 0)) + 0];
                G4 = color[3 * ((y + 0) * width + (x - 1)) + 1];
                B4 = color[3 * ((y + 0) * width + (x - 1)) + 0];
                G6 = color[3 * ((y + 0) * width + (x + 1)) + 1];
                B6 = color[3 * ((y + 0) * width + (x + 1)) + 0];
                G8 = color[3 * ((y + 1) * width + (x + 0)) + 1];
                B8 = color[3 * ((y + 1) * width + (x + 0)) + 0];

                if ((x % 2 == 0) && (y % 2 == 1))
                {
                    E2 = 1;
                    E8 = 1;
                    E4 = 0;
                    E6 = 0;
                }
                else
                {
                    E2 = 0;
                    E8 = 0;
                    E4 = 1;
                    E6 = 1;
                }
                d = (G5 *
                     (E2 * ((float) B2 / G2) + E4 * ((float) B4 / G4) +
                      E6 * ((float) B6 / G6) + E8 * ((float) B8 / G8)) / (E2 +
                                                                          E4 +
                                                                          E6 +
                                                                          E8));
                if (d < 256)
                    color[3 * (y * width + x) + 0] = (unsigned char) (d);
                else
                    color[3 * (y * width + x) + 0] = 255;
            }
        }

//          color[3*(y*width+x)+0] = 0;
//          color[3*(y*width+x)+2] = color[3*(y*width+x)+1]*(E1*())/();



//          red[y*width+x] = grey[y*width+x];
//          blu[y*width+x] = (grey[(y-1)*width+x-1]+grey[(y-1)*width+x+1]+grey[(y+1)*width+x-1]+grey[(y+1)*width+x+1])/4.0;
//          gre[y*width+x] = (grey[(y-1)*width+x]+grey[(y+1)*width+x]+grey[y*width+x-1]+grey[y*width+x+1])/4.0;

#endif
}


int
RCbuildC2LMapBayer (int nEcc, int nAng, int xSize, int ySize, double overlap,
                    double scaleFact, int mode, char *path)
{
    if (overlap <= -1.0)
        return 1;

	int cSize = xSize;
	
	if(xSize>ySize)
		cSize = ySize;

    double angle = (2.0 * PI / nAng);   //Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);

    int fov;
    int lim;

    double lambda;              //Log Index
    double firstRing;           //Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            //Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            //Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  //lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j;

    cart2LpPixel c2LpMap;

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    tangaxis = new double[nEcc];
    radialaxis = new double[nEcc];
    focus = new double[nEcc];
    radii2 = new double[nEcc];

    currRad = new double[nEcc];
    nextRad = new double[nEcc];

    if ((tangaxis == NULL) || (radialaxis == NULL) || (focus == NULL)
        || (radii2 == NULL))
        return 2;

    if ((currRad == NULL) || (nextRad == NULL))
        return 2;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < nEcc; rho++)
    {
        if (rho < fov)
            if (rho == 0)
            {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else
            {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else
        {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL))
        {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }
    }
    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            focus[rho] = 0;
        else
	{
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
	}		
        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }

    int totalcounter = 0;

    FILE *fout;
    char filename[256];

    sprintf (filename, "%sC2LPBayer.gio", path);

    fout = fopen (filename, "wb");

    fwrite (&totalcounter, sizeof (int), 1, fout);

    double *sintable;
    double *costable;

    int intx, inty;

    sintable = new double[nAng];
    costable = new double[nAng];

    if ((sintable == NULL) || (costable == NULL))
        return 2;

    for (j = 0; j < nAng; j++)
    {
        sintable[j] = sin (angle * (j + 0.5));
        costable[j] = cos (angle * (j + 0.5));
    }

    bool found;
    int mapsize;
    float *weight;
    double precision = 10.0;

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);

        c2LpMap.position = new int[mapsize];
        weight = new float[mapsize];
        c2LpMap.iweight = new int[mapsize];

        if ((c2LpMap.position == NULL) || (weight == NULL)
            || (c2LpMap.iweight == NULL))
            return 2;

        for (theta = 0; theta < nAng; theta++)
        {
            memset (c2LpMap.position, 0, mapsize);
            memset (weight, 0, mapsize);

            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0)
            {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else
            {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step)
                {
                    intx = (int) (locX + xSize / 2);
                    inty = (int) (locY + ySize / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                    {
                        if ((inty < ySize) && (inty >= 0))
                        {
                            if ((intx < xSize) && (intx >= 0))
                            {
                                if ((((intx + inty) % 2 == 1)
                                     && ((rho + theta) % 2 == 1))
                                    || ((intx % 2 == 0) && (inty % 2 == 0)
                                        && (rho % 2 == 0) && (theta % 2 == 0))
                                    || ((intx % 2 == 1) && (inty % 2 == 1)
                                        && (rho % 2 == 1)
                                        && (theta % 2 == 1)))
                                {
                                    found = false;
                                    j = 0;
                                    while ((j < mapsize)
                                           && (c2LpMap.position[j] != 0))
                                    {
                                        if (c2LpMap.position[j] ==
                                            (inty * xSize + intx))
                                        {
                                            weight[j]++;
                                            found = true;
                                            j = mapsize;
                                        }
                                        j++;
                                    }

                                    if (!found)
                                        for (j = 0; j < mapsize; j++)
                                        {
                                            if (c2LpMap.position[j] == 0)
                                            {
                                                c2LpMap.position[j] =
                                                    (inty * xSize + intx);
                                                weight[j]++;
                                                break;
                                            }
                                        }
                                }
                            }
                        }
                    }
                }

            for (j = 0; j < mapsize; j++)
                if (weight[j] == 0)
                    break;

            c2LpMap.divisor = j;

            if (j == 0)
            {
                intx = (int) (x0 + 0.0 + xSize / 2);
                inty = (int) (y0 + 0.0 + ySize / 2);
                if ((rho + theta) % 2 == 1)
                {
                    //find closest green pixel
                    if ((intx + inty) % 2 == 1) //I'm on green
                    {
                        c2LpMap.position[0] = (inty * xSize + intx);
                    }
                    else        //I'm not
                    {
                        if (intx > inty)
                            c2LpMap.position[0] = ((inty - 1) * xSize + intx);
                        else
                            c2LpMap.position[0] = ((inty) * xSize + intx - 1);
                    }
                    c2LpMap.divisor = 1;
                }
                else if ((rho % 2 == 0) && (theta % 2 == 0))
                {
                    if ((inty % 2 == 0) && (intx % 2 == 0))
                    {
                        c2LpMap.position[0] = (inty * xSize + intx);
                    }
                    else if ((inty % 2 == 0) && (intx % 2 == 1))
                    {
                        c2LpMap.position[0] = ((inty) * xSize + intx - 1);
                    }
                    else if ((inty % 2 == 1) && (intx % 2 == 0))
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx);
                    }
                    else
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx - 1);
                    }
                    c2LpMap.divisor = 1;

                }
                else
                {
                    if ((inty % 2 == 1) && (intx % 2 == 1))
                    {
                        c2LpMap.position[0] = (inty * xSize + intx);
                    }
                    else if ((inty % 2 == 0) && (intx % 2 == 1))
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx);
                    }
                    else if ((inty % 2 == 1) && (intx % 2 == 0))
                    {
                        c2LpMap.position[0] = ((inty) * xSize + intx - 1);
                    }
                    else
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx - 1);
                    }
                    c2LpMap.divisor = 1;

                }
                weight[0] = 1;
                j = 1;
            }

            totalcounter += c2LpMap.divisor;

            float sum = 0.0;
            int k;
            for (k = 0; k < j; k++)
                sum += weight[k];
            for (k = 0; k < j; k++)
            {
                weight[k] = weight[k] / sum;
                c2LpMap.iweight[k] = (int) (weight[k] * 65536.0);
            }

            fwrite (&c2LpMap.divisor, sizeof (int), 1, fout);
            fwrite (c2LpMap.position, sizeof (int), c2LpMap.divisor, fout);
            fwrite (c2LpMap.iweight, sizeof (int), c2LpMap.divisor, fout);

        }
        delete[]c2LpMap.position;
        delete[]weight;
        delete[]c2LpMap.iweight;
    }
    delete[]tangaxis;
    delete[]radialaxis;
    delete[]focus;
    delete[]radii2;
    delete[]currRad;
    delete[]nextRad;
    delete[]sintable;
    delete[]costable;

    fseek (fout, 0, SEEK_SET);
    fwrite (&totalcounter, sizeof (int), 1, fout);
    fclose (fout);

    return 0;

}

void
RCgetLpImg (unsigned char *lpImg, unsigned char *cartImg,
            cart2LpPixel * Table, int sizeLp, bool bayerImg)
{
    int i, j, k, z;
    int r[3];
    int pos;
    int t = 0;

    z = 3;

    if (bayerImg)
        z = 1;

    for (j = 0; j < sizeLp; j++)
    {
        for (k = 0; k < z; k++)
            r[k] = 0;

        t = 0;

        for (i = 0; i < Table[j].divisor; i++)
        {
            pos = Table[j].position[i];
            for (k = 0; k < z; k++)
                r[k] += cartImg[pos + k] * Table[j].iweight[i];
            t += Table[j].iweight[i];
        }
        for (k = 0; k < z; k++)
        {
            if (t == 0)
                printf ("t is zero!, position %d\n",j);
            lpImg[z * j + k] = (unsigned char) (r[k] / t);
        }
    }
}

void
RCgetCartImg (unsigned char *cartImg, unsigned char *lpImg,
              lp2CartPixel * Table, int cartSize)
{
    int i, j, k;

    int tempPixel[3];

    for (j = 0; j < cartSize; j++)
    {
        for (k = 0; k < 3; k++)
            tempPixel[k] = 0;

        if (Table[j].iweight != 0)
        {
            for (i = 0; i < Table[j].iweight; i++)
                for (k = 0; k < 3; k++)
                {
                    tempPixel[k] += lpImg[Table[j].position[i] + k];
                }
            for (k = 0; k < 3; k++)
                cartImg[3 * j + k] = tempPixel[k] / Table[j].iweight;
        }
    }
}

int
RCbuildL2CMap (int nEcc, int nAng, int xSize, int ySize, double overlap,
               double scaleFact, int hOffset, int vOffset, int mode,
               char *path)
{
    if (overlap <= -1.0)
        return 1;

	int cSize = xSize;
	
	if(xSize>ySize)
		cSize = ySize;

    double angle = (2.0 * PI / nAng);   //Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);

    int fov;                    //Number of rings in fovea
    int lim;

    double lambda;              //Log Index
    double firstRing;           //Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            //Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            //Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  //lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;              //Cartesian Coordinates
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    int *partCtr;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j;

    lp2CartPixel *Lp2CMap = new lp2CartPixel[xSize * ySize];

    partCtr = new int[xSize * ySize];
    memset (partCtr, 0, xSize * ySize * sizeof (int));

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    tangaxis = new double[nEcc];
    radialaxis = new double[nEcc];
    focus = new double[nEcc];
    radii2 = new double[nEcc];

    currRad = new double[nEcc];
    nextRad = new double[nEcc];

    if ((tangaxis == NULL) || (radialaxis == NULL) || (focus == NULL)
        || (radii2 == NULL))
        return 2;

    if ((currRad == NULL) || (nextRad == NULL))
        return 2;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < nEcc; rho++)
    {
        if (rho < fov)
            if (rho == 0)
            {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else
            {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else
        {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL))
        {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }

    }

    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < nEcc; rho++)
    {
        if (mode != ELLIPTICAL)
            focus[rho] = 0;
        else
	{
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
	}
        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }


    int totalcounter = 0;       //Table Size

    FILE *fout;
    char filename[256];

    sprintf (filename, "%sLP2C.gio", path);

    fout = fopen (filename, "wb");

    double *sintable;
    double *costable;

    int intx, inty;

    sintable = new double[nAng];
    costable = new double[nAng];

    if ((sintable == NULL) || (costable == NULL))
        return 2;

    for (j = 0; j < nAng; j++)
    {
        sintable[j] = sin (angle * (j + 0.5));  //Angular positions of the centers of the RF's
        costable[j] = cos (angle * (j + 0.5));
    }

    bool found;
    double precision = 10.0;

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        for (theta = 0; theta < nAng; theta++)
        {
            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0)
            {
                F0x = x0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * sintable[theta];

                F0y = y0 + focus[rho] * costable[theta];
                F1y = y0 - focus[rho] * costable[theta];

                maxaxis = tangaxis[rho];
            }
            else
            {
                F0x = x0 - focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * costable[theta];

                F0y = y0 - focus[rho] * sintable[theta];
                F1y = y0 + focus[rho] * sintable[theta];

                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step)
                {
                    intx = (int) (locX + xSize / 2);
                    inty = (int) (locY + ySize / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                        if ((inty + vOffset < ySize) && (inty + vOffset >= 0))
                            if ((intx + hOffset < xSize)
                                && (intx + hOffset >= 0))
                                partCtr[(inty + vOffset) * xSize + intx +
                                        hOffset]++;
                }

        }
    }

    for (j = 0; j < xSize * ySize; j++)
    {
        Lp2CMap[j].position = new int[partCtr[j]];
        memset (Lp2CMap[j].position, -1, sizeof (int) * partCtr[j]);
        Lp2CMap[j].iweight = 0;
    }

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        for (theta = 0; theta < nAng; theta++)
        {

            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0)
            {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else
            {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];


            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step)
                {
                    intx = (int) (locX + xSize / 2);
                    inty = (int) (locY + ySize / 2);


                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                        if ((inty + vOffset < ySize) && (inty + vOffset >= 0))
                            if ((intx + hOffset < xSize)
                                && (intx + hOffset >= 0))
                            {
                                found = false;
                                for (j = 0;
                                     j <
                                     partCtr[(inty + vOffset) * xSize + intx +
                                             hOffset]; j++)
                                {
                                    if (Lp2CMap
                                        [(inty + vOffset) * xSize + intx +
                                         hOffset].position[j] ==
                                        3 * (rho * nAng + theta))
                                    {
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    Lp2CMap[(inty + vOffset) * xSize + intx +
                                            hOffset].
                                        position[Lp2CMap
                                                 [(inty + vOffset) * xSize +
                                                  intx + hOffset].iweight] =
                                        3 * (rho * nAng + theta);
                                    Lp2CMap[(inty + vOffset) * xSize + intx +
                                            hOffset].iweight++;
                                }

                            }
                }
        }
    }


    fwrite (&totalcounter, sizeof (int), 1, fout);
    for (j = 0; j < xSize * ySize; j++)
    {
        fwrite (&Lp2CMap[j].iweight, sizeof (int), 1, fout);
        fwrite (Lp2CMap[j].position, sizeof (int), Lp2CMap[j].iweight, fout);
        totalcounter += Lp2CMap[j].iweight;
    }
    fseek (fout, 0, SEEK_SET);
    fwrite (&totalcounter, sizeof (int), 1, fout);
    fclose (fout);

    delete[]partCtr;

    return 0;
}
