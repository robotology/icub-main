// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* This code was written starting from tutorial code by Kosei Demura
* email:   vadim.tikhanoff@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/**
 * \file xloader.cpp
 * \brief This loads a mesh from a 3D file and loads it onto the simulator..
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#include "xloader.h"
#include <yarp/os/LogStream.h>

#if _MSC_VER
#pragma warning(disable:4996 4244 4305)
#endif

const float ModelScale = 1.01;
//---------------------------------------------------------------
//---------------------------------------------------------------
dxTriMeshX *dLoadMeshFromX(const char* FileName)
{
    dxTriMeshX *tmpTriMesh = new dxTriMeshX;
    char buff[256];
    char* word = buff;
    char*  symbol;
    int  indexCount;
    char *p;
    int  i, j=0;
    int ret;
    FILE *in;
    double dblval;
    int intval;

    if ((in = fopen(FileName, "r")) == NULL) {
        yError ("Can't open the file '%s'\n", FileName);
        return 0;
    }
    else {
        yDebug("Loading mesh data from '%s' ", FileName);

        while((symbol = fgets(buff, 256, in)) != (char*) NULL) { // Read till the end of file
            word = buff;
            while (word[0]==' ' || word[0]=='\t')
                word=word+1;
            if (strcmp(word, "template") == 0) {
                ret=fscanf(in, "%s", word);
                ret=fscanf(in, "%s", word);
            }else
                if (strncmp(word, "Mesh ", 5) == 0){								// All you need is Mesh !!
                if (strcmp(word, "{") != 0 )	// If the mesh has a name
                //fscanf(in, "%s", word);			// then skip it.
                ret=fscanf(in, "%d", &(tmpTriMesh->VertexCount));	// Get vertex count
                tmpTriMesh->Vertices = (float *)malloc(tmpTriMesh->VertexCount * 3 * sizeof(float));
                ret=fscanf(in, "%s", word);
                yDebug("...");
                if (fgets(word, 256, in)==0)
                    return 0;

                for (i = 0; i < tmpTriMesh->VertexCount; i++) {
                    //fscanf(in, "%s", word);
                    if (fgets(word, 256, in)==0)
                        return 0;

                //yDebug("Read(%d): '%s'\n", i, word);
                p = strtok(word, ",;");
                while (p != NULL) {
                    //yDebug("p = '%s'\n", p);
                    ret = sscanf(p, "%lf", &dblval);
                    if(ret > 0) { // only process if double was read
                        tmpTriMesh->Vertices[j] = dblval * ModelScale;
                        j++;
                    }
                    p = strtok(NULL, ",;");
                    //yDebug("j = %d\n", j);
                    }
                }
                yDebug("...");
                ret=fscanf(in, "%d", &indexCount);	// Get index count
                tmpTriMesh->IndexCount = indexCount * 3;
                tmpTriMesh->Indices = (int *)malloc(tmpTriMesh->IndexCount * sizeof(int));

                if (fgets(word, 256, in)==0)
                    return 0;

                for (i = 0; i < indexCount; i++) {
                    if (fgets(word, 256, in)==0)
                        return 0;

                    p = strtok(word, ",;");
                    ret = sscanf(p, "%d", &intval);
                    if(intval != 3) {
                        yError("Only triangular polygons supported! Convert your model!\n");
                        return 0;
                    }
                    for(j = 0; j < 3; j++) {//hardcoded 3
                        p = strtok(NULL, ",;");
                        ret = sscanf(p, "%d", &intval);
                        tmpTriMesh->Indices[i*3 + j] = intval;
                    }
            }
                yDebug("... OK!\n");
            }
        }
    }
    fclose(in);

    if ((in = fopen(FileName, "r")) == NULL) {
        yError ("Can't open the file '%s'\n", FileName);
        return 0;
    }
    else {
        yDebug("Loading texture coordinate from '%s' ", FileName);
        j=0;
        while((symbol = fgets(buff, 256, in)) != (char*) NULL) {
            word = buff;
            while (word[0]==' ' || word[0]=='\t')
                word=word+1;
            if (strncmp(word, "MeshTextureCoords", 17) == 0){
                ret=fscanf(in, "%d", &(tmpTriMesh->MeshCoordCount));
                if (fgets(word, 256, in)==NULL)
                    return 0; // consume newline

            tmpTriMesh->MeshCoord = (float *)malloc(tmpTriMesh->MeshCoordCount*2 * sizeof(float));
            yDebug("...");
            for (i = 0; i < tmpTriMesh->MeshCoordCount; i++) {
                if (fgets(word, 256, in)==0)
                    return 0;

                p = strtok(word, ",;");
                while (p != NULL) {
                    ret = sscanf(p, "%lf", &dblval);
                    if(ret > 0) {
                        tmpTriMesh->MeshCoord[j] = dblval ;//* ModelScale;
                        j++;
                    }
                    p = strtok(NULL, ",;");
                    }
                }
                yDebug("...");
            }
        }yDebug("... OK!\n");
    }

    fclose(in);
    //now load normals
    if ((in = fopen(FileName, "r")) == NULL) {
        yError ("Can't open the file '%s'\n", FileName);
        return 0;
    }
    else {
        yDebug("Loading normals from '%s' ", FileName);
        j=0;
        while((symbol = fgets(buff, 256, in)) != (char*) NULL) {
            word = buff;
            while (word[0]==' ' || word[0]=='\t')
                word=word+1;
            if (strncmp(word, "MeshNormals", 11) == 0){
                ret=fscanf(in, "%d", &(tmpTriMesh->NormCount));
                if (fgets(word, 256, in)==NULL)
                    return 0; // consume newline

            tmpTriMesh->NormCoord = (float *)malloc(tmpTriMesh->NormCount*3 * sizeof(float));
            yDebug("...");
            for (i = 0; i < tmpTriMesh->NormCount; i++) {
                if (fgets(word, 256, in)==0)
                    return 0;

                p = strtok(word, ",;");
                while (p != NULL) {
                    ret = sscanf(p, "%lf", &dblval);
                    if(ret > 0) {
                        tmpTriMesh->NormCoord[j] = dblval ;//* ModelScale;
                        j++;
                    }
                    p = strtok(NULL, ",;");
                    }
                }
                yDebug("...");
            }
        }yDebug("... OK!\n");
    }

    fclose(in);
    return tmpTriMesh;
}
void dTriMeshXDestroy(dTriMeshX TriMesh)
{
    free (TriMesh->Vertices);
    free (TriMesh->Indices);
    free (TriMesh->MeshCoord);
    free (TriMesh->NormCoord);
    free (TriMesh);
}
