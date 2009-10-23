/**
 * \file xloader.cpp
 * \brief This loads a mesh from a 3D file and loads it onto the simulator..
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#include "xloader.h"

#if _MSC_VER
#pragma warning(disable:4996)
#endif

#define MAX_SYMBOL 250  
#define MODEL_SCALE 10
//---------------------------------------------------------------
//---------------------------------------------------------------
void dLoadMeshFromX( const char* filename, dTriMesh* Trimesh )
{
  char word[MAX_SYMBOL];
  int  symbol;
  int  indexCount;
  char *p;
  int  i = 0, j = 0;
  FILE *in;

  if ((in = fopen(filename, "r")) == NULL) {
	 //ERROR!
  }
  else {
	printf("Loading mesh data from '%s' ", filename);
	while ((symbol = fscanf(in, "%s", word)) != EOF) { // Read till the end of file
	  // Skip templates
	  if (strcmp(word, "template") == 0) {
		int ret, ret1;
    	ret = fscanf(in, "%s", word);
		ret1 = fscanf(in, "%s", word);
	  }

	  else 

	  if (strcmp(word, "Mesh") == 0){	// All you need is Mesh !!
		int ret0 = fscanf(in, "%s", word);
		if (strcmp(word, "{") != 0 )	// If the mesh has a name
		int ret1 = fscanf(in, "%s", word);			// skip it.

		int ret2 = fscanf(in, "%d", &(Trimesh->VertexCount));	// Get vertex count

		Trimesh->Vertices = (float *)malloc(Trimesh->VertexCount * 3 *sizeof(float *)); 

		int ret3 = fscanf(in, "%s", word);
		printf("...");
		for (i = 0; i < Trimesh->VertexCount; i++) {
			int ret4 = fscanf(in, "%s", word);
			p = strtok(word, ",;");		
			while (p != NULL) {
					Trimesh->Vertices[j] = atof(p) * MODEL_SCALE;
				p = strtok(NULL, ",;");
				j++;
			}
		}
        
		printf("...");

		int ret5 = fscanf(in, "%d", &indexCount);	// Get index count

		Trimesh->IndexCount = indexCount * 3;

		Trimesh->Indices = (int *)malloc(Trimesh->IndexCount * sizeof(int *)); 

		int ret6 = fscanf(in, "%s", word);	

		for (i = 0; i < indexCount; i++) {
			j = 0;
			int ret7 = fscanf(in, "%s", word);
			p = strtok(word, ",;");
			while (p != NULL) {
				if (j != 0) {
					Trimesh->Indices[i*3+j-1] = atoi(p);
				}
				p = strtok(NULL, ",;");
			    j++;
			}
		}
		printf("...");
	  }
	}
  }

  printf(" OK!\n");

  fclose(in);
}
