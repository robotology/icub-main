/**
 * \file xloader.cpp
 * \brief This loads a mesh from a 3D file and loads it onto the simulator..
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#include "xloader.h"
#if _MSC_VER
#pragma warning(disable:4996 4244 4305)
#endif

const float ModelScale = 1.01;
//---------------------------------------------------------------
//---------------------------------------------------------------
dxTriMeshX *dLoadMeshFromX(const char* FileName)
{
	dxTriMeshX *tmpTriMesh = new dxTriMeshX;
	char word[256];
	char*  symbol;
	int  indexCount;
	char *p;
	int  i, j=0;
	int ret;
	FILE *in;
	double dblval;
	int intval;

	if ((in = fopen(FileName, "r")) == NULL) {
		printf ("Can't open the file '%s'\n", FileName);
		return 0;
		}
	else {
		printf("Loading mesh data from '%s' ", FileName);
    	
		while((symbol = fgets(word, 256, in)) != (char*) NULL) { // Read till the end of file
   			//printf("symbol[%s] word[%s]\n", symbol, word);
	  		// Skip templates
	  		if (strcmp(word, "template") == 0) {
				fscanf(in, "%s", word);
				fscanf(in, "%s", word);
	 		}else 
	  		if (strncmp(word, "Mesh ", 5) == 0){								// All you need is Mesh !!
      			//printf("\nWord: %s\n", word);
				//fscanf(in, "%s", word);
				if (strcmp(word, "{") != 0 )	// If the mesh has a name
				//fscanf(in, "%s", word);			// then skip it.
				fscanf(in, "%d", &(tmpTriMesh->VertexCount));	// Get vertex count
				tmpTriMesh->Vertices = (float *)malloc(tmpTriMesh->VertexCount * 3 * sizeof(float));
				//printf("\nVertexCount %d\n", tmpTriMesh->VertexCount );
				fscanf(in, "%s", word);
				printf("...");
        		fgets(word, 256, in); // consume newline
				for (i = 0; i < tmpTriMesh->VertexCount; i++) {
					//fscanf(in, "%s", word);
            		fgets(word, 256, in);
            		//printf("Read(%d): '%s'\n", i, word);
					p = strtok(word, ",;");		
					while (p != NULL) {
						//printf("p = '%s'\n", p);
                		ret = sscanf(p, "%lf", &dblval);
                		if(ret > 0) { // only process if double was read
				  			tmpTriMesh->Vertices[j] = dblval * ModelScale;
                  			j++;
                		}
						p = strtok(NULL, ",;");
						//printf("j = %d\n", j);
					}
				}
				printf("...");
        		//printf("\nVertexCount %d\n", tmpTriMesh->VertexCount );
				fscanf(in, "%d", &indexCount);	// Get index count
				//printf("IndexCount %d\n", indexCount );
				tmpTriMesh->IndexCount = indexCount * 3;
				tmpTriMesh->Indices = (int *)malloc(tmpTriMesh->IndexCount * sizeof(int)); 
	
        		fgets(word, 256, in);
        		//printf("Read(a): '%s'\n", word);
				//fscanf(in, "%s", word);	
	
				for (i = 0; i < indexCount; i++) {
            		//printf("Indices %d\n", i);
            		fgets(word, 256, in);
           		    p = strtok(word, ",;");
            		ret = sscanf(p, "%d", &intval);
					//printf("intVal %d \n", intval);
            		if(intval != 3) {
            	  		printf("Only triangular polygons supported! Convert your model!\n");
            	 		return 0;
            		}
            		//if(ret == 0) exit(1);
            		for(j = 0; j < 3; j++) {//hardcoded 3
            	    	p = strtok(NULL, ",;");
            	    	ret = sscanf(p, "%d", &intval);
						//printf("Read(%d,%d): '%s'\n", i, j, p);
            	    	//if(ret == 0) exit(1);
            	    	tmpTriMesh->Indices[i*3 + j] = intval;
            		}	
				}
			printf("... OK!\n");
	  	}
	}
}
  	//exit(1);
  	fclose(in);

	if ((in = fopen(FileName, "r")) == NULL) {
		printf ("Can't open the file '%s'\n", FileName);
		return 0;
		}
	else {
		printf("Loading texture coordinate from '%s' ", FileName);
    	j=0;
		while((symbol = fgets(word, 256, in)) != (char*) NULL) {
			if (strncmp(word, "MeshTextureCoords", 17) == 0){	
				fscanf(in, "%d", &(tmpTriMesh->MeshCoordCount));
        		fgets(word, 256, in); // consume newline
				//printf("MESH COORD COUNT = %d\n", tmpTriMesh->MeshCoordCount);
				tmpTriMesh->MeshCoord = (float *)malloc(tmpTriMesh->MeshCoordCount*2 * sizeof(float));
				printf("...");
				for (i = 0; i < tmpTriMesh->MeshCoordCount; i++) {	
					fgets(word, 256, in);
					//printf("Read(%d): '%s' \n", i, word );
					p = strtok(word, ",;");
					while (p != NULL) {
						ret = sscanf(p, "%lf", &dblval);
						if(ret > 0) {
							//printf("dblVal(%d) %lf \n",j, dblval);
							tmpTriMesh->MeshCoord[j] = dblval ;//* ModelScale;		
							j++;	
						}
						p = strtok(NULL, ",;");
					}
				}	
				printf("...");				
			}	
			
		}printf("... OK!\n");
	}
	
	//exit(1);
  	fclose(in);
  	return tmpTriMesh;
}
void dTriMeshXDestroy(dTriMeshX TriMesh)
{
	delete[] TriMesh->Vertices;
	delete[] TriMesh->Indices;
	delete TriMesh;
}
