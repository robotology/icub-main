// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "readPositions.h"
#include <stdio.h>

bool readCube(const char *filename, PositionList &list)
{
	list.clear();

	FILE *fp=fopen(filename, "rt");

	if (fp==0)
	{
		fprintf(stderr, "Error: could not open %s\n", filename);
		return false;
	}

	char c;
	int k=0;

	int elem=0;
	while(c!=EOF)
	{
		k=0;
        double *tmp=new double [7];
        double skip;
		for(k=0;k<7;k++)
            c=fscanf(fp, "%lf", &tmp[k]);
        
        // skip 3
		for(k=0;k<3;k++)
            c=fscanf(fp, "%lf", &skip);

        if (c!=EOF)
            {
                elem++;
                list.push_back(tmp);
            }
	}

	printf("Ok, read %d elements\n", elem);
	return true;
}
