#ifndef __READPOSITIONS__
#define __READPOSITIONS__

#include <vector>

typedef std::vector<double *> PositionList;
typedef std::vector<double *>::iterator PositionListIt;
typedef std::vector<double *>::const_iterator PositionListConstIt;

bool readCube(const char *filename, PositionList &list);

#endif
