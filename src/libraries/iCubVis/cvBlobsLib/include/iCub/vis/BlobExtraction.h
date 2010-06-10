//***********************************************************//
//* Blob analysis package  8 August 2003                    *//
//* Version 1.0                                             *//
//* Input: IplImage* binary image                           *//
//* Output: attributes of each connected region             *//
//* Author: Dave Grossman                                   *//
//* Email: dgrossman@cdr.stanford.edu                       *//
//* Acknowledgement: the algorithm has been around > 20 yrs *//
//* LICENSE: See README.TXT                                 *//
//***********************************************************//


#if !defined(_CLASSE_BLOBEXTRACTION_INCLUDED)
#define _CLASSE_BLOBEXTRACTION_INCLUDED


//! Extreu els blobs d'una imatge
bool BlobAnalysis(IplImage* inputImage, uchar threshold, IplImage* maskImage,
				    bool borderColor, bool findmoments, blob_vector &RegionData );

 
// FUNCIONS AUXILIARS

//! Fusiona dos blobs
void Subsume(blob_vector &RegionData, int, int*, CBlob*, CBlob*, bool, int, int );
//! Reallocata el vector auxiliar de blobs subsumats
int *NewSubsume(int *SubSumedRegion, int elems_inbuffer);
//! Retorna el perimetre extern d'una run lenght
double GetExternPerimeter( int start, int end, int row, int width, int height, IplImage *maskImage );

#endif //_CLASSE_BLOBEXTRACTION_INCLUDED
