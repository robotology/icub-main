
//	File: ImageStack.h
//
//	This class provides a template defintion for a circular buffer of 
//	images (or anything else you may wish to store) 
//
//	Author: Chris McCarthy (cdmcc)
//	Date: June 2007	
//

#ifndef IMAGESTACK_H
#define IMAGESTACK_H

#include <Defn.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <string.h>

using namespace yarp::os;
using namespace yarp::sig;

template <class T>
class ImageStack
{
	public: typedef T ContentType;
			  int count;
			  int *stack_count;
	public: 
//		ImageStack() { ImageStack(0, 0); }

	// creates a new, empty image stack
		ImageStack(int limit = 5, int bufferSize=5)
		{
			is_full = FALSE;
      	lock = FALSE;
      	stackLimit = limit;
      	this->bufferSize = bufferSize;
      	this->nSlices = 0;
      	stack = new (T *)[this->bufferSize];
			stack_count = new int [this->bufferSize];
			count = 0;
			for(int i = 0; i < bufferSize; i++){
				stack[i] = new T;
				stack_count[i] = -1;
			}
      	lo = hi = mid = -1;
      	buffer_time = 0;
      	total_frames=0;
		}


		~ImageStack()
		{
			is_full = FALSE;
   		delete[] stack;
		}

		void ImageStack::set_buffer_time(int time) { buffer_time = time; }

		int ImageStack::get_buffer_time()
		{
			return buffer_time;
		}


		// adds image to end of stack
		void addSlice(T *image)
		{
		   /*while(this->isLocked())
					  Time::delay(0.1);*/
			hi = ++hi % stackLimit;
   		mid = (++total_frames/2) % stackLimit;
   		if(total_frames == stackLimit) {
      		is_full = TRUE;
   		}
			stack_count[hi] = count++;

			stack[hi]->copy(*image);
			delete image;
   	/*	
			memcpy(stack[hi], image, sizeof(T));
			if(stack[hi]->getRawImage() == NULL){
				printf("In ImageStack addSlice() and data is NULL!!!\n");
			}
		*/
   		if(lo == hi)
      		lo = ++lo % stackLimit;
   		else {
      		lo = 0;
      		nSlices++;
   		}
   		return;
		}

		// dletes the whole stack of images
		void deleteStack()
		{
   		lo = -1;
   		hi = -1;
   		nSlices = 0;
   		total_frames = 0;
   		mid = -1;
		}

		// deletes image at index n (1<=n<=nSlices)
		void deleteSlice(int n)
		{
			if(n<1 || n>nSlices) {
      		printf("deleteSlice: n is out of range\n");
      		return;
   		}

   		if(nSlices < 1) return;

   		for(int i=n; i<nSlices; i++) {
      		stack[i-1] = stack[i];
   		}

   		//stack[nSlices-1] = NULL;
   		nSlices--;
   		mid = (--total_frames/2)%stackLimit;
		}

		// Returns ImageData for the specified slice
		T *getImage(int n)
		{
			/*
			while(this->isLocked())
			   Time::delay(0.1);
			this->Lock();	*/
			if(n > hi && n < lo) {
      		fprintf(stderr,"getImage: n is out of range");
				exit(1);
   		}
   		int index = (lo + n - 1) % stackLimit;

			if(stack[index]->getRawImage() == NULL){
				printf("IN IMAGESTACK and RAW IMAGE IS NULL! index is %d\n", index);
				printf("this_index = %d, lo = %d, hi = %d\n", index, lo, hi);
				for(int i = 0; i < 5; i++){
					printf("stack_count[%d] = %d\n", i, stack_count[i]);
				}
			}
			//this->UnLock();
   		return stack[index];
		}


  		// Get middle frames			  	
		T *getMidImage(){ return(getImage((lo + stackLimit/2) % stackLimit));}

		// Sets slice n with ImageData i
		void setImage(T i, int n)
		{
   		if(n > hi && n < lo) {
      		fprintf(stderr, "setImage: n out of range\n");
      		return;
   		}
   		int index = (lo + n - 1) % stackLimit;
   		stack[index] = image;
		}

		// Return Image array
		T **getImageArray() { return stack; }
	
		// Return number of slices in stack (1<=n<=nslices)
		int getSize() { return nSlices; }

		//Return physical limit of stack
		int getBufferSize(){ return bufferSize;}

		//Return stack size limit (soft limit)
		int getStackLimit(){ return stackLimit; } 	

		bool isFull(){return (is_full || (nSlices == stackLimit));} 

		bool isEmpty() { return nSlices == 0; }

		bool isLocked() { return lock;}

		void Lock() { lock = TRUE;}

		void UnLock() { lock = FALSE;}

	
	private: int nSlices;
    			T **stack;
   			int stackLimit;
   			bool lock;
   			int lo, hi, mid;
   			int is_full;
  				int buffer_time;
   			int total_frames;
				int bufferSize;

};
	
#endif
