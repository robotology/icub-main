// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "ring_buffer.h"

TRMRingBuffer *TRMRingBufferCreate(int aPadSize)
{
    TRMRingBuffer *newRingBuffer;
    int index;

    newRingBuffer = (TRMRingBuffer *)malloc(sizeof(TRMRingBuffer));
    if (newRingBuffer == NULL) {
        fprintf(stderr, "Failed to malloc() space for ring buffer.\n");
        return NULL;
    }

    for (index = 0; index < BUFFER_SIZE; index++)
        newRingBuffer->buffer[index] = 0;

    newRingBuffer->padSize = aPadSize;
    newRingBuffer->fillSize = BUFFER_SIZE - (2 * newRingBuffer->padSize);

    newRingBuffer->fillPtr = newRingBuffer->padSize;
    newRingBuffer->emptyPtr = 0;
    newRingBuffer->fillCounter = 0;

    newRingBuffer->context = NULL;
    newRingBuffer->callbackFunction = NULL;

    return newRingBuffer;
}

void TRMRingBufferFree(TRMRingBuffer *ringBuffer)
{
    if (ringBuffer == NULL)
        return;

    free(ringBuffer);
}

// Fills the ring buffer with a single sample, increments
// the counters and pointers, and empties the buffer when
// full.
void dataFill(TRMRingBuffer *ringBuffer, double data)
{
    ringBuffer->buffer[ringBuffer->fillPtr] = data;

    /*  INCREMENT THE FILL POINTER, MODULO THE BUFFER SIZE  */
    RBIncrement(ringBuffer);

    /*  INCREMENT THE COUNTER, AND EMPTY THE BUFFER IF FULL  */
    if (++(ringBuffer->fillCounter) >= ringBuffer->fillSize) {
	dataEmpty(ringBuffer);
	/* RESET THE FILL COUNTER  */
	ringBuffer->fillCounter = 0;
    }
}

void dataEmpty(TRMRingBuffer *ringBuffer)
{
    if (ringBuffer->callbackFunction == NULL) {
        // Just empty the buffer.
        fprintf(stderr, "No callback function set, should just empty it...\n");
    } else {
        (*(ringBuffer->callbackFunction))(ringBuffer, ringBuffer->context);
    }
}

void RBIncrement(TRMRingBuffer *ringBuffer)
{
    if (++(ringBuffer->fillPtr) >= BUFFER_SIZE)
	ringBuffer->fillPtr -= BUFFER_SIZE;
}

void RBDecrement(TRMRingBuffer *ringBuffer)
{
    if (--(ringBuffer->fillPtr) < 0)
	ringBuffer->fillPtr += BUFFER_SIZE;
}

// Pads the buffer with zero samples, and flushes it by converting the remaining samples.
void flushBuffer(TRMRingBuffer *ringBuffer)
{
    int index;

    /*  PAD END OF RING BUFFER WITH ZEROS  */
    for (index = 0; index < (ringBuffer->padSize * 2); index++)
	dataFill(ringBuffer, 0.0);

    /*  FLUSH UP TO FILL POINTER - PADSIZE  */
    dataEmpty(ringBuffer);
}

void RBIncrementIndex(int *index)
{
    if (++(*index) >= BUFFER_SIZE)
	(*index) -= BUFFER_SIZE;
}

void RBDecrementIndex(int *index)
{
    if (--(*index) < 0)
	(*index) += BUFFER_SIZE;
}
