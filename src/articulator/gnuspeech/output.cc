// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#include "output.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "tube.h"
#include "util.h"

// TODO (2004-05-03): Do we need to declare the function static here, at the implementation, or in both places?
//void writeAuFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile);
static void writeAiffFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile);
static void writeWaveFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile);
static void writeSamplesMonoMsb(FILE *tempFile, long int numberSamples, double scale, FILE *outputFile);
static void writeSamplesMonoLsb(FILE *tempFile, long int numberSamples, double scale, FILE *outputFile);
static void writeSamplesStereoMsb(FILE *tempFile, long int numberSamples, double leftScale, double rightScale, FILE *outputFile);
static void writeSamplesStereoLsb(FILE *tempFile, long int numberSamples, double leftScale, double rightScale, FILE *outputFile);
static size_t fwriteIntMsb(int data, FILE *stream);
static size_t fwriteIntLsb(int data, FILE *stream);
//size_t fwriteShortMsb(int data, FILE *stream);
static size_t fwriteShortLsb(int data, FILE *stream);
static void convertIntToFloat80(unsigned int value, unsigned char buffer[10]);

#include "local.h"


/******************************************************************************
*
*	function:	writeOutputToFile
*
*	purpose:	Scales the samples stored in the temporary file, and
*                       writes them to the output file, with the appropriate
*                       header.  Also does master volume scaling, and stereo
*                       balance scaling, if 2 channels of output.
*
*       arguments:      fileName
*
*	internal
*	functions:	writeAuFileHeader, writeSamplesMonoMsb,
*                       writeSamplesStereoMsb, writeAiffHeader,
*                       writeWaveHeader, writeSamplesMonoLsb,
*                       writeSamplesStereoLsb
*
*	library
*	functions:	fopen, printf, fclose
*
******************************************************************************/

void writeOutputToFile(TRMSampleRateConverter *sampleRateConverter, TRMData *data, const char *fileName)
{
    FILE *fd;
    double scale, leftScale = 0.0, rightScale = 0.0;


    /*  Calculate scaling constant  */
    //printf("maximumSampleValue: %g\n", sampleRateConverter->maximumSampleValue);
    scale = OUTPUT_SCALE * (RANGE_MAX / sampleRateConverter->maximumSampleValue) * amplitude(data->inputParameters.volume);

    /*  Print out info  */
    /*if (verbose)*/ {
	printf("\nnumber of samples:\t%-ld\n", sampleRateConverter->numberSamples);
	printf("maximum sample value:\t%.4f\n", sampleRateConverter->maximumSampleValue);
	printf("scale:\t\t\t%.4f\n", scale);
    }

    /*  If stereo, calculate left and right scaling constants  */
    if (data->inputParameters.channels == 2) {
	/*  Calculate left and right channel amplitudes  */
	leftScale = -((data->inputParameters.balance / 2.0) - 0.5) * scale * 2.0;
	rightScale = ((data->inputParameters.balance / 2.0) + 0.5) * scale * 2.0;

	/*  Print out info  */
	if (verbose) {
	    printf("left scale:\t\t%.4f\n", leftScale);
	    printf("right scale:\t\t%.4f\n", rightScale);
	}
    }

    /*  Rewind the temporary file to beginning  */
    rewind(sampleRateConverter->tempFilePtr);

    /*  Open the output file  */
    fd = fopen(fileName, "wb");

    /*  Scale and write out samples to the output file  */
    if (data->inputParameters.outputFileFormat == AU_FILE_FORMAT) {
        writeAuFileHeader(data->inputParameters.channels, sampleRateConverter->numberSamples, data->inputParameters.outputRate, fd);
        if (data->inputParameters.channels == 1)
            writeSamplesMonoMsb(sampleRateConverter->tempFilePtr, sampleRateConverter->numberSamples, scale, fd);
        else
            writeSamplesStereoMsb(sampleRateConverter->tempFilePtr, sampleRateConverter->numberSamples, leftScale, rightScale, fd);
    } else if (data->inputParameters.outputFileFormat == AIFF_FILE_FORMAT) {
        writeAiffFileHeader(data->inputParameters.channels, sampleRateConverter->numberSamples, data->inputParameters.outputRate, fd);
        if (data->inputParameters.channels == 1)
            writeSamplesMonoMsb(sampleRateConverter->tempFilePtr, sampleRateConverter->numberSamples, scale, fd);
        else
            writeSamplesStereoMsb(sampleRateConverter->tempFilePtr, sampleRateConverter->numberSamples, leftScale, rightScale, fd);
    } else if (data->inputParameters.outputFileFormat == WAVE_FILE_FORMAT) {
        writeWaveFileHeader(data->inputParameters.channels, sampleRateConverter->numberSamples, data->inputParameters.outputRate, fd);
        if (data->inputParameters.channels == 1)
            writeSamplesMonoLsb(sampleRateConverter->tempFilePtr, sampleRateConverter->numberSamples, scale, fd);
        else
            writeSamplesStereoLsb(sampleRateConverter->tempFilePtr, sampleRateConverter->numberSamples, leftScale, rightScale, fd);
    }

    /*  Close the output file  */
    fclose(fd);
}



/******************************************************************************
*
*       function:       writeAuFileHeader
*
*       purpose:        Writes the header in AU format to the output file.
*
*       internal
*       functions:      fwriteIntMsb
*
*       library
*       functions:      fputs
*
******************************************************************************/

void writeAuFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile)
{
    /*  AU magic string: ".snd"  */
    fputs(".snd", outputFile);

    /*  Header size (fixed in size at 28 bytes)  */
    fwriteIntMsb(28, outputFile);

    /*  Number of bytes of sound data  */
    fwriteIntMsb(channels * numberSamples * sizeof(short), outputFile);

    /*  Sound format:  3 is 16-bit linear  */
    fwriteIntMsb(3, outputFile);

    /*  Output sample rate in samples/second  */
    fwriteIntMsb((int)outputRate, outputFile);

    /*  Number of channels  */
    fwriteIntMsb(channels, outputFile);

    /*  Optional text description (4 bytes minimum)  */
    fwriteIntMsb(0, outputFile);
}



/******************************************************************************
*
*       function:       writeAiffFileHeader
*
*       purpose:        Writes the header in AIFF format to the output file.
*
*       internal
*       functions:      fwriteIntMsb, fwriteShortMsb, convertIntToFloat80
*
*       library
*       functions:      fputs, fwrite
*
******************************************************************************/

void writeAiffFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile)
{
    unsigned char sampleFramesPerSecond[10];
    int soundDataSize = channels * numberSamples * sizeof(short);
    int ssndChunkSize = soundDataSize + 8;
    int formSize = ssndChunkSize + 8 + 26 + 4;

    /*  Form container identifier  */
    fputs("FORM", outputFile);

    /*  Form size  */
    fwriteIntMsb(formSize, outputFile);

    /*  Form container type  */
    fputs("AIFF", outputFile);

    /*  Common chunk identifier  */
    fputs("COMM", outputFile);

    /*  Chunk size (fixed at 18 bytes)  */
    fwriteIntMsb(18, outputFile);

    /*  Number of channels  */
    fwriteShortMsb((short)channels, outputFile);

    /*  Number of sample frames  */
    fwriteIntMsb(numberSamples, outputFile);

    /*  Number of bits per samples (fixed at 16)  */
    fwriteShortMsb(BITS_PER_SAMPLE, outputFile);

    /*  Sample frames per second (output sample rate)  */
    /*  stored as an 80-bit (10-byte) float  */
    convertIntToFloat80((unsigned int)outputRate, sampleFramesPerSecond);
    fwrite(sampleFramesPerSecond, sizeof(unsigned char), 10, outputFile);

    /*  Sound Data chunk identifier  */
    fputs("SSND", outputFile);

    /*  Chunk size  */
    fwriteIntMsb(ssndChunkSize, outputFile);

    /*  Offset:  unused, so set to 0  */
    fwriteIntMsb(0, outputFile);

    /*  Block size:  unused, so set to 0  */
    fwriteIntMsb(0, outputFile);
}



/******************************************************************************
*
*       function:       writeWaveFileHeader
*
*       purpose:        Writes the header in WAVE format to the output file.
*
*       internal
*       functions:      fwriteIntLsb, fwriteShortLsb
*
*       library
*       functions:      fputs
*
******************************************************************************/

void writeWaveFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile)
{
    int soundDataSize = channels * numberSamples * sizeof(short);
    int dataChunkSize = soundDataSize;
    int formSize = dataChunkSize + 8 + 24 + 4;
    int frameSize = (int)ceil(channels * ((double)BITS_PER_SAMPLE / 8));
    int bytesPerSecond = (int)ceil(outputRate * frameSize);

    /*  Form container identifier  */
    fputs("RIFF", outputFile);

    /*  Form size  */
    fwriteIntLsb(formSize, outputFile);

    /*  Form container type  */
    fputs("WAVE", outputFile);

    /*  Format chunk identifier (Note:  space after 't' needed)  */
    fputs("fmt ", outputFile);

    /*  Chunk size (fixed at 16 bytes)  */
    fwriteIntLsb(16, outputFile);

    /*  Compression code:  1 = PCM  */
    fwriteShortLsb(1, outputFile);

    /*  Number of channels  */
    fwriteShortLsb((short)channels, outputFile);

    /*  Output Sample Rate  */
    fwriteIntLsb((int)outputRate, outputFile);

    /*  Bytes per second  */
    fwriteIntLsb(bytesPerSecond, outputFile);

    /*  Block alignment (frame size)  */
    fwriteShortLsb((short)frameSize, outputFile);

    /*  Bits per sample  */
    fwriteShortLsb((short)BITS_PER_SAMPLE, outputFile);

    /*  Sound Data chunk identifier  */
    fputs("data", outputFile);

    /*  Chunk size  */
    fwriteIntLsb(dataChunkSize, outputFile);
}



/******************************************************************************
*
*       function:       writeSamplesMonoMsb
*
*       purpose:        Reads the double f.p. samples in the temporary file,
*                       scales them, rounds them to a short (16-bit) integer,
*                       and writes them to the output file in big-endian
*                       format.
*
*       internal
*       functions:      fwriteShortMsb
*
*       library
*       functions:      fread
*
******************************************************************************/

void writeSamplesMonoMsb(FILE *tempFile, long int numberSamples, double scale, FILE *outputFile)
{
    long int i;

    /*  Write the samples to file, scaling each sample  */
    for (i = 0; i < numberSamples; i++) {
        double sample;

        fread(&sample, sizeof(sample), 1, tempFile);
        fwriteShortMsb((short)rint(sample * scale), outputFile);
        //printf("%8ld: %g -> %hd\n", i, sample, (short)rint(sample * scale));
    }
}



/******************************************************************************
*
*       function:       writeSamplesMonoLsb
*
*       purpose:        Reads the double f.p. samples in the temporary file,
*                       scales them, rounds them to a short (16-bit) integer,
*                       and writes them to the output file in little-endian
*                       format.
*
*       internal
*       functions:      fwriteShortLsb
*
*       library
*       functions:      fread
*
******************************************************************************/

void writeSamplesMonoLsb(FILE *tempFile, long int numberSamples, double scale, FILE *outputFile)
{
    long int i;

    /*  Write the samples to file, scaling each sample  */
    for (i = 0; i < numberSamples; i++) {
        double sample;

        fread(&sample, sizeof(sample), 1, tempFile);
        fwriteShortLsb((short)rint(sample * scale), outputFile);
    }
}



/******************************************************************************
*
*       function:       writeSamplesStereoMsb
*
*       purpose:        Reads the double f.p. samples in the temporary file,
*                       does stereo scaling, rounds them to a short (16-bit)
*                       integer, and writes them to the output file in
*                       big-endian format.
*
*       internal
*       functions:      fwriteShortMsb
*
*       library
*       functions:      fread
*
******************************************************************************/

void writeSamplesStereoMsb(FILE *tempFile, long int numberSamples, double leftScale, double rightScale, FILE *outputFile)
{
    long int i;

    /*  Write the samples to file, scaling each sample  */
    for (i = 0; i < numberSamples; i++) {
        double sample;

        fread(&sample, sizeof(sample), 1, tempFile);
        fwriteShortMsb((short)rint(sample * leftScale), outputFile);
        fwriteShortMsb((short)rint(sample * rightScale), outputFile);
    }
}



/******************************************************************************
*
*       function:       writeSamplesStereoLsb
*
*       purpose:        Reads the double f.p. samples in the temporary file,
*                       does stereo scaling, rounds them to a short (16-bit)
*                       integer, and writes them to the output file in
*                       little-endian format.
*
*       internal
*       functions:      fwriteShortLsb
*
*       library
*       functions:      fread
*
******************************************************************************/

void writeSamplesStereoLsb(FILE *tempFile, long int numberSamples, double leftScale, double rightScale, FILE *outputFile)
{
    long int i;

    /*  Write the samples to file, scaling each sample  */
    for (i = 0; i < numberSamples; i++) {
        double sample;

        fread(&sample, sizeof(sample), 1, tempFile);
        fwriteShortLsb((short)rint(sample * leftScale), outputFile);
        fwriteShortLsb((short)rint(sample * rightScale), outputFile);
    }
}



/******************************************************************************
*
*       function:       fwriteIntMsb
*
*       purpose:        Writes a 4-byte integer to the file stream, starting
*                       with the most significant byte (i.e. writes the int
*                       in big-endian form).  This routine will work on both
*                       big-endian and little-endian architectures.
*
*       internal
*       functions:      none
*
*       library
*       functions:      fwrite
*
******************************************************************************/

size_t fwriteIntMsb(int data, FILE *stream)
{
    unsigned char array[4];

    array[0] = (unsigned char)((data >> 24) & 0xFF);
    array[1] = (unsigned char)((data >> 16) & 0xFF);
    array[2] = (unsigned char)((data >> 8) & 0xFF);
    array[3] = (unsigned char)(data & 0xFF);
    return (fwrite(array, sizeof(unsigned char), 4, stream));
}



/******************************************************************************
*
*       function:       fwriteIntLsb
*
*       purpose:        Writes a 4-byte integer to the file stream, starting
*                       with the least significant byte (i.e. writes the int
*                       in little-endian form).  This routine will work on both
*                       big-endian and little-endian architectures.
*
*       internal
*       functions:      none
*
*       library
*       functions:      fwrite
*
******************************************************************************/

size_t fwriteIntLsb(int data, FILE *stream)
{
    unsigned char array[4];

    array[3] = (unsigned char)((data >> 24) & 0xFF);
    array[2] = (unsigned char)((data >> 16) & 0xFF);
    array[1] = (unsigned char)((data >> 8) & 0xFF);
    array[0] = (unsigned char)(data & 0xFF);
    return (fwrite(array, sizeof(unsigned char), 4, stream));
}



/******************************************************************************
*
*       function:       fwriteShortMsb
*
*       purpose:        Writes a 2-byte integer to the file stream, starting
*                       with the most significant byte (i.e. writes the int
*                       in big-endian form).  This routine will work on both
*                       big-endian and little-endian architectures.
*
*       internal
*       functions:      none
*
*       library
*       functions:      fwrite
*
******************************************************************************/

size_t fwriteShortMsb(int data, FILE *stream)
{
    unsigned char array[2];

    array[0] = (unsigned char)((data >> 8) & 0xFF);
    array[1] = (unsigned char)(data & 0xFF);
    return (fwrite(array, sizeof(unsigned char), 2, stream));
}



/******************************************************************************
*
*       function:       fwriteShortLsb
*
*       purpose:        Writes a 2-byte integer to the file stream, starting
*                       with the leastt significant byte (i.e. writes the int
*                       in little-endian form).  This routine will work on both
*                       big-endian and little-endian architectures.
*
*       internal
*       functions:      none
*
*       library
*       functions:      fwrite
*
******************************************************************************/

size_t fwriteShortLsb(int data, FILE *stream)
{
    unsigned char array[2];

    array[1] = (unsigned char)((data >> 8) & 0xFF);
    array[0] = (unsigned char)(data & 0xFF);
    return (fwrite(array, sizeof(unsigned char), 2, stream));
}



/******************************************************************************
*
*       function:       convertIntToFloat80
*
*       purpose:        Converts an unsigned 4-byte integer to an IEEE 754
*                       10-byte (80-bit) floating point number.
*
*       internal
*       functions:      none
*
*       library
*       functions:      memset
*
******************************************************************************/

void convertIntToFloat80(unsigned int value, unsigned char buffer[10])
{
    unsigned int exp;
    unsigned short i;

    /*  Set all bytes in buffer to 0  */
    memset(buffer, 0, 10);

    /*  Calculate the exponent  */
    exp = value;
    for (i = 0; i < 32; i++) {
        exp >>= 1;
        if (!exp)
            break;
    }

    /*  Add the bias to the exponent  */
    i += 0x3FFF;

    /*  Store the exponent  */
    buffer[0] = (unsigned char)((i >> 8) & 0x7F);
    buffer[1] = (unsigned char)(i & 0xFF);

    /*  Calculate the mantissa  */
    for (i = 32; i; i--) {
        if (value & 0x80000000)
            break;
        value <<= 1;
    }

    /*  Store the mantissa:  this should work on both big-endian and
        little-endian architectures  */
    buffer[2] = (unsigned char)((value >> 24) & 0xFF);
    buffer[3] = (unsigned char)((value >> 16) & 0xFF);
    buffer[4] = (unsigned char)((value >> 8) & 0xFF);
    buffer[5] = (unsigned char)(value & 0xFF);
}
