// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

// fix capitalization (important on linux) --paulfitz
#include "LogPolarGlobalDisparity.h"


int CLogPolarGlobalDisparity::GetDisparity(int ch)		{return m_pDisparityLUTArray[ch]->GetDisparity();};

bool CLogPolarGlobalDisparity::CreateLogPolarGlobalDisparity( int num_channels, int *disparities, CLogPolarSensor *lps )
{
	int i;

	if((m_iNumChannels=num_channels) <= 0)
		return false;
	if(!disparities)
		return false;
	
	if(m_pDisparityLUTArray)
	{
		int i;
		for(i = 0; i < m_iNumChannels; i++)
			delete m_pDisparityLUTArray[i];
		delete [] m_pDisparityLUTArray;
	}
		

	m_pDisparityLUTArray = new DISPARITY_CHANNEL_PTR[m_iNumChannels];
	if(!m_pDisparityLUTArray)
		return false;

	for(i = 0; i < m_iNumChannels; i++)
	{
		m_pDisparityLUTArray[i] = new CDisparityChannel();
		m_pDisparityLUTArray[i]->SetDisparity(disparities[i]);		
		m_pDisparityLUTArray[i]->CreateDisparityChannel(lps);
		m_pDisparityLUTArray[i]->CompressDisparityChannel(lps);
	}
	m_b_initialized = true;
	return true;
}

bool CLogPolarGlobalDisparity::CloseLogPolarGlobalDisparity()
{	
	if(!m_b_initialized)
		return false;
	int i;
	for(i = 0; i < m_iNumChannels; i++)
	{
		delete m_pDisparityLUTArray[i];
		m_pDisparityLUTArray[i] = NULL;
	}
	delete [] m_pDisparityLUTArray;
	m_pDisparityLUTArray = NULL;
	m_b_initialized = false;
	return true;
}

// COMPUTATION FUNCTIONS - CAREFULL - THE FOLLOWING FUNCTIONS WORK FOR SIZES < 16k !


int CLogPolarGlobalDisparity::ComputeDisparity( unsigned char *img1, unsigned char *img2, int size,  int n_color_channels, int *index)
{
	static double array[100];  //enough
	int i;
	double min = FLT_MAX;
	int minindex;
	int avg1, avg2;

	avg1 = SumOfElements(img1, size, n_color_channels)/size;
	avg2 = SumOfElements(img2, size, n_color_channels)/size;
	array[m_iNumChannels] = NormalizedCrossCorrelation(img1, avg1, img2, avg2, size, n_color_channels );
	for( i = 0; i < m_iNumChannels; i++)
	{
		array[i + 1 + m_iNumChannels] = NCC_channel(img1, avg1, img2, avg2, size, i, n_color_channels );
		array[m_iNumChannels - i - 1] = NCC_channel(img2, avg2, img1, avg1, size, i, n_color_channels );
	}
	for(i = 0; i < 2*m_iNumChannels + 1; i++)
	{
		if( array[i]<min )
		{
			min = array[i];
			minindex = i;
		}
	}
	if( minindex > m_iNumChannels )
	{
		*index = minindex - m_iNumChannels - 1;
		return( m_pDisparityLUTArray[minindex - m_iNumChannels - 1]->GetDisparity() );
	}
	if( minindex < m_iNumChannels )
	{
		*index = m_iNumChannels - minindex - 1;
		return( -1*m_pDisparityLUTArray[m_iNumChannels - minindex - 1]->GetDisparity() );
	}
	*index = 0;
	return 0;
}




double CLogPolarGlobalDisparity::ComputeChannelOutput( int channel, unsigned char *img1, unsigned char *img2, int size, int n_color_channels )
{
	int avg1, avg2;
	double retval;


//	assert(channel >= -m_iNumChannels);
//	assert(channel <= m_iNumChannels);

	avg1 = SumOfElements(img1, size, n_color_channels)/size/n_color_channels;
	avg2 = SumOfElements(img2, size, n_color_channels)/size/n_color_channels;

	if(channel == 0)
		retval = NormalizedCrossCorrelation(img1, avg1, img2, avg2, size, n_color_channels );
	else if( channel < 0 )
		retval = NCC_channel(img2, avg2, img1, avg1, size, -(channel+1), n_color_channels);
	else //channel > 0
		retval = NCC_channel(img1, avg1, img2, avg2, size, (channel-1), n_color_channels);

	return retval;
}



double CLogPolarGlobalDisparity::NormalizedCrossCorrelation( unsigned char *img1, int avg1, unsigned char *img2, int avg2, int size, int n_color_channels )
{
	int i; 
	int temp1, temp2;
	int s = 0, s1 = 0, s2 = 0;
	double temp, retval;

	// integer processing
	for( i = 0; i < size*n_color_channels; i++ )
	{
		temp1 = *img1++;
		temp2 = *img2++;
		s1 += temp1*temp1;
		s2 += temp2*temp2;
		s += temp1*temp2;
	}
	s  -= avg1*size*avg2;
	s1 -= avg1*size*avg1;
	s2 -= avg2*size*avg2;

	temp = static_cast<double>(s1)*static_cast<double>(s2);
	if(temp == 0.0) 
		return 0.0;
	retval = 1.0 - (static_cast<double>(s)/sqrt(temp));
	return retval;
}

double CLogPolarGlobalDisparity::NCC_channel(unsigned char *img1, int avg1, unsigned char *img2, int avg2, int size,  int channel, int n_color_channels )
{
	int crossp = 0, square1 = 0, square2 = 0, sum1 = 0, sum2 = 0;
	int temp1, temp2;
	int i, j;

	unsigned char *p1, *p2;
	int *dlt_pointer;
	int num_matches;

	double temp, retval;

//	ATLASSERT(m_pDisparityLUTArray);
//	ATLASSERT(channel >= 0);
//	ATLASSERT(channel < m_iNumChannels);

	dlt_pointer = m_pDisparityLUTArray[channel]->GetMatchesArray();
	num_matches = m_pDisparityLUTArray[channel]->GetNumMatches();
	p1 = img1;
	p2 = img2;

	for(i = 0; i < num_matches; i++)
	{
		 p1 += (*dlt_pointer++)*n_color_channels;
		 p2 += (*dlt_pointer++)*n_color_channels;
		 for(j = 0; j < n_color_channels; j++)
		 {
			temp1 = *(p1+j);
			temp2 = *(p2+j);
			crossp  += temp1 * temp2;
			square1 += temp1 * temp1;
			square2 += temp2 * temp2;
			sum1 += temp1;
			sum2 += temp2;
		 }
	}
	crossp = crossp - sum1*avg2 - sum2*avg1 + avg1*avg2*num_matches; 
	square1 = square1 - 2*sum1*avg1 + avg1*avg1*num_matches;
	square2 = square2 - 2*sum2*avg2 + avg2*avg2*num_matches;

	temp = static_cast<double>(square1)*static_cast<double>(square2);
	if( temp == 0.0 ) 
		return 0.0;
	retval = 1.0 - (static_cast<double>(crossp)/sqrt(temp)); 
	return retval;
}


int CLogPolarGlobalDisparity::SumOfElements( unsigned char *img, int size, int n_color_channels )
{
	int i, sum = 0;
	for( i = 0; i < size*n_color_channels; i++ )
		sum += *img++;
	return sum;
}
