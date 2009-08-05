// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef _DisparityChannel_h
#define _DisparityChannel_h

#include "CLogPolarSensor.h"

class CDisparityChannel
{
	typedef struct 	{
		int vert_coord;
		int horz_coord;
	}	img_coord_pair;

private:
	int				m_iDisparity;			//Disparity value for this map     
	int				m_iNumMatches;			//Number of pixel matches
	int				*m_pMatches;			//sparse array of interlaced increments (source/destination) for selecting matches
	img_coord_pair	*m_pSeqMap;				//auxiliary sequential array of matches	_DERIVED_		*m_pDMO;		        //the outer object

public:	
	void	SetDisparity(int setVal)	{m_iDisparity = setVal;};
	int		GetDisparity(void)		const {return m_iDisparity;};
	int		GetNumMatches(void)		const {return m_iNumMatches;};
	int*	GetMatchesArray(void)	const {return m_pMatches;};

	CDisparityChannel()
		: m_pMatches(0)
		, m_pSeqMap(0)
		, m_iDisparity(0)
	{
	};

	~CDisparityChannel()
	{	
		if(m_pMatches)
			free(m_pMatches);
		if(m_pSeqMap)
			free(m_pSeqMap);
	};
	
	bool CreateDisparityChannel( CLogPolarSensor *lps )
	{
		if(m_pSeqMap)
		{
			free(m_pSeqMap);
			m_pSeqMap = NULL;
		}
		long lines, cols;
		lines = lps->get_angles();
		cols = lps->get_eccentr();
		if( (lines == 0) || (cols == 0) ) 
			return false;

		//building sequential array 
		m_pSeqMap = (img_coord_pair *)malloc(lines*cols*sizeof(img_coord_pair));
		if(!m_pSeqMap)
			return false;

		m_iNumMatches = 0;

		int u, v, ui, vi;
		double x,y,xf;		
		double uf, vf;

        for( v = 0; v < lines; v++ )
		{
			for( u = 0; u < cols; u++ )
			{
				lps->invmap_coordinates((double)u, (double)v,&x,&y);
				xf = x + m_iDisparity;
				lps->map_coordinates(xf,y,&uf,&vf);				
				if( (uf>=0.0f) && (uf<(double)cols) && (vf>=0.0f) && (vf<(double)lines) ) 
				{
					ui = (int)uf;
					vi = (int)vf;
					m_pSeqMap[ v*cols + u ].vert_coord = vi;
					m_pSeqMap[ v*cols + u ].horz_coord = ui;
					m_iNumMatches++;					
				}
				else
				{
					m_pSeqMap[ v*cols + u ].vert_coord = -1;
					m_pSeqMap[ v*cols + u ].horz_coord = -1;
				}
			}
		}	
		
		return true;
	}

	bool CompressDisparityChannel(CLogPolarSensor *lps)
	{
		long lines, cols;
		
		lines = lps->get_angles();
		cols = lps->get_eccentr();
		
		int eta, csi, ef, cf;
		int jump_source, jump_dest, dest_index, last_dest_index, source_index, last_source_index;
		int *pointer;	

		if( (lines == 0) || (cols == 0) ) 
			return false;


		if(m_iNumMatches == 0)
			return false;

		// building sparse array
		if(m_pMatches)
		{
			free(m_pMatches);
			m_pMatches = NULL;
		}

		m_pMatches = (int*)calloc(m_iNumMatches*2L, sizeof(int));
		if(!m_pMatches)
			return false;
		
		last_dest_index=0;
		last_source_index=0;

		pointer = m_pMatches;
		for(eta = 0; eta < lines; eta++)
		{
			for(csi = 0; csi < cols; csi++)
			{
				ef = m_pSeqMap[eta*cols + csi].vert_coord;
				cf = m_pSeqMap[eta*cols + csi].horz_coord;
				if( (ef>=0) && (cf>=0) ) // valid match
				{
					dest_index = ef*cols + cf;
					source_index = eta*cols + csi;
					jump_dest = dest_index - last_dest_index;
					jump_source = source_index - last_source_index;
					last_dest_index = dest_index;
					last_source_index = source_index;
					*pointer++ =jump_source;
					*pointer++ =jump_dest;
				}
			}
		}
		return true;
	}
};

#endif
