//#include "C:\Program Files\OpenCV\cv\src\_cv.h"
//#include <_cv.h>
#include <iCub/weightedHistCalc.h>
//#include <cv.h>
#define  ICV_HIST_DUMMY_IDX  (INT_MIN/3)


static CvStatus
icvCalcHistLookupTables8u( const CvHistogram* hist, int dims, int* size, int* tab )
{
    const int lo = 0, hi = 256;
    int is_sparse = CV_IS_SPARSE_HIST( hist );
    int have_range = CV_HIST_HAS_RANGES(hist);
    int i, j;
    
    if( !have_range || CV_IS_UNIFORM_HIST(hist))
    {
        for( i = 0; i < dims; i++ )
        {
            double a = have_range ? hist->thresh[i][0] : 0;
            double b = have_range ? hist->thresh[i][1] : 256;
            int sz = size[i];
            double scale = sz/(b - a);
            int step = 1;

            if( !is_sparse )
                step = ((CvMatND*)(hist->bins))->dim[i].step/sizeof(float);

            for( j = lo; j < hi; j++ )
            {
                int idx = cvFloor((j - a)*scale);
                if( (unsigned)idx < (unsigned)sz )
                    idx *= step;
                else
                    idx = ICV_HIST_DUMMY_IDX;

                tab[i*(hi - lo) + j - lo] = idx;
            }
        }
    }
    else
    {
        for( i = 0; i < dims; i++ )
        {
            double limit = hist->thresh2[i][0];
            int idx = -1, write_idx = ICV_HIST_DUMMY_IDX, sz = size[i];
            int step = 1;

            if( !is_sparse )
                step = ((CvMatND*)(hist->bins))->dim[i].step/sizeof(float);

            if( limit > hi )
                limit = hi;
            
            j = lo;
            for(;;)
            {
                for( ; j < limit; j++ )
                    tab[i*(hi - lo) + j - lo] = write_idx;

                if( (unsigned)(++idx) < (unsigned)sz )
                {
                    limit = hist->thresh2[i][idx+1];
                    if( limit > hi )
                        limit = hi;
                    write_idx = idx*step;
                }
                else
                {
                    for( ; j < hi; j++ )
                        tab[i*(hi - lo) + j - lo] = ICV_HIST_DUMMY_IDX;
                    break;
                }
            }
        }
    }

    return CV_OK;
}

/***************************** C A L C   H I S T O G R A M *************************/

// Calculates histogram for one or more 8u arrays
static CvStatus CV_STDCALL
    icvCalcHistKernelWeighted_8u_C1R( uchar** img, int step, uchar *kernelMask, int kernelStep, uchar** binTable, int binTableStep,
						uchar* mask, int maskStep, CvSize size, CvHistogram* hist )
{
    int* tab;
    int is_sparse = CV_IS_SPARSE_HIST(hist);
    int dims, histsize[CV_MAX_DIM];
    int i, x;
    CvStatus status;

    dims = cvGetDims( hist->bins, histsize );

    tab = (int*)cvStackAlloc( dims*256*sizeof(int));
    status = icvCalcHistLookupTables8u( hist, dims, histsize, tab );

    if( status < 0 )
        return status;

    if( !is_sparse )
    {
        int total = 1;
        int* bins = ((CvMatND*)(hist->bins))->data.i;

        for( i = 0; i < dims; i++ )
            total *= histsize[i];

        if( dims <= 3 && total >= -ICV_HIST_DUMMY_IDX )
            return CV_BADSIZE_ERR; // too big histogram

        switch( dims )
        {
        case 1:
            {
            int tab1d[256];
            memset( tab1d, 0, sizeof(tab1d));

            for( ; size.height--; img[0] += step )
            {
                uchar* ptr = img[0];
				//float* kerPtr = kernelMask[0];
                if( !mask )
                {
                    for( x = 0; x <= size.width - 4; x += 4 )
                    {
                        int v0 = ptr[x];
                        int v1 = ptr[x+1];
						
						int m0 = kernelMask[x];
						int m1 = kernelMask[x+1];
						
						if (kernelMask!=0){
							tab1d[v0]+=m0;//tab1d[v0]++;
							tab1d[v1]+=m1;//tab1d[v1]++;
						}
						else{
							tab1d[v0]++;
							tab1d[v1]++;
						}

                        v0 = ptr[x+2];
                        v1 = ptr[x+3];

						m0 = kernelMask[x+2];
						m1 = kernelMask[x+3];

						if (kernelMask!=0){
							tab1d[v0]+=m0;//tab1d[v0]++;
							tab1d[v1]+=m1;//tab1d[v1]++;
						}
						else{
							tab1d[v0]++;
							tab1d[v1]++;
						}
                    }

					for( ; x < size.width; x++ ){
						if (kernelMask!=0)
							tab1d[ptr[x]]+=kernelMask[x];//tab1d[ptr[x]]++;
						else
							tab1d[ptr[x]]++;
					}
                }
                else
                {
                    for( x = 0; x < size.width; x++ )
						if( mask[x] ){
							if (kernelMask!=0)
								tab1d[ptr[x]]+=kernelMask[x];//tab1d[ptr[x]]++;
							else
								tab1d[ptr[x]]++;
						}
                    mask += maskStep;
					kernelMask += kernelStep;
                }
            }

            for( i = 0; i < 256; i++ )
            {
                int idx = tab[i];
                if( idx >= 0 )
                    bins[idx] += tab1d[i];
            }
            }
            break;
        case 2:
            for( ; size.height--; img[0] += step, img[1] += step )
            {
                uchar* ptr0 = img[0];
                uchar* ptr1 = img[1];
                if( !mask )
                {
                    for( x = 0; x < size.width; x++ )
                    {
                        int v0 = ptr0[x];
                        int v1 = ptr1[x];
                        int idx = tab[v0] + tab[256+v1];

						if( idx >= 0 ){
							if (kernelMask!=0)
								bins[idx]+=kernelMask[x];//bins[idx]++;
							else
								bins[idx]++;
						}
                    }
                }
                else
                {
                    for( x = 0; x < size.width; x++ )
                    {
                        if( mask[x] )
                        {
                            int v0 = ptr0[x];
                            int v1 = ptr1[x];

                            int idx = tab[v0] + tab[256+v1];

							if( idx >= 0 ){
								if (kernelMask!=0)
									bins[idx]+=kernelMask[x];//bins[idx]++;
								else
									bins[idx]++;
							}
                        }
                    }
                    mask += maskStep;
					kernelMask += kernelStep;
                }
            }
            break;
        case 3:
            for( ; size.height--; img[0] += step, img[1] += step, img[2] += step, kernelMask += kernelStep )
            {
                uchar* ptr0 = img[0];
                uchar* ptr1 = img[1];
                uchar* ptr2 = img[2];
                if( !mask )
                {
                    for( x = 0; x < size.width; x++ )
                    {
                        int v0 = ptr0[x];
                        int v1 = ptr1[x];
                        int v2 = ptr2[x];
                        int idx = tab[v0] + tab[256+v1] + tab[512+v2];

						if( idx >= 0 ){
							if (kernelMask!=0)
								bins[idx]+=kernelMask[x];//bins[idx]++;
							else
								bins[idx]++;
						}
                    }
                }
                else
                {
                    for( x = 0; x < size.width; x++ )
                    {
                        if( mask[x] )
                        {
                            int v0 = ptr0[x];
                            int v1 = ptr1[x];
                            int v2 = ptr2[x];
                            int idx = tab[v0] + tab[256+v1] + tab[512+v2];

							if( idx >= 0 ){
								if (kernelMask!=0)
									bins[idx]+=kernelMask[x];//bins[idx]++;
								else
									bins[idx]++;
							}
                        }
                    }
                    mask += maskStep;
					//kernelMask += kernelStep;
                }
            }
            break;
        default:
            for( ; size.height--; )
            {
                if( !mask )
                {
                    for( x = 0; x < size.width; x++ )
                    {
                        int* binptr = bins;
                        for( i = 0; i < dims; i++ )
                        {
                            int idx = tab[i*256 + img[i][x]];
                            if( idx < 0 )
                                break;
                            binptr += idx;
                        }
                        if( i == dims )
                            binptr[0]++;
                    }
                }
                else
                {
                    for( x = 0; x < size.width; x++ )
                    {
                        if( mask[x] )
                        {
                            int* binptr = bins;
                            for( i = 0; i < dims; i++ )
                            {
                                int idx = tab[i*256 + img[i][x]];
                                if( idx < 0 )
                                    break;
                                binptr += idx;
                            }
                            if( i == dims )
                                binptr[0]++;
                        }
                    }
                    mask += maskStep;
                }

                for( i = 0; i < dims; i++ )
                    img[i] += step;
            }
        }
    }
    else
    {
        CvSparseMat* mat = (CvSparseMat*)(hist->bins);
        int node_idx[CV_MAX_DIM];

        for( ; size.height--; )
        {
            if( !mask )
            {
                for( x = 0; x < size.width; x++ )
                {
                    for( i = 0; i < dims; i++ )
                    {
                        int idx = tab[i*256 + img[i][x]];
                        if( idx < 0 )
                            break;
                        node_idx[i] = idx;
                    }
                    if( i == dims )
                    {
                        int* bin = (int*)cvPtrND( mat, node_idx, 0, 1 );
                        bin[0]++;
                    }
                }
            }
            else
            {
                for( x = 0; x < size.width; x++ )
                {
                    if( mask[x] )
                    {
                        for( i = 0; i < dims; i++ )
                        {
                            int idx = tab[i*256 + img[i][x]];
                            if( idx < 0 )
                                break;
                            node_idx[i] = idx;
                        }
                        if( i == dims )
                        {
                            int* bin = (int*)cvPtrND( mat, node_idx, 0, 1, 0 );
                            bin[0]++;
                        }
                    }
                }
                mask += maskStep;
            }

            for( i = 0; i < dims; i++ )
                img[i] += step;
        }
    }

    return CV_OK;
}


// Calculates histogram for one or more 32f arrays
static CvStatus CV_STDCALL
    icvCalcHistKernelWeighted_32f_C1R( float** img, int step, float *kernelMask, int kernelStep, uchar** binTablePtr, int binTableStep,uchar* mask, int maskStep,
                         CvSize size, CvHistogram* hist )
{
    int is_sparse = CV_IS_SPARSE_HIST(hist);
    int uniform = CV_IS_UNIFORM_HIST(hist);
    int dims, histsize[CV_MAX_DIM];
    double uni_range[CV_MAX_DIM][2];
    int i, x;
	

    dims = cvGetDims( hist->bins, histsize );
    step /= sizeof(img[0][0]);

    if( uniform )
    {
        for( i = 0; i < dims; i++ )
        {
            double t = histsize[i]/((double)hist->thresh[i][1] - hist->thresh[i][0]);
            uni_range[i][0] = t;
            uni_range[i][1] = -t*hist->thresh[i][0];
        }
    }

    if( !is_sparse )
    {
        CvMatND* mat = (CvMatND*)(hist->bins);
        int* bins = mat->data.i;

        if( uniform )
        {
            switch( dims )
            {
            case 1:
                {
                double a = uni_range[0][0], b = uni_range[0][1];
                int sz = histsize[0];

                for( ; size.height--; img[0] += step , kernelMask+=kernelStep, binTablePtr[0] += binTableStep )
                {
                    float* ptr = img[0];
					uchar* binPtr = binTablePtr[0];
                    if( !mask )
                    {
                        for( x = 0; x <= size.width - 4; x += 4 )
                        {
                            int v0 = cvFloor(ptr[x]*a + b);
                            int v1 = cvFloor(ptr[x+1]*a + b);

							if( (unsigned)v0 < (unsigned)sz ){
								if (kernelMask!=0){
									bins[v0]+=(int)kernelMask[x];//bins[v0]++;
									if (binTablePtr!=0)
										binPtr[x]=v0;
								}
								else
									bins[v0]++;
							}
							if( (unsigned)v1 < (unsigned)sz ){
								if (kernelMask!=0){
									bins[v1]+=(int)kernelMask[x+1];//bins[v1]++;
									if (binTablePtr!=0)
										binPtr[x+1]=v1;
								}
								else
									bins[v1]++;
							}

                            v0 = cvFloor(ptr[x+2]*a + b);
                            v1 = cvFloor(ptr[x+3]*a + b);

							if( (unsigned)v0 < (unsigned)sz ){
								if (kernelMask!=0){
									bins[v0]+=(int)kernelMask[x+2];//bins[v0]++;
									if (binTablePtr!=0)
										binPtr[x+2]=v0;
								}
								else
									bins[v0]++;
							}
							if( (unsigned)v1 < (unsigned)sz ){
								if (kernelMask!=0){
									bins[v1]+=(int)kernelMask[x+3];//bins[v1]++;
									if (binTablePtr!=0)
										binPtr[x+3]=v1;
								}
								else
									bins[v1]++;
							}
                        }

                        for( ; x < size.width; x++ )
                        {
                            int v0 = cvFloor(ptr[x]*a + b);
							if( (unsigned)v0 < (unsigned)sz ){
								if (kernelMask!=0){
									bins[v0]+=(int)kernelMask[x];//bins[v0]++;
									if (binTablePtr!=0)
										binPtr[x]=v0;
								}
								else
									bins[v0]++;
							}
                        }
                    }
                    else
                    {
                        for( x = 0; x < size.width; x++ )
                            if( mask[x] )
                            {
                                int v0 = cvFloor(ptr[x]*a + b);
								if( (unsigned)v0 < (unsigned)sz ){
									if (kernelMask!=0){
										bins[v0]+=(int)kernelMask[x];
										//bins[v0]++;
										if (binTablePtr!=0)
											binPtr[x]=v0;
									}
									else
										bins[v0]++;
								}
                            }
                        mask += maskStep;
						//kernelMask += kernelStep;
						//binTablePtr[0] += binTableStep;
                    }
                }
                }
                break;
            case 2:
                {
                double  a0 = uni_range[0][0], b0 = uni_range[0][1];
                double  a1 = uni_range[1][0], b1 = uni_range[1][1];
                int sz0 = histsize[0], sz1 = histsize[1];
                int step0 = ((CvMatND*)(hist->bins))->dim[0].step/sizeof(float);
				
                for( ; size.height--; img[0] += step, img[1] += step, binTablePtr[0]+= binTableStep 
								,binTablePtr[1]+= binTableStep, kernelMask += kernelStep/4)
                {
                    float* ptr0 = img[0];
                    float* ptr1 = img[1];

					uchar* binPtr0 = binTablePtr[0];
					uchar* binPtr1 = binTablePtr[1];

                    if( !mask )
                    {
                        for( x = 0; x < size.width; x++ )
                        {
                            int v0 = cvFloor( ptr0[x]*a0 + b0 );
                            int v1 = cvFloor( ptr1[x]*a1 + b1 );

							/*if (kernelMask!=0 && kernelMask[x]<0)
								printf("1111\n");*/

                            if( (unsigned)v0 < (unsigned)sz0 &&
								(unsigned)v1 < (unsigned)sz1 ){
									if (kernelMask!=0){
										bins[v0*step0 + v1]+=(int)floor(kernelMask[x]*100000);
										//bins[v0*step0 + v1]++;
										if (binTablePtr!=0){
											//binTable[x]=v0*step0 + v1;
											binPtr0[x]=v0;
											binPtr1[x]=v1;
										}
									}
									else
										bins[v0*step0 + v1]++;
							}
                        }
                    }
                    else
                    {
                        for( x = 0; x < size.width; x++ )
                        {
                            if( mask[x] )
                            {
                                int v0 = cvFloor( ptr0[x]*a0 + b0 );
                                int v1 = cvFloor( ptr1[x]*a1 + b1 );

                                if( (unsigned)v0 < (unsigned)sz0 &&
									(unsigned)v1 < (unsigned)sz1 ){
										if (kernelMask!=0){
											bins[v0*step0 + v1]+=(int)floor(kernelMask[x]*10000);//kernelMask[x];//bins[v0*step0 + v1]++;
											if (binTablePtr!=0){
												//binTable[x]=v0*step0 + v1;
												binPtr0[x]=v0;
												binPtr1[x]=v1;
											}
										}
										else
											bins[v0*step0 + v1]++;
								}
                            }
                        }
                        mask += maskStep;
						//kernelMask += kernelStep;
						//binTable += binTableStep;
						//binTablePtr[0]+= binTableStep;
						//binTablePtr[1]+= binTableStep;
                    }
                }
                }
                break;
            default:
                for( ; size.height--; )
                {
                    if( !mask )
                    {
                        for( x = 0; x < size.width; x++ )
                        {
                            int* binptr = bins;
                            for( i = 0; i < dims; i++ )
                            {
                                int idx = cvFloor((double)img[i][x]*uni_range[i][0]
                                                 + uni_range[i][1]);
                                if( (unsigned)idx >= (unsigned)histsize[i] )
                                    break;
                                binptr += idx*(mat->dim[i].step/sizeof(float));
                            }
                            if( i == dims )
                                binptr[0]++;
                        }
                    }
                    else
                    {
                        for( x = 0; x < size.width; x++ )
                        {
                            if( mask[x] )
                            {
                                int* binptr = bins;
                                for( i = 0; i < dims; i++ )
                                {
                                    int idx = cvFloor((double)img[i][x]*uni_range[i][0]
                                                     + uni_range[i][1]);
                                    if( (unsigned)idx >= (unsigned)histsize[i] )
                                        break;
                                    binptr += idx*(mat->dim[i].step/sizeof(float));
                                }
                                if( i == dims )
                                    binptr[0]++;
                            }
                        }
                        mask += maskStep;
                    }

                    for( i = 0; i < dims; i++ )
                        img[i] += step;
                }
            }
        }
        else
        {
            for( ; size.height--; )
            {
                for( x = 0; x < size.width; x++ )
                {
                    if( !mask || mask[x] )
                    {
                        int* binptr = bins;
                        for( i = 0; i < dims; i++ )
                        {
                            float v = img[i][x];
                            float* thresh = hist->thresh2[i];
                            int idx = -1, sz = histsize[i];

                            while( v >= thresh[idx+1] && ++idx < sz )
                                /* nop */;

                            if( (unsigned)idx >= (unsigned)sz )
                                break;

                            binptr += idx*(mat->dim[i].step/sizeof(float));
                        }
                        if( i == dims )
                            binptr[0]++;
                    }
                }

                for( i = 0; i < dims; i++ )
                    img[i] += step;
                if( mask )
                    mask += maskStep;
            }
        }
    }
    else
    {
        CvSparseMat* mat = (CvSparseMat*)(hist->bins);
        int node_idx[CV_MAX_DIM];

        for( ; size.height--; )
        {
            if( uniform )
            {
                for( x = 0; x < size.width; x++ )
                {
                    if( !mask || mask[x] )
                    {
                        for( i = 0; i < dims; i++ )
                        {
                            int idx = cvFloor(img[i][x]*uni_range[i][0]
                                             + uni_range[i][1]);
                            if( (unsigned)idx >= (unsigned)histsize[i] )
                                break;
                            node_idx[i] = idx;
                        }
                        if( i == dims )
                        {
                            int* bin = (int*)cvPtrND( mat, node_idx, 0, 1, 0 );
                            bin[0]++;
                        }
                    }
                }
            }
            else
            {
                for( x = 0; x < size.width; x++ )
                {
                    if( !mask || mask[x] )
                    {
                        for( i = 0; i < dims; i++ )
                        {
                            float v = img[i][x];
                            float* thresh = hist->thresh2[i];
                            int idx = -1, sz = histsize[i];

                            while( v >= thresh[idx+1] && ++idx < sz )
                                /* nop */;

                            if( (unsigned)idx >= (unsigned)sz )
                                break;

                            node_idx[i] = idx;
                        }
                        if( i == dims )
                        {
                            int* bin = (int*)cvPtrND( mat, node_idx, 0, 1, 0 );
                            bin[0]++;
                        }
                    }
                }
            }

            for( i = 0; i < dims; i++ )
                img[i] += step;

            if( mask )
                mask += maskStep;
        }
    }

    return CV_OK;
}


CV_IMPL void
cvCalcArrHistKernelWeighted( CvArr** img, CvHistogram* hist, const CvArr *kernelMask, CvArr ** binTable,
               int do_not_clear, const CvArr* mask )
{
    CV_FUNCNAME( "cvCalcHistKernelWeighted" );

    __BEGIN__;

    uchar* ptr[CV_MAX_DIM];
    uchar* maskptr = 0;
	uchar* kernelPtr = 0;
	uchar* binTablePtr[CV_MAX_DIM];
    int maskstep = 0, step = 0;
	int kernelstep = 0;
	int binTableStep = 0;
    int i, dims;
    int cont_flag = -1;
    CvMat stub0, *mat0 = 0;
	CvMat stub3;
	CvMat *binT0=0;
    CvMatND dense;
    CvSize size;

    if( !CV_IS_HIST(hist))
        CV_ERROR( CV_StsBadArg, "Bad histogram pointer" );

    if( !img )
        CV_ERROR( CV_StsNullPtr, "Null double array pointer" );

    CV_CALL( dims = cvGetDims( hist->bins ));
    
    for( i = 0; i < dims; i++ )
    {
        CvMat stub, *mat = (CvMat*)img[i];
		CvMat *mat2;
		CvMat stub4;
        CV_CALL( mat = cvGetMat( mat, i == 0 ? &stub0 : &stub, 0, 1 ));
		if (binTable!=0){
		mat2 = (CvMat*)binTable[i];
		CV_CALL( mat2 = cvGetMat( mat2, i == 0 ? &stub3 : &stub4, 0, 1 ));
		}

        if( CV_MAT_CN( mat->type ) != 1 )
            CV_ERROR( CV_BadNumChannels, "Only 1-channel arrays are allowed here" );

        if( i == 0 )
        {
            mat0 = mat;
            step = mat0->step;
			if (binTable!=0)
			binT0 = mat2;
        }
        else
        {
            if( !CV_ARE_SIZES_EQ( mat0, mat ))
                CV_ERROR( CV_StsUnmatchedSizes, "Not all the planes have equal sizes" );

            if( mat0->step != mat->step )
                CV_ERROR( CV_StsUnmatchedSizes, "Not all the planes have equal steps" );

            if( !CV_ARE_TYPES_EQ( mat0, mat ))
                CV_ERROR( CV_StsUnmatchedFormats, "Not all the planes have equal types" );
			if (binTable!=0){
			if( !CV_ARE_SIZES_EQ( binT0, mat2 ))
				CV_ERROR( CV_StsUnmatchedSizes,"LUT size does not match to other arrays\' size" );
			}
        }

        cont_flag &= mat->type;
        ptr[i] = mat->data.ptr;
		if (binTable!=0){
		binTablePtr[i] = mat2->data.ptr;
		binTableStep = mat2->step;
		}
    }
	/*
	if (binTable){
		CvMat *mat2 = (CvMat*)binTable;
		if( !CV_ARE_SIZES_EQ( mat0, mat2 ))
            CV_ERROR( CV_StsUnmatchedSizes,
                "Mask size does not match to other arrays\' size" );
		binTablePtr = mat2->data.ptr;
		binTableStep = mat2->step;
	}*/


    if( mask )
    {
        CvMat stub, *mat = (CvMat*)mask;
        CV_CALL( mat = cvGetMat( mat, &stub, 0, 1 ));

        if( !CV_IS_MASK_ARR(mat))
            CV_ERROR( CV_StsBadMask, "Bad mask array" );

        if( !CV_ARE_SIZES_EQ( mat0, mat ))
            CV_ERROR( CV_StsUnmatchedSizes,
                "Mask size does not match to other arrays\' size" );
        maskptr = mat->data.ptr;
        maskstep = mat->step;
        cont_flag &= mat->type;
    }
	
	if( kernelMask )
    {
        CvMat stub, *mat1 = (CvMat*)kernelMask;
        CV_CALL( mat1 = cvGetMat( mat1, &stub, 0, 1 ));

        /*if( !CV_IS_MASK_ARR(mat))
            CV_ERROR( CV_StsBadMask, "Bad mask array" );*/

        if( !CV_ARE_SIZES_EQ( mat0, mat1 ))
            CV_ERROR( CV_StsUnmatchedSizes,
                "Mask size does not match to other arrays\' size" );
        kernelPtr = mat1->data.ptr;
        kernelstep = mat1->step;
        //cont_flag &= mat->type;
    }
	
    size = cvGetMatSize(mat0);
    if( CV_IS_MAT_CONT( cont_flag ))
    {
        size.width *= size.height;
        size.height = 1;
        maskstep = step = CV_STUB_STEP;
    }

    if( !CV_IS_SPARSE_HIST(hist))
    {
        dense = *(CvMatND*)hist->bins;
        dense.type = (dense.type & ~CV_MAT_TYPE_MASK) | CV_32SC1;
    }

    if( !do_not_clear )
    {
        CV_CALL( cvZero( hist->bins ));
    }
    else if( !CV_IS_SPARSE_HIST(hist))
    {
        CV_CALL( cvConvert( (CvMatND*)hist->bins, &dense ));
    }
    else
    {
        CvSparseMat* mat = (CvSparseMat*)(hist->bins);
        CvSparseMatIterator iterator;
        CvSparseNode* node;

        for( node = cvInitSparseMatIterator( mat, &iterator );
             node != 0; node = cvGetNextSparseNode( &iterator ))
        {
            Cv32suf* val = (Cv32suf*)CV_NODE_VAL( mat, node );
            val->i = cvRound( val->f );
        }
    }

    if( CV_MAT_DEPTH(mat0->type) > CV_8S && !CV_HIST_HAS_RANGES(hist))
        CV_ERROR( CV_StsBadArg, "histogram ranges must be set (via cvSetHistBinRanges) "
                                "before calling the function" );

    switch( CV_MAT_DEPTH(mat0->type) )
    {
    case CV_8U:
        IPPI_CALL( icvCalcHistKernelWeighted_8u_C1R( ptr, step, kernelPtr, kernelstep, binTablePtr, binTableStep, maskptr, maskstep, size, hist ));
	    break;
    case CV_32F:
        {
        union { uchar** ptr; float** fl; } v;
        v.ptr = ptr;
		union { uchar* kernelPtr; float* fl; } v1;
        v1.kernelPtr = kernelPtr;
	    IPPI_CALL( icvCalcHistKernelWeighted_32f_C1R( v.fl, step, v1.fl, kernelstep, binTablePtr, binTableStep, maskptr, maskstep, size, hist ));
        }
	    break;
    default:
        CV_ERROR( CV_StsUnsupportedFormat, "Unsupported array type" );
    }

    if( !CV_IS_SPARSE_HIST(hist))
    {
        CV_CALL( cvConvert( &dense, (CvMatND*)hist->bins ));
    }
    else
    {
        CvSparseMat* mat = (CvSparseMat*)(hist->bins);
        CvSparseMatIterator iterator;
        CvSparseNode* node;

        for( node = cvInitSparseMatIterator( mat, &iterator );
             node != 0; node = cvGetNextSparseNode( &iterator ))
        {
            Cv32suf* val = (Cv32suf*)CV_NODE_VAL( mat, node );
            val->f = (float)val->i;
        }
    }
    
    __END__;
}

