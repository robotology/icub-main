// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/math/Math.h>
#include <iCub/SalienceOperator.h>

//TODO:all the code related to the kinematic gaze of the robot head
SalienceOperator::SalienceOperator(const int width1, const int height1)//:_gaze( YMatrix(_dh_nrf, 5, DH_left[0]), YMatrix(_dh_nrf, 5, DH_right[0]), YMatrix(4, 4, TBaseline[0]) )
{
	resize(width1, height1);
	integralRG=new YARPIntegralImage(320,240);
	integralGR=new YARPIntegralImage(320,240);
	integralBY=new YARPIntegralImage(320,240);

	foveaBlob=new ImageOf<PixelMono>;
	foveaBlob->resize(320,240);
	colorVQ_img=new ImageOf<PixelBgr>;
	colorVQ_img->resize(320,240);

	colorVQ=new ColorVQ(320,240,10);
}

void SalienceOperator::DrawVQColor(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged)
{
	for (int r=0; r<height; r++)
		for (int c=0; c<width; c++)
			colorVQ->DominantQuantization(m_boxes[tagged(c, r)].meanColors, id(c,r), 0.3*255);
}

/*Defines Valid and not valid boxes

*/
void SalienceOperator::ComputeSalienceAll(int num_blob, int last_tag){	
	int i=1;
	for (; i <= last_tag; i++) {
	//__OLD//while (i<=last_tag && num_blob>0) {
		if (m_boxes[i].areaLP) {

			m_boxes[i].areaCart=TotalArea(m_boxes[i]);
			m_boxes[i].centroid_y = (double)m_boxes[i].ysum / m_boxes[i].areaLP;
			m_boxes[i].centroid_x = (double)m_boxes[i].xsum / m_boxes[i].areaLP;

			m_boxes[i].meanRG = m_boxes[i].rgSum / m_boxes[i].areaLP;
			m_boxes[i].meanGR = m_boxes[i].grSum / m_boxes[i].areaLP;
			m_boxes[i].meanBY = m_boxes[i].bySum / m_boxes[i].areaLP;
			
			//__OLD//m_boxes[box_num].id = max_tag;
		} else
			m_boxes[i].valid=false;
		//__OLD//i++;
	}
}

/*function that returns the PixelBgr mean of the selectioned blob
	inputs: the tagged image, the R+G- image, the G+R- image, the B+Y- image, the index of the blob
*/
PixelBgr SalienceOperator::varBlob(ImageOf<PixelInt>& tagged, ImageOf<PixelMono> &rg, ImageOf<PixelMono> &gr, ImageOf<PixelMono> &by, int tag)
{
	int tmpr, tmpg, tmpb;
	PixelBgr tmp;

	tmpr=0;
	tmpg=0;
	tmpb=0;

	for (int r=m_boxes[tag].rmin; r<=m_boxes[tag].rmax; r++)
		for (int c=m_boxes[tag].cmin; c<=m_boxes[tag].cmax; c++)
			if (tagged(c, r)==tag) {
				tmpr+=rg(c, r)*rg(c, r);
				tmpg+=gr(c, r)*gr(c, r);
				tmpb+=by(c, r)*by(c, r);
			}
	
	tmpr = tmpr / m_boxes[tag].areaLP;
	tmpg = tmpg / m_boxes[tag].areaLP;
	tmpb = tmpb / m_boxes[tag].areaLP;

	tmp.r = sqrt((double)tmpr - m_boxes[tag].meanRG*m_boxes[tag].meanRG);
	tmp.g = sqrt((double)tmpg - m_boxes[tag].meanGR*m_boxes[tag].meanGR);
	tmp.b = sqrt((double)tmpb - m_boxes[tag].meanBY*m_boxes[tag].meanBY);

	if (tmp.r==0) tmp.r=1;
	if (tmp.g==0) tmp.g=1;
	if (tmp.b==0) tmp.b=1;
	
	return tmp;
}

int SalienceOperator::countSmallBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int min_size)
{
	int num=0;
	
	for (int i=2; i<=max_tag; i++)
		if (blobList[i] && m_boxes[i].valid && m_boxes[i].areaCart < min_size)
			num++;
		else
			blobList[i]=0;

	return num;
}

void SalienceOperator::mergeBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int numBlob)
{
	for (int r=0; r<height; r++)
		for (int c=0; c<width; c++)
			if (blobList[tagged(c, r)]==1) {
				tagged(c, r)=numBlob;
				m_boxes[tagged(c, r)].valid=false;
			}
}

void SalienceOperator::RemoveNonValidNoRange(int last_tag, const int max_size, const int min_size)
{
	for (int i = 1; i <= last_tag;i++) {
		if (m_boxes[i].valid) {
			//__OLD//int area = TotalArea(m_boxes[i]);
			int area = m_boxes[i].areaCart;
			// eliminare i blob troppo piccoli nn è molto utile se
			// i blob vengono comunque ordinati x grandezza.
			if ( area < min_size ||	area > max_size ) {
				m_boxes[i].valid=false;
			}
			else
				printf("a valid Blob with valid Dimension \n");
		}
	}
}


void SalienceOperator::ComputeMeanColors(int last_tag)
{	
	// create the subset of attentional boxes.
	for (int i = 1; i <= last_tag; i++)
		if (m_boxes[i].areaLP!=0) {
			// note that I should cast to double and add 0.5 to the result to round
			// but the presence of noise in the image has the same effect
			// and in this way it is faster!
			m_boxes[i].meanColors.r = (double)m_boxes[i].rSum / m_boxes[i].areaLP + 0.5;
			m_boxes[i].meanColors.g = (double)m_boxes[i].gSum / m_boxes[i].areaLP + 0.5;
			m_boxes[i].meanColors.b = (double)m_boxes[i].bSum / m_boxes[i].areaLP + 0.5;
		}
}

void SalienceOperator::DrawMeanColorsLP(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged)
{
	/*__OLD//if (last_tag<numBlob || numBlob==-1) numBlob=last_tag;
	
	for (int i = 0; i < numBlob; i++) {
	//for (int i = 0; i < MaxBoxes; i++) {
		for (int r=m_boxes[i].rmin; r<=m_boxes[i].rmax; r++)
			for (int c=m_boxes[i].cmin; c<=m_boxes[i].cmax; c++)
				if (tagged(c, r)==m_boxes[i].id) {
					id(c ,r)=m_boxes[i].meanColors;
				}
	}

	return numBlob;*/

	for (int r=0; r<height; r++)
		for (int c=0; c<width; c++)
			id(c,r)=m_boxes[tagged(c,r)].meanColors;
}



int SalienceOperator::DrawContrastLP2(ImageOf<PixelMonoSigned>& rg, ImageOf<PixelMonoSigned>& gr,
								  ImageOf<PixelMonoSigned>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
								  int numBlob, float pBU, float pTD,
								  PixelMonoSigned prg, PixelMonoSigned pgr, PixelMonoSigned pby, PixelMono maxDest)
{
	int salienceBU, salienceTD;

	double a1,b1,a2,b2,a3,b3;

	int minSalienceBU=INT_MAX;
	int maxSalienceBU=INT_MIN;

	int minSalienceTD=INT_MAX;
	int maxSalienceTD=INT_MIN;

	int minSalienceTot=INT_MAX;
	int maxSalienceTot=INT_MIN;

	dst.zero();
	
	if (numBlob>imageSize) numBlob=imageSize;

	integralRG->computeCartesian(rg);
	integralGR->computeCartesian(gr);
	integralBY->computeCartesian(by);

	for (int i = 1; i < numBlob; i++) {
		// The salience should not change if a blob is eliminated because it is too small
		// or too big
		// I could not change the scale of the salience and change only the color of the
		// most salient blob
		if (m_boxes[i].valid) {
			int tmp;
			
			//__OLD//int rdim=(double)(m_boxes[i].rmax-m_boxes[i].rmin+1)*.75;
			//__OLD//int cdim=(double)(m_boxes[i].cmax-m_boxes[i].cmin+1)*.75;
			int rdim=(double)(m_boxes[i].rmax-m_boxes[i].rmin+1)*1;
			int cdim=(double)(m_boxes[i].cmax-m_boxes[i].cmin+1)*1;


			int a,b,c,d;
			c = m_boxes[i].rmax+rdim;
			d = m_boxes[i].rmin-rdim;
			if (cdim<width/3) {
				a = m_boxes[i].cmin+cdim;
				b = m_boxes[i].cmin-cdim;
			} else {
				a = 0;
				b = 251;
			}

			/*__OLD//tmp=255*height*width*integralRG.getSaliencyLp(m_boxes[i].cmax+cdim,
				m_boxes[i].cmin-cdim,
				m_boxes[i].rmax+rdim,
				m_boxes[i].rmin-rdim)/(rdim*cdim);
			m_boxes[i].cRG=abs(tmp-m_boxes[i].meanRG);
			tmp=255*height*width*integralGR.getSaliencyLp(m_boxes[i].cmax+cdim,
				m_boxes[i].cmin-cdim,
				m_boxes[i].rmax+rdim,
				m_boxes[i].rmin-rdim)/(rdim*cdim);
			m_boxes[i].cGR=abs(tmp-m_boxes[i].meanGR);
			tmp=255*height*width*integralBY.getSaliencyLp(m_boxes[i].cmax+cdim,
				m_boxes[i].cmin-cdim,
				m_boxes[i].rmax+rdim,
				m_boxes[i].rmin-rdim)/(rdim*cdim);
			m_boxes[i].cBY=abs(tmp-m_boxes[i].meanBY);*/

			tmp=255*height*width*integralRG->getMeanLp(a,b,c,d);
			m_boxes[i].cRG=abs(tmp-m_boxes[i].meanRG);

			tmp=255*height*width*integralGR->getMeanLp(a,b,c,d);
			m_boxes[i].cGR=abs(tmp-m_boxes[i].meanGR);
			
			tmp=255*height*width*integralBY->getMeanLp(a,b,c,d);
			m_boxes[i].cBY=abs(tmp-m_boxes[i].meanBY);

			/*__OLD//salienceBU=sqrt(m_boxes[i].cRG*m_boxes[i].cRG+
			                m_boxes[i].cGR*m_boxes[i].cGR+
			                m_boxes[i].cBY*m_boxes[i].cBY);
			salienceBU=salienceBU/sqrt(3);*/

			/*__OLD//salienceBU=m_boxes[i].cRG;
			if (salienceBU<m_boxes[i].cGR)
				salienceBU=m_boxes[i].cGR;
			if (salienceBU<m_boxes[i].cBY)
				salienceBU=m_boxes[i].cBY;*/

			//__OLD//sum of abs of contrast differences
			salienceBU=m_boxes[i].cRG+m_boxes[i].cGR+m_boxes[i].cBY;


			salienceTD=sqrt((double)(m_boxes[i].meanRG-prg)*(m_boxes[i].meanRG-prg)+
			                (m_boxes[i].meanGR-pgr)*(m_boxes[i].meanGR-pgr)+
			                (m_boxes[i].meanBY-pby)*(m_boxes[i].meanBY-pby));
			salienceTD=255-salienceTD/sqrt((double)3);
			
			/*__OLD//salienceTD=abs(m_boxes[i].meanRG-prg);
							
			if (salienceTD<abs(m_boxes[i].meanGR-pgr))
				salienceTD=abs(m_boxes[i].meanGR-pgr);
				
			if (salienceTD<abs(m_boxes[i].meanBY-pby))
				salienceTD=abs(m_boxes[i].meanBY-pby);

			salienceTD=255-salienceTD;*/

			//__OLD//if the color is too different it isn't in the scene!
			if (salienceTD<200) salienceTD=200;

			//__OLD//if (salienceTD<0) salienceTD=0;
			
			m_boxes[i].salienceBU=salienceBU;
			m_boxes[i].salienceTD=salienceTD;

			if (salienceBU<minSalienceBU)
				minSalienceBU=salienceBU;
			else if (salienceBU>maxSalienceBU)
				maxSalienceBU=salienceBU;

			if (salienceTD<minSalienceTD)
				minSalienceTD=salienceTD;
			else if (salienceTD>maxSalienceTD)
				maxSalienceTD=salienceTD;
		}
	}

	if (maxSalienceBU!=minSalienceBU) {
		//__OLD//a1=255.*(maxDest-1)/(maxSalienceBU-minSalienceBU);
		a1=254./(maxSalienceBU-minSalienceBU);
		b1=1.-a1*minSalienceBU;
	} else {
		a1=0;
		b1=0;
	}

	if (maxSalienceTD!=minSalienceTD) {
		//__OLD//a2=255.*(maxDest-1)/(maxSalienceTD-minSalienceTD);
		a2=254./(maxSalienceTD-minSalienceTD);
		b2=1.-a2*minSalienceTD;
	} else {
		a2=0;
		b2=0;
	}

	for (int i = 1; i < numBlob; i++) {
		if (m_boxes[i].valid) {
			m_boxes[i].salienceTotal=pBU*(a1*m_boxes[i].salienceBU+b1)+pTD*(a2*m_boxes[i].salienceTD+b2);

			if (m_boxes[i].salienceTotal<minSalienceTot)
				minSalienceTot=m_boxes[i].salienceTotal;
			else if (m_boxes[i].salienceTotal>maxSalienceTot)
				maxSalienceTot=m_boxes[i].salienceTotal;
		}
	}

	if (maxSalienceTot!=minSalienceTot) {
		a3=((double)(maxDest-1))/(maxSalienceTot-minSalienceTot);
		b3=1.-a3*minSalienceTot;
	} else {
		a3=0;
		b3=0;
	}

	for (int i = 1; i < numBlob; i++) {
		if (m_boxes[i].valid) {
			//__OLD//if ((m_boxes[i].salienceBU==maxSalienceBU && pBU==1) || (m_boxes[i].salienceTD==maxSalienceTD && pTD==1))
			if (m_boxes[i].salienceTotal==maxSalienceTot)
				m_boxes[i].salienceTotal=255;
			else
				m_boxes[i].salienceTotal=a3*m_boxes[i].salienceTotal+b3;

			for (int r=m_boxes[i].rmin; r<=m_boxes[i].rmax; r++)
				for (int c=m_boxes[i].cmin; c<=m_boxes[i].cmax; c++){
					int tag=tagged(c,r);
					if (tag==m_boxes[i].id) {
						//__OLD//if (sal>th) dst(c ,r)=sal;
						//__OLD//else dst(c ,r)=0;
						dst(c ,r)=(PixelMono)m_boxes[i].salienceTotal;
					}
				}
		}
	}
	
	return numBlob;
}



// Variance method
void SalienceOperator::fuseFoveaBlob3(ImageOf<PixelInt>& tagged, char *blobList, PixelBgr var, int max_tag)
{
	const PixelMono rg0=m_boxes[1].meanRG;
	const PixelMono gr0=m_boxes[1].meanGR;
	const PixelMono by0=m_boxes[1].meanBY;
	
	for (int i=2; i<=max_tag; i++) {
		if (blobList[i])
			if (abs(m_boxes[i].meanRG-rg0)/var.r+abs(m_boxes[i].meanGR-gr0)/var.g+abs(m_boxes[i].meanBY-by0)/var.b<=2) {
				blobList[i]=2;
				m_boxes[1].areaLP+=m_boxes[i].areaLP;
				//__OLD//m_boxes[1].areaCart+=TotalArea(m_boxes[tag]);
				m_boxes[1].areaCart+=m_boxes[i].areaCart;
				m_boxes[1].rgSum+=m_boxes[i].rgSum;
				m_boxes[1].grSum+=m_boxes[i].grSum;
				m_boxes[1].bySum+=m_boxes[i].bySum;
				m_boxes[1].rSum+=m_boxes[i].rSum;
				m_boxes[1].gSum+=m_boxes[i].gSum;
				m_boxes[1].bSum+=m_boxes[i].bSum;
				m_boxes[1].xsum+=m_boxes[i].xsum;
				m_boxes[1].ysum+=m_boxes[i].ysum;

				if (m_boxes[i].xmin<m_boxes[1].xmin) m_boxes[1].xmin=m_boxes[i].xmin;
				if (m_boxes[i].ymin<m_boxes[1].ymin) m_boxes[1].ymin=m_boxes[i].ymin;
				if (m_boxes[i].xmax>m_boxes[1].xmax) m_boxes[1].xmax=m_boxes[i].xmax;
				if (m_boxes[i].xmax>m_boxes[1].xmax) m_boxes[1].xmax=m_boxes[i].xmax;

				if (m_boxes[i].cmin<m_boxes[1].cmin) m_boxes[1].cmin=m_boxes[i].cmin;
				if (m_boxes[i].rmin<m_boxes[1].rmin) m_boxes[1].rmin=m_boxes[i].rmin;
				if (m_boxes[i].cmax>m_boxes[1].cmax) m_boxes[1].cmax=m_boxes[i].cmax;
				if (m_boxes[i].rmax>m_boxes[1].rmax) m_boxes[1].rmax=m_boxes[i].rmax;

				m_boxes[i].valid=false;
			}
	}
		
	m_boxes[1].centroid_y = (double)m_boxes[1].ysum / m_boxes[1].areaLP;
	m_boxes[1].centroid_x = (double)m_boxes[1].xsum / m_boxes[1].areaLP;

	m_boxes[1].meanRG = m_boxes[1].rgSum / m_boxes[1].areaLP;
	m_boxes[1].meanGR = m_boxes[1].grSum / m_boxes[1].areaLP;
	m_boxes[1].meanBY = m_boxes[1].bySum / m_boxes[1].areaLP;

	for (int r=0; r<height; r++)
		for (int c=0; c<width; c++)
			if (blobList[tagged(c,r)]==2) 
				tagged(c,r)=1;
}

void SalienceOperator::centerOfMassAndMass(ImageOf<PixelInt> &in, PixelInt tag, int *x, int *y, double *mass){
	/*int r,t;
	double areaT = 0.0;
	double sumX = 0.0;
	double sumY = 0.0;
	PixelInt *src;
	for(r = 0; r < height; r++) {
		int sumTmpX = 0;
		int sumTmpY = 0;
		int areaTmp = 0;

		src = (PixelInt *)in.getRawImage()[r];
		for(t = 0; t < width; t++) {
			if (*src==tag) {
				int tmpX;
				int tmpY;

				this->logPolar2Cartesian(r, t, tmpX, tmpY);

				sumTmpX += tmpX;
				sumTmpY += tmpY;
				areaTmp++;
			}
			src++;
		}
		double J=0.5;
		sumX+=sumTmpX*J;
		sumY+=sumTmpY*J;
		areaT+=areaTmp*J;
	}
	
	*mass=areaT;
	
	if (areaT != 0)	{
		*x = (int)(sumX/areaT + .5);
		*y = (int)(sumY/areaT + .5);
	} else {
		*x = 0;
		*y = 0;
	}*/
}


void SalienceOperator::maxSalienceBlob(ImageOf<PixelInt>& tagged, int max_tag, YARPBox &box)
{
	int max=1;
	int xcart;
	int ycart;
	
	for (int l = 1; l < max_tag; l++)
		if (m_boxes[l].valid)
			if (m_boxes[l].salienceTotal>m_boxes[max].salienceTotal)
				max=l;

	box=m_boxes[max];
	centerOfMassAndMass(tagged, box.id, &xcart, &ycart, &box.areaCart);
	box.centroid_x=xcart;
	box.centroid_y=ycart;
	//_gaze.computeRay(YARPBabybotHeadKin::KIN_LEFT_PERI, box.elev, box.az, (int)box.centroid_x, (int)box.centroid_y);
}


void SalienceOperator::resize(const int width1, const int height1)
{
	width=width1;
	height=height1;

	imageSize=width*height;

	m_boxes = new YARPBox[imageSize];

	_checkCutted = new bool[imageSize];

	/*tmpMsk = iplCreateImageHeader(
		1,
		0,
		IPL_DEPTH_1U,
		"GRAY",
		"G",
		IPL_DATA_ORDER_PLANE,
		IPL_ORIGIN_TL,
		IPL_ALIGN_QWORD,
		width1,
		height1,
		NULL,
		NULL,
		NULL,
		NULL);*/

	//iplAllocateImage(tmpMsk,1,0);

	//integralRG.resize(width1, height1);
	//integralGR.resize(width1, height1);
	//integralBY.resize(width1, height1);
}

/* //gets the tagged image of pixels,the R+G-, G*R-,B+Y-, the Red, blue and green Plans and creates a catalog of blobs*/
void SalienceOperator::blobCatalog(ImageOf<PixelInt>& tagged,
							   ImageOf<PixelMonoSigned> &rg,
							   ImageOf<PixelMonoSigned> &gr,
							   ImageOf<PixelMonoSigned> &by,
							   ImageOf<PixelMono> &r1,
							   ImageOf<PixelMono> &g1,
							   ImageOf<PixelMono> &b1,
							   int last_tag){
	
	for (int i = 0; i <= last_tag; i++) {
		m_boxes[i].cmax = m_boxes[i].rmax = 0;
		m_boxes[i].cmin = width;
		m_boxes[i].rmin = height;

		m_boxes[i].xmax = m_boxes[i].ymax = 0;
		m_boxes[i].xmin = m_boxes[i].ymin = 255;//<---REA_VERSION


		//m_boxes[i].xmin = m_lp.GetCWidth();
		//m_boxes[i].ymin = m_lp.GetCHeight();

		//__OLD//m_boxes[i].total_sal = 0;
		m_boxes[i].areaLP = 0;
		//__OLD//m_boxes[i].areaCart = 0;
		m_boxes[i].xsum = m_boxes[i].ysum = 0;
		m_boxes[i].valid = false;
		
		m_boxes[i].cutted = false;
		
		m_boxes[i].id = i;
		
		//__OLD//m_boxes[i].edge = false;

		m_boxes[i].rgSum=0;
		m_boxes[i].grSum=0;
		m_boxes[i].bySum=0;

		m_boxes[i].rSum=0;
		m_boxes[i].gSum=0;
		m_boxes[i].bSum=0;
	}
  
	// special case for the null tag (0)
	// null tag is not used!!!
	m_boxes[0].rmax = m_boxes[0].rmin = height/2;
	m_boxes[0].cmax = m_boxes[0].cmin = width/2;
	//m_boxes[0].xmax = m_boxes[0].xmin = m_lp.GetCWidth() / 2;
	//m_boxes[0].ymax = m_boxes[0].ymin = m_lp.GetCHeight() / 2;
	m_boxes[0].salienceTotal=0;
	m_boxes[0].valid = false;

	memset(_checkCutted, 0, sizeof(bool)*width*height);
	//----------
	for (int r = 0; r < height; r++)
		if (tagged(0, r)==tagged(width-1, r)) {
			m_boxes[tagged(0, r)].cutted=true;
			m_boxes[tagged(0, r)].indexCutted=r;
		}
	

	//----------
	// pixels are logpolar, averaging is done in cartesian
	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			long int tag_index = tagged(c, r);
			m_boxes[tag_index].areaLP++;
			//m_boxes[tag_index].areaCart+=pixelSize[r];

			m_boxes[tag_index].valid = true;

			int x, y;
			//get the x,y in order to have the averaging
			//m_lp.Logpolar2Cartesian(r, c, x, y);
			this->logPolar2Cartesian(r,c,x,y);

			if (m_boxes[tag_index].ymax < y) 
				m_boxes[tag_index].ymax = y;
			if (m_boxes[tag_index].ymin > y) 
				m_boxes[tag_index].ymin = y;
			if (m_boxes[tag_index].xmax < x) 
				m_boxes[tag_index].xmax = x;
			if (m_boxes[tag_index].xmin > x) 
				m_boxes[tag_index].xmin = x;

			m_boxes[tag_index].ysum += y;
			m_boxes[tag_index].xsum += x;

			if (m_boxes[tag_index].rmax < r) m_boxes[tag_index].rmax = r;
			if (m_boxes[tag_index].rmin > r) m_boxes[tag_index].rmin = r;
			if (!m_boxes[tag_index].cutted) {
				if (m_boxes[tag_index].cmax < c) m_boxes[tag_index].cmax = c;
				if (m_boxes[tag_index].cmin > c) m_boxes[tag_index].cmin = c;
			} else
				_checkCutted[m_boxes[tag_index].indexCutted*height+c]=true;
				

			m_boxes[tag_index].rgSum += rg(c, r);
			m_boxes[tag_index].grSum += gr(c, r);
			m_boxes[tag_index].bySum += by(c, r);

			m_boxes[tag_index].rSum += r1(c, r);
			m_boxes[tag_index].gSum += g1(c, r);
			m_boxes[tag_index].bSum += b1(c, r);
		}
	}
	
	//-----
	for (int i = 1; i <= last_tag; i++) {
		if (m_boxes[i].cutted) {
			int index = m_boxes[i].indexCutted;
			int c = 1;
			while (c<width && _checkCutted[index*height+c]==true)
				c++;
			if (c<width) {
				m_boxes[i].cmax=c-1;
				c++;
				while (_checkCutted[index*height+c]==false)
					c++;
				m_boxes[i].cmin=c-width;
			} else {
				m_boxes[i].cmin=0;
				m_boxes[i].cmax=width;
			}
		}
	}


}	

void SalienceOperator::logPolar2Cartesian(int irho, int itheta, int& ox, int& oy){
	double xx = 0;
	double yy = 0;
	double _xsize=2;
	double _ysize=2;

	//get_XY_Center(&xx, &yy, irho, itheta, &_img);
	xx=320/2;
	yy=240/2;

	//using namespace _logpolarParams;
	ox = int(xx + .5) + _xsize/2;
	oy = _ysize/2 - int(yy + .5);

	//return true;
}

void SalienceOperator::get_XY_Center(int *xx, int *yy, int irho, int itheta, yarp::sig::ImageOf<PixelRgb> *_img){
	int width=_img->width();
	int height=_img->height();
	*xx=width/2;
	*yy=height/2;
}