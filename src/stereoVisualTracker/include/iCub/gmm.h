#ifndef _MIXTURE_H_
#define _MIXTURE_H_

#include "public.h"

class Database {
protected:
	static const s32 maxnp = 720*576*2;
	static const s32 PtSize = 2;
	s32 *x;
	s32 *y;
	u32 np;
	u32 xsiz, ysiz;
	u32 col;
	bool lockflag;
public:

	Database(u32 xSize, u32 ySize, u32 c) {
		lockflag = false;
		x = new s32[maxnp];
		y = new s32[maxnp];
		xsiz = xSize;
		ysiz = ySize;
		col = c;
		np = 0;
	}

	~Database(){
		delete x;
		delete y;
	}

	void paint(IplImage *image) {
		if(!image) return;
		FOR(i,np)
			cvDrawLine(image, cvPoint(x[i],y[i]), cvPoint(x[i]+2,y[i]+2), CV_RGB(col,col,col),PtSize,8);
	}

	void push(s32 newx, s32 newy) {
		if(np < maxnp) {
			x[np] = newx;
			y[np] = newy;
			np++;
		}
	}

	void clearPoints() {
		np = 0;
	}

	void Process(IplImage *image){
		clearPoints();
		u32 w = image->width;
		u32 h = image->height;
		u32 step = image->widthStep;
		if(w*h < maxnp){
			FOR(i,h){
				FOR(j,w){
					if(rgb(image,i*step + j)){
						x[np] = j;
						y[np++] = i;
					}
				}
			}
		}
	}

	void randomPoints(u32 n) {
		FOR(i,n)
			push((s32) (xsiz * rand()/(f64)RAND_MAX ), (s32) (ysiz * rand()/(f64)RAND_MAX ));
	}

	s32 nPoints() {
		return np;
	}

	s32 xVal(s32 i) {
		return x[i];
	}

	s32 yVal(s32 i) {
		return y[i];
	}
};

class Module {
protected:
	s32 xsiz, ysiz;
	f64 weight;
	f64 *probs;
	CvPoint center;
public:
	Module(){
		center = cvPoint(0,0);
		xsiz = 2;
		ysiz = 2;
		weight = 1.0;
		probs = NULL; //micha
	}
	Module(s32 xSiz, s32 ySiz, f64 w) {
		xsiz = xSiz;
		ysiz = ySiz;
		weight = w;
		randomKernel(w);
		probs = NULL; //micha
	}
	
	CvPoint GetCoords(){return center;}

	void setweight(f64 w) {
		weight = w;
	}

	virtual void randomKernel(f64 w){};


	virtual void paint(IplImage *image, Database *db){};


	virtual f64 density(s32 x, s32 y){return 0;};


	f64 *calcp(Database *db) {
		s32 j, np;

		np = db->nPoints();
		DEL(probs)
			probs = new f64[np];
		for(j = 0; j < np; j++) probs[j] = density(db->xVal(j), db->yVal(j));
		return probs;
	}

	void EMprob(f64 *px, Database *db) {
		u32 np = db->nPoints();
		weight = 0;
		FOR(i,np){
			probs[i] /= px[i];
			weight += probs[i];
		}
		weight /= np;
	}  

	virtual void EMpar(Database *db, f64 prior){};
};

class CurvedGaussian : public Module {
protected:
	f64 kmx, kmy, ksx, ksy, ksxy;
	static const s32 mins = 5;
	bool plotcircle;

public:
	CurvedGaussian(s32 xSize, s32 ySize, f64 w)
		: Module(xSize, ySize, w)
	{
		center = cvPoint(0,0);
		plotcircle = true;
	}

	void randomKernel(f64 w) {
		weight = w;
		kmx = xsiz * rand()/(f64)RAND_MAX;
		kmy = ysiz * rand()/(f64)RAND_MAX;
		ksx = xsiz / 4 + mins;
		ksy = ysiz / 4 + mins;
		ksxy = 0;
	}

	void setplotline() {
		plotcircle = false;
	}

	f64 *getPar() {
		f64 *pars = new f64[6];
		pars[0] = weight;
		pars[1] = kmx;
		pars[2] = kmy;
		pars[3] = ksx;
		pars[4] = ksy;
		pars[5] = ksxy;
		return pars;
	}

	void paint(IplImage *image, Database *db) {
		if(!image) return;
		s32 j;
		f64 u1x, u1y, u2x, u2y, r1, r2, tmp, varx, vary;

		if(fabs(ksxy) <= 1e-4) { //micha was abs
			if(ksx > ksy) {
				u1x = u2y = 1;
				u1y = u2x = 0;
				r1 = ksx;
				r2 = ksy;
			} else {
				u1x = u2y = 0;
				u1y = u2x = 1;
				r1 = ksy;
				r2 = ksx;
			}
		} else {
			varx = ksx * ksx;
			vary = ksy * ksy;
			// eigen value
			tmp = varx - vary;
			tmp = sqrt(tmp * tmp + 4 * ksxy * ksxy);
			r1 = sqrt((varx + vary + tmp) / 2);
			r2 = sqrt((varx + vary - tmp) / 2);

			// eigen vectors
			u1x = r1 * r1 - vary;
			tmp = sqrt(u1x * u1x + ksxy * ksxy);
			u1x /= tmp;
			u1y = ksxy / tmp;

			u2x = r2 * r2 - vary;
			tmp = sqrt(u2x * u2x + ksxy * ksxy);
			u2x /= tmp;
			u2y = ksxy / tmp;
		}
		//printf("%d",weight);
		if(plotcircle) {
			for(j = 1; j < 4; j++) {
				drawCurvedOval(image, u1x, u1y, u2x, u2y, r1 * j, r2 * j,j==1 ? 2 : 1);
			}
		} else {
			cvDrawLine(image, cvPoint((s32)(kmx + 3 * r1 * u1x),(s32)(kmy + 3 * r1 * u1y)), cvPoint((s32)(kmx - 3 * r1 * u1x), (s32)(kmy - 3 * r1 * u1y)),CV_RGB(255,255,255),2,8);
		}
	}

	void drawCurvedOval(IplImage *image, f64 x1, f64 y1, f64 x2, f64 y2, f64 r1, f64 r2, s32 thick) {
		if(!image) return;
		s32 fx, fy, tx, ty;
		f64 w1, w2;

		fx = (s32) (kmx + r1 * x1);
		fy = (s32) (kmy + r1 * y1);
		for(f64 th = 0.1; th < 6.4; th += 0.1) {
			w1 = cos(th);
			w2 = sin(th);
			tx = (s32) (kmx + r1 * x1 * w1 + r2 * x2 * w2);
			ty = (s32) (kmy + r1 * y1 * w1 + r2 * y2 * w2);
			cvDrawLine(image, cvPoint(fx,fy),cvPoint(tx,ty),CV_RGB(255,255,255), thick, CV_AA);
			fx = tx;
			fy = ty;
		}
	}

	f64 density(s32 x, s32 y) {
		f64 tmpx, tmpy, tmpxy, det, varx, vary;
		varx = ksx * ksx;
		vary = ksy * ksy;
		det = varx * vary - ksxy * ksxy;
		tmpx = (x - kmx) * (x - kmx);
		tmpy = (y - kmy) * (y - kmy);
		tmpxy = (x - kmx) * (y - kmy);
		return weight / sqrt(det) / 6.28319 *
			exp(-(tmpx * vary + tmpy * varx - 2 * tmpxy * ksxy) / det / 2);
	}

	void EMpar(Database *db, f64 prior) {
		s32 j, np;
		f64 x, y, tmp, tmpsx, tmpsy, tmpsxy;

		np = db->nPoints();
		tmpsx = tmpsy = tmpsxy = kmx = kmy = 0;
		for(j = 0; j < np; j++) {
			x = db->xVal(j);
			y = db->yVal(j);
			kmx += probs[j] * x;
			kmy += probs[j] * y;
			tmpsx += probs[j] * x * x;
			tmpsy += probs[j] * y * y;
			tmpsxy += probs[j] * x * y;
		}
		tmp = np * weight;
		kmx /= tmp;
		kmy /= tmp;
		ksx = sqrt(tmpsx / tmp - kmx * kmx);
		ksy = sqrt(tmpsy / tmp - kmy * kmy);
		ksxy = tmpsxy / tmp - kmx * kmy;
		if(ksx < mins) ksx = mins;
		if(ksy < mins) ksy = mins;
		weight = 0.9 * weight + 0.1 * prior;
		center = !tmp ? cvPoint(0,0) : cvPoint((s32)kmx, (s32)kmy);
	}
};

class Uniform : Module {
public:
	Uniform(s32 xSize, s32 ySize, f64 w)
		: Module(xSize, ySize, w) {}

	void randomKernel(f64 w)  {
		weight = w;      
	}

	void paint(IplImage *image, Database *db)  {
	}

	f64 density(s32 x, s32 y)  {
		return weight / xsiz / ysiz;
	}

	void EMpar(Database *db, f64 prior)  {
	}
};

class Mixture {
protected:
	static const u32 maxkp = 20;
	static const s32 typegauss = 0;
	static const s32 typeuniform = 1;
	static const s32 typecurvedgauss = 2;
	static const s32 typescaleshift = 3;
	static const s32 typeline = 101;
	Module **kernel;;
	s32 *type;
	f64 *weight;
	u32 nk;
	u32 xsiz, ysiz;
	f64 *px;
	Database *db;

public:
	Mixture(s32 xSize, s32 ySize, Database *DB) {
		kernel = new Module *[maxkp];
		type = new s32[maxkp];
		weight = new f64[maxkp];
		xsiz = xSize;
		ysiz = ySize;  
		db = DB;
		px=NULL;
	}

	void initKernel(Module *mod, s32 tp, s32 pos) {
		kernel[pos] =  mod;
		type[pos] = tp;
	}

	void setnk(s32 nK) {
		if(nk == nK) return;
		nk = nK;
		randomKernel();
	}

	void setnk(s32 nK, f64 *ws) {
		f64 sum;
		f64 *w;

		nk = nK;
		w = new f64[nk];
		sum = 0;
		FOR(i,nk) sum += ws[i];
		FOR(i,nk) w[i] = ws[i] / sum;
		randomKernel(w);
	}

	void randomKernel(f64 *ws) {
	  f64 sum = 0;//micha tmp = 0;
		FOR(i,nk) sum += ws[i];
		FOR(i,nk) kernel[i]->randomKernel(ws[i] / sum);
	}

	void randomKernel() {
		f64 *ws = new f64[nk];
		FOR(i,nk) ws[i] = 1.0 / nk;
		randomKernel(ws);
	}

	void paint(IplImage *image) {
		FOR(i,nk) kernel[i]->paint(image, db);
	}

	void calcpx() {
		f64 *probs;

		u32 np = db->nPoints();
		DEL(px);
		px = new f64[np];
		FOR(i,np)px[i] = 0;
		FOR(i,nk){
			probs = kernel[i]->calcp(db);
			FOR(j,np) px[j] += probs[j];
		}
	}

	void EM(f64 *ws) {
		EMmain(ws);
	}

	void EMmain(f64 *ws) {
		f64 sum;
		if(db->nPoints() <= 2) return;
		sum = 0;
		FOR(i,nk) sum += ws[i];
		calcpx();
		FOR(i,nk){
			kernel[i]->EMprob(px, db);
			kernel[i]->EMpar(db, ws[i] / sum);
		}
	}

	f64 likelihood() {
		u32 np = db->nPoints(); 
		if(np == 0) return 0;
		calcpx();
		f64 tmp = 0;
		FOR(j, np) tmp += log(px[j]);
		return tmp / np;
	}

	CvPoint GetCoords(u32 index){
		return (kernel && kernel[index+1]) ? kernel[index+1]->GetCoords() : cvPoint(-1,-1);
	}
};

class CurvedGaussMixture : public Mixture {
protected:
	static const s32 kmax = 10;
public:
	CurvedGaussMixture(s32 xSize, s32 ySize, Database *DB, u32 kernels)
		: Mixture(xSize, ySize, DB)
	{
		initKernel((Module *)(new Uniform(xsiz, ysiz, 0.0)), typeuniform, 0);
		for(s32 i = 1; i < kmax; i++) {
			initKernel(new CurvedGaussian(xsiz, ysiz, 0.0), typecurvedgauss, i);
		}
		setnk(kernels+1);
	}

	void paint(IplImage *image){
		FOR(i, kmax){
			kernel[i]->paint(image, db);
		}
	}
};

class GaussLineMixture : public Mixture {
protected:
	static const s32 kmax = 10;
public:
	GaussLineMixture(s32 xSize, s32 ySize, Database *DB)
		: Mixture(xSize, ySize, DB)
	{
		CurvedGaussian *cgmix;
		initKernel((Module *)(new Uniform(xsiz, ysiz, 0.0)), typeuniform, 0);
		for(s32 i = 1; i < kmax; i++) {
			cgmix = new CurvedGaussian(xsiz, ysiz, 0.0);
			cgmix->setplotline();
			initKernel(cgmix, typecurvedgauss, i);
		}
		setnk(2);
	}

	void paint(IplImage *image){
		FOR(i,kmax){
			if(!kernel[i]){
				kernel[i]->paint(image, db);
			}
		}
	}
};

#endif // _MIXTURE_H_
