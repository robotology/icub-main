#include "eyeTriangulation.h"

#define KALMAN_INIT		0
#define KALMAN_NORMAL	1
#define KALMAN_NOINPUT	2


void eyeTriangulation::xInit(Vector xLeft, Vector xRight)
{
  xr = xRight;
  xl = xLeft;
}

void eyeTriangulation::xSetConstant(Vector xLeft, Vector xRight)
{
  xr = xRight;
  xl = xLeft;

  fprintf(stderr, "using a constant xr and xl\n");
  xConstant = true;
}

void eyeTriangulation::xDisConstant()
{
  fprintf(stderr, "using a input xr and xl\n");
  xConstant = false;
}

eyeTriangulation::eyeTriangulation(const string &configFile, Matrix PrjL, Matrix PrjR,
                                   bool _enableKalman, unsigned int _period) : 
                                   RateThread(_period), enableKalman(_enableKalman), Ts(_period/1000.0)
{
  T_RoLe.resize(4,4);
  T_RoRe.resize(4,4);
  T_LeRo.resize(4,4);
  T_ReRo.resize(4,4);
  Pr.resize(3,4);
  Pl.resize(3,4);

  rightEye = new iCubEye("right");
  leftEye = new iCubEye("left");

  rightEye->releaseLink(0);
  rightEye->releaseLink(1);
  rightEye->releaseLink(2);

  leftEye->releaseLink(0);
  leftEye->releaseLink(1);
  leftEye->releaseLink(2);

  chainRightEye=rightEye->asChain();
  chainLeftEye =leftEye->asChain();

  // add aligning links read from configuration file
  if (getAlignLinks(configFile,"RIGHT_ALIGN",&alignLnkRight1,&alignLnkRight2))
  {
      *chainRightEye<<*alignLnkRight1<<*alignLnkRight2;
      chainRightEye->blockLink(chainRightEye->getN()-1,0.0);
      chainRightEye->blockLink(chainRightEye->getN()-2,0.0);
  }
  else
      alignLnkRight1=alignLnkRight2=NULL;

  if (getAlignLinks(configFile,"LEFT_ALIGN",&alignLnkLeft1,&alignLnkLeft2))
  {
      *chainLeftEye<<*alignLnkLeft1<<*alignLnkLeft2;
      chainLeftEye->blockLink(chainLeftEye->getN()-1,0.0);
      chainLeftEye->blockLink(chainLeftEye->getN()-2,0.0);
  }
  else
      alignLnkLeft1=alignLnkLeft2=NULL;

  cout << "Left Eye kinematic parameters:" << endl;
  for (unsigned int i=0; i<chainLeftEye->getN(); i++)
  {
      cout << "#" << i << ": " <<  
      (*chainLeftEye)[i].getA()       << ", " <<
      (*chainLeftEye)[i].getD()       << ", " <<
      (*chainLeftEye)[i].getAlpha()   << ", " <<
      (*chainLeftEye)[i].getOffset()  << ", " <<
       endl;
  }

  cout << "Right Eye kinematic parameters:" << endl;
  for (unsigned int i=0; i<chainRightEye->getN(); i++)
  {
      cout << "#" << i << ": " <<  
      (*chainRightEye)[i].getA()       << ", " <<
      (*chainRightEye)[i].getD()       << ", " <<
      (*chainRightEye)[i].getAlpha()   << ", " <<
      (*chainRightEye)[i].getOffset()  << ", " <<
       endl;
  }

  Pr = PrjR;
  Pl = PrjL;

  nr = chainRightEye->getDOF();
  nl = chainLeftEye->getDOF();

  qr.resize(nr);
  ql.resize(nl);

  qr.zero();
  ql.zero();

  xr.resize(3);
  xl.resize(3);

  xr.zero();
  xl.zero();

  xl(2) = 1.0;
  xr(2) = 1.0;

  A.resize(4,3);
  b.resize(4);

  // Kalman stuff
  kalA=eye(3,3);			// the state is 3d pos
  kalH=eye(3,3);			// 3d pos is measured

  kalQ=0.01*eye(3,3);		// Process noise covariance
  kalR=0.02*eye(3,3);		// Measurement noise covariance

  // initial guess
  kalx0.resize(3); kalx0=0.0;
  kalP0=10*eye(3,3);

  // rate limiter set-up
  Vector rL(3); rL=0.4*Ts;	// max |x| m/s => max |x|*Ts m/sampling time

  rlim=new RateLimiter(-1.0*rL,rL);
  kal=new Kalman(kalA,kalH,kalQ,kalR);

  rlim->init(kalx0);
  kal->init(kalx0,kalx0,kalP0);

  kalState=KALMAN_INIT;
  kalTimer=0.0;

  qBottleTorso.addDouble(0.0);
  qBottleTorso.addDouble(0.0);
  qBottleTorso.addDouble(0.0);

  qBottleHead.addDouble(0.0);
  qBottleHead.addDouble(0.0);
  qBottleHead.addDouble(0.0);
  qBottleHead.addDouble(0.0);
  qBottleHead.addDouble(0.0);
  qBottleHead.addDouble(10.0);	// initial vergence != 0.0

  //not using a constant xr and xl
  xConstant = false;
}

bool eyeTriangulation::threadInit()
{
  xlrPort.open("/eyeTriangulation/x:i");
  XPort.open("/eyeTriangulation/X:o");

  qPortTorso.open("/eyeTriangulation/qTorso:i");
  qPortHead.open("/eyeTriangulation/qHead:i");
  Network::connect("/icub/torso/state:o", "/eyeTriangulation/qTorso:i");
  Network::connect("/icub/head/state:o", "/eyeTriangulation/qHead:i");

  return true;
}

void eyeTriangulation::afterStart(bool s)
{
	if (s)
		fprintf(stderr, "eT is running\n");
	else
		fprintf(stderr, "eT did not start\n");
}

void eyeTriangulation::threadRelease()
{
	fprintf(stderr, "Trying to stop eT...\n");

	delete rightEye;
	delete leftEye;
	delete rlim;
	delete kal;

    if (alignLnkLeft1)
        delete alignLnkLeft1;

    if (alignLnkLeft2)
        delete alignLnkLeft2;

    if (alignLnkRight1)
        delete alignLnkRight1;

    if (alignLnkRight2)
        delete alignLnkRight2;

    xlrPort.interrupt();
	XPort.interrupt();
	qPortTorso.interrupt();
	qPortHead.interrupt();

	xlrPort.close();
	XPort.close();
	qPortTorso.close();
	qPortHead.close();

	fprintf(stderr, "eT stopped successfully\n");
}

bool eyeTriangulation::getAlignLinks(const string &configFile, const string &type,
                                     iKinLink **link1, iKinLink **link2)
{
    *link1=*link2=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string error="unable to find aligning parameters for "+type+" eye";

        if (parType.size())
        {
            Bottle length=parType.findGroup("length");
            Bottle offset=parType.findGroup("offset");
            Bottle twist=parType.findGroup("twist");

            if (length.size()>=2 && offset.size()>=2 && twist.size()>=2)
            {
                *link1=new iKinLink(length.get(1).asDouble(),offset.get(1).asDouble(),
                                    twist.get(1).asDouble(),0.0,0.0,0.0);
                *link2=new iKinLink(length.get(2).asDouble(),offset.get(2).asDouble(),
                                    twist.get(2).asDouble(),0.0,0.0,0.0);

                return true;
            }
            else
                cerr << error << endl;
        }
        else
            cerr << error << endl;
    }

    return false;
}

bool eyeTriangulation::xCheckBottleFormat(Bottle* b, Vector &xL, Vector &xR)
{
	if (b->size()>=4)
	{
		xL(0)=b->get(0).asDouble();
		xL(1)=b->get(1).asDouble();
		xR(0)=b->get(2).asDouble();
		xR(1)=b->get(3).asDouble();
		return true;
	}
	else
	{
		fprintf(stderr, "Trowing away a x bottle with the wrong number of entries\n");
		return false;
	}
}

bool eyeTriangulation::qrGet(Vector q)
{
  if (q.size() != qr.size() + 1)
    {
      fprintf(stderr, "Getting a wrong q vector\n");
      return false;
    }
  else
    {
      for (int i=0; i < qr.size() -1; i++)
		  qr(i) = q(i)*M_PI/180;

	  int k = qr.size() - 1;
      qr(k) = (q(k) - q(k+1)/2.0)*M_PI/180;
      return true;
    }
}

bool eyeTriangulation::qlGet(Vector q)
{
  if (q.size() != ql.size() + 1)
    {
      fprintf(stderr, "Getting a wrong q vector\n");
      return false;
    }
  else
    {
      for (int i=0; i < ql.size() -1; i++)
		  ql(i) = q(i)*M_PI/180;

	  int k = ql.size() - 1;
      ql(k) = (q(k) + q(k+1)/2.0)*M_PI/180;
      return true;
    }
}

void eyeTriangulation::run()
{
      if (!xConstant)
		  {
			  if (Bottle *xlrBottle=xlrPort.read(false))
				  {
					xCheckBottleFormat(xlrBottle,xl,xr);
					kalTimer=0.0;
				  }
		  }
	  else
		  kalTimer=0.0;

	  //fprintf(stderr, "Trying to read torso position \n");
	  Bottle *qBottleTorsoNew=qPortTorso.read(false);
	  if (qBottleTorsoNew)
			qBottleTorso=*qBottleTorsoNew;
	  //fprintf(stderr, "Torso position read\n");

	  //fprintf(stderr, "Trying to read head position \n");
      Bottle *qBottleHeadNew=qPortHead.read(false);
	  if (qBottleHeadNew)
			qBottleHead=*qBottleHeadNew;
      //fprintf(stderr, "Head position read\n");

	  int szTorso=qBottleTorso.size();
	  int szHead=qBottleHead.size();
      q.resize(szTorso+szHead);
      for (int i=0; i<szTorso; i++)
		  q(i)=qBottleTorso.get(2-i).asDouble();	// invert torso encoders
	  for (int i=0; i<szHead; i++)
		  q(szTorso+i)=qBottleHead.get(i).asDouble();

      if (qrGet(q) && qlGet(q))
	{
	  //fprintf(stderr, "Loop in the eT loop\n");    
	  //fprintf(stderr, "qr value is %s\n", chainRightEye->getAng().toString().c_str());
	  T_RoRe = chainRightEye->getH(qr);
	  T_ReRo = SE3inv(T_RoRe);
	  //fprintf(stderr, "T_RoRe is: \n");    
	  //for (int i = 0; i < T_RoRe.rows(); i++)
	  //	fprintf(stderr, "%s\n", T_RoRe.getRow(i).toString().c_str());    
	  //fprintf(stderr, "ql value is %s\n", chainLeftEye->getAng().toString().c_str());
	  T_RoLe = chainLeftEye->getH(ql);
	  T_LeRo = SE3inv(T_RoLe);
	  //fprintf(stderr, "T_RoLe is: \n");    
	  //for (int i = 0; i < T_RoLe.rows(); i++)
	  //	fprintf(stderr, "%s\n", T_RoLe.getRow(i).toString().c_str());    
     
	  //Rewrite XLe(2)*xl = Pl*XLe and
	  //XRe(2)*xr = Pr*XRe as a linear
	  //system A*XRo = b recalling that
	  //XRe=T_ReRo*XRo and XLe=T_LeRo*XRo

	  //Let's first observe that:
	  //X(2)*x = [0 0 x 0]*X so that
	  //X(2)*x = P*X can be rewritten:
	  //(P - [0 0 x 0])*X = 0 i.e.
	  //(P - [0 0 x 0])*T*XRo = 0

	  Matrix tmp=zeros(3,4);
	  tmp(0,2) = xr(0); tmp(1,2) = xr(1); tmp(2,2) = xr(2);
	  Matrix Ar = (Pr - tmp) * T_ReRo;

	  //fprintf(stderr, "Ar is: \n");    
	  //for (int i = 0; i < Ar.rows(); i++)
	  //	fprintf(stderr, "%s\n", Ar.getRow(i).toString().c_str());    

	  tmp(0,2) = xl(0); tmp(1,2) = xl(1); tmp(2,2) = xl(2);
	  Matrix Al = (Pl - tmp) * T_LeRo;

	  //fprintf(stderr, "Al is: \n");    
	  //for (int i = 0; i < Al.rows(); i++)
	  //	fprintf(stderr, "%s\n", Al.getRow(i).toString().c_str());    

	  //fprintf(stderr, "Copying up A1 \n");
	  for (int i = 0; i < 2; i++)
	    {
		  b(i) = -Ar(i,3);
	      for (int j = 0; j < 3; j++)
		{
		  A(i,j) = Ar(i,j);
		}
	    }
	  //fprintf(stderr, "Copying up A2 \n");
	  for (int i = 0; i < 2; i++)
	    {
		  b(i+2) = -Al(i,3);
	      for (int j = 0; j < 3; j++)
	      A(i+2,j) = Al(i,j);
	    }

	  //fprintf(stderr, "A is: \n");    
	  //for (int i = 0; i < A.rows(); i++)
	  //	fprintf(stderr, "%s\n", A.getRow(i).toString().c_str());          

	  //fprintf(stderr, "b is: \n");    
	  //for (int i = 0; i < b.rows(); i++)
	  //fprintf(stderr, "%s\n", b.getRow(i).toString().c_str());          


	  //Matrix X(4,1);
	  //X(0,0)=0; X(1,0)=0; X(2,0)=0; X(3,0)=1;
	  //Vector Temp = Pr*T_ReRo*X;
	  //Temp = (Temp(2)*xr) - Temp;
	  //fprintf(stderr, "xr should be: %s\n", Temp.toString().c_str());

	  //Temp = Pl*T_LeRo*X;
	  //Temp = (Temp(2)*xl) - Temp;
	  //fprintf(stderr, "xl should be: %s\n", Temp.toString().c_str());

	  //Matrix Temp = Ar*X;
	  //fprintf(stderr, "Residual error is: %s\n", Temp.toString().c_str());

	  Vector XRo=pinv(A)*b;
	  //fprintf(stderr, "Final solution is %s\n", XRo.toString().c_str());
	  
	  Vector &X=XPort.prepare();
	  if (enableKalman)
		  {
			 if (kalTimer>1.0 && kalState==KALMAN_NORMAL)
				 kalState=KALMAN_NOINPUT;

			 if (kalState==KALMAN_INIT || kalTimer<1.0 && kalState==KALMAN_NOINPUT)
				 {
				    kalx0(0)=XRo(0);
					kalx0(1)=XRo(1);
					kalx0(2)=XRo(2);
					rlim->init(kalx0);
					kal->init(kalx0,kalx0,kalP0);
					kalState=KALMAN_NORMAL;
				 }

			 Vector estX=kal->filt(rlim->filt(XRo));
			 X.resize(3);
			 X(0)=estX(0);
			 X(1)=estX(1);
			 X(2)=estX(2);
		  }
	  else
		  X=XRo;
	  XPort.write();
	}
      else
	fprintf(stderr, "Unable to get the head position\n");

	  kalTimer+=Ts;
}
    
    
