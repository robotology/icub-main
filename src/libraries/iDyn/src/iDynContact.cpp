/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Andrea Del Prete
* email:   andrea.delprete@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <iostream>
#include <iCub/iDyn/iDynContact.h>
#include <yarp/math/SVD.h>
#include <stdio.h>

//using namespace std;
//using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::iDyn;
using namespace iCub::ctrl;


//~~~~~~~~~~~~~~~~~~~~~~
//   IDYN CONTACT
//~~~~~~~~~~~~~~~~~~~~~~
iDynContact::iDynContact(unsigned int _linkNumber, const Vector &_pos){
    init(_linkNumber, _pos);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynContact::iDynContact(unsigned int _linkNumber, const Vector &_pos, const Vector &_Mu){
    init(_linkNumber, _pos);
    setMoment(_Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynContact::iDynContact(unsigned int _linkNumber, const Vector &_pos, const Vector &_Mu, const Vector &_Fdir){
    init(_linkNumber, _pos);
    setMoment(_Mu);
    setForceDirection(_Fdir);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::init(unsigned int _linkNumber, const Vector &_pos){
    setLinkNumber(_linkNumber); 
    setPosition(_pos);
    muKnown = false;
    fDirKnown = false;
}
//~~~~~~~~~~~~~~~~~~~~~~
//   GET methods
//~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContact::getForceMoment() const{
	Vector ret(6); ret.zero();
    Vector F = Fmodule*Fdir;
	ret[0]=F[0]; ret[1]=F[1]; ret[2]=F[2];
	ret[3]=Mu[0]; ret[4]=Mu[1]; ret[5]=Mu[2];
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContact::getForce() const{ return Fmodule*Fdir;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	iDynContact::getForceDirection() const{ return Fdir;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynContact::getForceModule() const{ return Fmodule;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContact::getMoment() const{ return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContact::getPosition() const{ return pos;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynContact::getLinkNumber() const{ return linkNumber;}
//~~~~~~~~~~~~~~~~~~~~~~
//   IS methods
//~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::isMomentKnown() const{ return muKnown;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::isForceDirectionKnown() const{ return fDirKnown;}
//~~~~~~~~~~~~~~~~~~~~~~
//   SET methods
//~~~~~~~~~~~~~~~~~~~~~~    
bool iDynContact::setForce(const Vector &_F){
    if(!checkVectorDim(_F, 3, "force"))
        return false;
    Fmodule = norm(_F);
    Fdir = _F / Fmodule;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForceModule(double _Fmodule){
    if(_Fmodule<0){
        if(verbose)
            fprintf(stderr, "Error in iDynContact: negative force module, %f\n", _Fmodule);
        return false;
    }
    Fmodule = _Fmodule;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForceDirection(const Vector &_Fdir){
    if(!checkVectorDim(_Fdir, 3, "force direction"))
        return false;
    Fdir = _Fdir / norm(_Fdir);
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setMoment(const Vector &_Mu){
    if(!checkVectorDim(_Mu, 3, "moment"))
        return false;
    Mu = _Mu;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setPosition(const Vector &_pos){
    if(!checkVectorDim(_pos, 3, "position"))
        return false;
    pos = _pos;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setLinkNumber(unsigned int _linkNum){
    linkNumber = _linkNum;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~
//   FIX/UNFIX methods
//~~~~~~~~~~~~~~~~~~~~~~ 
bool iDynContact::fixForceDirection(const Vector &_Fdir){
    if(setForceDirection(_Fdir)){
        fDirKnown = true;
        return true;
    }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::fixMoment(){
    Vector zeroMu(3);
	zeroMu.zero();
    return fixMoment(zeroMu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::fixMoment(const Vector &_Mu){
    if(setMoment(_Mu)){
        muKnown = true;
        return true;
    }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::unfixForceDirection(){ fDirKnown=false;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::unfixMoment(){ muKnown=false;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynContact::toString() const{
    return "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::setVerbose(unsigned int verb){
    verbose = verb;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::checkVectorDim(const Vector &v, unsigned int dim, const string &descr){
    if(v.length() != dim){
        if(verbose)
            fprintf(stderr, "Error in iDynContact: unexpected dimension of vector %s, %d\n", descr.c_str(), v.length());
        return false;
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynContact::norm(const Vector &v){
    double res = 0;
    for(int i=0; i<v.length(); i++)
        res += pow(v(i), 2);
    return sqrt(res);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// IDYN CONTACT SOLVER
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynContactSolver::iDynContactSolver(iDynChain *_c, const string &_info, const NewEulMode _mode, unsigned int verb)
:iDynSensor(_c, _info, _mode, verb){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynContactSolver::iDynContactSolver(iDynChain *_c, unsigned int sensLink, SensorLinkNewtonEuler *sensor, 
                                    const string &_info, const NewEulMode _mode, unsigned int verb)
:iDynSensor(_c, _info, _mode, verb){
    lSens = sensLink;
    sens = sensor;
}

iDynContactSolver::iDynContactSolver(iDynChain *_c, unsigned int sensLink, const Matrix &_H, const Matrix &_HC, double _m, 
                                     const Matrix &_I, const string &_info, const NewEulMode _mode, unsigned int verb)
:iDynSensor(_c, sensLink, _H, _HC, _m, _I, _info, _mode, verb){

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynContactSolver::~iDynContactSolver(){

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContactSolver::addContact(const iDynContact &contact){
    contactList.push_back(contact);
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContactSolver::addContacts(const deque<iDynContact> &contacts){
    contactList.insert(contactList.end(), contacts.begin(), contacts.end());
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContactSolver::removeContact(const int contactId){
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContactSolver::clearContactList(){
    if(!contactList.empty())
        contactList.clear();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
deque<iDynContact> iDynContactSolver::computeExternalContacts(const Vector &FMsens){
	deque<iDynContact> nullList;
	if(!setSensorMeasures(FMsens))                      // set the sensor measure
        return nullList;
	return computeExternalContacts();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
deque<iDynContact> iDynContactSolver::computeExternalContacts(){
    deque<iDynContact> nullList;
    if(contactList.size()==0)
        return nullList;

	// initialize the dynamics at the end effector and the kinematics at the base
	if(chain->NE == NULL){
		chain->prepareNewtonEuler(mode);
		chain->initNewtonEuler();       // here I assume the chain base is still
	}
    
    // KINEMATIC PHASE
	chain->computeKinematicNewtonEuler();               // compute kinematics of all the links
	sens->ForwardAttachToLink(chain->refLink(lSens));   // compute kinematics of the sensor sub-link

    // FIND THE BOUND OF THE SUBCHAIN WHERE EXT CONTACTS ARE APPLIED
    unsigned int firstContactLink, lastContactLink;
    findContactSubChain(firstContactLink, lastContactLink);
	if(verbose)
		fprintf(stdout, "Estimating %d contacts between link %d and %d\n", 
			contactList.size(), firstContactLink, lastContactLink);
    if(firstContactLink<lSens){
        if(verbose)
            fprintf(stderr, "Contacts before the sensor link. Impossible to compute the external contacts!\n");
        return nullList;
    }

    // PROPAGATE WRENCH FORWARD, UP TO THE FIRST LINK A CONTACT IS APPLIED TO
	sens->ForwardForcesMomentsToLink(chain->refLink(lSens));    // propagate wrench from sensor to hosting link
    //printMatrix("Forces after forward sensor", chain->getForces());
    if(firstContactLink==lSens){
        // if there're contacts on the sensor link we need to propagate the wrench to the previous link
        // (here we are assuming that the contacts are detected by the F/T sensor, no matter their application point)
        chain->NE->BackwardWrenchFromAtoB(lSens-1, lSens-1);
        /*Vector zeroF(3), zeroMu(3); zeroF.zero(); zeroMu.zero();
        chain->refLink(lSens)->setForceMoment(zeroF, zeroMu);*/    
    }else if(firstContactLink>lSens+1){
        chain->NE->ForwardWrenchFromAtoB(lSens+1, firstContactLink-1);
    }
    //printMatrix("Forces after firstContactLink", chain->getForces());

    // PROPAGATE WRENCH BACKWARD, FROM THE E.E. UP TO THE LAST LINK A CONTACT IS APPLIED TO    
    Vector Fend(3); Fend.zero(); Vector Mend(Fend);
    chain->NE->initWrenchEnd(Fend, Mend);
    chain->NE->BackwardWrenchFromAtoB(chain->getN()-1, lastContactLink);
    //printMatrix("Forces after lastContactLink", chain->getForces());

    // BUILD AND SOLVE THE LINEAR SYSTEM AX=B RELATIVE TO THE CONTACT SUB-CHAIN
    // the reference frame is the <firstContactLink-1> 
    Matrix A = buildA(firstContactLink, lastContactLink);
    Vector B = buildB(firstContactLink, lastContactLink);
	Matrix pinv_A;
	double tollerance = 10e-08;
	if(A.rows() < A.cols())	// SVD is not implemented for "fat" matrixes in GSL
		pinv_A = pinv(A.transposed(), tollerance).transposed();
	else
		pinv_A = pinv(A, tollerance);
    Vector X = pinv_A * B;
	//Vector AX = A*X;
    //if(verbose){
        //printMatrix("A", A);
        //printVector("B", B);
        //printVector("X", X);
		//printVector("AX", AX);
    //}
    
    // SET THE COMPUTED VALUES IN THE CONTACT LIST
    unsigned int unknownInd = 0;
    Matrix H, R;
    deque<iDynContact>::iterator it = contactList.begin();
    for(; it!=contactList.end(); it++){        
        if(it->isForceDirectionKnown()){
            it->setForceModule( X(unknownInd++));
        }else{
            H = getHFromAtoB(it->getLinkNumber(), firstContactLink-1);
            R = H.submatrix(0,2,0,2);
            it->setForce( R * X.subVector(unknownInd, unknownInd+2));
            unknownInd += 3;
            if(!it->isMomentKnown()){
                it->setMoment(R * X.subVector(unknownInd, unknownInd+2));
                unknownInd += 3;
            }
        }
    }

    // RETURN A COPY OF THE CONTACT LIST
    return contactList;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContactSolver::computeWrenchFromSensorNewtonEuler(){
	// if there is no contact specified, suppose contact at the end effector
	if(contactList.empty()){
		iDynSensor::computeWrenchFromSensorNewtonEuler();
		return;
	}

	// in the external contact computations also the internal wrenches
	// are computed, form the sensor link to the end effector
	// (except for the contact subchain, but now we suppose only 1 contact at a time)
	computeExternalContacts();
    //printMatrix("Forces after computeExternalContacts", chain->getForces());

	//propagate the wrench from lSens to Base
	chain->NE->BackwardWrenchToBase(lSens-1);
    //printMatrix("Forces after backward To Base", chain->getForces());

    // now we can compute all torques
    chain->NE->computeTorques();
    //printMatrix("Forces after computeTorques", chain->getForces());
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynContactSolver::buildA(unsigned int firstContactLink, unsigned int lastContactLink){
    unsigned int unknownNum = getUnknownNumber();
    Matrix A(6, unknownNum);

    // For each contact add some columns to A:
    //    * wrench: add 6 columns, 3 composed by I_3 above and the S(r_E) below, 3 composed by 0_3 above and I_3 below
    //    * pure force: add 3 columns composed by I_3 above and the S(r_E) below
    //    * force module: add 1 column composed by the force direction unit vector above and the cross product between 
    //          the contact point and the force direction unit vector below    
    unsigned int colInd = 0;
    Matrix H, R;
    Vector r;
    deque<iDynContact>::const_iterator it = contactList.begin();
    for(; it!=contactList.end(); it++){
        // compute the rototranslation matrix from <firstContactLink-1> to the current link
        H = getHFromAtoB(firstContactLink-1, it->getLinkNumber());
        R = H.submatrix(0,2,0,2);
        r = H.submatrix(0,2,3,3).getCol(0);

        if(it->isForceDirectionKnown()){                    // 1 UNKNOWN: FORCE MODULE
            Vector v = R*it->getForceDirection();           // force direction unit vector
            Vector v2 = cross(R*it->getPosition() + r, v);  // contact point X force direction
            v.resize(6);
            v(3) = v2(0); v(4)=v2(1); v(5)=v2(2);
            A.setCol(colInd++, v);
        }else{                                              // 3 UNKNOWNS: FORCE
            Matrix m(6,3); m.zero();
            m(0,0) = 1; m(1,1) = 1; m(2,2) = 1; 
            Vector pos = R*it->getPosition() + r;
            Matrix m2 = crossProductMatrix(pos);
            m.setRow(3, m2.getRow(0));
            m.setRow(4, m2.getRow(1));
            m.setRow(5, m2.getRow(2));
            A.setCol(colInd++, m.getCol(0));
            A.setCol(colInd++, m.getCol(1));
            A.setCol(colInd++, m.getCol(2));
            
            if(!it->isMomentKnown()){                       // 6 UNKNOWNS: FORCE AND MOMENT
                m.zero();
                m(3,0)=1; m(4,1)=1; m(5,2)=1;
                A.setCol(colInd++, m.getCol(0));
                A.setCol(colInd++, m.getCol(1));
                A.setCol(colInd++, m.getCol(2));
            }
        }
    }
    return A;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContactSolver::buildB(unsigned int firstContactLink, unsigned int lastContactLink){
    Matrix H, R;
    Vector r;

    // Initialize the force part of the B vector (first 3 components) as:
    //    * minus the force applied on the first link
    //    * plus the force exchanged by the last link on the next one
    Matrix Hlast = getHFromAtoB(firstContactLink-1, lastContactLink);
    Matrix Rlast = Hlast.submatrix(0,2,0,2);
    Vector rLast = Hlast.submatrix(0,2,3,3).getCol(0);
    Vector Bforce = Rlast * chain->getForce(lastContactLink) - chain->getForce(firstContactLink-1);    
    /*if(verbose){
        printMatrix("H from <i-1> to <j>", Hlast);
        printVector("Last link out force", Rlast*chain->getForce(lastContactLink));
        printVector("First link in force", chain->getForce(firstContactLink-1));
    }*/
    // For each link add the mass multiplied by the linear accelleration of the COM
    for(unsigned int i=firstContactLink; i<=lastContactLink; i++){
        H = getHFromAtoB(firstContactLink-1, i);
        R = H.submatrix(0,2,0,2);
        Bforce = Bforce + chain->getMass(i) * R * chain->getLinAccCOM(i);
        /*if(verbose){
            printVector("Grav force", chain->getMass(i) * R * chain->getLinAccCOM(i));
        }*/
    }

    // initialize the moment part (computed w.r.t. the begin of the first link) of the B vector (last 3 components) as:
    //  * the moment exerted by the last link on the next one
    //  * minus the moment applied on the first link
    //  * the displacement between the beginning of the first link and the end of the last link, 
    //    vector product the force exerted by the last link on the next one
    Vector Bmoment = Rlast * chain->getMoment(lastContactLink) - chain->getMoment(firstContactLink-1);    
    Bmoment = Bmoment + cross(rLast, Rlast* chain->getForce(lastContactLink));    
    // Then for each link add:
    //  * the gravitational contribution
    //  * the inertia contribution
    //  * the Coriolì contribution
    iDynLink* link;
    for(unsigned int i=firstContactLink; i<=lastContactLink; i++){
        H = getHFromAtoB(firstContactLink-1, i);
        H = H * chain->refLink(i)->getCOM();
        R = H.submatrix(0,2,0,2);
        r = H.submatrix(0,2,3,3).getCol(0); // vector from <firstContactLink-1> to COM of i
        link = chain->refLink(i);
        Bmoment = Bmoment + link->getMass() * cross(r, R * link->getLinAccC());
        Bmoment = Bmoment + R * (link->getInertia() * link->getdW() +
                                 cross(link->getW(), link->getInertia()*link->getW()));
        /*if(verbose){
            printVector("Out moment (zero)", Rlast * chain->getMoment(lastContactLink));
            printVector("In moment", -1.0*chain->getMoment(firstContactLink-1));
            printVector("Out force moment (zero)", cross(rLast, Rlast* chain->getForce(lastContactLink)));
            printVector("Grav moment", link->getMass() * cross(r, R * link->getLinAccC()));
            printVector("Grav force m*a", link->getMass() * R * link->getLinAccC());
            printVector("Inertia moment", R * (link->getInertia() * link->getdW()));
            printVector("Coriolli moment", R * cross(link->getW(), link->getInertia()*link->getW()));
        }*/
    }

    Vector B(Bforce);
    B.push_back(Bmoment(0));
    B.push_back(Bmoment(1));
    B.push_back(Bmoment(2));
    return B;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
deque<iDynContact> iDynContactSolver::getContactList() const{
    return contactList;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContactSolver::findContactSubChain(unsigned int &firstLink, unsigned int &lastLink){
    deque<iDynContact>::const_iterator it=contactList.begin();
    firstLink = chain->getN()+1; 
    lastLink = 0;

    for(; it!=contactList.end(); it++){
        unsigned int l = it->getLinkNumber();
        if(l>lastLink)
            lastLink = l;
        if(l<firstLink)
            firstLink = l;
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynContactSolver::crossProductMatrix(const Vector &v){
    Matrix res(3,3); res.zero();
    res(1,0) = v(2); 
    res(0,1) = -v(2); 
    res(2,0) = -v(1); 
    res(0,2) = v(1);
    res(2,1) = v(0); 
    res(1,2) = -v(0);
    return res;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynContactSolver::getHFromAtoB(unsigned int a, unsigned int b){
    if(a>b){
        Matrix Hinv = getHFromAtoB(b, a);
        return SE3inv(Hinv);
    }
    if(a==b){
        Matrix identity(4,4);
        return identity.eye();
    }
    Matrix H = chain->refLink(a+1)->getH(true);     // initialize H with the rototranslation from <a> to <a+1>
    for(unsigned int i=a+2; i<=b; i++){
        H = H * chain->refLink(i)->getH(true);
    }
    return H;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContactSolver::projectContact2Root(const iDynContact &c){
	Vector wrench = c.getForceMoment();
	Matrix H_02L = getHFromAtoB(0, c.getLinkNumber());	// rototraslation from 0 to contact link
	Matrix H_r20 = chain->getH0();	// rototraslation from root to 0
	Matrix R_r2L = H_r20.submatrix(0,2,0,2) * H_02L.submatrix(0,2,0,2);	// rotation from root to contact link
	return R_r2L*wrench;	// project the wrench from the link frame to the root frame
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynContactSolver::getUnknownNumber() const{
    deque<iDynContact>::const_iterator it=contactList.begin();
    unsigned int unknowns=0;

    for(; it!=contactList.end(); it++){        
        if(it->isMomentKnown()){
            if(it->isForceDirectionKnown()){
                unknowns++;     // 1 unknown (force module)
            }else{
                unknowns+=3;    // 3 unknowns (force)
            }
        }else{
            unknowns+=6;        // 6 unknowns (force and moment)
        }
    }
    return unknowns;
}
