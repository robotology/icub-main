// include header
#include <Tracking.h>

// namespaces
using namespace thesis::gui;
using namespace thesis::tools;
using namespace thesis::tracking;
using namespace thesis::mathematics;
using namespace thesis::imageprocessing;
using namespace thesis::controlsprocessing;
using namespace thesis::imageprocessing::imgproc_helpers;

using std::vector;

// ***************************************************************************

/**
 *
 * Implementation of PatchTracker class
 *
 */

// Init constructor 

PatchTracker::PatchTracker(ImageOf<PixelMono> *_img_ptr, Decider *_dModule_ptr, Connector *_cModule_ptr, MICube *_mIModule_ptr, int _timewindow) {
	
	//printf("Start:\t[PatchTracker]\n");
	// Declarations & Initializing
	this->frame_width		= _img_ptr->width();
	this->frame_height		= _img_ptr->height();
	this->decider_ptr		= _dModule_ptr;
	this->connector_ptr		= _cModule_ptr;
	this->motorInfo_ptr		= _mIModule_ptr;
	this->ready				= false;
	this->counter			= 0;
	this->dimNum			= _timewindow;
	this->time				= 0;
	this->initTime			= 0;

	vector<patchElement> patchesVector;
	this->trackingPatches_vec = patchesVector;
}

// Destructor
PatchTracker::~PatchTracker() {
	//printf("Quit:\t[PatchTracker]\n");
}

bool PatchTracker::reset() {
	this->ready		= false;
	this->counter	= 0;
	this->time		= 0;
	this->initTime	= 0;
	/*	int j;
	for (j=0; j<this->dataNum; j++) {
		this->disqualifyPatch(j);
	}*/
	this->trackingPatches_vec.clear();
	this->dataNum	= 0;

	return true;
}

bool PatchTracker::paramChange(int _timewindow, int _clusternumb) {
	this->reset();
	//printf("before %d\n", this->dimNum);
	this->dimNum = _timewindow;
	//printf("after %d\n", this->dimNum);
	this->clusterNum = _clusternumb;

	return true;
}


bool PatchTracker::isReady() {
	return this->ready;
}

void PatchTracker::updatePatches(ImageOf<PixelMono> *_img_ptr) {
	this->tmp_mono_img_ptr = _img_ptr;
	this->time++;
	if (this->isReady()) {
		this->trackPatches();
	}
	else {
		if (this->decider_ptr->decide("tracking")) {
			this->ready = true;
			this->determinePatches();
		} // end trigger if
	}
}

void PatchTracker::determinePatches() {
	Gui g;

	int len1;
	dPoint2D pos;
	dPoint2D val;

	this->initTime		= this->time;
	this->patches_ptr	= this->connector_ptr->flow_ptr->getRelevantPatches();
	len1				= this->patches_ptr->patch_id.x.size();
	this->nofPatches	= len1;

	for (int i = 0; i < len1; i ++) {
		pos.x = (double)UNPATCHEDPOS(this->patches_ptr->patch_id.x.get(i).asDouble());
		pos.y = (double)UNPATCHEDPOS(this->patches_ptr->patch_id.y.get(i).asDouble());
		this->addPatch(pos);
		val.x = pos.x + this->patches_ptr->vector_value.x.get(i).asDouble();
		val.y = pos.y + this->patches_ptr->vector_value.y.get(i).asDouble();
		this->addPosition(i, val);
		//addRectangleOutline(*this->tmp_mono_img_ptr, g.c_m.grey, pos.x, pos.y, ROUNDTOINT(PATCH_SIZE/2), ROUNDTOINT(PATCH_SIZE/2));
		addRectangleOutline(*this->tmp_mono_img_ptr, g.c_m.grey, (int)val.x, (int)val.y, ROUNDTOINT(PATCH_SIZE/2), ROUNDTOINT(PATCH_SIZE/2));
	}
}

void PatchTracker::trackPatches() {

	Gui g;
	iPoint2D		p;
	dPoint2D		q;
	CvPoint2D32f	*thisGoodFeatures_ptr	= this->connector_ptr->flow_ptr->getThisFeatures();
	CvPoint2D32f	*nextGoodFeatures_ptr	= this->connector_ptr->flow_ptr->getNextFeatures();
	const char		*error_ptr				= this->connector_ptr->flow_ptr->getError();
	dPoint2D		**lookUp_ptr			= new dPoint2D* [this->frame_height];
	
	// EVERY field MUST be set to zero, because it is not necessary that every field will be visited.
    int j=0;
    int i=0;
	for (j=0; j < this->frame_height; j++) {
		lookUp_ptr[j] = new dPoint2D [this->frame_width];
		for (i = 0; i < this->frame_width; i++) {
			lookUp_ptr[j][i].x = 0.0;
			lookUp_ptr[j][i].y = 0.0;
		}
	}

	if (thisGoodFeatures_ptr != NULL) {
		for (i = 0; i < NOF_FEATURES_OPTFLOW; i++) {
			if (error_ptr[i] == 0) continue;

			p.x = ROUNDTOINT(thisGoodFeatures_ptr[i].x);
			p.y = ROUNDTOINT(thisGoodFeatures_ptr[i].y);
			q.x = nextGoodFeatures_ptr[i].x;
			q.y = nextGoodFeatures_ptr[i].y;

			if (( p.x < this->frame_width) & ( p.x >= 0)) {
				if (( p.y < this->frame_height) & ( p.y >= 0)) {
					lookUp_ptr[p.y][p.x] = q;
				}
			}
		} // end for NOF_FEATURES_OPTFLOW
	}
	//starting the actual tracking procedure
	Helpers h;
	Geometry m;
	Statistics s;

	iPoint2D	ipos, size, n_tmp;
	dPoint2D	dpos, diff, value, conf_interval_x, conf_interval_y, avg;

	int			offset, howmany;
	double		len;

	Bottle		distr_x;
	Bottle		distr_y;

	size.x		= this->frame_width;
	size.y		= this->frame_height;
	offset		= ROUNDTOINT(PATCH_SIZE/2);

	for (i = 0; i < this->getNumberOfPatches(); i++ ) {
		distr_x.clear();
		distr_y.clear();

		howmany		= 0;
		len			= 0.0;
		dpos		= this->getPosition(i);
		ipos.x		= ROUNDTOINT(dpos.x);
		ipos.y		= ROUNDTOINT(dpos.y);
		// test whether the position is within the range
		if (h.withinBorders(ipos, size)) {
			// travers the patch around the center pos (ipos)
			for (int r = (-1*offset); r < offset; r++) {
				n_tmp.y = ipos.y + r;
				for (int c = (-1*offset); c < offset; c++) {
					n_tmp.x = ipos.x + c;
					if ((n_tmp.y == ipos.y) & (n_tmp.x == ipos.x)) {
					}
					else {
						if (h.withinBorders(n_tmp, size)) {
							value = lookUp_ptr[n_tmp.y][n_tmp.x];
							if ((value.x > 0.0) & (value.y > 0.0)) {
								diff.x = 0.0;
								diff.y = 0.0;
								diff.x = value.x - (double)n_tmp.x;
								diff.y = value.y - (double)n_tmp.y;
								len = sqrt(diff.x * diff.y + diff.y * diff.y);
								if (len > 0.0) {							
									howmany++;
									distr_x.addDouble(diff.x);
									distr_y.addDouble(diff.y);
								}
							}
						}
					} // end else (centre)
				} // end for c (columns)
			} // end for r (rows)
		} // end if 
		avg.x = 0.0;
		avg.y = 0.0;
		if (howmany > 0 ) {
			conf_interval_x = s.confidenceIntervalND(&distr_x, 90);
			conf_interval_y = s.confidenceIntervalND(&distr_y, 90);
			howmany = 0;
			for (int b=0; b < distr_x.size(); b++) {
				diff.x = distr_x.get(b).asDouble();
				diff.y = distr_y.get(b).asDouble();
				if (((diff.x >= conf_interval_x.x) & (diff.x <= conf_interval_x.y)) & ((diff.y >= conf_interval_y.x) & (diff.y <= conf_interval_y.y))) {
					avg.x = avg.x + diff.x;
					avg.y = avg.y + diff.y;
					howmany++;
				}
			} // end for b
			if (howmany > 0) {
				avg.x = avg.x / howmany;
				avg.y = avg.y / howmany;
			}
		}
		// store the new value into the correspondend patch i
		avg.x = dpos.x + avg.x;
		avg.y = dpos.y + avg.y;
		this->addPosition(i, avg);

		// new!!! tonight (sunday)
		if (this->isPartOfRobot(i)) {
			//printf("yes %d @ %d", i, this->time);
			addCircleOutline(*this->tmp_mono_img_ptr, g.c_m.white, ROUNDTOINT(avg.x), ROUNDTOINT(avg.y), ROUNDTOINT(PATCH_SIZE/2));
		}
		else {
			addRectangleOutline(*this->tmp_mono_img_ptr, g.c_m.black, ROUNDTOINT(avg.x), ROUNDTOINT(avg.y), ROUNDTOINT(PATCH_SIZE/2), ROUNDTOINT(PATCH_SIZE/2));
		}
	} // end for patches

	// deleting allocated mem
	for (j= 0; j<this->frame_height; j++) {
		delete [] lookUp_ptr[j];
	}
	delete [] lookUp_ptr;
}

int PatchTracker::getNumberOfPatches() {
	return this->nofPatches;
}

void PatchTracker::addPatch(dPoint2D _pos) {
	patchElement element;

	element.id = (int)this->counter;
	element.pos.push_back(_pos);
	element.clusterNr = -1;
	element.partOfJoint = -1;
	element.partOfRobot = false;
	element.partOfClustering = false;
	this->trackingPatches_vec.push_back(element);
	this->counter++;
}

dPoint2D PatchTracker::getPosition(int _id) {
	//printf("getPosition\n");
	dPoint2D ret;
	int last = 0;
	int nr = this->trackingPatches_vec[_id].id;
	int size = this->trackingPatches_vec[_id].pos.size();

	if (nr == _id) {
		ret = this->trackingPatches_vec[_id].pos.back();
	}
	else {
		printf("Error: getPostion\n");
		exit(-99);
	}
	return ret;
}

void PatchTracker::addPosition(int _id, dPoint2D _pos) {
	patchElement *el;
	el = &this->trackingPatches_vec[_id];
	if (el->id == _id) {
		el->pos.push_back(_pos);
	}
	else {
		printf("%d not equal to  %d \n", _id, el->id);
	}	
}

bool PatchTracker::isPartOfRobot(int _patchNumber) {
	if (_patchNumber < this->getNumberOfPatches()) {
		return this->trackingPatches_vec[_patchNumber].partOfRobot;
	}
	else {
		return false;
	}
}

int PatchTracker::getTime() {
	return this->time;
}

void PatchTracker::qualifyPatch(int _patchNumber) {
	this->trackingPatches_vec[_patchNumber].partOfRobot = true;
}

void PatchTracker::disqualifyPatch(int _patchNumber) {
	this->trackingPatches_vec[_patchNumber].partOfRobot = false;
	this->trackingPatches_vec[_patchNumber].partOfClustering = false;
}

void PatchTracker::qualifyPatchForClustering(int _patchNumber) {
	this->trackingPatches_vec[_patchNumber].partOfClustering = true;
}

bool PatchTracker::isPatchToCluster(int _patchNumber) {
	return this->trackingPatches_vec[_patchNumber].partOfClustering;
}

bool PatchTracker::prepareValuesForClustering(double **_visualvalues_ptr, bool *_bool_arr) {
	int count;
	double		len, x_tmp, y_tmp, std_j;
	Statistics	stats;
	dPoint2D	firstvalue, el;

	count = 0;
	for ( int j=0; j < this->dataNum; j++ ) {
		firstvalue.x	= this->trackingPatches_vec[j].pos[0].x;
		firstvalue.y	= this->trackingPatches_vec[j].pos[0].y;
		for ( int i=0; i < this->dimNum; i++ ) {
			el		= this->trackingPatches_vec[j].pos[i];
			x_tmp	= fabs(el.x - firstvalue.x);
			y_tmp	= fabs(el.y - firstvalue.y);
			len		= sqrt(x_tmp*x_tmp + y_tmp*y_tmp);
			_visualvalues_ptr[j][i] = len;
		}
		std_j = stats.standardDeviationND(_visualvalues_ptr[j], this->dimNum);
		if (std_j <= 0.01) {
			count++;
			_bool_arr[j] = false;
		}
		else {
			_bool_arr[j] = true;
			this->qualifyPatchForClustering(j);
			this->patchIDCluster.addInt(j);
		}
	}
	if (count > 0) {
		return true;
	}
	else {
		return false;
	}
}

bool PatchTracker::prepareMotorValuesForClustering(double **_motorvalues_ptr, bool *_bool_arr) {
	//printf("get motor values\n");
	int count;
	double		x_tmp, std_j, firstvalue, el;
	Statistics	stats;
	double **mi_ptr = this->motorInfo_ptr->getAccessToMotorInformationValues();

	count = 0;
	int zero = this->initTime;
	//printf("init time: %d\n", this->initTime);
	for ( int j=0; j < this->nofJoints; j++ ) {
		firstvalue	= mi_ptr[j][zero];
		for ( int i=0; i < this->dimNum; i++ ) {
			el		= mi_ptr[j][zero+i];
			x_tmp	= fabs(el - firstvalue);
			_motorvalues_ptr[j][i] = x_tmp;
		}
		std_j = stats.standardDeviationND(_motorvalues_ptr[j], this->dimNum);
		//printf("std_j = %f", std_j);
		if (std_j <= 0.01) {
			count++;
			_bool_arr[j] = false;
		}
		else {
			_bool_arr[j] = true;
		}
	}
	if (count > 0) {
		return true;
	}
	else {
		return false;
	}
}

void PatchTracker::sortOut() {
	int			i, j;
	bool		change, change2;
	Statistics	stats;
	Cluster		clusteringModule;

	this->dataNum		= this->getNumberOfPatches();
	this->nofJoints		= this->motorInfo_ptr->getNofJoints();
	this->clusterNum	= CLUSTERNUMBER;
	//init arrays
	double		**visualvalues_ptr	= new double*	[this->dataNum];
	double		**motorvalues_ptr	= new double*	[this->nofJoints];
	bool		*boolArr_ptr		= new bool		[this->dataNum];
	bool		*boolArr2_ptr		= new bool		[this->nofJoints];

	// loop for how many patches
	for ( j=0; j<this->dataNum; j++ ) { 
		visualvalues_ptr[j]			= new double	[this->dimNum];
		boolArr_ptr[j]				= false;
	}
	for ( j=0; j<this->nofJoints; j++ ) {
		motorvalues_ptr[j]			= new double	[this->dimNum];
		boolArr2_ptr[j]				= false;
	}

	//printf("%d x %d, %d, %d\n", this->dataNum, this->dimNum, this->nofJoints, this->clusterNum);
	change		= this->prepareValuesForClustering(visualvalues_ptr, boolArr_ptr);
	change2		= this->prepareMotorValuesForClustering(motorvalues_ptr, boolArr2_ptr);

	// change and change2 tell us if there has been a patch or a joint with std <= 0.01
	// if true, then all id are kept with std > 0.01
	Bottle val_ids, mot_ids;
	val_ids.clear();
	mot_ids.clear();
	if (change) {
		for (j=0; j<this->dataNum; j++) {
			if (boolArr_ptr[j]) {
				val_ids.addInt(j);
			}
			else {
			
			}

		}
	} 
	if (change2) {
		for (j=0; j<this->nofJoints; j++) {
			if (boolArr2_ptr[j]) {
				mot_ids.addInt(j);
			}
		}
	}
	int bottleindex1 = 0;
	int bottleindex2 = 0;
	int vv_len1, mv_len2;
	
	if (val_ids.size() > 0) {
		vv_len1 = val_ids.size();
	}
	else {
		vv_len1 = this->dataNum;
	}
	if (mot_ids.size() > 0) {
		mv_len2 = mot_ids.size();
	}
	else {
		mv_len2 = this->nofJoints;
	}
	this->totalNum = vv_len1 + mv_len2;

	double **clusteringvalues_ptr	= new double*		[this->totalNum];
	for (j=0; j<this->totalNum; j++) {
		clusteringvalues_ptr[j]		= new double		[this->dimNum];
		if (j<(this->dataNum - val_ids.size())) {
			for (i=0; i<this->dimNum; i++) {
				if (change) {
					clusteringvalues_ptr[j][i] = visualvalues_ptr[val_ids.get(bottleindex1).asInt()][i];
				}
				else {
					clusteringvalues_ptr[j][i] = visualvalues_ptr[j][i];
				}
			}
			bottleindex1++;
		}
		else {
			for (i=0; i<this->dimNum; i++) {
				if (change2) {
					clusteringvalues_ptr[j][i] = motorvalues_ptr[mot_ids.get(bottleindex2).asInt()][i];
				}
				else {
					clusteringvalues_ptr[j][i] = motorvalues_ptr[j][i];
				}
			}
			bottleindex2++;
		}
	}

	int			*label_ptr			= new int		[this->totalNum];
	double		**clusterCenter_ptr = new double*	[clusterNum];
	for ( i=0; i < this->clusterNum; i++ ) {
		clusterCenter_ptr[i]		= new double	[this->dimNum];
		for (int k=0; k<this->dimNum; k++) {
			clusterCenter_ptr[i][k] = 0.0;
		}
	}

	clusteringModule.kMeans(clusteringvalues_ptr, "distance correlation", clusterNum, this->totalNum, dimNum, 0.0000001, 10, label_ptr, clusterCenter_ptr);
	// determine the patches belonging to the robot
	int int_tmp, lab_nr;
	//dPoint2D avg;
	//Gui g;
	for (int k=0; k < mot_ids.size(); k++) {
		lab_nr = label_ptr[this->totalNum - mot_ids.size() + k];
		for (i = 0; i<this->patchIDCluster.size(); i++) {
			int_tmp = this->patchIDCluster.get(i).asInt();
			if (label_ptr[i] == lab_nr) {
				this->qualifyPatch(int_tmp);
				//printf("qualified patch %d\n", int_tmp);
				//avg = getPosition(int_tmp);
				//addCircleOutline(*this->tmp_mono_img_ptr, g.c_m.white, ROUNDTOINT(avg.x), ROUNDTOINT(avg.y), ROUNDTOINT(PATCH_SIZE/2));
			}
		}
	}

	// free allocated memory
	delete [] label_ptr;
	delete [] boolArr_ptr;
	delete [] boolArr2_ptr;

	for ( i=0; i < this->clusterNum; i++ ) {
		delete [] clusterCenter_ptr[i];
	}
	delete [] clusterCenter_ptr;

	for (i=0; i< this->nofJoints; i++) {
		delete [] motorvalues_ptr[i];
	}
	delete [] motorvalues_ptr;

	for ( i=0; i < this->dataNum; i++ ) {
		delete [] visualvalues_ptr[i];
	}
	delete [] visualvalues_ptr;

	for (i=0; i<this->totalNum; i++) {
		delete [] clusteringvalues_ptr[i];
	}
	delete [] clusteringvalues_ptr;
}

ImageOf<PixelMono>* PatchTracker::getTmp() {
	return this->tmp_mono_img_ptr;
}
