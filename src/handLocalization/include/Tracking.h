#ifndef _TRACKING_
#define _TRACKING_

#include <includes.h>
#include <Gui.h>
#include <Tools.h>
#include <Mathematics.h>
#include <ImageProcessing.h>
#include <ControlsProcessing.h>

#include <vector>
// namespaces 
namespace thesis {
    /**
     * Motor Controls Processing.
     */
    namespace tracking {
		class PatchTracker;
    }
}

// ***************************************************************************
// ***************         namespace thesis::tracking          ***************
// ***************************************************************************

// ***************************************************************************

/**
 *  Class PatchTracker: definition
 */
class thesis::tracking::PatchTracker {
protected:
	// Declaration of hidden class variables for the storage part
	int		counter, nofPatches, nofJoints;
	struct	patchElement {
		int id;
		int clusterNr, partOfJoint;
		bool partOfRobot, partOfClustering;
		std::vector<dPoint2D> pos;
	};

	// Declaration of hidden class variables for the tracking part
	bool	ready;
	int		frame_width, frame_height, cols, rows, dataNum, clusterNum, dimNum, totalNum, time, initTime;
	Bottle 	patchIDCluster;

	patchInfo							*patches_ptr;
	ImageOf<PixelMono>					*tmp_mono_img_ptr;
	ImageOf<PixelRgb>					tmp2_rgb_img;
	thesis::tools::Decider				*decider_ptr;
	thesis::imageprocessing::Connector	*connector_ptr;
	thesis::controlsprocessing::MICube	*motorInfo_ptr;

	std::vector<patchElement>			trackingPatches_vec;
	
	// Declaration of hidden  methods
	void addPatch(dPoint2D _pos);
	void addPosition(int _id, dPoint2D _pos);

	dPoint2D getPosition(int _id);

	// methods for tracking
	void trackPatches();
	void determinePatches();

	// methods for clustering
	bool prepareValuesForClustering(double **_visualvalues_ptr, bool *_bool_arr);
	bool prepareMotorValuesForClustering(double **_motorvalues_ptr, bool *_boolArr_ptr);
	// identification
	void qualifyPatch(int _patchNumber);
	void disqualifyPatch(int _patchNumber);
	void qualifyPatchForClustering(int _patchNumber);
	void drawClusters(double **_visualvalues_ptr, int *_label_ptr);
	void drawMotorCorrenspondingClusters(double **_visualvalues_ptr, int *_label_ptr, Bottle *_motorvalues_ptr);
	bool isPatchToCluster(int _patchNumber);
	bool isPartOfRobot(int _patchNumber);

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	PatchTracker (ImageOf<PixelMono> *_img_ptr, thesis::tools::Decider *_dModule_ptr, thesis::imageprocessing::Connector *_cModule_ptr, thesis::controlsprocessing::MICube *_mIModule_ptr, int _timewindow);
	~PatchTracker ();

	bool reset();
	bool paramChange(int _timewindow, int _clusternumb);
	// methods for storing & handling the patch information
	bool isReady();
	int getNumberOfPatches();

	// methods for tracking
	void updatePatches(ImageOf<PixelMono> *_img_ptr);
	
	// methods for clustering
	void sortOut();
	int getTime();

	ImageOf<PixelMono>* getTmp();
	ImageOf<PixelRgb>* getClusters();

};


#endif
