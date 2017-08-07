#ifndef __RPC_SKIN_MANAGER_H__
#define __RPC_SKIN_MANAGER_H__

#include <string>

namespace iCub{

namespace skinManager{

// Enum containing all the commands accepted by the rpc port of the SkinManager.
// The last element of the enum (SkinManagerCommandSize) represents the total number of commands accepted by the module.
typedef enum { 
    calibrate,          get_touch_thr,
    set_binarization,   get_binarization, 
    set_smooth_filter,  get_smooth_filter,  set_smooth_factor, 
    get_smooth_factor,  set_threshold,      get_threshold, 
    set_gain,           get_gain,           set_cont_gain,           
    get_cont_gain,      is_calibrating,     get_info,
    get_pose,           set_pose,
    get_position,       set_position,
    get_confidence,
    get_max_neigh_dist, set_max_neigh_dist, get_skin_parts,     
    enable_skin_part,   disable_skin_part,  is_skin_enabled,
    help,               quit,               
    SkinManagerCommandSize} SkinManagerCommand;
    
// Enum containing the responses to the the set commands
typedef enum{
    skin_manager_ok,
    skin_manager_error
} SkinManagerResponse;

// the order of the command in this list MUST correspond to the order of the enum SkinManagerCommand
const std::string SkinManagerCommandList[]  = {
    "calib",                "get touch thr",
    "set binarization",     "get binarization", 
    "set smooth filter",    "get smooth filter",    "set smooth factor", 
    "get smooth factor",    "set threshold",        "get threshold", 
    "set gain",             "get gain",             "set contact gain",
    "get contact gain",     "is calibrating",       "get info",      
    "get pose",             "set pose",
    "get position",         "set position",
    "get confidence",
    "get neigh dist",       "set neigh dist",       "get skin",             
    "enable skin",          "disable skin",         "is skin enabled",
    "help",                 "quit"};

// the order in SkinManagerCommandDesc must correspond to the order in SkinManagerCommandList
const std::string SkinManagerCommandDesc[]  = {
    "calibrate the skin, i.e. reset the baseline (for 5 sec no touch should occur)", 
    "get touch thresholds (i.e. 95 percentile)", 
    "enable or disable the binarization filter (255 touch, 0 no touch)",
    "get the binarization filter state (on, off)",
    "enable or disable the smooth filter",
    "get the smooth filter state (on, off)",
    "set the value of the smooth factor (in [0,1])",
    "get the smooth factor value",
    "set the safety threshold that is added to the touch thresholds (int in [0, 254])",
    "get the safety threshold that is added to the touch threshold",
    "set the compensation gain", 
    "get the compensation gain",
    "set the contact compensation gain", 
    "get the contact compensation gain",
    "tell whether the skin calibration is in progress",
    "get information about the module",
    "get taxel pose(s) with input params: skin part, taxel index (if taxel index is not specified return all taxel positions)",
    "set taxel pose(s) with input params: skin part, taxel index, pose(s) (if taxel index is not specified set all taxel positions)",
    "get taxel position(s) with input params: skin part, taxel index (if taxel index is not specified return all taxel positions)",
    "set taxel position(s) with input params: skin part, taxel index, pose(s) (if taxel index is not specified set all taxel positions)",
    "get taxel pose confidence with input params: skin part, taxel index",
    "get the max neighbor distance",
    "set the max neighbor distance",
    "get the list of the skin parts",
    "enable the specified skin part",
    "disable the specified skin part",
    "check whether the specified skin part is enabled",
    "get this list", 
    "quit the module"};

static const double MAX_NEIGHBOR_DISTANCE = 0.012;

}

}

#endif

