Notes
=====

The calibrations of the following parts have been created from CAD models:
- `left_leg_lower.txt`
- `left_leg_upper.txt`
- `right_leg_lower.txt`
- `right_leg_upper.txt`

## Code
The code is available at: https://gitlab.fel.cvut.cz/body-schema/icub/icub-skin-calibration/-/tree/main/skin_models.
- `CStransformations_*.npy` and `CSlabels_*.npy` files contain transformations to triangle centers extracted from CAD files and corresponding triangle IDs, respectively.
- `skin_from_CS.py` uses these files and .ini files from `skinGui` to generate taxel position files (visualization included).
- `read_calib.py` is a script for visualization.
- The `calibrations` folder contains the position files.