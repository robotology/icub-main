% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 267.702079658425700 ; 271.860209772795770 ];

%-- Principal point:
cc = [ 152.470420424151060 ; 127.145089665918730 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.396577135703983 ; 0.152337895808908 ; -0.006987705912976 ; 0.005684134134953 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.445748666291359 ; 4.597096470263632 ];

%-- Principal point uncertainty:
cc_error = [ 3.455582966330412 ; 3.701586894989490 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.023134023213832 ; 0.035539316511668 ; 0.002624483958390 ; 0.002034335995654 ; 0.000000000000000 ];

%-- Image size:
nx = 320;
ny = 240;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 1406;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.046873e+000 ; -2.313273e+000 ; 9.181958e-002 ];
Tc_1  = [ -8.905635e+001 ; -1.340604e+002 ; 2.771063e+002 ];
omc_error_1 = [ 1.790904e-002 ; 1.764282e-002 ; 3.814938e-002 ];
Tc_error_1  = [ 3.752782e+000 ; 3.883384e+000 ; 5.838029e+000 ];

%-- Image #2:
omc_2 = [ NaN ; NaN ; NaN ];
Tc_2  = [ NaN ; NaN ; NaN ];
omc_error_2 = [ NaN ; NaN ; NaN ];
Tc_error_2  = [ NaN ; NaN ; NaN ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ NaN ; NaN ; NaN ];
Tc_4  = [ NaN ; NaN ; NaN ];
omc_error_4 = [ NaN ; NaN ; NaN ];
Tc_error_4  = [ NaN ; NaN ; NaN ];

%-- Image #5:
omc_5 = [ NaN ; NaN ; NaN ];
Tc_5  = [ NaN ; NaN ; NaN ];
omc_error_5 = [ NaN ; NaN ; NaN ];
Tc_error_5  = [ NaN ; NaN ; NaN ];

%-- Image #6:
omc_6 = [ NaN ; NaN ; NaN ];
Tc_6  = [ NaN ; NaN ; NaN ];
omc_error_6 = [ NaN ; NaN ; NaN ];
Tc_error_6  = [ NaN ; NaN ; NaN ];

%-- Image #7:
omc_7 = [ NaN ; NaN ; NaN ];
Tc_7  = [ NaN ; NaN ; NaN ];
omc_error_7 = [ NaN ; NaN ; NaN ];
Tc_error_7  = [ NaN ; NaN ; NaN ];

%-- Image #8:
omc_8 = [ NaN ; NaN ; NaN ];
Tc_8  = [ NaN ; NaN ; NaN ];
omc_error_8 = [ NaN ; NaN ; NaN ];
Tc_error_8  = [ NaN ; NaN ; NaN ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ NaN ; NaN ; NaN ];
Tc_10  = [ NaN ; NaN ; NaN ];
omc_error_10 = [ NaN ; NaN ; NaN ];
Tc_error_10  = [ NaN ; NaN ; NaN ];

%-- Image #11:
omc_11 = [ NaN ; NaN ; NaN ];
Tc_11  = [ NaN ; NaN ; NaN ];
omc_error_11 = [ NaN ; NaN ; NaN ];
Tc_error_11  = [ NaN ; NaN ; NaN ];

%-- Image #12:
omc_12 = [ NaN ; NaN ; NaN ];
Tc_12  = [ NaN ; NaN ; NaN ];
omc_error_12 = [ NaN ; NaN ; NaN ];
Tc_error_12  = [ NaN ; NaN ; NaN ];

%-- Image #13:
omc_13 = [ NaN ; NaN ; NaN ];
Tc_13  = [ NaN ; NaN ; NaN ];
omc_error_13 = [ NaN ; NaN ; NaN ];
Tc_error_13  = [ NaN ; NaN ; NaN ];

%-- Image #14:
omc_14 = [ NaN ; NaN ; NaN ];
Tc_14  = [ NaN ; NaN ; NaN ];
omc_error_14 = [ NaN ; NaN ; NaN ];
Tc_error_14  = [ NaN ; NaN ; NaN ];

%-- Image #15:
omc_15 = [ NaN ; NaN ; NaN ];
Tc_15  = [ NaN ; NaN ; NaN ];
omc_error_15 = [ NaN ; NaN ; NaN ];
Tc_error_15  = [ NaN ; NaN ; NaN ];

%-- Image #16:
omc_16 = [ NaN ; NaN ; NaN ];
Tc_16  = [ NaN ; NaN ; NaN ];
omc_error_16 = [ NaN ; NaN ; NaN ];
Tc_error_16  = [ NaN ; NaN ; NaN ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ NaN ; NaN ; NaN ];
Tc_19  = [ NaN ; NaN ; NaN ];
omc_error_19 = [ NaN ; NaN ; NaN ];
Tc_error_19  = [ NaN ; NaN ; NaN ];

%-- Image #20:
omc_20 = [ NaN ; NaN ; NaN ];
Tc_20  = [ NaN ; NaN ; NaN ];
omc_error_20 = [ NaN ; NaN ; NaN ];
Tc_error_20  = [ NaN ; NaN ; NaN ];

%-- Image #21:
omc_21 = [ NaN ; NaN ; NaN ];
Tc_21  = [ NaN ; NaN ; NaN ];
omc_error_21 = [ NaN ; NaN ; NaN ];
Tc_error_21  = [ NaN ; NaN ; NaN ];

%-- Image #22:
omc_22 = [ NaN ; NaN ; NaN ];
Tc_22  = [ NaN ; NaN ; NaN ];
omc_error_22 = [ NaN ; NaN ; NaN ];
Tc_error_22  = [ NaN ; NaN ; NaN ];

%-- Image #23:
omc_23 = [ NaN ; NaN ; NaN ];
Tc_23  = [ NaN ; NaN ; NaN ];
omc_error_23 = [ NaN ; NaN ; NaN ];
Tc_error_23  = [ NaN ; NaN ; NaN ];

%-- Image #24:
omc_24 = [ NaN ; NaN ; NaN ];
Tc_24  = [ NaN ; NaN ; NaN ];
omc_error_24 = [ NaN ; NaN ; NaN ];
Tc_error_24  = [ NaN ; NaN ; NaN ];

%-- Image #25:
omc_25 = [ NaN ; NaN ; NaN ];
Tc_25  = [ NaN ; NaN ; NaN ];
omc_error_25 = [ NaN ; NaN ; NaN ];
Tc_error_25  = [ NaN ; NaN ; NaN ];

%-- Image #26:
omc_26 = [ NaN ; NaN ; NaN ];
Tc_26  = [ NaN ; NaN ; NaN ];
omc_error_26 = [ NaN ; NaN ; NaN ];
Tc_error_26  = [ NaN ; NaN ; NaN ];

%-- Image #27:
omc_27 = [ NaN ; NaN ; NaN ];
Tc_27  = [ NaN ; NaN ; NaN ];
omc_error_27 = [ NaN ; NaN ; NaN ];
Tc_error_27  = [ NaN ; NaN ; NaN ];

%-- Image #28:
omc_28 = [ NaN ; NaN ; NaN ];
Tc_28  = [ NaN ; NaN ; NaN ];
omc_error_28 = [ NaN ; NaN ; NaN ];
Tc_error_28  = [ NaN ; NaN ; NaN ];

%-- Image #29:
omc_29 = [ NaN ; NaN ; NaN ];
Tc_29  = [ NaN ; NaN ; NaN ];
omc_error_29 = [ NaN ; NaN ; NaN ];
Tc_error_29  = [ NaN ; NaN ; NaN ];

%-- Image #30:
omc_30 = [ NaN ; NaN ; NaN ];
Tc_30  = [ NaN ; NaN ; NaN ];
omc_error_30 = [ NaN ; NaN ; NaN ];
Tc_error_30  = [ NaN ; NaN ; NaN ];

%-- Image #31:
omc_31 = [ NaN ; NaN ; NaN ];
Tc_31  = [ NaN ; NaN ; NaN ];
omc_error_31 = [ NaN ; NaN ; NaN ];
Tc_error_31  = [ NaN ; NaN ; NaN ];

%-- Image #32:
omc_32 = [ NaN ; NaN ; NaN ];
Tc_32  = [ NaN ; NaN ; NaN ];
omc_error_32 = [ NaN ; NaN ; NaN ];
Tc_error_32  = [ NaN ; NaN ; NaN ];

%-- Image #33:
omc_33 = [ NaN ; NaN ; NaN ];
Tc_33  = [ NaN ; NaN ; NaN ];
omc_error_33 = [ NaN ; NaN ; NaN ];
Tc_error_33  = [ NaN ; NaN ; NaN ];

%-- Image #34:
omc_34 = [ NaN ; NaN ; NaN ];
Tc_34  = [ NaN ; NaN ; NaN ];
omc_error_34 = [ NaN ; NaN ; NaN ];
Tc_error_34  = [ NaN ; NaN ; NaN ];

%-- Image #35:
omc_35 = [ NaN ; NaN ; NaN ];
Tc_35  = [ NaN ; NaN ; NaN ];
omc_error_35 = [ NaN ; NaN ; NaN ];
Tc_error_35  = [ NaN ; NaN ; NaN ];

%-- Image #36:
omc_36 = [ NaN ; NaN ; NaN ];
Tc_36  = [ NaN ; NaN ; NaN ];
omc_error_36 = [ NaN ; NaN ; NaN ];
Tc_error_36  = [ NaN ; NaN ; NaN ];

%-- Image #37:
omc_37 = [ NaN ; NaN ; NaN ];
Tc_37  = [ NaN ; NaN ; NaN ];
omc_error_37 = [ NaN ; NaN ; NaN ];
Tc_error_37  = [ NaN ; NaN ; NaN ];

%-- Image #38:
omc_38 = [ NaN ; NaN ; NaN ];
Tc_38  = [ NaN ; NaN ; NaN ];
omc_error_38 = [ NaN ; NaN ; NaN ];
Tc_error_38  = [ NaN ; NaN ; NaN ];

%-- Image #39:
omc_39 = [ NaN ; NaN ; NaN ];
Tc_39  = [ NaN ; NaN ; NaN ];
omc_error_39 = [ NaN ; NaN ; NaN ];
Tc_error_39  = [ NaN ; NaN ; NaN ];

%-- Image #40:
omc_40 = [ NaN ; NaN ; NaN ];
Tc_40  = [ NaN ; NaN ; NaN ];
omc_error_40 = [ NaN ; NaN ; NaN ];
Tc_error_40  = [ NaN ; NaN ; NaN ];

%-- Image #41:
omc_41 = [ NaN ; NaN ; NaN ];
Tc_41  = [ NaN ; NaN ; NaN ];
omc_error_41 = [ NaN ; NaN ; NaN ];
Tc_error_41  = [ NaN ; NaN ; NaN ];

%-- Image #42:
omc_42 = [ NaN ; NaN ; NaN ];
Tc_42  = [ NaN ; NaN ; NaN ];
omc_error_42 = [ NaN ; NaN ; NaN ];
Tc_error_42  = [ NaN ; NaN ; NaN ];

%-- Image #43:
omc_43 = [ NaN ; NaN ; NaN ];
Tc_43  = [ NaN ; NaN ; NaN ];
omc_error_43 = [ NaN ; NaN ; NaN ];
Tc_error_43  = [ NaN ; NaN ; NaN ];

%-- Image #44:
omc_44 = [ NaN ; NaN ; NaN ];
Tc_44  = [ NaN ; NaN ; NaN ];
omc_error_44 = [ NaN ; NaN ; NaN ];
Tc_error_44  = [ NaN ; NaN ; NaN ];

%-- Image #45:
omc_45 = [ NaN ; NaN ; NaN ];
Tc_45  = [ NaN ; NaN ; NaN ];
omc_error_45 = [ NaN ; NaN ; NaN ];
Tc_error_45  = [ NaN ; NaN ; NaN ];

%-- Image #46:
omc_46 = [ NaN ; NaN ; NaN ];
Tc_46  = [ NaN ; NaN ; NaN ];
omc_error_46 = [ NaN ; NaN ; NaN ];
Tc_error_46  = [ NaN ; NaN ; NaN ];

%-- Image #47:
omc_47 = [ NaN ; NaN ; NaN ];
Tc_47  = [ NaN ; NaN ; NaN ];
omc_error_47 = [ NaN ; NaN ; NaN ];
Tc_error_47  = [ NaN ; NaN ; NaN ];

%-- Image #48:
omc_48 = [ NaN ; NaN ; NaN ];
Tc_48  = [ NaN ; NaN ; NaN ];
omc_error_48 = [ NaN ; NaN ; NaN ];
Tc_error_48  = [ NaN ; NaN ; NaN ];

%-- Image #49:
omc_49 = [ NaN ; NaN ; NaN ];
Tc_49  = [ NaN ; NaN ; NaN ];
omc_error_49 = [ NaN ; NaN ; NaN ];
Tc_error_49  = [ NaN ; NaN ; NaN ];

%-- Image #50:
omc_50 = [ NaN ; NaN ; NaN ];
Tc_50  = [ NaN ; NaN ; NaN ];
omc_error_50 = [ NaN ; NaN ; NaN ];
Tc_error_50  = [ NaN ; NaN ; NaN ];

%-- Image #51:
omc_51 = [ NaN ; NaN ; NaN ];
Tc_51  = [ NaN ; NaN ; NaN ];
omc_error_51 = [ NaN ; NaN ; NaN ];
Tc_error_51  = [ NaN ; NaN ; NaN ];

%-- Image #52:
omc_52 = [ NaN ; NaN ; NaN ];
Tc_52  = [ NaN ; NaN ; NaN ];
omc_error_52 = [ NaN ; NaN ; NaN ];
Tc_error_52  = [ NaN ; NaN ; NaN ];

%-- Image #53:
omc_53 = [ NaN ; NaN ; NaN ];
Tc_53  = [ NaN ; NaN ; NaN ];
omc_error_53 = [ NaN ; NaN ; NaN ];
Tc_error_53  = [ NaN ; NaN ; NaN ];

%-- Image #54:
omc_54 = [ NaN ; NaN ; NaN ];
Tc_54  = [ NaN ; NaN ; NaN ];
omc_error_54 = [ NaN ; NaN ; NaN ];
Tc_error_54  = [ NaN ; NaN ; NaN ];

%-- Image #55:
omc_55 = [ NaN ; NaN ; NaN ];
Tc_55  = [ NaN ; NaN ; NaN ];
omc_error_55 = [ NaN ; NaN ; NaN ];
Tc_error_55  = [ NaN ; NaN ; NaN ];

%-- Image #56:
omc_56 = [ NaN ; NaN ; NaN ];
Tc_56  = [ NaN ; NaN ; NaN ];
omc_error_56 = [ NaN ; NaN ; NaN ];
Tc_error_56  = [ NaN ; NaN ; NaN ];

%-- Image #57:
omc_57 = [ NaN ; NaN ; NaN ];
Tc_57  = [ NaN ; NaN ; NaN ];
omc_error_57 = [ NaN ; NaN ; NaN ];
Tc_error_57  = [ NaN ; NaN ; NaN ];

%-- Image #58:
omc_58 = [ NaN ; NaN ; NaN ];
Tc_58  = [ NaN ; NaN ; NaN ];
omc_error_58 = [ NaN ; NaN ; NaN ];
Tc_error_58  = [ NaN ; NaN ; NaN ];

%-- Image #59:
omc_59 = [ NaN ; NaN ; NaN ];
Tc_59  = [ NaN ; NaN ; NaN ];
omc_error_59 = [ NaN ; NaN ; NaN ];
Tc_error_59  = [ NaN ; NaN ; NaN ];

%-- Image #60:
omc_60 = [ NaN ; NaN ; NaN ];
Tc_60  = [ NaN ; NaN ; NaN ];
omc_error_60 = [ NaN ; NaN ; NaN ];
Tc_error_60  = [ NaN ; NaN ; NaN ];

%-- Image #61:
omc_61 = [ NaN ; NaN ; NaN ];
Tc_61  = [ NaN ; NaN ; NaN ];
omc_error_61 = [ NaN ; NaN ; NaN ];
Tc_error_61  = [ NaN ; NaN ; NaN ];

%-- Image #62:
omc_62 = [ NaN ; NaN ; NaN ];
Tc_62  = [ NaN ; NaN ; NaN ];
omc_error_62 = [ NaN ; NaN ; NaN ];
Tc_error_62  = [ NaN ; NaN ; NaN ];

%-- Image #63:
omc_63 = [ NaN ; NaN ; NaN ];
Tc_63  = [ NaN ; NaN ; NaN ];
omc_error_63 = [ NaN ; NaN ; NaN ];
Tc_error_63  = [ NaN ; NaN ; NaN ];

%-- Image #64:
omc_64 = [ NaN ; NaN ; NaN ];
Tc_64  = [ NaN ; NaN ; NaN ];
omc_error_64 = [ NaN ; NaN ; NaN ];
Tc_error_64  = [ NaN ; NaN ; NaN ];

%-- Image #65:
omc_65 = [ NaN ; NaN ; NaN ];
Tc_65  = [ NaN ; NaN ; NaN ];
omc_error_65 = [ NaN ; NaN ; NaN ];
Tc_error_65  = [ NaN ; NaN ; NaN ];

%-- Image #66:
omc_66 = [ NaN ; NaN ; NaN ];
Tc_66  = [ NaN ; NaN ; NaN ];
omc_error_66 = [ NaN ; NaN ; NaN ];
Tc_error_66  = [ NaN ; NaN ; NaN ];

%-- Image #67:
omc_67 = [ NaN ; NaN ; NaN ];
Tc_67  = [ NaN ; NaN ; NaN ];
omc_error_67 = [ NaN ; NaN ; NaN ];
Tc_error_67  = [ NaN ; NaN ; NaN ];

%-- Image #68:
omc_68 = [ NaN ; NaN ; NaN ];
Tc_68  = [ NaN ; NaN ; NaN ];
omc_error_68 = [ NaN ; NaN ; NaN ];
Tc_error_68  = [ NaN ; NaN ; NaN ];

%-- Image #69:
omc_69 = [ NaN ; NaN ; NaN ];
Tc_69  = [ NaN ; NaN ; NaN ];
omc_error_69 = [ NaN ; NaN ; NaN ];
Tc_error_69  = [ NaN ; NaN ; NaN ];

%-- Image #70:
omc_70 = [ NaN ; NaN ; NaN ];
Tc_70  = [ NaN ; NaN ; NaN ];
omc_error_70 = [ NaN ; NaN ; NaN ];
Tc_error_70  = [ NaN ; NaN ; NaN ];

%-- Image #71:
omc_71 = [ NaN ; NaN ; NaN ];
Tc_71  = [ NaN ; NaN ; NaN ];
omc_error_71 = [ NaN ; NaN ; NaN ];
Tc_error_71  = [ NaN ; NaN ; NaN ];

%-- Image #72:
omc_72 = [ NaN ; NaN ; NaN ];
Tc_72  = [ NaN ; NaN ; NaN ];
omc_error_72 = [ NaN ; NaN ; NaN ];
Tc_error_72  = [ NaN ; NaN ; NaN ];

%-- Image #73:
omc_73 = [ NaN ; NaN ; NaN ];
Tc_73  = [ NaN ; NaN ; NaN ];
omc_error_73 = [ NaN ; NaN ; NaN ];
Tc_error_73  = [ NaN ; NaN ; NaN ];

%-- Image #74:
omc_74 = [ NaN ; NaN ; NaN ];
Tc_74  = [ NaN ; NaN ; NaN ];
omc_error_74 = [ NaN ; NaN ; NaN ];
Tc_error_74  = [ NaN ; NaN ; NaN ];

%-- Image #75:
omc_75 = [ NaN ; NaN ; NaN ];
Tc_75  = [ NaN ; NaN ; NaN ];
omc_error_75 = [ NaN ; NaN ; NaN ];
Tc_error_75  = [ NaN ; NaN ; NaN ];

%-- Image #76:
omc_76 = [ NaN ; NaN ; NaN ];
Tc_76  = [ NaN ; NaN ; NaN ];
omc_error_76 = [ NaN ; NaN ; NaN ];
Tc_error_76  = [ NaN ; NaN ; NaN ];

%-- Image #77:
omc_77 = [ NaN ; NaN ; NaN ];
Tc_77  = [ NaN ; NaN ; NaN ];
omc_error_77 = [ NaN ; NaN ; NaN ];
Tc_error_77  = [ NaN ; NaN ; NaN ];

%-- Image #78:
omc_78 = [ NaN ; NaN ; NaN ];
Tc_78  = [ NaN ; NaN ; NaN ];
omc_error_78 = [ NaN ; NaN ; NaN ];
Tc_error_78  = [ NaN ; NaN ; NaN ];

%-- Image #79:
omc_79 = [ NaN ; NaN ; NaN ];
Tc_79  = [ NaN ; NaN ; NaN ];
omc_error_79 = [ NaN ; NaN ; NaN ];
Tc_error_79  = [ NaN ; NaN ; NaN ];

%-- Image #80:
omc_80 = [ NaN ; NaN ; NaN ];
Tc_80  = [ NaN ; NaN ; NaN ];
omc_error_80 = [ NaN ; NaN ; NaN ];
Tc_error_80  = [ NaN ; NaN ; NaN ];

%-- Image #81:
omc_81 = [ NaN ; NaN ; NaN ];
Tc_81  = [ NaN ; NaN ; NaN ];
omc_error_81 = [ NaN ; NaN ; NaN ];
Tc_error_81  = [ NaN ; NaN ; NaN ];

%-- Image #82:
omc_82 = [ NaN ; NaN ; NaN ];
Tc_82  = [ NaN ; NaN ; NaN ];
omc_error_82 = [ NaN ; NaN ; NaN ];
Tc_error_82  = [ NaN ; NaN ; NaN ];

%-- Image #83:
omc_83 = [ NaN ; NaN ; NaN ];
Tc_83  = [ NaN ; NaN ; NaN ];
omc_error_83 = [ NaN ; NaN ; NaN ];
Tc_error_83  = [ NaN ; NaN ; NaN ];

%-- Image #84:
omc_84 = [ NaN ; NaN ; NaN ];
Tc_84  = [ NaN ; NaN ; NaN ];
omc_error_84 = [ NaN ; NaN ; NaN ];
Tc_error_84  = [ NaN ; NaN ; NaN ];

%-- Image #85:
omc_85 = [ NaN ; NaN ; NaN ];
Tc_85  = [ NaN ; NaN ; NaN ];
omc_error_85 = [ NaN ; NaN ; NaN ];
Tc_error_85  = [ NaN ; NaN ; NaN ];

%-- Image #86:
omc_86 = [ NaN ; NaN ; NaN ];
Tc_86  = [ NaN ; NaN ; NaN ];
omc_error_86 = [ NaN ; NaN ; NaN ];
Tc_error_86  = [ NaN ; NaN ; NaN ];

%-- Image #87:
omc_87 = [ NaN ; NaN ; NaN ];
Tc_87  = [ NaN ; NaN ; NaN ];
omc_error_87 = [ NaN ; NaN ; NaN ];
Tc_error_87  = [ NaN ; NaN ; NaN ];

%-- Image #88:
omc_88 = [ NaN ; NaN ; NaN ];
Tc_88  = [ NaN ; NaN ; NaN ];
omc_error_88 = [ NaN ; NaN ; NaN ];
Tc_error_88  = [ NaN ; NaN ; NaN ];

%-- Image #89:
omc_89 = [ NaN ; NaN ; NaN ];
Tc_89  = [ NaN ; NaN ; NaN ];
omc_error_89 = [ NaN ; NaN ; NaN ];
Tc_error_89  = [ NaN ; NaN ; NaN ];

%-- Image #90:
omc_90 = [ NaN ; NaN ; NaN ];
Tc_90  = [ NaN ; NaN ; NaN ];
omc_error_90 = [ NaN ; NaN ; NaN ];
Tc_error_90  = [ NaN ; NaN ; NaN ];

%-- Image #91:
omc_91 = [ NaN ; NaN ; NaN ];
Tc_91  = [ NaN ; NaN ; NaN ];
omc_error_91 = [ NaN ; NaN ; NaN ];
Tc_error_91  = [ NaN ; NaN ; NaN ];

%-- Image #92:
omc_92 = [ NaN ; NaN ; NaN ];
Tc_92  = [ NaN ; NaN ; NaN ];
omc_error_92 = [ NaN ; NaN ; NaN ];
Tc_error_92  = [ NaN ; NaN ; NaN ];

%-- Image #93:
omc_93 = [ NaN ; NaN ; NaN ];
Tc_93  = [ NaN ; NaN ; NaN ];
omc_error_93 = [ NaN ; NaN ; NaN ];
Tc_error_93  = [ NaN ; NaN ; NaN ];

%-- Image #94:
omc_94 = [ NaN ; NaN ; NaN ];
Tc_94  = [ NaN ; NaN ; NaN ];
omc_error_94 = [ NaN ; NaN ; NaN ];
Tc_error_94  = [ NaN ; NaN ; NaN ];

%-- Image #95:
omc_95 = [ NaN ; NaN ; NaN ];
Tc_95  = [ NaN ; NaN ; NaN ];
omc_error_95 = [ NaN ; NaN ; NaN ];
Tc_error_95  = [ NaN ; NaN ; NaN ];

%-- Image #96:
omc_96 = [ NaN ; NaN ; NaN ];
Tc_96  = [ NaN ; NaN ; NaN ];
omc_error_96 = [ NaN ; NaN ; NaN ];
Tc_error_96  = [ NaN ; NaN ; NaN ];

%-- Image #97:
omc_97 = [ NaN ; NaN ; NaN ];
Tc_97  = [ NaN ; NaN ; NaN ];
omc_error_97 = [ NaN ; NaN ; NaN ];
Tc_error_97  = [ NaN ; NaN ; NaN ];

%-- Image #98:
omc_98 = [ NaN ; NaN ; NaN ];
Tc_98  = [ NaN ; NaN ; NaN ];
omc_error_98 = [ NaN ; NaN ; NaN ];
Tc_error_98  = [ NaN ; NaN ; NaN ];

%-- Image #99:
omc_99 = [ NaN ; NaN ; NaN ];
Tc_99  = [ NaN ; NaN ; NaN ];
omc_error_99 = [ NaN ; NaN ; NaN ];
Tc_error_99  = [ NaN ; NaN ; NaN ];

%-- Image #100:
omc_100 = [ NaN ; NaN ; NaN ];
Tc_100  = [ NaN ; NaN ; NaN ];
omc_error_100 = [ NaN ; NaN ; NaN ];
Tc_error_100  = [ NaN ; NaN ; NaN ];

%-- Image #101:
omc_101 = [ NaN ; NaN ; NaN ];
Tc_101  = [ NaN ; NaN ; NaN ];
omc_error_101 = [ NaN ; NaN ; NaN ];
Tc_error_101  = [ NaN ; NaN ; NaN ];

%-- Image #102:
omc_102 = [ NaN ; NaN ; NaN ];
Tc_102  = [ NaN ; NaN ; NaN ];
omc_error_102 = [ NaN ; NaN ; NaN ];
Tc_error_102  = [ NaN ; NaN ; NaN ];

%-- Image #103:
omc_103 = [ NaN ; NaN ; NaN ];
Tc_103  = [ NaN ; NaN ; NaN ];
omc_error_103 = [ NaN ; NaN ; NaN ];
Tc_error_103  = [ NaN ; NaN ; NaN ];

%-- Image #104:
omc_104 = [ NaN ; NaN ; NaN ];
Tc_104  = [ NaN ; NaN ; NaN ];
omc_error_104 = [ NaN ; NaN ; NaN ];
Tc_error_104  = [ NaN ; NaN ; NaN ];

%-- Image #105:
omc_105 = [ NaN ; NaN ; NaN ];
Tc_105  = [ NaN ; NaN ; NaN ];
omc_error_105 = [ NaN ; NaN ; NaN ];
Tc_error_105  = [ NaN ; NaN ; NaN ];

%-- Image #106:
omc_106 = [ NaN ; NaN ; NaN ];
Tc_106  = [ NaN ; NaN ; NaN ];
omc_error_106 = [ NaN ; NaN ; NaN ];
Tc_error_106  = [ NaN ; NaN ; NaN ];

%-- Image #107:
omc_107 = [ NaN ; NaN ; NaN ];
Tc_107  = [ NaN ; NaN ; NaN ];
omc_error_107 = [ NaN ; NaN ; NaN ];
Tc_error_107  = [ NaN ; NaN ; NaN ];

%-- Image #108:
omc_108 = [ NaN ; NaN ; NaN ];
Tc_108  = [ NaN ; NaN ; NaN ];
omc_error_108 = [ NaN ; NaN ; NaN ];
Tc_error_108  = [ NaN ; NaN ; NaN ];

%-- Image #109:
omc_109 = [ NaN ; NaN ; NaN ];
Tc_109  = [ NaN ; NaN ; NaN ];
omc_error_109 = [ NaN ; NaN ; NaN ];
Tc_error_109  = [ NaN ; NaN ; NaN ];

%-- Image #110:
omc_110 = [ NaN ; NaN ; NaN ];
Tc_110  = [ NaN ; NaN ; NaN ];
omc_error_110 = [ NaN ; NaN ; NaN ];
Tc_error_110  = [ NaN ; NaN ; NaN ];

%-- Image #111:
omc_111 = [ -1.942411e+000 ; -2.256183e+000 ; -2.071341e-001 ];
Tc_111  = [ -1.228642e+002 ; -1.463302e+002 ; 2.872499e+002 ];
omc_error_111 = [ 2.266498e-002 ; 2.092288e-002 ; 4.376954e-002 ];
Tc_error_111  = [ 3.966609e+000 ; 4.301493e+000 ; 6.686319e+000 ];

%-- Image #112:
omc_112 = [ NaN ; NaN ; NaN ];
Tc_112  = [ NaN ; NaN ; NaN ];
omc_error_112 = [ NaN ; NaN ; NaN ];
Tc_error_112  = [ NaN ; NaN ; NaN ];

%-- Image #113:
omc_113 = [ NaN ; NaN ; NaN ];
Tc_113  = [ NaN ; NaN ; NaN ];
omc_error_113 = [ NaN ; NaN ; NaN ];
Tc_error_113  = [ NaN ; NaN ; NaN ];

%-- Image #114:
omc_114 = [ NaN ; NaN ; NaN ];
Tc_114  = [ NaN ; NaN ; NaN ];
omc_error_114 = [ NaN ; NaN ; NaN ];
Tc_error_114  = [ NaN ; NaN ; NaN ];

%-- Image #115:
omc_115 = [ NaN ; NaN ; NaN ];
Tc_115  = [ NaN ; NaN ; NaN ];
omc_error_115 = [ NaN ; NaN ; NaN ];
Tc_error_115  = [ NaN ; NaN ; NaN ];

%-- Image #116:
omc_116 = [ NaN ; NaN ; NaN ];
Tc_116  = [ NaN ; NaN ; NaN ];
omc_error_116 = [ NaN ; NaN ; NaN ];
Tc_error_116  = [ NaN ; NaN ; NaN ];

%-- Image #117:
omc_117 = [ NaN ; NaN ; NaN ];
Tc_117  = [ NaN ; NaN ; NaN ];
omc_error_117 = [ NaN ; NaN ; NaN ];
Tc_error_117  = [ NaN ; NaN ; NaN ];

%-- Image #118:
omc_118 = [ NaN ; NaN ; NaN ];
Tc_118  = [ NaN ; NaN ; NaN ];
omc_error_118 = [ NaN ; NaN ; NaN ];
Tc_error_118  = [ NaN ; NaN ; NaN ];

%-- Image #119:
omc_119 = [ NaN ; NaN ; NaN ];
Tc_119  = [ NaN ; NaN ; NaN ];
omc_error_119 = [ NaN ; NaN ; NaN ];
Tc_error_119  = [ NaN ; NaN ; NaN ];

%-- Image #120:
omc_120 = [ NaN ; NaN ; NaN ];
Tc_120  = [ NaN ; NaN ; NaN ];
omc_error_120 = [ NaN ; NaN ; NaN ];
Tc_error_120  = [ NaN ; NaN ; NaN ];

%-- Image #121:
omc_121 = [ NaN ; NaN ; NaN ];
Tc_121  = [ NaN ; NaN ; NaN ];
omc_error_121 = [ NaN ; NaN ; NaN ];
Tc_error_121  = [ NaN ; NaN ; NaN ];

%-- Image #122:
omc_122 = [ NaN ; NaN ; NaN ];
Tc_122  = [ NaN ; NaN ; NaN ];
omc_error_122 = [ NaN ; NaN ; NaN ];
Tc_error_122  = [ NaN ; NaN ; NaN ];

%-- Image #123:
omc_123 = [ NaN ; NaN ; NaN ];
Tc_123  = [ NaN ; NaN ; NaN ];
omc_error_123 = [ NaN ; NaN ; NaN ];
Tc_error_123  = [ NaN ; NaN ; NaN ];

%-- Image #124:
omc_124 = [ NaN ; NaN ; NaN ];
Tc_124  = [ NaN ; NaN ; NaN ];
omc_error_124 = [ NaN ; NaN ; NaN ];
Tc_error_124  = [ NaN ; NaN ; NaN ];

%-- Image #125:
omc_125 = [ NaN ; NaN ; NaN ];
Tc_125  = [ NaN ; NaN ; NaN ];
omc_error_125 = [ NaN ; NaN ; NaN ];
Tc_error_125  = [ NaN ; NaN ; NaN ];

%-- Image #126:
omc_126 = [ NaN ; NaN ; NaN ];
Tc_126  = [ NaN ; NaN ; NaN ];
omc_error_126 = [ NaN ; NaN ; NaN ];
Tc_error_126  = [ NaN ; NaN ; NaN ];

%-- Image #127:
omc_127 = [ NaN ; NaN ; NaN ];
Tc_127  = [ NaN ; NaN ; NaN ];
omc_error_127 = [ NaN ; NaN ; NaN ];
Tc_error_127  = [ NaN ; NaN ; NaN ];

%-- Image #128:
omc_128 = [ NaN ; NaN ; NaN ];
Tc_128  = [ NaN ; NaN ; NaN ];
omc_error_128 = [ NaN ; NaN ; NaN ];
Tc_error_128  = [ NaN ; NaN ; NaN ];

%-- Image #129:
omc_129 = [ NaN ; NaN ; NaN ];
Tc_129  = [ NaN ; NaN ; NaN ];
omc_error_129 = [ NaN ; NaN ; NaN ];
Tc_error_129  = [ NaN ; NaN ; NaN ];

%-- Image #130:
omc_130 = [ NaN ; NaN ; NaN ];
Tc_130  = [ NaN ; NaN ; NaN ];
omc_error_130 = [ NaN ; NaN ; NaN ];
Tc_error_130  = [ NaN ; NaN ; NaN ];

%-- Image #131:
omc_131 = [ NaN ; NaN ; NaN ];
Tc_131  = [ NaN ; NaN ; NaN ];
omc_error_131 = [ NaN ; NaN ; NaN ];
Tc_error_131  = [ NaN ; NaN ; NaN ];

%-- Image #132:
omc_132 = [ NaN ; NaN ; NaN ];
Tc_132  = [ NaN ; NaN ; NaN ];
omc_error_132 = [ NaN ; NaN ; NaN ];
Tc_error_132  = [ NaN ; NaN ; NaN ];

%-- Image #133:
omc_133 = [ NaN ; NaN ; NaN ];
Tc_133  = [ NaN ; NaN ; NaN ];
omc_error_133 = [ NaN ; NaN ; NaN ];
Tc_error_133  = [ NaN ; NaN ; NaN ];

%-- Image #134:
omc_134 = [ NaN ; NaN ; NaN ];
Tc_134  = [ NaN ; NaN ; NaN ];
omc_error_134 = [ NaN ; NaN ; NaN ];
Tc_error_134  = [ NaN ; NaN ; NaN ];

%-- Image #135:
omc_135 = [ NaN ; NaN ; NaN ];
Tc_135  = [ NaN ; NaN ; NaN ];
omc_error_135 = [ NaN ; NaN ; NaN ];
Tc_error_135  = [ NaN ; NaN ; NaN ];

%-- Image #136:
omc_136 = [ NaN ; NaN ; NaN ];
Tc_136  = [ NaN ; NaN ; NaN ];
omc_error_136 = [ NaN ; NaN ; NaN ];
Tc_error_136  = [ NaN ; NaN ; NaN ];

%-- Image #137:
omc_137 = [ NaN ; NaN ; NaN ];
Tc_137  = [ NaN ; NaN ; NaN ];
omc_error_137 = [ NaN ; NaN ; NaN ];
Tc_error_137  = [ NaN ; NaN ; NaN ];

%-- Image #138:
omc_138 = [ NaN ; NaN ; NaN ];
Tc_138  = [ NaN ; NaN ; NaN ];
omc_error_138 = [ NaN ; NaN ; NaN ];
Tc_error_138  = [ NaN ; NaN ; NaN ];

%-- Image #139:
omc_139 = [ NaN ; NaN ; NaN ];
Tc_139  = [ NaN ; NaN ; NaN ];
omc_error_139 = [ NaN ; NaN ; NaN ];
Tc_error_139  = [ NaN ; NaN ; NaN ];

%-- Image #140:
omc_140 = [ NaN ; NaN ; NaN ];
Tc_140  = [ NaN ; NaN ; NaN ];
omc_error_140 = [ NaN ; NaN ; NaN ];
Tc_error_140  = [ NaN ; NaN ; NaN ];

%-- Image #141:
omc_141 = [ NaN ; NaN ; NaN ];
Tc_141  = [ NaN ; NaN ; NaN ];
omc_error_141 = [ NaN ; NaN ; NaN ];
Tc_error_141  = [ NaN ; NaN ; NaN ];

%-- Image #142:
omc_142 = [ NaN ; NaN ; NaN ];
Tc_142  = [ NaN ; NaN ; NaN ];
omc_error_142 = [ NaN ; NaN ; NaN ];
Tc_error_142  = [ NaN ; NaN ; NaN ];

%-- Image #143:
omc_143 = [ NaN ; NaN ; NaN ];
Tc_143  = [ NaN ; NaN ; NaN ];
omc_error_143 = [ NaN ; NaN ; NaN ];
Tc_error_143  = [ NaN ; NaN ; NaN ];

%-- Image #144:
omc_144 = [ NaN ; NaN ; NaN ];
Tc_144  = [ NaN ; NaN ; NaN ];
omc_error_144 = [ NaN ; NaN ; NaN ];
Tc_error_144  = [ NaN ; NaN ; NaN ];

%-- Image #145:
omc_145 = [ NaN ; NaN ; NaN ];
Tc_145  = [ NaN ; NaN ; NaN ];
omc_error_145 = [ NaN ; NaN ; NaN ];
Tc_error_145  = [ NaN ; NaN ; NaN ];

%-- Image #146:
omc_146 = [ NaN ; NaN ; NaN ];
Tc_146  = [ NaN ; NaN ; NaN ];
omc_error_146 = [ NaN ; NaN ; NaN ];
Tc_error_146  = [ NaN ; NaN ; NaN ];

%-- Image #147:
omc_147 = [ NaN ; NaN ; NaN ];
Tc_147  = [ NaN ; NaN ; NaN ];
omc_error_147 = [ NaN ; NaN ; NaN ];
Tc_error_147  = [ NaN ; NaN ; NaN ];

%-- Image #148:
omc_148 = [ NaN ; NaN ; NaN ];
Tc_148  = [ NaN ; NaN ; NaN ];
omc_error_148 = [ NaN ; NaN ; NaN ];
Tc_error_148  = [ NaN ; NaN ; NaN ];

%-- Image #149:
omc_149 = [ NaN ; NaN ; NaN ];
Tc_149  = [ NaN ; NaN ; NaN ];
omc_error_149 = [ NaN ; NaN ; NaN ];
Tc_error_149  = [ NaN ; NaN ; NaN ];

%-- Image #150:
omc_150 = [ NaN ; NaN ; NaN ];
Tc_150  = [ NaN ; NaN ; NaN ];
omc_error_150 = [ NaN ; NaN ; NaN ];
Tc_error_150  = [ NaN ; NaN ; NaN ];

%-- Image #151:
omc_151 = [ NaN ; NaN ; NaN ];
Tc_151  = [ NaN ; NaN ; NaN ];
omc_error_151 = [ NaN ; NaN ; NaN ];
Tc_error_151  = [ NaN ; NaN ; NaN ];

%-- Image #152:
omc_152 = [ NaN ; NaN ; NaN ];
Tc_152  = [ NaN ; NaN ; NaN ];
omc_error_152 = [ NaN ; NaN ; NaN ];
Tc_error_152  = [ NaN ; NaN ; NaN ];

%-- Image #153:
omc_153 = [ NaN ; NaN ; NaN ];
Tc_153  = [ NaN ; NaN ; NaN ];
omc_error_153 = [ NaN ; NaN ; NaN ];
Tc_error_153  = [ NaN ; NaN ; NaN ];

%-- Image #154:
omc_154 = [ NaN ; NaN ; NaN ];
Tc_154  = [ NaN ; NaN ; NaN ];
omc_error_154 = [ NaN ; NaN ; NaN ];
Tc_error_154  = [ NaN ; NaN ; NaN ];

%-- Image #155:
omc_155 = [ NaN ; NaN ; NaN ];
Tc_155  = [ NaN ; NaN ; NaN ];
omc_error_155 = [ NaN ; NaN ; NaN ];
Tc_error_155  = [ NaN ; NaN ; NaN ];

%-- Image #156:
omc_156 = [ NaN ; NaN ; NaN ];
Tc_156  = [ NaN ; NaN ; NaN ];
omc_error_156 = [ NaN ; NaN ; NaN ];
Tc_error_156  = [ NaN ; NaN ; NaN ];

%-- Image #157:
omc_157 = [ NaN ; NaN ; NaN ];
Tc_157  = [ NaN ; NaN ; NaN ];
omc_error_157 = [ NaN ; NaN ; NaN ];
Tc_error_157  = [ NaN ; NaN ; NaN ];

%-- Image #158:
omc_158 = [ NaN ; NaN ; NaN ];
Tc_158  = [ NaN ; NaN ; NaN ];
omc_error_158 = [ NaN ; NaN ; NaN ];
Tc_error_158  = [ NaN ; NaN ; NaN ];

%-- Image #159:
omc_159 = [ NaN ; NaN ; NaN ];
Tc_159  = [ NaN ; NaN ; NaN ];
omc_error_159 = [ NaN ; NaN ; NaN ];
Tc_error_159  = [ NaN ; NaN ; NaN ];

%-- Image #160:
omc_160 = [ NaN ; NaN ; NaN ];
Tc_160  = [ NaN ; NaN ; NaN ];
omc_error_160 = [ NaN ; NaN ; NaN ];
Tc_error_160  = [ NaN ; NaN ; NaN ];

%-- Image #161:
omc_161 = [ NaN ; NaN ; NaN ];
Tc_161  = [ NaN ; NaN ; NaN ];
omc_error_161 = [ NaN ; NaN ; NaN ];
Tc_error_161  = [ NaN ; NaN ; NaN ];

%-- Image #162:
omc_162 = [ -1.930079e+000 ; -2.192833e+000 ; 2.984771e-001 ];
Tc_162  = [ -1.305827e+002 ; -1.273769e+002 ; 3.080681e+002 ];
omc_error_162 = [ 1.746210e-002 ; 1.525073e-002 ; 3.217950e-002 ];
Tc_error_162  = [ 4.127072e+000 ; 4.345749e+000 ; 5.905024e+000 ];

%-- Image #163:
omc_163 = [ NaN ; NaN ; NaN ];
Tc_163  = [ NaN ; NaN ; NaN ];
omc_error_163 = [ NaN ; NaN ; NaN ];
Tc_error_163  = [ NaN ; NaN ; NaN ];

%-- Image #164:
omc_164 = [ NaN ; NaN ; NaN ];
Tc_164  = [ NaN ; NaN ; NaN ];
omc_error_164 = [ NaN ; NaN ; NaN ];
Tc_error_164  = [ NaN ; NaN ; NaN ];

%-- Image #165:
omc_165 = [ NaN ; NaN ; NaN ];
Tc_165  = [ NaN ; NaN ; NaN ];
omc_error_165 = [ NaN ; NaN ; NaN ];
Tc_error_165  = [ NaN ; NaN ; NaN ];

%-- Image #166:
omc_166 = [ NaN ; NaN ; NaN ];
Tc_166  = [ NaN ; NaN ; NaN ];
omc_error_166 = [ NaN ; NaN ; NaN ];
Tc_error_166  = [ NaN ; NaN ; NaN ];

%-- Image #167:
omc_167 = [ NaN ; NaN ; NaN ];
Tc_167  = [ NaN ; NaN ; NaN ];
omc_error_167 = [ NaN ; NaN ; NaN ];
Tc_error_167  = [ NaN ; NaN ; NaN ];

%-- Image #168:
omc_168 = [ NaN ; NaN ; NaN ];
Tc_168  = [ NaN ; NaN ; NaN ];
omc_error_168 = [ NaN ; NaN ; NaN ];
Tc_error_168  = [ NaN ; NaN ; NaN ];

%-- Image #169:
omc_169 = [ NaN ; NaN ; NaN ];
Tc_169  = [ NaN ; NaN ; NaN ];
omc_error_169 = [ NaN ; NaN ; NaN ];
Tc_error_169  = [ NaN ; NaN ; NaN ];

%-- Image #170:
omc_170 = [ NaN ; NaN ; NaN ];
Tc_170  = [ NaN ; NaN ; NaN ];
omc_error_170 = [ NaN ; NaN ; NaN ];
Tc_error_170  = [ NaN ; NaN ; NaN ];

%-- Image #171:
omc_171 = [ NaN ; NaN ; NaN ];
Tc_171  = [ NaN ; NaN ; NaN ];
omc_error_171 = [ NaN ; NaN ; NaN ];
Tc_error_171  = [ NaN ; NaN ; NaN ];

%-- Image #172:
omc_172 = [ NaN ; NaN ; NaN ];
Tc_172  = [ NaN ; NaN ; NaN ];
omc_error_172 = [ NaN ; NaN ; NaN ];
Tc_error_172  = [ NaN ; NaN ; NaN ];

%-- Image #173:
omc_173 = [ NaN ; NaN ; NaN ];
Tc_173  = [ NaN ; NaN ; NaN ];
omc_error_173 = [ NaN ; NaN ; NaN ];
Tc_error_173  = [ NaN ; NaN ; NaN ];

%-- Image #174:
omc_174 = [ NaN ; NaN ; NaN ];
Tc_174  = [ NaN ; NaN ; NaN ];
omc_error_174 = [ NaN ; NaN ; NaN ];
Tc_error_174  = [ NaN ; NaN ; NaN ];

%-- Image #175:
omc_175 = [ NaN ; NaN ; NaN ];
Tc_175  = [ NaN ; NaN ; NaN ];
omc_error_175 = [ NaN ; NaN ; NaN ];
Tc_error_175  = [ NaN ; NaN ; NaN ];

%-- Image #176:
omc_176 = [ NaN ; NaN ; NaN ];
Tc_176  = [ NaN ; NaN ; NaN ];
omc_error_176 = [ NaN ; NaN ; NaN ];
Tc_error_176  = [ NaN ; NaN ; NaN ];

%-- Image #177:
omc_177 = [ NaN ; NaN ; NaN ];
Tc_177  = [ NaN ; NaN ; NaN ];
omc_error_177 = [ NaN ; NaN ; NaN ];
Tc_error_177  = [ NaN ; NaN ; NaN ];

%-- Image #178:
omc_178 = [ NaN ; NaN ; NaN ];
Tc_178  = [ NaN ; NaN ; NaN ];
omc_error_178 = [ NaN ; NaN ; NaN ];
Tc_error_178  = [ NaN ; NaN ; NaN ];

%-- Image #179:
omc_179 = [ NaN ; NaN ; NaN ];
Tc_179  = [ NaN ; NaN ; NaN ];
omc_error_179 = [ NaN ; NaN ; NaN ];
Tc_error_179  = [ NaN ; NaN ; NaN ];

%-- Image #180:
omc_180 = [ NaN ; NaN ; NaN ];
Tc_180  = [ NaN ; NaN ; NaN ];
omc_error_180 = [ NaN ; NaN ; NaN ];
Tc_error_180  = [ NaN ; NaN ; NaN ];

%-- Image #181:
omc_181 = [ NaN ; NaN ; NaN ];
Tc_181  = [ NaN ; NaN ; NaN ];
omc_error_181 = [ NaN ; NaN ; NaN ];
Tc_error_181  = [ NaN ; NaN ; NaN ];

%-- Image #182:
omc_182 = [ NaN ; NaN ; NaN ];
Tc_182  = [ NaN ; NaN ; NaN ];
omc_error_182 = [ NaN ; NaN ; NaN ];
Tc_error_182  = [ NaN ; NaN ; NaN ];

%-- Image #183:
omc_183 = [ NaN ; NaN ; NaN ];
Tc_183  = [ NaN ; NaN ; NaN ];
omc_error_183 = [ NaN ; NaN ; NaN ];
Tc_error_183  = [ NaN ; NaN ; NaN ];

%-- Image #184:
omc_184 = [ NaN ; NaN ; NaN ];
Tc_184  = [ NaN ; NaN ; NaN ];
omc_error_184 = [ NaN ; NaN ; NaN ];
Tc_error_184  = [ NaN ; NaN ; NaN ];

%-- Image #185:
omc_185 = [ NaN ; NaN ; NaN ];
Tc_185  = [ NaN ; NaN ; NaN ];
omc_error_185 = [ NaN ; NaN ; NaN ];
Tc_error_185  = [ NaN ; NaN ; NaN ];

%-- Image #186:
omc_186 = [ NaN ; NaN ; NaN ];
Tc_186  = [ NaN ; NaN ; NaN ];
omc_error_186 = [ NaN ; NaN ; NaN ];
Tc_error_186  = [ NaN ; NaN ; NaN ];

%-- Image #187:
omc_187 = [ NaN ; NaN ; NaN ];
Tc_187  = [ NaN ; NaN ; NaN ];
omc_error_187 = [ NaN ; NaN ; NaN ];
Tc_error_187  = [ NaN ; NaN ; NaN ];

%-- Image #188:
omc_188 = [ NaN ; NaN ; NaN ];
Tc_188  = [ NaN ; NaN ; NaN ];
omc_error_188 = [ NaN ; NaN ; NaN ];
Tc_error_188  = [ NaN ; NaN ; NaN ];

%-- Image #189:
omc_189 = [ NaN ; NaN ; NaN ];
Tc_189  = [ NaN ; NaN ; NaN ];
omc_error_189 = [ NaN ; NaN ; NaN ];
Tc_error_189  = [ NaN ; NaN ; NaN ];

%-- Image #190:
omc_190 = [ NaN ; NaN ; NaN ];
Tc_190  = [ NaN ; NaN ; NaN ];
omc_error_190 = [ NaN ; NaN ; NaN ];
Tc_error_190  = [ NaN ; NaN ; NaN ];

%-- Image #191:
omc_191 = [ NaN ; NaN ; NaN ];
Tc_191  = [ NaN ; NaN ; NaN ];
omc_error_191 = [ NaN ; NaN ; NaN ];
Tc_error_191  = [ NaN ; NaN ; NaN ];

%-- Image #192:
omc_192 = [ NaN ; NaN ; NaN ];
Tc_192  = [ NaN ; NaN ; NaN ];
omc_error_192 = [ NaN ; NaN ; NaN ];
Tc_error_192  = [ NaN ; NaN ; NaN ];

%-- Image #193:
omc_193 = [ NaN ; NaN ; NaN ];
Tc_193  = [ NaN ; NaN ; NaN ];
omc_error_193 = [ NaN ; NaN ; NaN ];
Tc_error_193  = [ NaN ; NaN ; NaN ];

%-- Image #194:
omc_194 = [ NaN ; NaN ; NaN ];
Tc_194  = [ NaN ; NaN ; NaN ];
omc_error_194 = [ NaN ; NaN ; NaN ];
Tc_error_194  = [ NaN ; NaN ; NaN ];

%-- Image #195:
omc_195 = [ NaN ; NaN ; NaN ];
Tc_195  = [ NaN ; NaN ; NaN ];
omc_error_195 = [ NaN ; NaN ; NaN ];
Tc_error_195  = [ NaN ; NaN ; NaN ];

%-- Image #196:
omc_196 = [ NaN ; NaN ; NaN ];
Tc_196  = [ NaN ; NaN ; NaN ];
omc_error_196 = [ NaN ; NaN ; NaN ];
Tc_error_196  = [ NaN ; NaN ; NaN ];

%-- Image #197:
omc_197 = [ NaN ; NaN ; NaN ];
Tc_197  = [ NaN ; NaN ; NaN ];
omc_error_197 = [ NaN ; NaN ; NaN ];
Tc_error_197  = [ NaN ; NaN ; NaN ];

%-- Image #198:
omc_198 = [ NaN ; NaN ; NaN ];
Tc_198  = [ NaN ; NaN ; NaN ];
omc_error_198 = [ NaN ; NaN ; NaN ];
Tc_error_198  = [ NaN ; NaN ; NaN ];

%-- Image #199:
omc_199 = [ NaN ; NaN ; NaN ];
Tc_199  = [ NaN ; NaN ; NaN ];
omc_error_199 = [ NaN ; NaN ; NaN ];
Tc_error_199  = [ NaN ; NaN ; NaN ];

%-- Image #200:
omc_200 = [ NaN ; NaN ; NaN ];
Tc_200  = [ NaN ; NaN ; NaN ];
omc_error_200 = [ NaN ; NaN ; NaN ];
Tc_error_200  = [ NaN ; NaN ; NaN ];

%-- Image #201:
omc_201 = [ NaN ; NaN ; NaN ];
Tc_201  = [ NaN ; NaN ; NaN ];
omc_error_201 = [ NaN ; NaN ; NaN ];
Tc_error_201  = [ NaN ; NaN ; NaN ];

%-- Image #202:
omc_202 = [ NaN ; NaN ; NaN ];
Tc_202  = [ NaN ; NaN ; NaN ];
omc_error_202 = [ NaN ; NaN ; NaN ];
Tc_error_202  = [ NaN ; NaN ; NaN ];

%-- Image #203:
omc_203 = [ NaN ; NaN ; NaN ];
Tc_203  = [ NaN ; NaN ; NaN ];
omc_error_203 = [ NaN ; NaN ; NaN ];
Tc_error_203  = [ NaN ; NaN ; NaN ];

%-- Image #204:
omc_204 = [ NaN ; NaN ; NaN ];
Tc_204  = [ NaN ; NaN ; NaN ];
omc_error_204 = [ NaN ; NaN ; NaN ];
Tc_error_204  = [ NaN ; NaN ; NaN ];

%-- Image #205:
omc_205 = [ NaN ; NaN ; NaN ];
Tc_205  = [ NaN ; NaN ; NaN ];
omc_error_205 = [ NaN ; NaN ; NaN ];
Tc_error_205  = [ NaN ; NaN ; NaN ];

%-- Image #206:
omc_206 = [ NaN ; NaN ; NaN ];
Tc_206  = [ NaN ; NaN ; NaN ];
omc_error_206 = [ NaN ; NaN ; NaN ];
Tc_error_206  = [ NaN ; NaN ; NaN ];

%-- Image #207:
omc_207 = [ NaN ; NaN ; NaN ];
Tc_207  = [ NaN ; NaN ; NaN ];
omc_error_207 = [ NaN ; NaN ; NaN ];
Tc_error_207  = [ NaN ; NaN ; NaN ];

%-- Image #208:
omc_208 = [ -2.046224e+000 ; -2.214921e+000 ; 9.515313e-002 ];
Tc_208  = [ -8.254605e+001 ; -1.113756e+002 ; 2.473065e+002 ];
omc_error_208 = [ 1.527354e-002 ; 1.581616e-002 ; 3.204765e-002 ];
Tc_error_208  = [ 3.339069e+000 ; 3.486005e+000 ; 5.100124e+000 ];

%-- Image #209:
omc_209 = [ NaN ; NaN ; NaN ];
Tc_209  = [ NaN ; NaN ; NaN ];
omc_error_209 = [ NaN ; NaN ; NaN ];
Tc_error_209  = [ NaN ; NaN ; NaN ];

%-- Image #210:
omc_210 = [ NaN ; NaN ; NaN ];
Tc_210  = [ NaN ; NaN ; NaN ];
omc_error_210 = [ NaN ; NaN ; NaN ];
Tc_error_210  = [ NaN ; NaN ; NaN ];

%-- Image #211:
omc_211 = [ NaN ; NaN ; NaN ];
Tc_211  = [ NaN ; NaN ; NaN ];
omc_error_211 = [ NaN ; NaN ; NaN ];
Tc_error_211  = [ NaN ; NaN ; NaN ];

%-- Image #212:
omc_212 = [ NaN ; NaN ; NaN ];
Tc_212  = [ NaN ; NaN ; NaN ];
omc_error_212 = [ NaN ; NaN ; NaN ];
Tc_error_212  = [ NaN ; NaN ; NaN ];

%-- Image #213:
omc_213 = [ NaN ; NaN ; NaN ];
Tc_213  = [ NaN ; NaN ; NaN ];
omc_error_213 = [ NaN ; NaN ; NaN ];
Tc_error_213  = [ NaN ; NaN ; NaN ];

%-- Image #214:
omc_214 = [ NaN ; NaN ; NaN ];
Tc_214  = [ NaN ; NaN ; NaN ];
omc_error_214 = [ NaN ; NaN ; NaN ];
Tc_error_214  = [ NaN ; NaN ; NaN ];

%-- Image #215:
omc_215 = [ NaN ; NaN ; NaN ];
Tc_215  = [ NaN ; NaN ; NaN ];
omc_error_215 = [ NaN ; NaN ; NaN ];
Tc_error_215  = [ NaN ; NaN ; NaN ];

%-- Image #216:
omc_216 = [ NaN ; NaN ; NaN ];
Tc_216  = [ NaN ; NaN ; NaN ];
omc_error_216 = [ NaN ; NaN ; NaN ];
Tc_error_216  = [ NaN ; NaN ; NaN ];

%-- Image #217:
omc_217 = [ NaN ; NaN ; NaN ];
Tc_217  = [ NaN ; NaN ; NaN ];
omc_error_217 = [ NaN ; NaN ; NaN ];
Tc_error_217  = [ NaN ; NaN ; NaN ];

%-- Image #218:
omc_218 = [ NaN ; NaN ; NaN ];
Tc_218  = [ NaN ; NaN ; NaN ];
omc_error_218 = [ NaN ; NaN ; NaN ];
Tc_error_218  = [ NaN ; NaN ; NaN ];

%-- Image #219:
omc_219 = [ NaN ; NaN ; NaN ];
Tc_219  = [ NaN ; NaN ; NaN ];
omc_error_219 = [ NaN ; NaN ; NaN ];
Tc_error_219  = [ NaN ; NaN ; NaN ];

%-- Image #220:
omc_220 = [ NaN ; NaN ; NaN ];
Tc_220  = [ NaN ; NaN ; NaN ];
omc_error_220 = [ NaN ; NaN ; NaN ];
Tc_error_220  = [ NaN ; NaN ; NaN ];

%-- Image #221:
omc_221 = [ NaN ; NaN ; NaN ];
Tc_221  = [ NaN ; NaN ; NaN ];
omc_error_221 = [ NaN ; NaN ; NaN ];
Tc_error_221  = [ NaN ; NaN ; NaN ];

%-- Image #222:
omc_222 = [ NaN ; NaN ; NaN ];
Tc_222  = [ NaN ; NaN ; NaN ];
omc_error_222 = [ NaN ; NaN ; NaN ];
Tc_error_222  = [ NaN ; NaN ; NaN ];

%-- Image #223:
omc_223 = [ NaN ; NaN ; NaN ];
Tc_223  = [ NaN ; NaN ; NaN ];
omc_error_223 = [ NaN ; NaN ; NaN ];
Tc_error_223  = [ NaN ; NaN ; NaN ];

%-- Image #224:
omc_224 = [ NaN ; NaN ; NaN ];
Tc_224  = [ NaN ; NaN ; NaN ];
omc_error_224 = [ NaN ; NaN ; NaN ];
Tc_error_224  = [ NaN ; NaN ; NaN ];

%-- Image #225:
omc_225 = [ NaN ; NaN ; NaN ];
Tc_225  = [ NaN ; NaN ; NaN ];
omc_error_225 = [ NaN ; NaN ; NaN ];
Tc_error_225  = [ NaN ; NaN ; NaN ];

%-- Image #226:
omc_226 = [ NaN ; NaN ; NaN ];
Tc_226  = [ NaN ; NaN ; NaN ];
omc_error_226 = [ NaN ; NaN ; NaN ];
Tc_error_226  = [ NaN ; NaN ; NaN ];

%-- Image #227:
omc_227 = [ NaN ; NaN ; NaN ];
Tc_227  = [ NaN ; NaN ; NaN ];
omc_error_227 = [ NaN ; NaN ; NaN ];
Tc_error_227  = [ NaN ; NaN ; NaN ];

%-- Image #228:
omc_228 = [ NaN ; NaN ; NaN ];
Tc_228  = [ NaN ; NaN ; NaN ];
omc_error_228 = [ NaN ; NaN ; NaN ];
Tc_error_228  = [ NaN ; NaN ; NaN ];

%-- Image #229:
omc_229 = [ NaN ; NaN ; NaN ];
Tc_229  = [ NaN ; NaN ; NaN ];
omc_error_229 = [ NaN ; NaN ; NaN ];
Tc_error_229  = [ NaN ; NaN ; NaN ];

%-- Image #230:
omc_230 = [ NaN ; NaN ; NaN ];
Tc_230  = [ NaN ; NaN ; NaN ];
omc_error_230 = [ NaN ; NaN ; NaN ];
Tc_error_230  = [ NaN ; NaN ; NaN ];

%-- Image #231:
omc_231 = [ NaN ; NaN ; NaN ];
Tc_231  = [ NaN ; NaN ; NaN ];
omc_error_231 = [ NaN ; NaN ; NaN ];
Tc_error_231  = [ NaN ; NaN ; NaN ];

%-- Image #232:
omc_232 = [ NaN ; NaN ; NaN ];
Tc_232  = [ NaN ; NaN ; NaN ];
omc_error_232 = [ NaN ; NaN ; NaN ];
Tc_error_232  = [ NaN ; NaN ; NaN ];

%-- Image #233:
omc_233 = [ NaN ; NaN ; NaN ];
Tc_233  = [ NaN ; NaN ; NaN ];
omc_error_233 = [ NaN ; NaN ; NaN ];
Tc_error_233  = [ NaN ; NaN ; NaN ];

%-- Image #234:
omc_234 = [ NaN ; NaN ; NaN ];
Tc_234  = [ NaN ; NaN ; NaN ];
omc_error_234 = [ NaN ; NaN ; NaN ];
Tc_error_234  = [ NaN ; NaN ; NaN ];

%-- Image #235:
omc_235 = [ NaN ; NaN ; NaN ];
Tc_235  = [ NaN ; NaN ; NaN ];
omc_error_235 = [ NaN ; NaN ; NaN ];
Tc_error_235  = [ NaN ; NaN ; NaN ];

%-- Image #236:
omc_236 = [ NaN ; NaN ; NaN ];
Tc_236  = [ NaN ; NaN ; NaN ];
omc_error_236 = [ NaN ; NaN ; NaN ];
Tc_error_236  = [ NaN ; NaN ; NaN ];

%-- Image #237:
omc_237 = [ NaN ; NaN ; NaN ];
Tc_237  = [ NaN ; NaN ; NaN ];
omc_error_237 = [ NaN ; NaN ; NaN ];
Tc_error_237  = [ NaN ; NaN ; NaN ];

%-- Image #238:
omc_238 = [ NaN ; NaN ; NaN ];
Tc_238  = [ NaN ; NaN ; NaN ];
omc_error_238 = [ NaN ; NaN ; NaN ];
Tc_error_238  = [ NaN ; NaN ; NaN ];

%-- Image #239:
omc_239 = [ NaN ; NaN ; NaN ];
Tc_239  = [ NaN ; NaN ; NaN ];
omc_error_239 = [ NaN ; NaN ; NaN ];
Tc_error_239  = [ NaN ; NaN ; NaN ];

%-- Image #240:
omc_240 = [ NaN ; NaN ; NaN ];
Tc_240  = [ NaN ; NaN ; NaN ];
omc_error_240 = [ NaN ; NaN ; NaN ];
Tc_error_240  = [ NaN ; NaN ; NaN ];

%-- Image #241:
omc_241 = [ NaN ; NaN ; NaN ];
Tc_241  = [ NaN ; NaN ; NaN ];
omc_error_241 = [ NaN ; NaN ; NaN ];
Tc_error_241  = [ NaN ; NaN ; NaN ];

%-- Image #242:
omc_242 = [ NaN ; NaN ; NaN ];
Tc_242  = [ NaN ; NaN ; NaN ];
omc_error_242 = [ NaN ; NaN ; NaN ];
Tc_error_242  = [ NaN ; NaN ; NaN ];

%-- Image #243:
omc_243 = [ NaN ; NaN ; NaN ];
Tc_243  = [ NaN ; NaN ; NaN ];
omc_error_243 = [ NaN ; NaN ; NaN ];
Tc_error_243  = [ NaN ; NaN ; NaN ];

%-- Image #244:
omc_244 = [ NaN ; NaN ; NaN ];
Tc_244  = [ NaN ; NaN ; NaN ];
omc_error_244 = [ NaN ; NaN ; NaN ];
Tc_error_244  = [ NaN ; NaN ; NaN ];

%-- Image #245:
omc_245 = [ NaN ; NaN ; NaN ];
Tc_245  = [ NaN ; NaN ; NaN ];
omc_error_245 = [ NaN ; NaN ; NaN ];
Tc_error_245  = [ NaN ; NaN ; NaN ];

%-- Image #246:
omc_246 = [ NaN ; NaN ; NaN ];
Tc_246  = [ NaN ; NaN ; NaN ];
omc_error_246 = [ NaN ; NaN ; NaN ];
Tc_error_246  = [ NaN ; NaN ; NaN ];

%-- Image #247:
omc_247 = [ NaN ; NaN ; NaN ];
Tc_247  = [ NaN ; NaN ; NaN ];
omc_error_247 = [ NaN ; NaN ; NaN ];
Tc_error_247  = [ NaN ; NaN ; NaN ];

%-- Image #248:
omc_248 = [ NaN ; NaN ; NaN ];
Tc_248  = [ NaN ; NaN ; NaN ];
omc_error_248 = [ NaN ; NaN ; NaN ];
Tc_error_248  = [ NaN ; NaN ; NaN ];

%-- Image #249:
omc_249 = [ NaN ; NaN ; NaN ];
Tc_249  = [ NaN ; NaN ; NaN ];
omc_error_249 = [ NaN ; NaN ; NaN ];
Tc_error_249  = [ NaN ; NaN ; NaN ];

%-- Image #250:
omc_250 = [ NaN ; NaN ; NaN ];
Tc_250  = [ NaN ; NaN ; NaN ];
omc_error_250 = [ NaN ; NaN ; NaN ];
Tc_error_250  = [ NaN ; NaN ; NaN ];

%-- Image #251:
omc_251 = [ NaN ; NaN ; NaN ];
Tc_251  = [ NaN ; NaN ; NaN ];
omc_error_251 = [ NaN ; NaN ; NaN ];
Tc_error_251  = [ NaN ; NaN ; NaN ];

%-- Image #252:
omc_252 = [ NaN ; NaN ; NaN ];
Tc_252  = [ NaN ; NaN ; NaN ];
omc_error_252 = [ NaN ; NaN ; NaN ];
Tc_error_252  = [ NaN ; NaN ; NaN ];

%-- Image #253:
omc_253 = [ NaN ; NaN ; NaN ];
Tc_253  = [ NaN ; NaN ; NaN ];
omc_error_253 = [ NaN ; NaN ; NaN ];
Tc_error_253  = [ NaN ; NaN ; NaN ];

%-- Image #254:
omc_254 = [ NaN ; NaN ; NaN ];
Tc_254  = [ NaN ; NaN ; NaN ];
omc_error_254 = [ NaN ; NaN ; NaN ];
Tc_error_254  = [ NaN ; NaN ; NaN ];

%-- Image #255:
omc_255 = [ NaN ; NaN ; NaN ];
Tc_255  = [ NaN ; NaN ; NaN ];
omc_error_255 = [ NaN ; NaN ; NaN ];
Tc_error_255  = [ NaN ; NaN ; NaN ];

%-- Image #256:
omc_256 = [ NaN ; NaN ; NaN ];
Tc_256  = [ NaN ; NaN ; NaN ];
omc_error_256 = [ NaN ; NaN ; NaN ];
Tc_error_256  = [ NaN ; NaN ; NaN ];

%-- Image #257:
omc_257 = [ NaN ; NaN ; NaN ];
Tc_257  = [ NaN ; NaN ; NaN ];
omc_error_257 = [ NaN ; NaN ; NaN ];
Tc_error_257  = [ NaN ; NaN ; NaN ];

%-- Image #258:
omc_258 = [ NaN ; NaN ; NaN ];
Tc_258  = [ NaN ; NaN ; NaN ];
omc_error_258 = [ NaN ; NaN ; NaN ];
Tc_error_258  = [ NaN ; NaN ; NaN ];

%-- Image #259:
omc_259 = [ NaN ; NaN ; NaN ];
Tc_259  = [ NaN ; NaN ; NaN ];
omc_error_259 = [ NaN ; NaN ; NaN ];
Tc_error_259  = [ NaN ; NaN ; NaN ];

%-- Image #260:
omc_260 = [ NaN ; NaN ; NaN ];
Tc_260  = [ NaN ; NaN ; NaN ];
omc_error_260 = [ NaN ; NaN ; NaN ];
Tc_error_260  = [ NaN ; NaN ; NaN ];

%-- Image #261:
omc_261 = [ NaN ; NaN ; NaN ];
Tc_261  = [ NaN ; NaN ; NaN ];
omc_error_261 = [ NaN ; NaN ; NaN ];
Tc_error_261  = [ NaN ; NaN ; NaN ];

%-- Image #262:
omc_262 = [ NaN ; NaN ; NaN ];
Tc_262  = [ NaN ; NaN ; NaN ];
omc_error_262 = [ NaN ; NaN ; NaN ];
Tc_error_262  = [ NaN ; NaN ; NaN ];

%-- Image #263:
omc_263 = [ NaN ; NaN ; NaN ];
Tc_263  = [ NaN ; NaN ; NaN ];
omc_error_263 = [ NaN ; NaN ; NaN ];
Tc_error_263  = [ NaN ; NaN ; NaN ];

%-- Image #264:
omc_264 = [ NaN ; NaN ; NaN ];
Tc_264  = [ NaN ; NaN ; NaN ];
omc_error_264 = [ NaN ; NaN ; NaN ];
Tc_error_264  = [ NaN ; NaN ; NaN ];

%-- Image #265:
omc_265 = [ NaN ; NaN ; NaN ];
Tc_265  = [ NaN ; NaN ; NaN ];
omc_error_265 = [ NaN ; NaN ; NaN ];
Tc_error_265  = [ NaN ; NaN ; NaN ];

%-- Image #266:
omc_266 = [ NaN ; NaN ; NaN ];
Tc_266  = [ NaN ; NaN ; NaN ];
omc_error_266 = [ NaN ; NaN ; NaN ];
Tc_error_266  = [ NaN ; NaN ; NaN ];

%-- Image #267:
omc_267 = [ NaN ; NaN ; NaN ];
Tc_267  = [ NaN ; NaN ; NaN ];
omc_error_267 = [ NaN ; NaN ; NaN ];
Tc_error_267  = [ NaN ; NaN ; NaN ];

%-- Image #268:
omc_268 = [ -1.931407e+000 ; -2.071300e+000 ; -9.219575e-002 ];
Tc_268  = [ -8.896381e+001 ; -1.029524e+002 ; 2.179458e+002 ];
omc_error_268 = [ 1.396905e-002 ; 1.477282e-002 ; 2.661241e-002 ];
Tc_error_268  = [ 2.967840e+000 ; 3.192627e+000 ; 4.515601e+000 ];

%-- Image #269:
omc_269 = [ NaN ; NaN ; NaN ];
Tc_269  = [ NaN ; NaN ; NaN ];
omc_error_269 = [ NaN ; NaN ; NaN ];
Tc_error_269  = [ NaN ; NaN ; NaN ];

%-- Image #270:
omc_270 = [ NaN ; NaN ; NaN ];
Tc_270  = [ NaN ; NaN ; NaN ];
omc_error_270 = [ NaN ; NaN ; NaN ];
Tc_error_270  = [ NaN ; NaN ; NaN ];

%-- Image #271:
omc_271 = [ NaN ; NaN ; NaN ];
Tc_271  = [ NaN ; NaN ; NaN ];
omc_error_271 = [ NaN ; NaN ; NaN ];
Tc_error_271  = [ NaN ; NaN ; NaN ];

%-- Image #272:
omc_272 = [ NaN ; NaN ; NaN ];
Tc_272  = [ NaN ; NaN ; NaN ];
omc_error_272 = [ NaN ; NaN ; NaN ];
Tc_error_272  = [ NaN ; NaN ; NaN ];

%-- Image #273:
omc_273 = [ NaN ; NaN ; NaN ];
Tc_273  = [ NaN ; NaN ; NaN ];
omc_error_273 = [ NaN ; NaN ; NaN ];
Tc_error_273  = [ NaN ; NaN ; NaN ];

%-- Image #274:
omc_274 = [ NaN ; NaN ; NaN ];
Tc_274  = [ NaN ; NaN ; NaN ];
omc_error_274 = [ NaN ; NaN ; NaN ];
Tc_error_274  = [ NaN ; NaN ; NaN ];

%-- Image #275:
omc_275 = [ NaN ; NaN ; NaN ];
Tc_275  = [ NaN ; NaN ; NaN ];
omc_error_275 = [ NaN ; NaN ; NaN ];
Tc_error_275  = [ NaN ; NaN ; NaN ];

%-- Image #276:
omc_276 = [ NaN ; NaN ; NaN ];
Tc_276  = [ NaN ; NaN ; NaN ];
omc_error_276 = [ NaN ; NaN ; NaN ];
Tc_error_276  = [ NaN ; NaN ; NaN ];

%-- Image #277:
omc_277 = [ NaN ; NaN ; NaN ];
Tc_277  = [ NaN ; NaN ; NaN ];
omc_error_277 = [ NaN ; NaN ; NaN ];
Tc_error_277  = [ NaN ; NaN ; NaN ];

%-- Image #278:
omc_278 = [ NaN ; NaN ; NaN ];
Tc_278  = [ NaN ; NaN ; NaN ];
omc_error_278 = [ NaN ; NaN ; NaN ];
Tc_error_278  = [ NaN ; NaN ; NaN ];

%-- Image #279:
omc_279 = [ NaN ; NaN ; NaN ];
Tc_279  = [ NaN ; NaN ; NaN ];
omc_error_279 = [ NaN ; NaN ; NaN ];
Tc_error_279  = [ NaN ; NaN ; NaN ];

%-- Image #280:
omc_280 = [ NaN ; NaN ; NaN ];
Tc_280  = [ NaN ; NaN ; NaN ];
omc_error_280 = [ NaN ; NaN ; NaN ];
Tc_error_280  = [ NaN ; NaN ; NaN ];

%-- Image #281:
omc_281 = [ NaN ; NaN ; NaN ];
Tc_281  = [ NaN ; NaN ; NaN ];
omc_error_281 = [ NaN ; NaN ; NaN ];
Tc_error_281  = [ NaN ; NaN ; NaN ];

%-- Image #282:
omc_282 = [ NaN ; NaN ; NaN ];
Tc_282  = [ NaN ; NaN ; NaN ];
omc_error_282 = [ NaN ; NaN ; NaN ];
Tc_error_282  = [ NaN ; NaN ; NaN ];

%-- Image #283:
omc_283 = [ NaN ; NaN ; NaN ];
Tc_283  = [ NaN ; NaN ; NaN ];
omc_error_283 = [ NaN ; NaN ; NaN ];
Tc_error_283  = [ NaN ; NaN ; NaN ];

%-- Image #284:
omc_284 = [ NaN ; NaN ; NaN ];
Tc_284  = [ NaN ; NaN ; NaN ];
omc_error_284 = [ NaN ; NaN ; NaN ];
Tc_error_284  = [ NaN ; NaN ; NaN ];

%-- Image #285:
omc_285 = [ NaN ; NaN ; NaN ];
Tc_285  = [ NaN ; NaN ; NaN ];
omc_error_285 = [ NaN ; NaN ; NaN ];
Tc_error_285  = [ NaN ; NaN ; NaN ];

%-- Image #286:
omc_286 = [ NaN ; NaN ; NaN ];
Tc_286  = [ NaN ; NaN ; NaN ];
omc_error_286 = [ NaN ; NaN ; NaN ];
Tc_error_286  = [ NaN ; NaN ; NaN ];

%-- Image #287:
omc_287 = [ NaN ; NaN ; NaN ];
Tc_287  = [ NaN ; NaN ; NaN ];
omc_error_287 = [ NaN ; NaN ; NaN ];
Tc_error_287  = [ NaN ; NaN ; NaN ];

%-- Image #288:
omc_288 = [ NaN ; NaN ; NaN ];
Tc_288  = [ NaN ; NaN ; NaN ];
omc_error_288 = [ NaN ; NaN ; NaN ];
Tc_error_288  = [ NaN ; NaN ; NaN ];

%-- Image #289:
omc_289 = [ NaN ; NaN ; NaN ];
Tc_289  = [ NaN ; NaN ; NaN ];
omc_error_289 = [ NaN ; NaN ; NaN ];
Tc_error_289  = [ NaN ; NaN ; NaN ];

%-- Image #290:
omc_290 = [ NaN ; NaN ; NaN ];
Tc_290  = [ NaN ; NaN ; NaN ];
omc_error_290 = [ NaN ; NaN ; NaN ];
Tc_error_290  = [ NaN ; NaN ; NaN ];

%-- Image #291:
omc_291 = [ NaN ; NaN ; NaN ];
Tc_291  = [ NaN ; NaN ; NaN ];
omc_error_291 = [ NaN ; NaN ; NaN ];
Tc_error_291  = [ NaN ; NaN ; NaN ];

%-- Image #292:
omc_292 = [ NaN ; NaN ; NaN ];
Tc_292  = [ NaN ; NaN ; NaN ];
omc_error_292 = [ NaN ; NaN ; NaN ];
Tc_error_292  = [ NaN ; NaN ; NaN ];

%-- Image #293:
omc_293 = [ NaN ; NaN ; NaN ];
Tc_293  = [ NaN ; NaN ; NaN ];
omc_error_293 = [ NaN ; NaN ; NaN ];
Tc_error_293  = [ NaN ; NaN ; NaN ];

%-- Image #294:
omc_294 = [ NaN ; NaN ; NaN ];
Tc_294  = [ NaN ; NaN ; NaN ];
omc_error_294 = [ NaN ; NaN ; NaN ];
Tc_error_294  = [ NaN ; NaN ; NaN ];

%-- Image #295:
omc_295 = [ NaN ; NaN ; NaN ];
Tc_295  = [ NaN ; NaN ; NaN ];
omc_error_295 = [ NaN ; NaN ; NaN ];
Tc_error_295  = [ NaN ; NaN ; NaN ];

%-- Image #296:
omc_296 = [ NaN ; NaN ; NaN ];
Tc_296  = [ NaN ; NaN ; NaN ];
omc_error_296 = [ NaN ; NaN ; NaN ];
Tc_error_296  = [ NaN ; NaN ; NaN ];

%-- Image #297:
omc_297 = [ NaN ; NaN ; NaN ];
Tc_297  = [ NaN ; NaN ; NaN ];
omc_error_297 = [ NaN ; NaN ; NaN ];
Tc_error_297  = [ NaN ; NaN ; NaN ];

%-- Image #298:
omc_298 = [ NaN ; NaN ; NaN ];
Tc_298  = [ NaN ; NaN ; NaN ];
omc_error_298 = [ NaN ; NaN ; NaN ];
Tc_error_298  = [ NaN ; NaN ; NaN ];

%-- Image #299:
omc_299 = [ NaN ; NaN ; NaN ];
Tc_299  = [ NaN ; NaN ; NaN ];
omc_error_299 = [ NaN ; NaN ; NaN ];
Tc_error_299  = [ NaN ; NaN ; NaN ];

%-- Image #300:
omc_300 = [ NaN ; NaN ; NaN ];
Tc_300  = [ NaN ; NaN ; NaN ];
omc_error_300 = [ NaN ; NaN ; NaN ];
Tc_error_300  = [ NaN ; NaN ; NaN ];

%-- Image #301:
omc_301 = [ NaN ; NaN ; NaN ];
Tc_301  = [ NaN ; NaN ; NaN ];
omc_error_301 = [ NaN ; NaN ; NaN ];
Tc_error_301  = [ NaN ; NaN ; NaN ];

%-- Image #302:
omc_302 = [ NaN ; NaN ; NaN ];
Tc_302  = [ NaN ; NaN ; NaN ];
omc_error_302 = [ NaN ; NaN ; NaN ];
Tc_error_302  = [ NaN ; NaN ; NaN ];

%-- Image #303:
omc_303 = [ NaN ; NaN ; NaN ];
Tc_303  = [ NaN ; NaN ; NaN ];
omc_error_303 = [ NaN ; NaN ; NaN ];
Tc_error_303  = [ NaN ; NaN ; NaN ];

%-- Image #304:
omc_304 = [ NaN ; NaN ; NaN ];
Tc_304  = [ NaN ; NaN ; NaN ];
omc_error_304 = [ NaN ; NaN ; NaN ];
Tc_error_304  = [ NaN ; NaN ; NaN ];

%-- Image #305:
omc_305 = [ NaN ; NaN ; NaN ];
Tc_305  = [ NaN ; NaN ; NaN ];
omc_error_305 = [ NaN ; NaN ; NaN ];
Tc_error_305  = [ NaN ; NaN ; NaN ];

%-- Image #306:
omc_306 = [ NaN ; NaN ; NaN ];
Tc_306  = [ NaN ; NaN ; NaN ];
omc_error_306 = [ NaN ; NaN ; NaN ];
Tc_error_306  = [ NaN ; NaN ; NaN ];

%-- Image #307:
omc_307 = [ NaN ; NaN ; NaN ];
Tc_307  = [ NaN ; NaN ; NaN ];
omc_error_307 = [ NaN ; NaN ; NaN ];
Tc_error_307  = [ NaN ; NaN ; NaN ];

%-- Image #308:
omc_308 = [ NaN ; NaN ; NaN ];
Tc_308  = [ NaN ; NaN ; NaN ];
omc_error_308 = [ NaN ; NaN ; NaN ];
Tc_error_308  = [ NaN ; NaN ; NaN ];

%-- Image #309:
omc_309 = [ NaN ; NaN ; NaN ];
Tc_309  = [ NaN ; NaN ; NaN ];
omc_error_309 = [ NaN ; NaN ; NaN ];
Tc_error_309  = [ NaN ; NaN ; NaN ];

%-- Image #310:
omc_310 = [ NaN ; NaN ; NaN ];
Tc_310  = [ NaN ; NaN ; NaN ];
omc_error_310 = [ NaN ; NaN ; NaN ];
Tc_error_310  = [ NaN ; NaN ; NaN ];

%-- Image #311:
omc_311 = [ NaN ; NaN ; NaN ];
Tc_311  = [ NaN ; NaN ; NaN ];
omc_error_311 = [ NaN ; NaN ; NaN ];
Tc_error_311  = [ NaN ; NaN ; NaN ];

%-- Image #312:
omc_312 = [ NaN ; NaN ; NaN ];
Tc_312  = [ NaN ; NaN ; NaN ];
omc_error_312 = [ NaN ; NaN ; NaN ];
Tc_error_312  = [ NaN ; NaN ; NaN ];

%-- Image #313:
omc_313 = [ NaN ; NaN ; NaN ];
Tc_313  = [ NaN ; NaN ; NaN ];
omc_error_313 = [ NaN ; NaN ; NaN ];
Tc_error_313  = [ NaN ; NaN ; NaN ];

%-- Image #314:
omc_314 = [ NaN ; NaN ; NaN ];
Tc_314  = [ NaN ; NaN ; NaN ];
omc_error_314 = [ NaN ; NaN ; NaN ];
Tc_error_314  = [ NaN ; NaN ; NaN ];

%-- Image #315:
omc_315 = [ NaN ; NaN ; NaN ];
Tc_315  = [ NaN ; NaN ; NaN ];
omc_error_315 = [ NaN ; NaN ; NaN ];
Tc_error_315  = [ NaN ; NaN ; NaN ];

%-- Image #316:
omc_316 = [ NaN ; NaN ; NaN ];
Tc_316  = [ NaN ; NaN ; NaN ];
omc_error_316 = [ NaN ; NaN ; NaN ];
Tc_error_316  = [ NaN ; NaN ; NaN ];

%-- Image #317:
omc_317 = [ NaN ; NaN ; NaN ];
Tc_317  = [ NaN ; NaN ; NaN ];
omc_error_317 = [ NaN ; NaN ; NaN ];
Tc_error_317  = [ NaN ; NaN ; NaN ];

%-- Image #318:
omc_318 = [ NaN ; NaN ; NaN ];
Tc_318  = [ NaN ; NaN ; NaN ];
omc_error_318 = [ NaN ; NaN ; NaN ];
Tc_error_318  = [ NaN ; NaN ; NaN ];

%-- Image #319:
omc_319 = [ NaN ; NaN ; NaN ];
Tc_319  = [ NaN ; NaN ; NaN ];
omc_error_319 = [ NaN ; NaN ; NaN ];
Tc_error_319  = [ NaN ; NaN ; NaN ];

%-- Image #320:
omc_320 = [ NaN ; NaN ; NaN ];
Tc_320  = [ NaN ; NaN ; NaN ];
omc_error_320 = [ NaN ; NaN ; NaN ];
Tc_error_320  = [ NaN ; NaN ; NaN ];

%-- Image #321:
omc_321 = [ NaN ; NaN ; NaN ];
Tc_321  = [ NaN ; NaN ; NaN ];
omc_error_321 = [ NaN ; NaN ; NaN ];
Tc_error_321  = [ NaN ; NaN ; NaN ];

%-- Image #322:
omc_322 = [ NaN ; NaN ; NaN ];
Tc_322  = [ NaN ; NaN ; NaN ];
omc_error_322 = [ NaN ; NaN ; NaN ];
Tc_error_322  = [ NaN ; NaN ; NaN ];

%-- Image #323:
omc_323 = [ NaN ; NaN ; NaN ];
Tc_323  = [ NaN ; NaN ; NaN ];
omc_error_323 = [ NaN ; NaN ; NaN ];
Tc_error_323  = [ NaN ; NaN ; NaN ];

%-- Image #324:
omc_324 = [ NaN ; NaN ; NaN ];
Tc_324  = [ NaN ; NaN ; NaN ];
omc_error_324 = [ NaN ; NaN ; NaN ];
Tc_error_324  = [ NaN ; NaN ; NaN ];

%-- Image #325:
omc_325 = [ NaN ; NaN ; NaN ];
Tc_325  = [ NaN ; NaN ; NaN ];
omc_error_325 = [ NaN ; NaN ; NaN ];
Tc_error_325  = [ NaN ; NaN ; NaN ];

%-- Image #326:
omc_326 = [ NaN ; NaN ; NaN ];
Tc_326  = [ NaN ; NaN ; NaN ];
omc_error_326 = [ NaN ; NaN ; NaN ];
Tc_error_326  = [ NaN ; NaN ; NaN ];

%-- Image #327:
omc_327 = [ NaN ; NaN ; NaN ];
Tc_327  = [ NaN ; NaN ; NaN ];
omc_error_327 = [ NaN ; NaN ; NaN ];
Tc_error_327  = [ NaN ; NaN ; NaN ];

%-- Image #328:
omc_328 = [ NaN ; NaN ; NaN ];
Tc_328  = [ NaN ; NaN ; NaN ];
omc_error_328 = [ NaN ; NaN ; NaN ];
Tc_error_328  = [ NaN ; NaN ; NaN ];

%-- Image #329:
omc_329 = [ NaN ; NaN ; NaN ];
Tc_329  = [ NaN ; NaN ; NaN ];
omc_error_329 = [ NaN ; NaN ; NaN ];
Tc_error_329  = [ NaN ; NaN ; NaN ];

%-- Image #330:
omc_330 = [ NaN ; NaN ; NaN ];
Tc_330  = [ NaN ; NaN ; NaN ];
omc_error_330 = [ NaN ; NaN ; NaN ];
Tc_error_330  = [ NaN ; NaN ; NaN ];

%-- Image #331:
omc_331 = [ NaN ; NaN ; NaN ];
Tc_331  = [ NaN ; NaN ; NaN ];
omc_error_331 = [ NaN ; NaN ; NaN ];
Tc_error_331  = [ NaN ; NaN ; NaN ];

%-- Image #332:
omc_332 = [ NaN ; NaN ; NaN ];
Tc_332  = [ NaN ; NaN ; NaN ];
omc_error_332 = [ NaN ; NaN ; NaN ];
Tc_error_332  = [ NaN ; NaN ; NaN ];

%-- Image #333:
omc_333 = [ NaN ; NaN ; NaN ];
Tc_333  = [ NaN ; NaN ; NaN ];
omc_error_333 = [ NaN ; NaN ; NaN ];
Tc_error_333  = [ NaN ; NaN ; NaN ];

%-- Image #334:
omc_334 = [ NaN ; NaN ; NaN ];
Tc_334  = [ NaN ; NaN ; NaN ];
omc_error_334 = [ NaN ; NaN ; NaN ];
Tc_error_334  = [ NaN ; NaN ; NaN ];

%-- Image #335:
omc_335 = [ NaN ; NaN ; NaN ];
Tc_335  = [ NaN ; NaN ; NaN ];
omc_error_335 = [ NaN ; NaN ; NaN ];
Tc_error_335  = [ NaN ; NaN ; NaN ];

%-- Image #336:
omc_336 = [ NaN ; NaN ; NaN ];
Tc_336  = [ NaN ; NaN ; NaN ];
omc_error_336 = [ NaN ; NaN ; NaN ];
Tc_error_336  = [ NaN ; NaN ; NaN ];

%-- Image #337:
omc_337 = [ NaN ; NaN ; NaN ];
Tc_337  = [ NaN ; NaN ; NaN ];
omc_error_337 = [ NaN ; NaN ; NaN ];
Tc_error_337  = [ NaN ; NaN ; NaN ];

%-- Image #338:
omc_338 = [ NaN ; NaN ; NaN ];
Tc_338  = [ NaN ; NaN ; NaN ];
omc_error_338 = [ NaN ; NaN ; NaN ];
Tc_error_338  = [ NaN ; NaN ; NaN ];

%-- Image #339:
omc_339 = [ NaN ; NaN ; NaN ];
Tc_339  = [ NaN ; NaN ; NaN ];
omc_error_339 = [ NaN ; NaN ; NaN ];
Tc_error_339  = [ NaN ; NaN ; NaN ];

%-- Image #340:
omc_340 = [ NaN ; NaN ; NaN ];
Tc_340  = [ NaN ; NaN ; NaN ];
omc_error_340 = [ NaN ; NaN ; NaN ];
Tc_error_340  = [ NaN ; NaN ; NaN ];

%-- Image #341:
omc_341 = [ NaN ; NaN ; NaN ];
Tc_341  = [ NaN ; NaN ; NaN ];
omc_error_341 = [ NaN ; NaN ; NaN ];
Tc_error_341  = [ NaN ; NaN ; NaN ];

%-- Image #342:
omc_342 = [ NaN ; NaN ; NaN ];
Tc_342  = [ NaN ; NaN ; NaN ];
omc_error_342 = [ NaN ; NaN ; NaN ];
Tc_error_342  = [ NaN ; NaN ; NaN ];

%-- Image #343:
omc_343 = [ NaN ; NaN ; NaN ];
Tc_343  = [ NaN ; NaN ; NaN ];
omc_error_343 = [ NaN ; NaN ; NaN ];
Tc_error_343  = [ NaN ; NaN ; NaN ];

%-- Image #344:
omc_344 = [ NaN ; NaN ; NaN ];
Tc_344  = [ NaN ; NaN ; NaN ];
omc_error_344 = [ NaN ; NaN ; NaN ];
Tc_error_344  = [ NaN ; NaN ; NaN ];

%-- Image #345:
omc_345 = [ NaN ; NaN ; NaN ];
Tc_345  = [ NaN ; NaN ; NaN ];
omc_error_345 = [ NaN ; NaN ; NaN ];
Tc_error_345  = [ NaN ; NaN ; NaN ];

%-- Image #346:
omc_346 = [ NaN ; NaN ; NaN ];
Tc_346  = [ NaN ; NaN ; NaN ];
omc_error_346 = [ NaN ; NaN ; NaN ];
Tc_error_346  = [ NaN ; NaN ; NaN ];

%-- Image #347:
omc_347 = [ NaN ; NaN ; NaN ];
Tc_347  = [ NaN ; NaN ; NaN ];
omc_error_347 = [ NaN ; NaN ; NaN ];
Tc_error_347  = [ NaN ; NaN ; NaN ];

%-- Image #348:
omc_348 = [ NaN ; NaN ; NaN ];
Tc_348  = [ NaN ; NaN ; NaN ];
omc_error_348 = [ NaN ; NaN ; NaN ];
Tc_error_348  = [ NaN ; NaN ; NaN ];

%-- Image #349:
omc_349 = [ NaN ; NaN ; NaN ];
Tc_349  = [ NaN ; NaN ; NaN ];
omc_error_349 = [ NaN ; NaN ; NaN ];
Tc_error_349  = [ NaN ; NaN ; NaN ];

%-- Image #350:
omc_350 = [ NaN ; NaN ; NaN ];
Tc_350  = [ NaN ; NaN ; NaN ];
omc_error_350 = [ NaN ; NaN ; NaN ];
Tc_error_350  = [ NaN ; NaN ; NaN ];

%-- Image #351:
omc_351 = [ -2.104685e+000 ; -2.261954e+000 ; -4.074904e-001 ];
Tc_351  = [ -1.301986e+002 ; -1.151842e+002 ; 2.291767e+002 ];
omc_error_351 = [ 1.893883e-002 ; 1.709247e-002 ; 3.817930e-002 ];
Tc_error_351  = [ 3.338971e+000 ; 3.574618e+000 ; 5.537508e+000 ];

%-- Image #352:
omc_352 = [ NaN ; NaN ; NaN ];
Tc_352  = [ NaN ; NaN ; NaN ];
omc_error_352 = [ NaN ; NaN ; NaN ];
Tc_error_352  = [ NaN ; NaN ; NaN ];

%-- Image #353:
omc_353 = [ NaN ; NaN ; NaN ];
Tc_353  = [ NaN ; NaN ; NaN ];
omc_error_353 = [ NaN ; NaN ; NaN ];
Tc_error_353  = [ NaN ; NaN ; NaN ];

%-- Image #354:
omc_354 = [ NaN ; NaN ; NaN ];
Tc_354  = [ NaN ; NaN ; NaN ];
omc_error_354 = [ NaN ; NaN ; NaN ];
Tc_error_354  = [ NaN ; NaN ; NaN ];

%-- Image #355:
omc_355 = [ NaN ; NaN ; NaN ];
Tc_355  = [ NaN ; NaN ; NaN ];
omc_error_355 = [ NaN ; NaN ; NaN ];
Tc_error_355  = [ NaN ; NaN ; NaN ];

%-- Image #356:
omc_356 = [ NaN ; NaN ; NaN ];
Tc_356  = [ NaN ; NaN ; NaN ];
omc_error_356 = [ NaN ; NaN ; NaN ];
Tc_error_356  = [ NaN ; NaN ; NaN ];

%-- Image #357:
omc_357 = [ NaN ; NaN ; NaN ];
Tc_357  = [ NaN ; NaN ; NaN ];
omc_error_357 = [ NaN ; NaN ; NaN ];
Tc_error_357  = [ NaN ; NaN ; NaN ];

%-- Image #358:
omc_358 = [ NaN ; NaN ; NaN ];
Tc_358  = [ NaN ; NaN ; NaN ];
omc_error_358 = [ NaN ; NaN ; NaN ];
Tc_error_358  = [ NaN ; NaN ; NaN ];

%-- Image #359:
omc_359 = [ NaN ; NaN ; NaN ];
Tc_359  = [ NaN ; NaN ; NaN ];
omc_error_359 = [ NaN ; NaN ; NaN ];
Tc_error_359  = [ NaN ; NaN ; NaN ];

%-- Image #360:
omc_360 = [ NaN ; NaN ; NaN ];
Tc_360  = [ NaN ; NaN ; NaN ];
omc_error_360 = [ NaN ; NaN ; NaN ];
Tc_error_360  = [ NaN ; NaN ; NaN ];

%-- Image #361:
omc_361 = [ NaN ; NaN ; NaN ];
Tc_361  = [ NaN ; NaN ; NaN ];
omc_error_361 = [ NaN ; NaN ; NaN ];
Tc_error_361  = [ NaN ; NaN ; NaN ];

%-- Image #362:
omc_362 = [ NaN ; NaN ; NaN ];
Tc_362  = [ NaN ; NaN ; NaN ];
omc_error_362 = [ NaN ; NaN ; NaN ];
Tc_error_362  = [ NaN ; NaN ; NaN ];

%-- Image #363:
omc_363 = [ NaN ; NaN ; NaN ];
Tc_363  = [ NaN ; NaN ; NaN ];
omc_error_363 = [ NaN ; NaN ; NaN ];
Tc_error_363  = [ NaN ; NaN ; NaN ];

%-- Image #364:
omc_364 = [ NaN ; NaN ; NaN ];
Tc_364  = [ NaN ; NaN ; NaN ];
omc_error_364 = [ NaN ; NaN ; NaN ];
Tc_error_364  = [ NaN ; NaN ; NaN ];

%-- Image #365:
omc_365 = [ NaN ; NaN ; NaN ];
Tc_365  = [ NaN ; NaN ; NaN ];
omc_error_365 = [ NaN ; NaN ; NaN ];
Tc_error_365  = [ NaN ; NaN ; NaN ];

%-- Image #366:
omc_366 = [ NaN ; NaN ; NaN ];
Tc_366  = [ NaN ; NaN ; NaN ];
omc_error_366 = [ NaN ; NaN ; NaN ];
Tc_error_366  = [ NaN ; NaN ; NaN ];

%-- Image #367:
omc_367 = [ NaN ; NaN ; NaN ];
Tc_367  = [ NaN ; NaN ; NaN ];
omc_error_367 = [ NaN ; NaN ; NaN ];
Tc_error_367  = [ NaN ; NaN ; NaN ];

%-- Image #368:
omc_368 = [ NaN ; NaN ; NaN ];
Tc_368  = [ NaN ; NaN ; NaN ];
omc_error_368 = [ NaN ; NaN ; NaN ];
Tc_error_368  = [ NaN ; NaN ; NaN ];

%-- Image #369:
omc_369 = [ NaN ; NaN ; NaN ];
Tc_369  = [ NaN ; NaN ; NaN ];
omc_error_369 = [ NaN ; NaN ; NaN ];
Tc_error_369  = [ NaN ; NaN ; NaN ];

%-- Image #370:
omc_370 = [ NaN ; NaN ; NaN ];
Tc_370  = [ NaN ; NaN ; NaN ];
omc_error_370 = [ NaN ; NaN ; NaN ];
Tc_error_370  = [ NaN ; NaN ; NaN ];

%-- Image #371:
omc_371 = [ NaN ; NaN ; NaN ];
Tc_371  = [ NaN ; NaN ; NaN ];
omc_error_371 = [ NaN ; NaN ; NaN ];
Tc_error_371  = [ NaN ; NaN ; NaN ];

%-- Image #372:
omc_372 = [ NaN ; NaN ; NaN ];
Tc_372  = [ NaN ; NaN ; NaN ];
omc_error_372 = [ NaN ; NaN ; NaN ];
Tc_error_372  = [ NaN ; NaN ; NaN ];

%-- Image #373:
omc_373 = [ NaN ; NaN ; NaN ];
Tc_373  = [ NaN ; NaN ; NaN ];
omc_error_373 = [ NaN ; NaN ; NaN ];
Tc_error_373  = [ NaN ; NaN ; NaN ];

%-- Image #374:
omc_374 = [ NaN ; NaN ; NaN ];
Tc_374  = [ NaN ; NaN ; NaN ];
omc_error_374 = [ NaN ; NaN ; NaN ];
Tc_error_374  = [ NaN ; NaN ; NaN ];

%-- Image #375:
omc_375 = [ NaN ; NaN ; NaN ];
Tc_375  = [ NaN ; NaN ; NaN ];
omc_error_375 = [ NaN ; NaN ; NaN ];
Tc_error_375  = [ NaN ; NaN ; NaN ];

%-- Image #376:
omc_376 = [ NaN ; NaN ; NaN ];
Tc_376  = [ NaN ; NaN ; NaN ];
omc_error_376 = [ NaN ; NaN ; NaN ];
Tc_error_376  = [ NaN ; NaN ; NaN ];

%-- Image #377:
omc_377 = [ NaN ; NaN ; NaN ];
Tc_377  = [ NaN ; NaN ; NaN ];
omc_error_377 = [ NaN ; NaN ; NaN ];
Tc_error_377  = [ NaN ; NaN ; NaN ];

%-- Image #378:
omc_378 = [ NaN ; NaN ; NaN ];
Tc_378  = [ NaN ; NaN ; NaN ];
omc_error_378 = [ NaN ; NaN ; NaN ];
Tc_error_378  = [ NaN ; NaN ; NaN ];

%-- Image #379:
omc_379 = [ NaN ; NaN ; NaN ];
Tc_379  = [ NaN ; NaN ; NaN ];
omc_error_379 = [ NaN ; NaN ; NaN ];
Tc_error_379  = [ NaN ; NaN ; NaN ];

%-- Image #380:
omc_380 = [ NaN ; NaN ; NaN ];
Tc_380  = [ NaN ; NaN ; NaN ];
omc_error_380 = [ NaN ; NaN ; NaN ];
Tc_error_380  = [ NaN ; NaN ; NaN ];

%-- Image #381:
omc_381 = [ NaN ; NaN ; NaN ];
Tc_381  = [ NaN ; NaN ; NaN ];
omc_error_381 = [ NaN ; NaN ; NaN ];
Tc_error_381  = [ NaN ; NaN ; NaN ];

%-- Image #382:
omc_382 = [ NaN ; NaN ; NaN ];
Tc_382  = [ NaN ; NaN ; NaN ];
omc_error_382 = [ NaN ; NaN ; NaN ];
Tc_error_382  = [ NaN ; NaN ; NaN ];

%-- Image #383:
omc_383 = [ NaN ; NaN ; NaN ];
Tc_383  = [ NaN ; NaN ; NaN ];
omc_error_383 = [ NaN ; NaN ; NaN ];
Tc_error_383  = [ NaN ; NaN ; NaN ];

%-- Image #384:
omc_384 = [ NaN ; NaN ; NaN ];
Tc_384  = [ NaN ; NaN ; NaN ];
omc_error_384 = [ NaN ; NaN ; NaN ];
Tc_error_384  = [ NaN ; NaN ; NaN ];

%-- Image #385:
omc_385 = [ NaN ; NaN ; NaN ];
Tc_385  = [ NaN ; NaN ; NaN ];
omc_error_385 = [ NaN ; NaN ; NaN ];
Tc_error_385  = [ NaN ; NaN ; NaN ];

%-- Image #386:
omc_386 = [ NaN ; NaN ; NaN ];
Tc_386  = [ NaN ; NaN ; NaN ];
omc_error_386 = [ NaN ; NaN ; NaN ];
Tc_error_386  = [ NaN ; NaN ; NaN ];

%-- Image #387:
omc_387 = [ NaN ; NaN ; NaN ];
Tc_387  = [ NaN ; NaN ; NaN ];
omc_error_387 = [ NaN ; NaN ; NaN ];
Tc_error_387  = [ NaN ; NaN ; NaN ];

%-- Image #388:
omc_388 = [ NaN ; NaN ; NaN ];
Tc_388  = [ NaN ; NaN ; NaN ];
omc_error_388 = [ NaN ; NaN ; NaN ];
Tc_error_388  = [ NaN ; NaN ; NaN ];

%-- Image #389:
omc_389 = [ NaN ; NaN ; NaN ];
Tc_389  = [ NaN ; NaN ; NaN ];
omc_error_389 = [ NaN ; NaN ; NaN ];
Tc_error_389  = [ NaN ; NaN ; NaN ];

%-- Image #390:
omc_390 = [ NaN ; NaN ; NaN ];
Tc_390  = [ NaN ; NaN ; NaN ];
omc_error_390 = [ NaN ; NaN ; NaN ];
Tc_error_390  = [ NaN ; NaN ; NaN ];

%-- Image #391:
omc_391 = [ NaN ; NaN ; NaN ];
Tc_391  = [ NaN ; NaN ; NaN ];
omc_error_391 = [ NaN ; NaN ; NaN ];
Tc_error_391  = [ NaN ; NaN ; NaN ];

%-- Image #392:
omc_392 = [ NaN ; NaN ; NaN ];
Tc_392  = [ NaN ; NaN ; NaN ];
omc_error_392 = [ NaN ; NaN ; NaN ];
Tc_error_392  = [ NaN ; NaN ; NaN ];

%-- Image #393:
omc_393 = [ NaN ; NaN ; NaN ];
Tc_393  = [ NaN ; NaN ; NaN ];
omc_error_393 = [ NaN ; NaN ; NaN ];
Tc_error_393  = [ NaN ; NaN ; NaN ];

%-- Image #394:
omc_394 = [ NaN ; NaN ; NaN ];
Tc_394  = [ NaN ; NaN ; NaN ];
omc_error_394 = [ NaN ; NaN ; NaN ];
Tc_error_394  = [ NaN ; NaN ; NaN ];

%-- Image #395:
omc_395 = [ NaN ; NaN ; NaN ];
Tc_395  = [ NaN ; NaN ; NaN ];
omc_error_395 = [ NaN ; NaN ; NaN ];
Tc_error_395  = [ NaN ; NaN ; NaN ];

%-- Image #396:
omc_396 = [ NaN ; NaN ; NaN ];
Tc_396  = [ NaN ; NaN ; NaN ];
omc_error_396 = [ NaN ; NaN ; NaN ];
Tc_error_396  = [ NaN ; NaN ; NaN ];

%-- Image #397:
omc_397 = [ NaN ; NaN ; NaN ];
Tc_397  = [ NaN ; NaN ; NaN ];
omc_error_397 = [ NaN ; NaN ; NaN ];
Tc_error_397  = [ NaN ; NaN ; NaN ];

%-- Image #398:
omc_398 = [ NaN ; NaN ; NaN ];
Tc_398  = [ NaN ; NaN ; NaN ];
omc_error_398 = [ NaN ; NaN ; NaN ];
Tc_error_398  = [ NaN ; NaN ; NaN ];

%-- Image #399:
omc_399 = [ NaN ; NaN ; NaN ];
Tc_399  = [ NaN ; NaN ; NaN ];
omc_error_399 = [ NaN ; NaN ; NaN ];
Tc_error_399  = [ NaN ; NaN ; NaN ];

%-- Image #400:
omc_400 = [ NaN ; NaN ; NaN ];
Tc_400  = [ NaN ; NaN ; NaN ];
omc_error_400 = [ NaN ; NaN ; NaN ];
Tc_error_400  = [ NaN ; NaN ; NaN ];

%-- Image #401:
omc_401 = [ NaN ; NaN ; NaN ];
Tc_401  = [ NaN ; NaN ; NaN ];
omc_error_401 = [ NaN ; NaN ; NaN ];
Tc_error_401  = [ NaN ; NaN ; NaN ];

%-- Image #402:
omc_402 = [ NaN ; NaN ; NaN ];
Tc_402  = [ NaN ; NaN ; NaN ];
omc_error_402 = [ NaN ; NaN ; NaN ];
Tc_error_402  = [ NaN ; NaN ; NaN ];

%-- Image #403:
omc_403 = [ NaN ; NaN ; NaN ];
Tc_403  = [ NaN ; NaN ; NaN ];
omc_error_403 = [ NaN ; NaN ; NaN ];
Tc_error_403  = [ NaN ; NaN ; NaN ];

%-- Image #404:
omc_404 = [ NaN ; NaN ; NaN ];
Tc_404  = [ NaN ; NaN ; NaN ];
omc_error_404 = [ NaN ; NaN ; NaN ];
Tc_error_404  = [ NaN ; NaN ; NaN ];

%-- Image #405:
omc_405 = [ NaN ; NaN ; NaN ];
Tc_405  = [ NaN ; NaN ; NaN ];
omc_error_405 = [ NaN ; NaN ; NaN ];
Tc_error_405  = [ NaN ; NaN ; NaN ];

%-- Image #406:
omc_406 = [ NaN ; NaN ; NaN ];
Tc_406  = [ NaN ; NaN ; NaN ];
omc_error_406 = [ NaN ; NaN ; NaN ];
Tc_error_406  = [ NaN ; NaN ; NaN ];

%-- Image #407:
omc_407 = [ NaN ; NaN ; NaN ];
Tc_407  = [ NaN ; NaN ; NaN ];
omc_error_407 = [ NaN ; NaN ; NaN ];
Tc_error_407  = [ NaN ; NaN ; NaN ];

%-- Image #408:
omc_408 = [ NaN ; NaN ; NaN ];
Tc_408  = [ NaN ; NaN ; NaN ];
omc_error_408 = [ NaN ; NaN ; NaN ];
Tc_error_408  = [ NaN ; NaN ; NaN ];

%-- Image #409:
omc_409 = [ NaN ; NaN ; NaN ];
Tc_409  = [ NaN ; NaN ; NaN ];
omc_error_409 = [ NaN ; NaN ; NaN ];
Tc_error_409  = [ NaN ; NaN ; NaN ];

%-- Image #410:
omc_410 = [ NaN ; NaN ; NaN ];
Tc_410  = [ NaN ; NaN ; NaN ];
omc_error_410 = [ NaN ; NaN ; NaN ];
Tc_error_410  = [ NaN ; NaN ; NaN ];

%-- Image #411:
omc_411 = [ NaN ; NaN ; NaN ];
Tc_411  = [ NaN ; NaN ; NaN ];
omc_error_411 = [ NaN ; NaN ; NaN ];
Tc_error_411  = [ NaN ; NaN ; NaN ];

%-- Image #412:
omc_412 = [ NaN ; NaN ; NaN ];
Tc_412  = [ NaN ; NaN ; NaN ];
omc_error_412 = [ NaN ; NaN ; NaN ];
Tc_error_412  = [ NaN ; NaN ; NaN ];

%-- Image #413:
omc_413 = [ NaN ; NaN ; NaN ];
Tc_413  = [ NaN ; NaN ; NaN ];
omc_error_413 = [ NaN ; NaN ; NaN ];
Tc_error_413  = [ NaN ; NaN ; NaN ];

%-- Image #414:
omc_414 = [ NaN ; NaN ; NaN ];
Tc_414  = [ NaN ; NaN ; NaN ];
omc_error_414 = [ NaN ; NaN ; NaN ];
Tc_error_414  = [ NaN ; NaN ; NaN ];

%-- Image #415:
omc_415 = [ NaN ; NaN ; NaN ];
Tc_415  = [ NaN ; NaN ; NaN ];
omc_error_415 = [ NaN ; NaN ; NaN ];
Tc_error_415  = [ NaN ; NaN ; NaN ];

%-- Image #416:
omc_416 = [ NaN ; NaN ; NaN ];
Tc_416  = [ NaN ; NaN ; NaN ];
omc_error_416 = [ NaN ; NaN ; NaN ];
Tc_error_416  = [ NaN ; NaN ; NaN ];

%-- Image #417:
omc_417 = [ NaN ; NaN ; NaN ];
Tc_417  = [ NaN ; NaN ; NaN ];
omc_error_417 = [ NaN ; NaN ; NaN ];
Tc_error_417  = [ NaN ; NaN ; NaN ];

%-- Image #418:
omc_418 = [ NaN ; NaN ; NaN ];
Tc_418  = [ NaN ; NaN ; NaN ];
omc_error_418 = [ NaN ; NaN ; NaN ];
Tc_error_418  = [ NaN ; NaN ; NaN ];

%-- Image #419:
omc_419 = [ NaN ; NaN ; NaN ];
Tc_419  = [ NaN ; NaN ; NaN ];
omc_error_419 = [ NaN ; NaN ; NaN ];
Tc_error_419  = [ NaN ; NaN ; NaN ];

%-- Image #420:
omc_420 = [ NaN ; NaN ; NaN ];
Tc_420  = [ NaN ; NaN ; NaN ];
omc_error_420 = [ NaN ; NaN ; NaN ];
Tc_error_420  = [ NaN ; NaN ; NaN ];

%-- Image #421:
omc_421 = [ NaN ; NaN ; NaN ];
Tc_421  = [ NaN ; NaN ; NaN ];
omc_error_421 = [ NaN ; NaN ; NaN ];
Tc_error_421  = [ NaN ; NaN ; NaN ];

%-- Image #422:
omc_422 = [ NaN ; NaN ; NaN ];
Tc_422  = [ NaN ; NaN ; NaN ];
omc_error_422 = [ NaN ; NaN ; NaN ];
Tc_error_422  = [ NaN ; NaN ; NaN ];

%-- Image #423:
omc_423 = [ NaN ; NaN ; NaN ];
Tc_423  = [ NaN ; NaN ; NaN ];
omc_error_423 = [ NaN ; NaN ; NaN ];
Tc_error_423  = [ NaN ; NaN ; NaN ];

%-- Image #424:
omc_424 = [ NaN ; NaN ; NaN ];
Tc_424  = [ NaN ; NaN ; NaN ];
omc_error_424 = [ NaN ; NaN ; NaN ];
Tc_error_424  = [ NaN ; NaN ; NaN ];

%-- Image #425:
omc_425 = [ NaN ; NaN ; NaN ];
Tc_425  = [ NaN ; NaN ; NaN ];
omc_error_425 = [ NaN ; NaN ; NaN ];
Tc_error_425  = [ NaN ; NaN ; NaN ];

%-- Image #426:
omc_426 = [ NaN ; NaN ; NaN ];
Tc_426  = [ NaN ; NaN ; NaN ];
omc_error_426 = [ NaN ; NaN ; NaN ];
Tc_error_426  = [ NaN ; NaN ; NaN ];

%-- Image #427:
omc_427 = [ NaN ; NaN ; NaN ];
Tc_427  = [ NaN ; NaN ; NaN ];
omc_error_427 = [ NaN ; NaN ; NaN ];
Tc_error_427  = [ NaN ; NaN ; NaN ];

%-- Image #428:
omc_428 = [ NaN ; NaN ; NaN ];
Tc_428  = [ NaN ; NaN ; NaN ];
omc_error_428 = [ NaN ; NaN ; NaN ];
Tc_error_428  = [ NaN ; NaN ; NaN ];

%-- Image #429:
omc_429 = [ NaN ; NaN ; NaN ];
Tc_429  = [ NaN ; NaN ; NaN ];
omc_error_429 = [ NaN ; NaN ; NaN ];
Tc_error_429  = [ NaN ; NaN ; NaN ];

%-- Image #430:
omc_430 = [ NaN ; NaN ; NaN ];
Tc_430  = [ NaN ; NaN ; NaN ];
omc_error_430 = [ NaN ; NaN ; NaN ];
Tc_error_430  = [ NaN ; NaN ; NaN ];

%-- Image #431:
omc_431 = [ NaN ; NaN ; NaN ];
Tc_431  = [ NaN ; NaN ; NaN ];
omc_error_431 = [ NaN ; NaN ; NaN ];
Tc_error_431  = [ NaN ; NaN ; NaN ];

%-- Image #432:
omc_432 = [ NaN ; NaN ; NaN ];
Tc_432  = [ NaN ; NaN ; NaN ];
omc_error_432 = [ NaN ; NaN ; NaN ];
Tc_error_432  = [ NaN ; NaN ; NaN ];

%-- Image #433:
omc_433 = [ NaN ; NaN ; NaN ];
Tc_433  = [ NaN ; NaN ; NaN ];
omc_error_433 = [ NaN ; NaN ; NaN ];
Tc_error_433  = [ NaN ; NaN ; NaN ];

%-- Image #434:
omc_434 = [ NaN ; NaN ; NaN ];
Tc_434  = [ NaN ; NaN ; NaN ];
omc_error_434 = [ NaN ; NaN ; NaN ];
Tc_error_434  = [ NaN ; NaN ; NaN ];

%-- Image #435:
omc_435 = [ NaN ; NaN ; NaN ];
Tc_435  = [ NaN ; NaN ; NaN ];
omc_error_435 = [ NaN ; NaN ; NaN ];
Tc_error_435  = [ NaN ; NaN ; NaN ];

%-- Image #436:
omc_436 = [ NaN ; NaN ; NaN ];
Tc_436  = [ NaN ; NaN ; NaN ];
omc_error_436 = [ NaN ; NaN ; NaN ];
Tc_error_436  = [ NaN ; NaN ; NaN ];

%-- Image #437:
omc_437 = [ NaN ; NaN ; NaN ];
Tc_437  = [ NaN ; NaN ; NaN ];
omc_error_437 = [ NaN ; NaN ; NaN ];
Tc_error_437  = [ NaN ; NaN ; NaN ];

%-- Image #438:
omc_438 = [ NaN ; NaN ; NaN ];
Tc_438  = [ NaN ; NaN ; NaN ];
omc_error_438 = [ NaN ; NaN ; NaN ];
Tc_error_438  = [ NaN ; NaN ; NaN ];

%-- Image #439:
omc_439 = [ NaN ; NaN ; NaN ];
Tc_439  = [ NaN ; NaN ; NaN ];
omc_error_439 = [ NaN ; NaN ; NaN ];
Tc_error_439  = [ NaN ; NaN ; NaN ];

%-- Image #440:
omc_440 = [ NaN ; NaN ; NaN ];
Tc_440  = [ NaN ; NaN ; NaN ];
omc_error_440 = [ NaN ; NaN ; NaN ];
Tc_error_440  = [ NaN ; NaN ; NaN ];

%-- Image #441:
omc_441 = [ NaN ; NaN ; NaN ];
Tc_441  = [ NaN ; NaN ; NaN ];
omc_error_441 = [ NaN ; NaN ; NaN ];
Tc_error_441  = [ NaN ; NaN ; NaN ];

%-- Image #442:
omc_442 = [ NaN ; NaN ; NaN ];
Tc_442  = [ NaN ; NaN ; NaN ];
omc_error_442 = [ NaN ; NaN ; NaN ];
Tc_error_442  = [ NaN ; NaN ; NaN ];

%-- Image #443:
omc_443 = [ 1.967013e+000 ; 1.949049e+000 ; -4.871993e-001 ];
Tc_443  = [ -1.304677e+002 ; -7.326726e+001 ; 2.981743e+002 ];
omc_error_443 = [ 1.300916e-002 ; 1.528324e-002 ; 2.782337e-002 ];
Tc_error_443  = [ 3.887896e+000 ; 4.192118e+000 ; 4.958612e+000 ];

%-- Image #444:
omc_444 = [ NaN ; NaN ; NaN ];
Tc_444  = [ NaN ; NaN ; NaN ];
omc_error_444 = [ NaN ; NaN ; NaN ];
Tc_error_444  = [ NaN ; NaN ; NaN ];

%-- Image #445:
omc_445 = [ NaN ; NaN ; NaN ];
Tc_445  = [ NaN ; NaN ; NaN ];
omc_error_445 = [ NaN ; NaN ; NaN ];
Tc_error_445  = [ NaN ; NaN ; NaN ];

%-- Image #446:
omc_446 = [ NaN ; NaN ; NaN ];
Tc_446  = [ NaN ; NaN ; NaN ];
omc_error_446 = [ NaN ; NaN ; NaN ];
Tc_error_446  = [ NaN ; NaN ; NaN ];

%-- Image #447:
omc_447 = [ NaN ; NaN ; NaN ];
Tc_447  = [ NaN ; NaN ; NaN ];
omc_error_447 = [ NaN ; NaN ; NaN ];
Tc_error_447  = [ NaN ; NaN ; NaN ];

%-- Image #448:
omc_448 = [ NaN ; NaN ; NaN ];
Tc_448  = [ NaN ; NaN ; NaN ];
omc_error_448 = [ NaN ; NaN ; NaN ];
Tc_error_448  = [ NaN ; NaN ; NaN ];

%-- Image #449:
omc_449 = [ NaN ; NaN ; NaN ];
Tc_449  = [ NaN ; NaN ; NaN ];
omc_error_449 = [ NaN ; NaN ; NaN ];
Tc_error_449  = [ NaN ; NaN ; NaN ];

%-- Image #450:
omc_450 = [ NaN ; NaN ; NaN ];
Tc_450  = [ NaN ; NaN ; NaN ];
omc_error_450 = [ NaN ; NaN ; NaN ];
Tc_error_450  = [ NaN ; NaN ; NaN ];

%-- Image #451:
omc_451 = [ NaN ; NaN ; NaN ];
Tc_451  = [ NaN ; NaN ; NaN ];
omc_error_451 = [ NaN ; NaN ; NaN ];
Tc_error_451  = [ NaN ; NaN ; NaN ];

%-- Image #452:
omc_452 = [ NaN ; NaN ; NaN ];
Tc_452  = [ NaN ; NaN ; NaN ];
omc_error_452 = [ NaN ; NaN ; NaN ];
Tc_error_452  = [ NaN ; NaN ; NaN ];

%-- Image #453:
omc_453 = [ NaN ; NaN ; NaN ];
Tc_453  = [ NaN ; NaN ; NaN ];
omc_error_453 = [ NaN ; NaN ; NaN ];
Tc_error_453  = [ NaN ; NaN ; NaN ];

%-- Image #454:
omc_454 = [ NaN ; NaN ; NaN ];
Tc_454  = [ NaN ; NaN ; NaN ];
omc_error_454 = [ NaN ; NaN ; NaN ];
Tc_error_454  = [ NaN ; NaN ; NaN ];

%-- Image #455:
omc_455 = [ NaN ; NaN ; NaN ];
Tc_455  = [ NaN ; NaN ; NaN ];
omc_error_455 = [ NaN ; NaN ; NaN ];
Tc_error_455  = [ NaN ; NaN ; NaN ];

%-- Image #456:
omc_456 = [ NaN ; NaN ; NaN ];
Tc_456  = [ NaN ; NaN ; NaN ];
omc_error_456 = [ NaN ; NaN ; NaN ];
Tc_error_456  = [ NaN ; NaN ; NaN ];

%-- Image #457:
omc_457 = [ NaN ; NaN ; NaN ];
Tc_457  = [ NaN ; NaN ; NaN ];
omc_error_457 = [ NaN ; NaN ; NaN ];
Tc_error_457  = [ NaN ; NaN ; NaN ];

%-- Image #458:
omc_458 = [ NaN ; NaN ; NaN ];
Tc_458  = [ NaN ; NaN ; NaN ];
omc_error_458 = [ NaN ; NaN ; NaN ];
Tc_error_458  = [ NaN ; NaN ; NaN ];

%-- Image #459:
omc_459 = [ NaN ; NaN ; NaN ];
Tc_459  = [ NaN ; NaN ; NaN ];
omc_error_459 = [ NaN ; NaN ; NaN ];
Tc_error_459  = [ NaN ; NaN ; NaN ];

%-- Image #460:
omc_460 = [ NaN ; NaN ; NaN ];
Tc_460  = [ NaN ; NaN ; NaN ];
omc_error_460 = [ NaN ; NaN ; NaN ];
Tc_error_460  = [ NaN ; NaN ; NaN ];

%-- Image #461:
omc_461 = [ NaN ; NaN ; NaN ];
Tc_461  = [ NaN ; NaN ; NaN ];
omc_error_461 = [ NaN ; NaN ; NaN ];
Tc_error_461  = [ NaN ; NaN ; NaN ];

%-- Image #462:
omc_462 = [ NaN ; NaN ; NaN ];
Tc_462  = [ NaN ; NaN ; NaN ];
omc_error_462 = [ NaN ; NaN ; NaN ];
Tc_error_462  = [ NaN ; NaN ; NaN ];

%-- Image #463:
omc_463 = [ NaN ; NaN ; NaN ];
Tc_463  = [ NaN ; NaN ; NaN ];
omc_error_463 = [ NaN ; NaN ; NaN ];
Tc_error_463  = [ NaN ; NaN ; NaN ];

%-- Image #464:
omc_464 = [ NaN ; NaN ; NaN ];
Tc_464  = [ NaN ; NaN ; NaN ];
omc_error_464 = [ NaN ; NaN ; NaN ];
Tc_error_464  = [ NaN ; NaN ; NaN ];

%-- Image #465:
omc_465 = [ NaN ; NaN ; NaN ];
Tc_465  = [ NaN ; NaN ; NaN ];
omc_error_465 = [ NaN ; NaN ; NaN ];
Tc_error_465  = [ NaN ; NaN ; NaN ];

%-- Image #466:
omc_466 = [ NaN ; NaN ; NaN ];
Tc_466  = [ NaN ; NaN ; NaN ];
omc_error_466 = [ NaN ; NaN ; NaN ];
Tc_error_466  = [ NaN ; NaN ; NaN ];

%-- Image #467:
omc_467 = [ NaN ; NaN ; NaN ];
Tc_467  = [ NaN ; NaN ; NaN ];
omc_error_467 = [ NaN ; NaN ; NaN ];
Tc_error_467  = [ NaN ; NaN ; NaN ];

%-- Image #468:
omc_468 = [ NaN ; NaN ; NaN ];
Tc_468  = [ NaN ; NaN ; NaN ];
omc_error_468 = [ NaN ; NaN ; NaN ];
Tc_error_468  = [ NaN ; NaN ; NaN ];

%-- Image #469:
omc_469 = [ NaN ; NaN ; NaN ];
Tc_469  = [ NaN ; NaN ; NaN ];
omc_error_469 = [ NaN ; NaN ; NaN ];
Tc_error_469  = [ NaN ; NaN ; NaN ];

%-- Image #470:
omc_470 = [ NaN ; NaN ; NaN ];
Tc_470  = [ NaN ; NaN ; NaN ];
omc_error_470 = [ NaN ; NaN ; NaN ];
Tc_error_470  = [ NaN ; NaN ; NaN ];

%-- Image #471:
omc_471 = [ NaN ; NaN ; NaN ];
Tc_471  = [ NaN ; NaN ; NaN ];
omc_error_471 = [ NaN ; NaN ; NaN ];
Tc_error_471  = [ NaN ; NaN ; NaN ];

%-- Image #472:
omc_472 = [ NaN ; NaN ; NaN ];
Tc_472  = [ NaN ; NaN ; NaN ];
omc_error_472 = [ NaN ; NaN ; NaN ];
Tc_error_472  = [ NaN ; NaN ; NaN ];

%-- Image #473:
omc_473 = [ NaN ; NaN ; NaN ];
Tc_473  = [ NaN ; NaN ; NaN ];
omc_error_473 = [ NaN ; NaN ; NaN ];
Tc_error_473  = [ NaN ; NaN ; NaN ];

%-- Image #474:
omc_474 = [ NaN ; NaN ; NaN ];
Tc_474  = [ NaN ; NaN ; NaN ];
omc_error_474 = [ NaN ; NaN ; NaN ];
Tc_error_474  = [ NaN ; NaN ; NaN ];

%-- Image #475:
omc_475 = [ NaN ; NaN ; NaN ];
Tc_475  = [ NaN ; NaN ; NaN ];
omc_error_475 = [ NaN ; NaN ; NaN ];
Tc_error_475  = [ NaN ; NaN ; NaN ];

%-- Image #476:
omc_476 = [ NaN ; NaN ; NaN ];
Tc_476  = [ NaN ; NaN ; NaN ];
omc_error_476 = [ NaN ; NaN ; NaN ];
Tc_error_476  = [ NaN ; NaN ; NaN ];

%-- Image #477:
omc_477 = [ NaN ; NaN ; NaN ];
Tc_477  = [ NaN ; NaN ; NaN ];
omc_error_477 = [ NaN ; NaN ; NaN ];
Tc_error_477  = [ NaN ; NaN ; NaN ];

%-- Image #478:
omc_478 = [ 2.012347e+000 ; 2.309683e+000 ; -2.749905e-001 ];
Tc_478  = [ -1.156946e+002 ; -1.295901e+002 ; 2.733901e+002 ];
omc_error_478 = [ 1.258445e-002 ; 1.758870e-002 ; 3.296623e-002 ];
Tc_error_478  = [ 3.665625e+000 ; 3.819343e+000 ; 5.397642e+000 ];

%-- Image #479:
omc_479 = [ NaN ; NaN ; NaN ];
Tc_479  = [ NaN ; NaN ; NaN ];
omc_error_479 = [ NaN ; NaN ; NaN ];
Tc_error_479  = [ NaN ; NaN ; NaN ];

%-- Image #480:
omc_480 = [ NaN ; NaN ; NaN ];
Tc_480  = [ NaN ; NaN ; NaN ];
omc_error_480 = [ NaN ; NaN ; NaN ];
Tc_error_480  = [ NaN ; NaN ; NaN ];

%-- Image #481:
omc_481 = [ NaN ; NaN ; NaN ];
Tc_481  = [ NaN ; NaN ; NaN ];
omc_error_481 = [ NaN ; NaN ; NaN ];
Tc_error_481  = [ NaN ; NaN ; NaN ];

%-- Image #482:
omc_482 = [ NaN ; NaN ; NaN ];
Tc_482  = [ NaN ; NaN ; NaN ];
omc_error_482 = [ NaN ; NaN ; NaN ];
Tc_error_482  = [ NaN ; NaN ; NaN ];

%-- Image #483:
omc_483 = [ NaN ; NaN ; NaN ];
Tc_483  = [ NaN ; NaN ; NaN ];
omc_error_483 = [ NaN ; NaN ; NaN ];
Tc_error_483  = [ NaN ; NaN ; NaN ];

%-- Image #484:
omc_484 = [ NaN ; NaN ; NaN ];
Tc_484  = [ NaN ; NaN ; NaN ];
omc_error_484 = [ NaN ; NaN ; NaN ];
Tc_error_484  = [ NaN ; NaN ; NaN ];

%-- Image #485:
omc_485 = [ NaN ; NaN ; NaN ];
Tc_485  = [ NaN ; NaN ; NaN ];
omc_error_485 = [ NaN ; NaN ; NaN ];
Tc_error_485  = [ NaN ; NaN ; NaN ];

%-- Image #486:
omc_486 = [ NaN ; NaN ; NaN ];
Tc_486  = [ NaN ; NaN ; NaN ];
omc_error_486 = [ NaN ; NaN ; NaN ];
Tc_error_486  = [ NaN ; NaN ; NaN ];

%-- Image #487:
omc_487 = [ NaN ; NaN ; NaN ];
Tc_487  = [ NaN ; NaN ; NaN ];
omc_error_487 = [ NaN ; NaN ; NaN ];
Tc_error_487  = [ NaN ; NaN ; NaN ];

%-- Image #488:
omc_488 = [ NaN ; NaN ; NaN ];
Tc_488  = [ NaN ; NaN ; NaN ];
omc_error_488 = [ NaN ; NaN ; NaN ];
Tc_error_488  = [ NaN ; NaN ; NaN ];

%-- Image #489:
omc_489 = [ NaN ; NaN ; NaN ];
Tc_489  = [ NaN ; NaN ; NaN ];
omc_error_489 = [ NaN ; NaN ; NaN ];
Tc_error_489  = [ NaN ; NaN ; NaN ];

%-- Image #490:
omc_490 = [ NaN ; NaN ; NaN ];
Tc_490  = [ NaN ; NaN ; NaN ];
omc_error_490 = [ NaN ; NaN ; NaN ];
Tc_error_490  = [ NaN ; NaN ; NaN ];

%-- Image #491:
omc_491 = [ NaN ; NaN ; NaN ];
Tc_491  = [ NaN ; NaN ; NaN ];
omc_error_491 = [ NaN ; NaN ; NaN ];
Tc_error_491  = [ NaN ; NaN ; NaN ];

%-- Image #492:
omc_492 = [ NaN ; NaN ; NaN ];
Tc_492  = [ NaN ; NaN ; NaN ];
omc_error_492 = [ NaN ; NaN ; NaN ];
Tc_error_492  = [ NaN ; NaN ; NaN ];

%-- Image #493:
omc_493 = [ NaN ; NaN ; NaN ];
Tc_493  = [ NaN ; NaN ; NaN ];
omc_error_493 = [ NaN ; NaN ; NaN ];
Tc_error_493  = [ NaN ; NaN ; NaN ];

%-- Image #494:
omc_494 = [ NaN ; NaN ; NaN ];
Tc_494  = [ NaN ; NaN ; NaN ];
omc_error_494 = [ NaN ; NaN ; NaN ];
Tc_error_494  = [ NaN ; NaN ; NaN ];

%-- Image #495:
omc_495 = [ NaN ; NaN ; NaN ];
Tc_495  = [ NaN ; NaN ; NaN ];
omc_error_495 = [ NaN ; NaN ; NaN ];
Tc_error_495  = [ NaN ; NaN ; NaN ];

%-- Image #496:
omc_496 = [ NaN ; NaN ; NaN ];
Tc_496  = [ NaN ; NaN ; NaN ];
omc_error_496 = [ NaN ; NaN ; NaN ];
Tc_error_496  = [ NaN ; NaN ; NaN ];

%-- Image #497:
omc_497 = [ NaN ; NaN ; NaN ];
Tc_497  = [ NaN ; NaN ; NaN ];
omc_error_497 = [ NaN ; NaN ; NaN ];
Tc_error_497  = [ NaN ; NaN ; NaN ];

%-- Image #498:
omc_498 = [ NaN ; NaN ; NaN ];
Tc_498  = [ NaN ; NaN ; NaN ];
omc_error_498 = [ NaN ; NaN ; NaN ];
Tc_error_498  = [ NaN ; NaN ; NaN ];

%-- Image #499:
omc_499 = [ NaN ; NaN ; NaN ];
Tc_499  = [ NaN ; NaN ; NaN ];
omc_error_499 = [ NaN ; NaN ; NaN ];
Tc_error_499  = [ NaN ; NaN ; NaN ];

%-- Image #500:
omc_500 = [ NaN ; NaN ; NaN ];
Tc_500  = [ NaN ; NaN ; NaN ];
omc_error_500 = [ NaN ; NaN ; NaN ];
Tc_error_500  = [ NaN ; NaN ; NaN ];

%-- Image #501:
omc_501 = [ NaN ; NaN ; NaN ];
Tc_501  = [ NaN ; NaN ; NaN ];
omc_error_501 = [ NaN ; NaN ; NaN ];
Tc_error_501  = [ NaN ; NaN ; NaN ];

%-- Image #502:
omc_502 = [ NaN ; NaN ; NaN ];
Tc_502  = [ NaN ; NaN ; NaN ];
omc_error_502 = [ NaN ; NaN ; NaN ];
Tc_error_502  = [ NaN ; NaN ; NaN ];

%-- Image #503:
omc_503 = [ NaN ; NaN ; NaN ];
Tc_503  = [ NaN ; NaN ; NaN ];
omc_error_503 = [ NaN ; NaN ; NaN ];
Tc_error_503  = [ NaN ; NaN ; NaN ];

%-- Image #504:
omc_504 = [ NaN ; NaN ; NaN ];
Tc_504  = [ NaN ; NaN ; NaN ];
omc_error_504 = [ NaN ; NaN ; NaN ];
Tc_error_504  = [ NaN ; NaN ; NaN ];

%-- Image #505:
omc_505 = [ -1.907753e+000 ; -2.134141e+000 ; -9.460062e-002 ];
Tc_505  = [ -1.288292e+002 ; -1.466575e+002 ; 3.651502e+002 ];
omc_error_505 = [ 2.611155e-002 ; 2.387040e-002 ; 4.703744e-002 ];
Tc_error_505  = [ 4.905090e+000 ; 5.285745e+000 ; 8.034396e+000 ];

%-- Image #506:
omc_506 = [ NaN ; NaN ; NaN ];
Tc_506  = [ NaN ; NaN ; NaN ];
omc_error_506 = [ NaN ; NaN ; NaN ];
Tc_error_506  = [ NaN ; NaN ; NaN ];

%-- Image #507:
omc_507 = [ NaN ; NaN ; NaN ];
Tc_507  = [ NaN ; NaN ; NaN ];
omc_error_507 = [ NaN ; NaN ; NaN ];
Tc_error_507  = [ NaN ; NaN ; NaN ];

%-- Image #508:
omc_508 = [ NaN ; NaN ; NaN ];
Tc_508  = [ NaN ; NaN ; NaN ];
omc_error_508 = [ NaN ; NaN ; NaN ];
Tc_error_508  = [ NaN ; NaN ; NaN ];

%-- Image #509:
omc_509 = [ NaN ; NaN ; NaN ];
Tc_509  = [ NaN ; NaN ; NaN ];
omc_error_509 = [ NaN ; NaN ; NaN ];
Tc_error_509  = [ NaN ; NaN ; NaN ];

%-- Image #510:
omc_510 = [ NaN ; NaN ; NaN ];
Tc_510  = [ NaN ; NaN ; NaN ];
omc_error_510 = [ NaN ; NaN ; NaN ];
Tc_error_510  = [ NaN ; NaN ; NaN ];

%-- Image #511:
omc_511 = [ NaN ; NaN ; NaN ];
Tc_511  = [ NaN ; NaN ; NaN ];
omc_error_511 = [ NaN ; NaN ; NaN ];
Tc_error_511  = [ NaN ; NaN ; NaN ];

%-- Image #512:
omc_512 = [ NaN ; NaN ; NaN ];
Tc_512  = [ NaN ; NaN ; NaN ];
omc_error_512 = [ NaN ; NaN ; NaN ];
Tc_error_512  = [ NaN ; NaN ; NaN ];

%-- Image #513:
omc_513 = [ NaN ; NaN ; NaN ];
Tc_513  = [ NaN ; NaN ; NaN ];
omc_error_513 = [ NaN ; NaN ; NaN ];
Tc_error_513  = [ NaN ; NaN ; NaN ];

%-- Image #514:
omc_514 = [ NaN ; NaN ; NaN ];
Tc_514  = [ NaN ; NaN ; NaN ];
omc_error_514 = [ NaN ; NaN ; NaN ];
Tc_error_514  = [ NaN ; NaN ; NaN ];

%-- Image #515:
omc_515 = [ NaN ; NaN ; NaN ];
Tc_515  = [ NaN ; NaN ; NaN ];
omc_error_515 = [ NaN ; NaN ; NaN ];
Tc_error_515  = [ NaN ; NaN ; NaN ];

%-- Image #516:
omc_516 = [ NaN ; NaN ; NaN ];
Tc_516  = [ NaN ; NaN ; NaN ];
omc_error_516 = [ NaN ; NaN ; NaN ];
Tc_error_516  = [ NaN ; NaN ; NaN ];

%-- Image #517:
omc_517 = [ NaN ; NaN ; NaN ];
Tc_517  = [ NaN ; NaN ; NaN ];
omc_error_517 = [ NaN ; NaN ; NaN ];
Tc_error_517  = [ NaN ; NaN ; NaN ];

%-- Image #518:
omc_518 = [ NaN ; NaN ; NaN ];
Tc_518  = [ NaN ; NaN ; NaN ];
omc_error_518 = [ NaN ; NaN ; NaN ];
Tc_error_518  = [ NaN ; NaN ; NaN ];

%-- Image #519:
omc_519 = [ NaN ; NaN ; NaN ];
Tc_519  = [ NaN ; NaN ; NaN ];
omc_error_519 = [ NaN ; NaN ; NaN ];
Tc_error_519  = [ NaN ; NaN ; NaN ];

%-- Image #520:
omc_520 = [ NaN ; NaN ; NaN ];
Tc_520  = [ NaN ; NaN ; NaN ];
omc_error_520 = [ NaN ; NaN ; NaN ];
Tc_error_520  = [ NaN ; NaN ; NaN ];

%-- Image #521:
omc_521 = [ NaN ; NaN ; NaN ];
Tc_521  = [ NaN ; NaN ; NaN ];
omc_error_521 = [ NaN ; NaN ; NaN ];
Tc_error_521  = [ NaN ; NaN ; NaN ];

%-- Image #522:
omc_522 = [ NaN ; NaN ; NaN ];
Tc_522  = [ NaN ; NaN ; NaN ];
omc_error_522 = [ NaN ; NaN ; NaN ];
Tc_error_522  = [ NaN ; NaN ; NaN ];

%-- Image #523:
omc_523 = [ NaN ; NaN ; NaN ];
Tc_523  = [ NaN ; NaN ; NaN ];
omc_error_523 = [ NaN ; NaN ; NaN ];
Tc_error_523  = [ NaN ; NaN ; NaN ];

%-- Image #524:
omc_524 = [ NaN ; NaN ; NaN ];
Tc_524  = [ NaN ; NaN ; NaN ];
omc_error_524 = [ NaN ; NaN ; NaN ];
Tc_error_524  = [ NaN ; NaN ; NaN ];

%-- Image #525:
omc_525 = [ NaN ; NaN ; NaN ];
Tc_525  = [ NaN ; NaN ; NaN ];
omc_error_525 = [ NaN ; NaN ; NaN ];
Tc_error_525  = [ NaN ; NaN ; NaN ];

%-- Image #526:
omc_526 = [ NaN ; NaN ; NaN ];
Tc_526  = [ NaN ; NaN ; NaN ];
omc_error_526 = [ NaN ; NaN ; NaN ];
Tc_error_526  = [ NaN ; NaN ; NaN ];

%-- Image #527:
omc_527 = [ NaN ; NaN ; NaN ];
Tc_527  = [ NaN ; NaN ; NaN ];
omc_error_527 = [ NaN ; NaN ; NaN ];
Tc_error_527  = [ NaN ; NaN ; NaN ];

%-- Image #528:
omc_528 = [ NaN ; NaN ; NaN ];
Tc_528  = [ NaN ; NaN ; NaN ];
omc_error_528 = [ NaN ; NaN ; NaN ];
Tc_error_528  = [ NaN ; NaN ; NaN ];

%-- Image #529:
omc_529 = [ NaN ; NaN ; NaN ];
Tc_529  = [ NaN ; NaN ; NaN ];
omc_error_529 = [ NaN ; NaN ; NaN ];
Tc_error_529  = [ NaN ; NaN ; NaN ];

%-- Image #530:
omc_530 = [ NaN ; NaN ; NaN ];
Tc_530  = [ NaN ; NaN ; NaN ];
omc_error_530 = [ NaN ; NaN ; NaN ];
Tc_error_530  = [ NaN ; NaN ; NaN ];

%-- Image #531:
omc_531 = [ NaN ; NaN ; NaN ];
Tc_531  = [ NaN ; NaN ; NaN ];
omc_error_531 = [ NaN ; NaN ; NaN ];
Tc_error_531  = [ NaN ; NaN ; NaN ];

%-- Image #532:
omc_532 = [ NaN ; NaN ; NaN ];
Tc_532  = [ NaN ; NaN ; NaN ];
omc_error_532 = [ NaN ; NaN ; NaN ];
Tc_error_532  = [ NaN ; NaN ; NaN ];

%-- Image #533:
omc_533 = [ NaN ; NaN ; NaN ];
Tc_533  = [ NaN ; NaN ; NaN ];
omc_error_533 = [ NaN ; NaN ; NaN ];
Tc_error_533  = [ NaN ; NaN ; NaN ];

%-- Image #534:
omc_534 = [ NaN ; NaN ; NaN ];
Tc_534  = [ NaN ; NaN ; NaN ];
omc_error_534 = [ NaN ; NaN ; NaN ];
Tc_error_534  = [ NaN ; NaN ; NaN ];

%-- Image #535:
omc_535 = [ NaN ; NaN ; NaN ];
Tc_535  = [ NaN ; NaN ; NaN ];
omc_error_535 = [ NaN ; NaN ; NaN ];
Tc_error_535  = [ NaN ; NaN ; NaN ];

%-- Image #536:
omc_536 = [ NaN ; NaN ; NaN ];
Tc_536  = [ NaN ; NaN ; NaN ];
omc_error_536 = [ NaN ; NaN ; NaN ];
Tc_error_536  = [ NaN ; NaN ; NaN ];

%-- Image #537:
omc_537 = [ NaN ; NaN ; NaN ];
Tc_537  = [ NaN ; NaN ; NaN ];
omc_error_537 = [ NaN ; NaN ; NaN ];
Tc_error_537  = [ NaN ; NaN ; NaN ];

%-- Image #538:
omc_538 = [ NaN ; NaN ; NaN ];
Tc_538  = [ NaN ; NaN ; NaN ];
omc_error_538 = [ NaN ; NaN ; NaN ];
Tc_error_538  = [ NaN ; NaN ; NaN ];

%-- Image #539:
omc_539 = [ NaN ; NaN ; NaN ];
Tc_539  = [ NaN ; NaN ; NaN ];
omc_error_539 = [ NaN ; NaN ; NaN ];
Tc_error_539  = [ NaN ; NaN ; NaN ];

%-- Image #540:
omc_540 = [ NaN ; NaN ; NaN ];
Tc_540  = [ NaN ; NaN ; NaN ];
omc_error_540 = [ NaN ; NaN ; NaN ];
Tc_error_540  = [ NaN ; NaN ; NaN ];

%-- Image #541:
omc_541 = [ NaN ; NaN ; NaN ];
Tc_541  = [ NaN ; NaN ; NaN ];
omc_error_541 = [ NaN ; NaN ; NaN ];
Tc_error_541  = [ NaN ; NaN ; NaN ];

%-- Image #542:
omc_542 = [ -1.834852e+000 ; -1.955494e+000 ; -1.623727e-001 ];
Tc_542  = [ -1.257302e+002 ; -1.069743e+002 ; 2.447983e+002 ];
omc_error_542 = [ 1.565234e-002 ; 1.492436e-002 ; 2.554490e-002 ];
Tc_error_542  = [ 3.294203e+000 ; 3.625000e+000 ; 5.127470e+000 ];

%-- Image #543:
omc_543 = [ NaN ; NaN ; NaN ];
Tc_543  = [ NaN ; NaN ; NaN ];
omc_error_543 = [ NaN ; NaN ; NaN ];
Tc_error_543  = [ NaN ; NaN ; NaN ];

%-- Image #544:
omc_544 = [ NaN ; NaN ; NaN ];
Tc_544  = [ NaN ; NaN ; NaN ];
omc_error_544 = [ NaN ; NaN ; NaN ];
Tc_error_544  = [ NaN ; NaN ; NaN ];

%-- Image #545:
omc_545 = [ NaN ; NaN ; NaN ];
Tc_545  = [ NaN ; NaN ; NaN ];
omc_error_545 = [ NaN ; NaN ; NaN ];
Tc_error_545  = [ NaN ; NaN ; NaN ];

%-- Image #546:
omc_546 = [ NaN ; NaN ; NaN ];
Tc_546  = [ NaN ; NaN ; NaN ];
omc_error_546 = [ NaN ; NaN ; NaN ];
Tc_error_546  = [ NaN ; NaN ; NaN ];

%-- Image #547:
omc_547 = [ NaN ; NaN ; NaN ];
Tc_547  = [ NaN ; NaN ; NaN ];
omc_error_547 = [ NaN ; NaN ; NaN ];
Tc_error_547  = [ NaN ; NaN ; NaN ];

%-- Image #548:
omc_548 = [ NaN ; NaN ; NaN ];
Tc_548  = [ NaN ; NaN ; NaN ];
omc_error_548 = [ NaN ; NaN ; NaN ];
Tc_error_548  = [ NaN ; NaN ; NaN ];

%-- Image #549:
omc_549 = [ NaN ; NaN ; NaN ];
Tc_549  = [ NaN ; NaN ; NaN ];
omc_error_549 = [ NaN ; NaN ; NaN ];
Tc_error_549  = [ NaN ; NaN ; NaN ];

%-- Image #550:
omc_550 = [ NaN ; NaN ; NaN ];
Tc_550  = [ NaN ; NaN ; NaN ];
omc_error_550 = [ NaN ; NaN ; NaN ];
Tc_error_550  = [ NaN ; NaN ; NaN ];

%-- Image #551:
omc_551 = [ NaN ; NaN ; NaN ];
Tc_551  = [ NaN ; NaN ; NaN ];
omc_error_551 = [ NaN ; NaN ; NaN ];
Tc_error_551  = [ NaN ; NaN ; NaN ];

%-- Image #552:
omc_552 = [ NaN ; NaN ; NaN ];
Tc_552  = [ NaN ; NaN ; NaN ];
omc_error_552 = [ NaN ; NaN ; NaN ];
Tc_error_552  = [ NaN ; NaN ; NaN ];

%-- Image #553:
omc_553 = [ NaN ; NaN ; NaN ];
Tc_553  = [ NaN ; NaN ; NaN ];
omc_error_553 = [ NaN ; NaN ; NaN ];
Tc_error_553  = [ NaN ; NaN ; NaN ];

%-- Image #554:
omc_554 = [ NaN ; NaN ; NaN ];
Tc_554  = [ NaN ; NaN ; NaN ];
omc_error_554 = [ NaN ; NaN ; NaN ];
Tc_error_554  = [ NaN ; NaN ; NaN ];

%-- Image #555:
omc_555 = [ NaN ; NaN ; NaN ];
Tc_555  = [ NaN ; NaN ; NaN ];
omc_error_555 = [ NaN ; NaN ; NaN ];
Tc_error_555  = [ NaN ; NaN ; NaN ];

%-- Image #556:
omc_556 = [ NaN ; NaN ; NaN ];
Tc_556  = [ NaN ; NaN ; NaN ];
omc_error_556 = [ NaN ; NaN ; NaN ];
Tc_error_556  = [ NaN ; NaN ; NaN ];

%-- Image #557:
omc_557 = [ NaN ; NaN ; NaN ];
Tc_557  = [ NaN ; NaN ; NaN ];
omc_error_557 = [ NaN ; NaN ; NaN ];
Tc_error_557  = [ NaN ; NaN ; NaN ];

%-- Image #558:
omc_558 = [ NaN ; NaN ; NaN ];
Tc_558  = [ NaN ; NaN ; NaN ];
omc_error_558 = [ NaN ; NaN ; NaN ];
Tc_error_558  = [ NaN ; NaN ; NaN ];

%-- Image #559:
omc_559 = [ NaN ; NaN ; NaN ];
Tc_559  = [ NaN ; NaN ; NaN ];
omc_error_559 = [ NaN ; NaN ; NaN ];
Tc_error_559  = [ NaN ; NaN ; NaN ];

%-- Image #560:
omc_560 = [ NaN ; NaN ; NaN ];
Tc_560  = [ NaN ; NaN ; NaN ];
omc_error_560 = [ NaN ; NaN ; NaN ];
Tc_error_560  = [ NaN ; NaN ; NaN ];

%-- Image #561:
omc_561 = [ NaN ; NaN ; NaN ];
Tc_561  = [ NaN ; NaN ; NaN ];
omc_error_561 = [ NaN ; NaN ; NaN ];
Tc_error_561  = [ NaN ; NaN ; NaN ];

%-- Image #562:
omc_562 = [ NaN ; NaN ; NaN ];
Tc_562  = [ NaN ; NaN ; NaN ];
omc_error_562 = [ NaN ; NaN ; NaN ];
Tc_error_562  = [ NaN ; NaN ; NaN ];

%-- Image #563:
omc_563 = [ NaN ; NaN ; NaN ];
Tc_563  = [ NaN ; NaN ; NaN ];
omc_error_563 = [ NaN ; NaN ; NaN ];
Tc_error_563  = [ NaN ; NaN ; NaN ];

%-- Image #564:
omc_564 = [ -1.837343e+000 ; -1.969197e+000 ; -1.718693e-001 ];
Tc_564  = [ -8.391723e+001 ; -1.062332e+002 ; 2.342012e+002 ];
omc_error_564 = [ 1.448154e-002 ; 1.552478e-002 ; 2.602776e-002 ];
Tc_error_564  = [ 3.181402e+000 ; 3.442781e+000 ; 4.767101e+000 ];

%-- Image #565:
omc_565 = [ NaN ; NaN ; NaN ];
Tc_565  = [ NaN ; NaN ; NaN ];
omc_error_565 = [ NaN ; NaN ; NaN ];
Tc_error_565  = [ NaN ; NaN ; NaN ];

%-- Image #566:
omc_566 = [ NaN ; NaN ; NaN ];
Tc_566  = [ NaN ; NaN ; NaN ];
omc_error_566 = [ NaN ; NaN ; NaN ];
Tc_error_566  = [ NaN ; NaN ; NaN ];

%-- Image #567:
omc_567 = [ NaN ; NaN ; NaN ];
Tc_567  = [ NaN ; NaN ; NaN ];
omc_error_567 = [ NaN ; NaN ; NaN ];
Tc_error_567  = [ NaN ; NaN ; NaN ];

%-- Image #568:
omc_568 = [ NaN ; NaN ; NaN ];
Tc_568  = [ NaN ; NaN ; NaN ];
omc_error_568 = [ NaN ; NaN ; NaN ];
Tc_error_568  = [ NaN ; NaN ; NaN ];

%-- Image #569:
omc_569 = [ NaN ; NaN ; NaN ];
Tc_569  = [ NaN ; NaN ; NaN ];
omc_error_569 = [ NaN ; NaN ; NaN ];
Tc_error_569  = [ NaN ; NaN ; NaN ];

%-- Image #570:
omc_570 = [ NaN ; NaN ; NaN ];
Tc_570  = [ NaN ; NaN ; NaN ];
omc_error_570 = [ NaN ; NaN ; NaN ];
Tc_error_570  = [ NaN ; NaN ; NaN ];

%-- Image #571:
omc_571 = [ NaN ; NaN ; NaN ];
Tc_571  = [ NaN ; NaN ; NaN ];
omc_error_571 = [ NaN ; NaN ; NaN ];
Tc_error_571  = [ NaN ; NaN ; NaN ];

%-- Image #572:
omc_572 = [ NaN ; NaN ; NaN ];
Tc_572  = [ NaN ; NaN ; NaN ];
omc_error_572 = [ NaN ; NaN ; NaN ];
Tc_error_572  = [ NaN ; NaN ; NaN ];

%-- Image #573:
omc_573 = [ NaN ; NaN ; NaN ];
Tc_573  = [ NaN ; NaN ; NaN ];
omc_error_573 = [ NaN ; NaN ; NaN ];
Tc_error_573  = [ NaN ; NaN ; NaN ];

%-- Image #574:
omc_574 = [ NaN ; NaN ; NaN ];
Tc_574  = [ NaN ; NaN ; NaN ];
omc_error_574 = [ NaN ; NaN ; NaN ];
Tc_error_574  = [ NaN ; NaN ; NaN ];

%-- Image #575:
omc_575 = [ NaN ; NaN ; NaN ];
Tc_575  = [ NaN ; NaN ; NaN ];
omc_error_575 = [ NaN ; NaN ; NaN ];
Tc_error_575  = [ NaN ; NaN ; NaN ];

%-- Image #576:
omc_576 = [ NaN ; NaN ; NaN ];
Tc_576  = [ NaN ; NaN ; NaN ];
omc_error_576 = [ NaN ; NaN ; NaN ];
Tc_error_576  = [ NaN ; NaN ; NaN ];

%-- Image #577:
omc_577 = [ NaN ; NaN ; NaN ];
Tc_577  = [ NaN ; NaN ; NaN ];
omc_error_577 = [ NaN ; NaN ; NaN ];
Tc_error_577  = [ NaN ; NaN ; NaN ];

%-- Image #578:
omc_578 = [ NaN ; NaN ; NaN ];
Tc_578  = [ NaN ; NaN ; NaN ];
omc_error_578 = [ NaN ; NaN ; NaN ];
Tc_error_578  = [ NaN ; NaN ; NaN ];

%-- Image #579:
omc_579 = [ NaN ; NaN ; NaN ];
Tc_579  = [ NaN ; NaN ; NaN ];
omc_error_579 = [ NaN ; NaN ; NaN ];
Tc_error_579  = [ NaN ; NaN ; NaN ];

%-- Image #580:
omc_580 = [ NaN ; NaN ; NaN ];
Tc_580  = [ NaN ; NaN ; NaN ];
omc_error_580 = [ NaN ; NaN ; NaN ];
Tc_error_580  = [ NaN ; NaN ; NaN ];

%-- Image #581:
omc_581 = [ NaN ; NaN ; NaN ];
Tc_581  = [ NaN ; NaN ; NaN ];
omc_error_581 = [ NaN ; NaN ; NaN ];
Tc_error_581  = [ NaN ; NaN ; NaN ];

%-- Image #582:
omc_582 = [ NaN ; NaN ; NaN ];
Tc_582  = [ NaN ; NaN ; NaN ];
omc_error_582 = [ NaN ; NaN ; NaN ];
Tc_error_582  = [ NaN ; NaN ; NaN ];

%-- Image #583:
omc_583 = [ NaN ; NaN ; NaN ];
Tc_583  = [ NaN ; NaN ; NaN ];
omc_error_583 = [ NaN ; NaN ; NaN ];
Tc_error_583  = [ NaN ; NaN ; NaN ];

%-- Image #584:
omc_584 = [ NaN ; NaN ; NaN ];
Tc_584  = [ NaN ; NaN ; NaN ];
omc_error_584 = [ NaN ; NaN ; NaN ];
Tc_error_584  = [ NaN ; NaN ; NaN ];

%-- Image #585:
omc_585 = [ NaN ; NaN ; NaN ];
Tc_585  = [ NaN ; NaN ; NaN ];
omc_error_585 = [ NaN ; NaN ; NaN ];
Tc_error_585  = [ NaN ; NaN ; NaN ];

%-- Image #586:
omc_586 = [ NaN ; NaN ; NaN ];
Tc_586  = [ NaN ; NaN ; NaN ];
omc_error_586 = [ NaN ; NaN ; NaN ];
Tc_error_586  = [ NaN ; NaN ; NaN ];

%-- Image #587:
omc_587 = [ NaN ; NaN ; NaN ];
Tc_587  = [ NaN ; NaN ; NaN ];
omc_error_587 = [ NaN ; NaN ; NaN ];
Tc_error_587  = [ NaN ; NaN ; NaN ];

%-- Image #588:
omc_588 = [ NaN ; NaN ; NaN ];
Tc_588  = [ NaN ; NaN ; NaN ];
omc_error_588 = [ NaN ; NaN ; NaN ];
Tc_error_588  = [ NaN ; NaN ; NaN ];

%-- Image #589:
omc_589 = [ NaN ; NaN ; NaN ];
Tc_589  = [ NaN ; NaN ; NaN ];
omc_error_589 = [ NaN ; NaN ; NaN ];
Tc_error_589  = [ NaN ; NaN ; NaN ];

%-- Image #590:
omc_590 = [ NaN ; NaN ; NaN ];
Tc_590  = [ NaN ; NaN ; NaN ];
omc_error_590 = [ NaN ; NaN ; NaN ];
Tc_error_590  = [ NaN ; NaN ; NaN ];

%-- Image #591:
omc_591 = [ NaN ; NaN ; NaN ];
Tc_591  = [ NaN ; NaN ; NaN ];
omc_error_591 = [ NaN ; NaN ; NaN ];
Tc_error_591  = [ NaN ; NaN ; NaN ];

%-- Image #592:
omc_592 = [ NaN ; NaN ; NaN ];
Tc_592  = [ NaN ; NaN ; NaN ];
omc_error_592 = [ NaN ; NaN ; NaN ];
Tc_error_592  = [ NaN ; NaN ; NaN ];

%-- Image #593:
omc_593 = [ NaN ; NaN ; NaN ];
Tc_593  = [ NaN ; NaN ; NaN ];
omc_error_593 = [ NaN ; NaN ; NaN ];
Tc_error_593  = [ NaN ; NaN ; NaN ];

%-- Image #594:
omc_594 = [ NaN ; NaN ; NaN ];
Tc_594  = [ NaN ; NaN ; NaN ];
omc_error_594 = [ NaN ; NaN ; NaN ];
Tc_error_594  = [ NaN ; NaN ; NaN ];

%-- Image #595:
omc_595 = [ NaN ; NaN ; NaN ];
Tc_595  = [ NaN ; NaN ; NaN ];
omc_error_595 = [ NaN ; NaN ; NaN ];
Tc_error_595  = [ NaN ; NaN ; NaN ];

%-- Image #596:
omc_596 = [ NaN ; NaN ; NaN ];
Tc_596  = [ NaN ; NaN ; NaN ];
omc_error_596 = [ NaN ; NaN ; NaN ];
Tc_error_596  = [ NaN ; NaN ; NaN ];

%-- Image #597:
omc_597 = [ NaN ; NaN ; NaN ];
Tc_597  = [ NaN ; NaN ; NaN ];
omc_error_597 = [ NaN ; NaN ; NaN ];
Tc_error_597  = [ NaN ; NaN ; NaN ];

%-- Image #598:
omc_598 = [ NaN ; NaN ; NaN ];
Tc_598  = [ NaN ; NaN ; NaN ];
omc_error_598 = [ NaN ; NaN ; NaN ];
Tc_error_598  = [ NaN ; NaN ; NaN ];

%-- Image #599:
omc_599 = [ NaN ; NaN ; NaN ];
Tc_599  = [ NaN ; NaN ; NaN ];
omc_error_599 = [ NaN ; NaN ; NaN ];
Tc_error_599  = [ NaN ; NaN ; NaN ];

%-- Image #600:
omc_600 = [ NaN ; NaN ; NaN ];
Tc_600  = [ NaN ; NaN ; NaN ];
omc_error_600 = [ NaN ; NaN ; NaN ];
Tc_error_600  = [ NaN ; NaN ; NaN ];

%-- Image #601:
omc_601 = [ NaN ; NaN ; NaN ];
Tc_601  = [ NaN ; NaN ; NaN ];
omc_error_601 = [ NaN ; NaN ; NaN ];
Tc_error_601  = [ NaN ; NaN ; NaN ];

%-- Image #602:
omc_602 = [ NaN ; NaN ; NaN ];
Tc_602  = [ NaN ; NaN ; NaN ];
omc_error_602 = [ NaN ; NaN ; NaN ];
Tc_error_602  = [ NaN ; NaN ; NaN ];

%-- Image #603:
omc_603 = [ NaN ; NaN ; NaN ];
Tc_603  = [ NaN ; NaN ; NaN ];
omc_error_603 = [ NaN ; NaN ; NaN ];
Tc_error_603  = [ NaN ; NaN ; NaN ];

%-- Image #604:
omc_604 = [ NaN ; NaN ; NaN ];
Tc_604  = [ NaN ; NaN ; NaN ];
omc_error_604 = [ NaN ; NaN ; NaN ];
Tc_error_604  = [ NaN ; NaN ; NaN ];

%-- Image #605:
omc_605 = [ NaN ; NaN ; NaN ];
Tc_605  = [ NaN ; NaN ; NaN ];
omc_error_605 = [ NaN ; NaN ; NaN ];
Tc_error_605  = [ NaN ; NaN ; NaN ];

%-- Image #606:
omc_606 = [ NaN ; NaN ; NaN ];
Tc_606  = [ NaN ; NaN ; NaN ];
omc_error_606 = [ NaN ; NaN ; NaN ];
Tc_error_606  = [ NaN ; NaN ; NaN ];

%-- Image #607:
omc_607 = [ NaN ; NaN ; NaN ];
Tc_607  = [ NaN ; NaN ; NaN ];
omc_error_607 = [ NaN ; NaN ; NaN ];
Tc_error_607  = [ NaN ; NaN ; NaN ];

%-- Image #608:
omc_608 = [ NaN ; NaN ; NaN ];
Tc_608  = [ NaN ; NaN ; NaN ];
omc_error_608 = [ NaN ; NaN ; NaN ];
Tc_error_608  = [ NaN ; NaN ; NaN ];

%-- Image #609:
omc_609 = [ NaN ; NaN ; NaN ];
Tc_609  = [ NaN ; NaN ; NaN ];
omc_error_609 = [ NaN ; NaN ; NaN ];
Tc_error_609  = [ NaN ; NaN ; NaN ];

%-- Image #610:
omc_610 = [ NaN ; NaN ; NaN ];
Tc_610  = [ NaN ; NaN ; NaN ];
omc_error_610 = [ NaN ; NaN ; NaN ];
Tc_error_610  = [ NaN ; NaN ; NaN ];

%-- Image #611:
omc_611 = [ NaN ; NaN ; NaN ];
Tc_611  = [ NaN ; NaN ; NaN ];
omc_error_611 = [ NaN ; NaN ; NaN ];
Tc_error_611  = [ NaN ; NaN ; NaN ];

%-- Image #612:
omc_612 = [ -2.044179e+000 ; -2.153320e+000 ; -5.940257e-001 ];
Tc_612  = [ -1.238118e+002 ; -9.310952e+001 ; 1.870175e+002 ];
omc_error_612 = [ 1.486980e-002 ; 1.488872e-002 ; 3.020779e-002 ];
Tc_error_612  = [ 2.778021e+000 ; 3.026726e+000 ; 4.608699e+000 ];

%-- Image #613:
omc_613 = [ NaN ; NaN ; NaN ];
Tc_613  = [ NaN ; NaN ; NaN ];
omc_error_613 = [ NaN ; NaN ; NaN ];
Tc_error_613  = [ NaN ; NaN ; NaN ];

%-- Image #614:
omc_614 = [ NaN ; NaN ; NaN ];
Tc_614  = [ NaN ; NaN ; NaN ];
omc_error_614 = [ NaN ; NaN ; NaN ];
Tc_error_614  = [ NaN ; NaN ; NaN ];

%-- Image #615:
omc_615 = [ NaN ; NaN ; NaN ];
Tc_615  = [ NaN ; NaN ; NaN ];
omc_error_615 = [ NaN ; NaN ; NaN ];
Tc_error_615  = [ NaN ; NaN ; NaN ];

%-- Image #616:
omc_616 = [ NaN ; NaN ; NaN ];
Tc_616  = [ NaN ; NaN ; NaN ];
omc_error_616 = [ NaN ; NaN ; NaN ];
Tc_error_616  = [ NaN ; NaN ; NaN ];

%-- Image #617:
omc_617 = [ NaN ; NaN ; NaN ];
Tc_617  = [ NaN ; NaN ; NaN ];
omc_error_617 = [ NaN ; NaN ; NaN ];
Tc_error_617  = [ NaN ; NaN ; NaN ];

%-- Image #618:
omc_618 = [ NaN ; NaN ; NaN ];
Tc_618  = [ NaN ; NaN ; NaN ];
omc_error_618 = [ NaN ; NaN ; NaN ];
Tc_error_618  = [ NaN ; NaN ; NaN ];

%-- Image #619:
omc_619 = [ NaN ; NaN ; NaN ];
Tc_619  = [ NaN ; NaN ; NaN ];
omc_error_619 = [ NaN ; NaN ; NaN ];
Tc_error_619  = [ NaN ; NaN ; NaN ];

%-- Image #620:
omc_620 = [ NaN ; NaN ; NaN ];
Tc_620  = [ NaN ; NaN ; NaN ];
omc_error_620 = [ NaN ; NaN ; NaN ];
Tc_error_620  = [ NaN ; NaN ; NaN ];

%-- Image #621:
omc_621 = [ NaN ; NaN ; NaN ];
Tc_621  = [ NaN ; NaN ; NaN ];
omc_error_621 = [ NaN ; NaN ; NaN ];
Tc_error_621  = [ NaN ; NaN ; NaN ];

%-- Image #622:
omc_622 = [ NaN ; NaN ; NaN ];
Tc_622  = [ NaN ; NaN ; NaN ];
omc_error_622 = [ NaN ; NaN ; NaN ];
Tc_error_622  = [ NaN ; NaN ; NaN ];

%-- Image #623:
omc_623 = [ NaN ; NaN ; NaN ];
Tc_623  = [ NaN ; NaN ; NaN ];
omc_error_623 = [ NaN ; NaN ; NaN ];
Tc_error_623  = [ NaN ; NaN ; NaN ];

%-- Image #624:
omc_624 = [ NaN ; NaN ; NaN ];
Tc_624  = [ NaN ; NaN ; NaN ];
omc_error_624 = [ NaN ; NaN ; NaN ];
Tc_error_624  = [ NaN ; NaN ; NaN ];

%-- Image #625:
omc_625 = [ NaN ; NaN ; NaN ];
Tc_625  = [ NaN ; NaN ; NaN ];
omc_error_625 = [ NaN ; NaN ; NaN ];
Tc_error_625  = [ NaN ; NaN ; NaN ];

%-- Image #626:
omc_626 = [ NaN ; NaN ; NaN ];
Tc_626  = [ NaN ; NaN ; NaN ];
omc_error_626 = [ NaN ; NaN ; NaN ];
Tc_error_626  = [ NaN ; NaN ; NaN ];

%-- Image #627:
omc_627 = [ NaN ; NaN ; NaN ];
Tc_627  = [ NaN ; NaN ; NaN ];
omc_error_627 = [ NaN ; NaN ; NaN ];
Tc_error_627  = [ NaN ; NaN ; NaN ];

%-- Image #628:
omc_628 = [ NaN ; NaN ; NaN ];
Tc_628  = [ NaN ; NaN ; NaN ];
omc_error_628 = [ NaN ; NaN ; NaN ];
Tc_error_628  = [ NaN ; NaN ; NaN ];

%-- Image #629:
omc_629 = [ NaN ; NaN ; NaN ];
Tc_629  = [ NaN ; NaN ; NaN ];
omc_error_629 = [ NaN ; NaN ; NaN ];
Tc_error_629  = [ NaN ; NaN ; NaN ];

%-- Image #630:
omc_630 = [ NaN ; NaN ; NaN ];
Tc_630  = [ NaN ; NaN ; NaN ];
omc_error_630 = [ NaN ; NaN ; NaN ];
Tc_error_630  = [ NaN ; NaN ; NaN ];

%-- Image #631:
omc_631 = [ NaN ; NaN ; NaN ];
Tc_631  = [ NaN ; NaN ; NaN ];
omc_error_631 = [ NaN ; NaN ; NaN ];
Tc_error_631  = [ NaN ; NaN ; NaN ];

%-- Image #632:
omc_632 = [ NaN ; NaN ; NaN ];
Tc_632  = [ NaN ; NaN ; NaN ];
omc_error_632 = [ NaN ; NaN ; NaN ];
Tc_error_632  = [ NaN ; NaN ; NaN ];

%-- Image #633:
omc_633 = [ NaN ; NaN ; NaN ];
Tc_633  = [ NaN ; NaN ; NaN ];
omc_error_633 = [ NaN ; NaN ; NaN ];
Tc_error_633  = [ NaN ; NaN ; NaN ];

%-- Image #634:
omc_634 = [ NaN ; NaN ; NaN ];
Tc_634  = [ NaN ; NaN ; NaN ];
omc_error_634 = [ NaN ; NaN ; NaN ];
Tc_error_634  = [ NaN ; NaN ; NaN ];

%-- Image #635:
omc_635 = [ NaN ; NaN ; NaN ];
Tc_635  = [ NaN ; NaN ; NaN ];
omc_error_635 = [ NaN ; NaN ; NaN ];
Tc_error_635  = [ NaN ; NaN ; NaN ];

%-- Image #636:
omc_636 = [ NaN ; NaN ; NaN ];
Tc_636  = [ NaN ; NaN ; NaN ];
omc_error_636 = [ NaN ; NaN ; NaN ];
Tc_error_636  = [ NaN ; NaN ; NaN ];

%-- Image #637:
omc_637 = [ NaN ; NaN ; NaN ];
Tc_637  = [ NaN ; NaN ; NaN ];
omc_error_637 = [ NaN ; NaN ; NaN ];
Tc_error_637  = [ NaN ; NaN ; NaN ];

%-- Image #638:
omc_638 = [ NaN ; NaN ; NaN ];
Tc_638  = [ NaN ; NaN ; NaN ];
omc_error_638 = [ NaN ; NaN ; NaN ];
Tc_error_638  = [ NaN ; NaN ; NaN ];

%-- Image #639:
omc_639 = [ NaN ; NaN ; NaN ];
Tc_639  = [ NaN ; NaN ; NaN ];
omc_error_639 = [ NaN ; NaN ; NaN ];
Tc_error_639  = [ NaN ; NaN ; NaN ];

%-- Image #640:
omc_640 = [ NaN ; NaN ; NaN ];
Tc_640  = [ NaN ; NaN ; NaN ];
omc_error_640 = [ NaN ; NaN ; NaN ];
Tc_error_640  = [ NaN ; NaN ; NaN ];

%-- Image #641:
omc_641 = [ NaN ; NaN ; NaN ];
Tc_641  = [ NaN ; NaN ; NaN ];
omc_error_641 = [ NaN ; NaN ; NaN ];
Tc_error_641  = [ NaN ; NaN ; NaN ];

%-- Image #642:
omc_642 = [ NaN ; NaN ; NaN ];
Tc_642  = [ NaN ; NaN ; NaN ];
omc_error_642 = [ NaN ; NaN ; NaN ];
Tc_error_642  = [ NaN ; NaN ; NaN ];

%-- Image #643:
omc_643 = [ 2.108522e+000 ; 2.263317e+000 ; -6.584796e-003 ];
Tc_643  = [ -1.229687e+002 ; -1.134654e+002 ; 2.253573e+002 ];
omc_error_643 = [ 1.338228e-002 ; 1.657967e-002 ; 3.159891e-002 ];
Tc_error_643  = [ 3.092482e+000 ; 3.262953e+000 ; 4.969605e+000 ];

%-- Image #644:
omc_644 = [ NaN ; NaN ; NaN ];
Tc_644  = [ NaN ; NaN ; NaN ];
omc_error_644 = [ NaN ; NaN ; NaN ];
Tc_error_644  = [ NaN ; NaN ; NaN ];

%-- Image #645:
omc_645 = [ NaN ; NaN ; NaN ];
Tc_645  = [ NaN ; NaN ; NaN ];
omc_error_645 = [ NaN ; NaN ; NaN ];
Tc_error_645  = [ NaN ; NaN ; NaN ];

%-- Image #646:
omc_646 = [ NaN ; NaN ; NaN ];
Tc_646  = [ NaN ; NaN ; NaN ];
omc_error_646 = [ NaN ; NaN ; NaN ];
Tc_error_646  = [ NaN ; NaN ; NaN ];

%-- Image #647:
omc_647 = [ NaN ; NaN ; NaN ];
Tc_647  = [ NaN ; NaN ; NaN ];
omc_error_647 = [ NaN ; NaN ; NaN ];
Tc_error_647  = [ NaN ; NaN ; NaN ];

%-- Image #648:
omc_648 = [ NaN ; NaN ; NaN ];
Tc_648  = [ NaN ; NaN ; NaN ];
omc_error_648 = [ NaN ; NaN ; NaN ];
Tc_error_648  = [ NaN ; NaN ; NaN ];

%-- Image #649:
omc_649 = [ NaN ; NaN ; NaN ];
Tc_649  = [ NaN ; NaN ; NaN ];
omc_error_649 = [ NaN ; NaN ; NaN ];
Tc_error_649  = [ NaN ; NaN ; NaN ];

%-- Image #650:
omc_650 = [ NaN ; NaN ; NaN ];
Tc_650  = [ NaN ; NaN ; NaN ];
omc_error_650 = [ NaN ; NaN ; NaN ];
Tc_error_650  = [ NaN ; NaN ; NaN ];

%-- Image #651:
omc_651 = [ NaN ; NaN ; NaN ];
Tc_651  = [ NaN ; NaN ; NaN ];
omc_error_651 = [ NaN ; NaN ; NaN ];
Tc_error_651  = [ NaN ; NaN ; NaN ];

%-- Image #652:
omc_652 = [ NaN ; NaN ; NaN ];
Tc_652  = [ NaN ; NaN ; NaN ];
omc_error_652 = [ NaN ; NaN ; NaN ];
Tc_error_652  = [ NaN ; NaN ; NaN ];

%-- Image #653:
omc_653 = [ NaN ; NaN ; NaN ];
Tc_653  = [ NaN ; NaN ; NaN ];
omc_error_653 = [ NaN ; NaN ; NaN ];
Tc_error_653  = [ NaN ; NaN ; NaN ];

%-- Image #654:
omc_654 = [ NaN ; NaN ; NaN ];
Tc_654  = [ NaN ; NaN ; NaN ];
omc_error_654 = [ NaN ; NaN ; NaN ];
Tc_error_654  = [ NaN ; NaN ; NaN ];

%-- Image #655:
omc_655 = [ NaN ; NaN ; NaN ];
Tc_655  = [ NaN ; NaN ; NaN ];
omc_error_655 = [ NaN ; NaN ; NaN ];
Tc_error_655  = [ NaN ; NaN ; NaN ];

%-- Image #656:
omc_656 = [ NaN ; NaN ; NaN ];
Tc_656  = [ NaN ; NaN ; NaN ];
omc_error_656 = [ NaN ; NaN ; NaN ];
Tc_error_656  = [ NaN ; NaN ; NaN ];

%-- Image #657:
omc_657 = [ NaN ; NaN ; NaN ];
Tc_657  = [ NaN ; NaN ; NaN ];
omc_error_657 = [ NaN ; NaN ; NaN ];
Tc_error_657  = [ NaN ; NaN ; NaN ];

%-- Image #658:
omc_658 = [ NaN ; NaN ; NaN ];
Tc_658  = [ NaN ; NaN ; NaN ];
omc_error_658 = [ NaN ; NaN ; NaN ];
Tc_error_658  = [ NaN ; NaN ; NaN ];

%-- Image #659:
omc_659 = [ NaN ; NaN ; NaN ];
Tc_659  = [ NaN ; NaN ; NaN ];
omc_error_659 = [ NaN ; NaN ; NaN ];
Tc_error_659  = [ NaN ; NaN ; NaN ];

%-- Image #660:
omc_660 = [ NaN ; NaN ; NaN ];
Tc_660  = [ NaN ; NaN ; NaN ];
omc_error_660 = [ NaN ; NaN ; NaN ];
Tc_error_660  = [ NaN ; NaN ; NaN ];

%-- Image #661:
omc_661 = [ NaN ; NaN ; NaN ];
Tc_661  = [ NaN ; NaN ; NaN ];
omc_error_661 = [ NaN ; NaN ; NaN ];
Tc_error_661  = [ NaN ; NaN ; NaN ];

%-- Image #662:
omc_662 = [ NaN ; NaN ; NaN ];
Tc_662  = [ NaN ; NaN ; NaN ];
omc_error_662 = [ NaN ; NaN ; NaN ];
Tc_error_662  = [ NaN ; NaN ; NaN ];

%-- Image #663:
omc_663 = [ NaN ; NaN ; NaN ];
Tc_663  = [ NaN ; NaN ; NaN ];
omc_error_663 = [ NaN ; NaN ; NaN ];
Tc_error_663  = [ NaN ; NaN ; NaN ];

%-- Image #664:
omc_664 = [ NaN ; NaN ; NaN ];
Tc_664  = [ NaN ; NaN ; NaN ];
omc_error_664 = [ NaN ; NaN ; NaN ];
Tc_error_664  = [ NaN ; NaN ; NaN ];

%-- Image #665:
omc_665 = [ NaN ; NaN ; NaN ];
Tc_665  = [ NaN ; NaN ; NaN ];
omc_error_665 = [ NaN ; NaN ; NaN ];
Tc_error_665  = [ NaN ; NaN ; NaN ];

%-- Image #666:
omc_666 = [ NaN ; NaN ; NaN ];
Tc_666  = [ NaN ; NaN ; NaN ];
omc_error_666 = [ NaN ; NaN ; NaN ];
Tc_error_666  = [ NaN ; NaN ; NaN ];

%-- Image #667:
omc_667 = [ NaN ; NaN ; NaN ];
Tc_667  = [ NaN ; NaN ; NaN ];
omc_error_667 = [ NaN ; NaN ; NaN ];
Tc_error_667  = [ NaN ; NaN ; NaN ];

%-- Image #668:
omc_668 = [ NaN ; NaN ; NaN ];
Tc_668  = [ NaN ; NaN ; NaN ];
omc_error_668 = [ NaN ; NaN ; NaN ];
Tc_error_668  = [ NaN ; NaN ; NaN ];

%-- Image #669:
omc_669 = [ NaN ; NaN ; NaN ];
Tc_669  = [ NaN ; NaN ; NaN ];
omc_error_669 = [ NaN ; NaN ; NaN ];
Tc_error_669  = [ NaN ; NaN ; NaN ];

%-- Image #670:
omc_670 = [ NaN ; NaN ; NaN ];
Tc_670  = [ NaN ; NaN ; NaN ];
omc_error_670 = [ NaN ; NaN ; NaN ];
Tc_error_670  = [ NaN ; NaN ; NaN ];

%-- Image #671:
omc_671 = [ NaN ; NaN ; NaN ];
Tc_671  = [ NaN ; NaN ; NaN ];
omc_error_671 = [ NaN ; NaN ; NaN ];
Tc_error_671  = [ NaN ; NaN ; NaN ];

%-- Image #672:
omc_672 = [ NaN ; NaN ; NaN ];
Tc_672  = [ NaN ; NaN ; NaN ];
omc_error_672 = [ NaN ; NaN ; NaN ];
Tc_error_672  = [ NaN ; NaN ; NaN ];

%-- Image #673:
omc_673 = [ NaN ; NaN ; NaN ];
Tc_673  = [ NaN ; NaN ; NaN ];
omc_error_673 = [ NaN ; NaN ; NaN ];
Tc_error_673  = [ NaN ; NaN ; NaN ];

%-- Image #674:
omc_674 = [ NaN ; NaN ; NaN ];
Tc_674  = [ NaN ; NaN ; NaN ];
omc_error_674 = [ NaN ; NaN ; NaN ];
Tc_error_674  = [ NaN ; NaN ; NaN ];

%-- Image #675:
omc_675 = [ NaN ; NaN ; NaN ];
Tc_675  = [ NaN ; NaN ; NaN ];
omc_error_675 = [ NaN ; NaN ; NaN ];
Tc_error_675  = [ NaN ; NaN ; NaN ];

%-- Image #676:
omc_676 = [ NaN ; NaN ; NaN ];
Tc_676  = [ NaN ; NaN ; NaN ];
omc_error_676 = [ NaN ; NaN ; NaN ];
Tc_error_676  = [ NaN ; NaN ; NaN ];

%-- Image #677:
omc_677 = [ NaN ; NaN ; NaN ];
Tc_677  = [ NaN ; NaN ; NaN ];
omc_error_677 = [ NaN ; NaN ; NaN ];
Tc_error_677  = [ NaN ; NaN ; NaN ];

%-- Image #678:
omc_678 = [ NaN ; NaN ; NaN ];
Tc_678  = [ NaN ; NaN ; NaN ];
omc_error_678 = [ NaN ; NaN ; NaN ];
Tc_error_678  = [ NaN ; NaN ; NaN ];

%-- Image #679:
omc_679 = [ NaN ; NaN ; NaN ];
Tc_679  = [ NaN ; NaN ; NaN ];
omc_error_679 = [ NaN ; NaN ; NaN ];
Tc_error_679  = [ NaN ; NaN ; NaN ];

%-- Image #680:
omc_680 = [ NaN ; NaN ; NaN ];
Tc_680  = [ NaN ; NaN ; NaN ];
omc_error_680 = [ NaN ; NaN ; NaN ];
Tc_error_680  = [ NaN ; NaN ; NaN ];

%-- Image #681:
omc_681 = [ NaN ; NaN ; NaN ];
Tc_681  = [ NaN ; NaN ; NaN ];
omc_error_681 = [ NaN ; NaN ; NaN ];
Tc_error_681  = [ NaN ; NaN ; NaN ];

%-- Image #682:
omc_682 = [ NaN ; NaN ; NaN ];
Tc_682  = [ NaN ; NaN ; NaN ];
omc_error_682 = [ NaN ; NaN ; NaN ];
Tc_error_682  = [ NaN ; NaN ; NaN ];

%-- Image #683:
omc_683 = [ NaN ; NaN ; NaN ];
Tc_683  = [ NaN ; NaN ; NaN ];
omc_error_683 = [ NaN ; NaN ; NaN ];
Tc_error_683  = [ NaN ; NaN ; NaN ];

%-- Image #684:
omc_684 = [ NaN ; NaN ; NaN ];
Tc_684  = [ NaN ; NaN ; NaN ];
omc_error_684 = [ NaN ; NaN ; NaN ];
Tc_error_684  = [ NaN ; NaN ; NaN ];

%-- Image #685:
omc_685 = [ NaN ; NaN ; NaN ];
Tc_685  = [ NaN ; NaN ; NaN ];
omc_error_685 = [ NaN ; NaN ; NaN ];
Tc_error_685  = [ NaN ; NaN ; NaN ];

%-- Image #686:
omc_686 = [ NaN ; NaN ; NaN ];
Tc_686  = [ NaN ; NaN ; NaN ];
omc_error_686 = [ NaN ; NaN ; NaN ];
Tc_error_686  = [ NaN ; NaN ; NaN ];

%-- Image #687:
omc_687 = [ NaN ; NaN ; NaN ];
Tc_687  = [ NaN ; NaN ; NaN ];
omc_error_687 = [ NaN ; NaN ; NaN ];
Tc_error_687  = [ NaN ; NaN ; NaN ];

%-- Image #688:
omc_688 = [ NaN ; NaN ; NaN ];
Tc_688  = [ NaN ; NaN ; NaN ];
omc_error_688 = [ NaN ; NaN ; NaN ];
Tc_error_688  = [ NaN ; NaN ; NaN ];

%-- Image #689:
omc_689 = [ NaN ; NaN ; NaN ];
Tc_689  = [ NaN ; NaN ; NaN ];
omc_error_689 = [ NaN ; NaN ; NaN ];
Tc_error_689  = [ NaN ; NaN ; NaN ];

%-- Image #690:
omc_690 = [ NaN ; NaN ; NaN ];
Tc_690  = [ NaN ; NaN ; NaN ];
omc_error_690 = [ NaN ; NaN ; NaN ];
Tc_error_690  = [ NaN ; NaN ; NaN ];

%-- Image #691:
omc_691 = [ NaN ; NaN ; NaN ];
Tc_691  = [ NaN ; NaN ; NaN ];
omc_error_691 = [ NaN ; NaN ; NaN ];
Tc_error_691  = [ NaN ; NaN ; NaN ];

%-- Image #692:
omc_692 = [ NaN ; NaN ; NaN ];
Tc_692  = [ NaN ; NaN ; NaN ];
omc_error_692 = [ NaN ; NaN ; NaN ];
Tc_error_692  = [ NaN ; NaN ; NaN ];

%-- Image #693:
omc_693 = [ NaN ; NaN ; NaN ];
Tc_693  = [ NaN ; NaN ; NaN ];
omc_error_693 = [ NaN ; NaN ; NaN ];
Tc_error_693  = [ NaN ; NaN ; NaN ];

%-- Image #694:
omc_694 = [ NaN ; NaN ; NaN ];
Tc_694  = [ NaN ; NaN ; NaN ];
omc_error_694 = [ NaN ; NaN ; NaN ];
Tc_error_694  = [ NaN ; NaN ; NaN ];

%-- Image #695:
omc_695 = [ NaN ; NaN ; NaN ];
Tc_695  = [ NaN ; NaN ; NaN ];
omc_error_695 = [ NaN ; NaN ; NaN ];
Tc_error_695  = [ NaN ; NaN ; NaN ];

%-- Image #696:
omc_696 = [ NaN ; NaN ; NaN ];
Tc_696  = [ NaN ; NaN ; NaN ];
omc_error_696 = [ NaN ; NaN ; NaN ];
Tc_error_696  = [ NaN ; NaN ; NaN ];

%-- Image #697:
omc_697 = [ NaN ; NaN ; NaN ];
Tc_697  = [ NaN ; NaN ; NaN ];
omc_error_697 = [ NaN ; NaN ; NaN ];
Tc_error_697  = [ NaN ; NaN ; NaN ];

%-- Image #698:
omc_698 = [ NaN ; NaN ; NaN ];
Tc_698  = [ NaN ; NaN ; NaN ];
omc_error_698 = [ NaN ; NaN ; NaN ];
Tc_error_698  = [ NaN ; NaN ; NaN ];

%-- Image #699:
omc_699 = [ NaN ; NaN ; NaN ];
Tc_699  = [ NaN ; NaN ; NaN ];
omc_error_699 = [ NaN ; NaN ; NaN ];
Tc_error_699  = [ NaN ; NaN ; NaN ];

%-- Image #700:
omc_700 = [ NaN ; NaN ; NaN ];
Tc_700  = [ NaN ; NaN ; NaN ];
omc_error_700 = [ NaN ; NaN ; NaN ];
Tc_error_700  = [ NaN ; NaN ; NaN ];

%-- Image #701:
omc_701 = [ NaN ; NaN ; NaN ];
Tc_701  = [ NaN ; NaN ; NaN ];
omc_error_701 = [ NaN ; NaN ; NaN ];
Tc_error_701  = [ NaN ; NaN ; NaN ];

%-- Image #702:
omc_702 = [ NaN ; NaN ; NaN ];
Tc_702  = [ NaN ; NaN ; NaN ];
omc_error_702 = [ NaN ; NaN ; NaN ];
Tc_error_702  = [ NaN ; NaN ; NaN ];

%-- Image #703:
omc_703 = [ 1.910567e+000 ; 2.289045e+000 ; -4.681886e-001 ];
Tc_703  = [ -6.744019e+001 ; -1.230959e+002 ; 2.598564e+002 ];
omc_error_703 = [ 1.105639e-002 ; 1.413976e-002 ; 2.717909e-002 ];
Tc_error_703  = [ 3.468962e+000 ; 3.489430e+000 ; 4.519567e+000 ];

%-- Image #704:
omc_704 = [ NaN ; NaN ; NaN ];
Tc_704  = [ NaN ; NaN ; NaN ];
omc_error_704 = [ NaN ; NaN ; NaN ];
Tc_error_704  = [ NaN ; NaN ; NaN ];

%-- Image #705:
omc_705 = [ NaN ; NaN ; NaN ];
Tc_705  = [ NaN ; NaN ; NaN ];
omc_error_705 = [ NaN ; NaN ; NaN ];
Tc_error_705  = [ NaN ; NaN ; NaN ];

%-- Image #706:
omc_706 = [ NaN ; NaN ; NaN ];
Tc_706  = [ NaN ; NaN ; NaN ];
omc_error_706 = [ NaN ; NaN ; NaN ];
Tc_error_706  = [ NaN ; NaN ; NaN ];

%-- Image #707:
omc_707 = [ NaN ; NaN ; NaN ];
Tc_707  = [ NaN ; NaN ; NaN ];
omc_error_707 = [ NaN ; NaN ; NaN ];
Tc_error_707  = [ NaN ; NaN ; NaN ];

%-- Image #708:
omc_708 = [ NaN ; NaN ; NaN ];
Tc_708  = [ NaN ; NaN ; NaN ];
omc_error_708 = [ NaN ; NaN ; NaN ];
Tc_error_708  = [ NaN ; NaN ; NaN ];

%-- Image #709:
omc_709 = [ NaN ; NaN ; NaN ];
Tc_709  = [ NaN ; NaN ; NaN ];
omc_error_709 = [ NaN ; NaN ; NaN ];
Tc_error_709  = [ NaN ; NaN ; NaN ];

%-- Image #710:
omc_710 = [ NaN ; NaN ; NaN ];
Tc_710  = [ NaN ; NaN ; NaN ];
omc_error_710 = [ NaN ; NaN ; NaN ];
Tc_error_710  = [ NaN ; NaN ; NaN ];

%-- Image #711:
omc_711 = [ NaN ; NaN ; NaN ];
Tc_711  = [ NaN ; NaN ; NaN ];
omc_error_711 = [ NaN ; NaN ; NaN ];
Tc_error_711  = [ NaN ; NaN ; NaN ];

%-- Image #712:
omc_712 = [ NaN ; NaN ; NaN ];
Tc_712  = [ NaN ; NaN ; NaN ];
omc_error_712 = [ NaN ; NaN ; NaN ];
Tc_error_712  = [ NaN ; NaN ; NaN ];

%-- Image #713:
omc_713 = [ NaN ; NaN ; NaN ];
Tc_713  = [ NaN ; NaN ; NaN ];
omc_error_713 = [ NaN ; NaN ; NaN ];
Tc_error_713  = [ NaN ; NaN ; NaN ];

%-- Image #714:
omc_714 = [ NaN ; NaN ; NaN ];
Tc_714  = [ NaN ; NaN ; NaN ];
omc_error_714 = [ NaN ; NaN ; NaN ];
Tc_error_714  = [ NaN ; NaN ; NaN ];

%-- Image #715:
omc_715 = [ NaN ; NaN ; NaN ];
Tc_715  = [ NaN ; NaN ; NaN ];
omc_error_715 = [ NaN ; NaN ; NaN ];
Tc_error_715  = [ NaN ; NaN ; NaN ];

%-- Image #716:
omc_716 = [ NaN ; NaN ; NaN ];
Tc_716  = [ NaN ; NaN ; NaN ];
omc_error_716 = [ NaN ; NaN ; NaN ];
Tc_error_716  = [ NaN ; NaN ; NaN ];

%-- Image #717:
omc_717 = [ NaN ; NaN ; NaN ];
Tc_717  = [ NaN ; NaN ; NaN ];
omc_error_717 = [ NaN ; NaN ; NaN ];
Tc_error_717  = [ NaN ; NaN ; NaN ];

%-- Image #718:
omc_718 = [ NaN ; NaN ; NaN ];
Tc_718  = [ NaN ; NaN ; NaN ];
omc_error_718 = [ NaN ; NaN ; NaN ];
Tc_error_718  = [ NaN ; NaN ; NaN ];

%-- Image #719:
omc_719 = [ NaN ; NaN ; NaN ];
Tc_719  = [ NaN ; NaN ; NaN ];
omc_error_719 = [ NaN ; NaN ; NaN ];
Tc_error_719  = [ NaN ; NaN ; NaN ];

%-- Image #720:
omc_720 = [ NaN ; NaN ; NaN ];
Tc_720  = [ NaN ; NaN ; NaN ];
omc_error_720 = [ NaN ; NaN ; NaN ];
Tc_error_720  = [ NaN ; NaN ; NaN ];

%-- Image #721:
omc_721 = [ NaN ; NaN ; NaN ];
Tc_721  = [ NaN ; NaN ; NaN ];
omc_error_721 = [ NaN ; NaN ; NaN ];
Tc_error_721  = [ NaN ; NaN ; NaN ];

%-- Image #722:
omc_722 = [ NaN ; NaN ; NaN ];
Tc_722  = [ NaN ; NaN ; NaN ];
omc_error_722 = [ NaN ; NaN ; NaN ];
Tc_error_722  = [ NaN ; NaN ; NaN ];

%-- Image #723:
omc_723 = [ NaN ; NaN ; NaN ];
Tc_723  = [ NaN ; NaN ; NaN ];
omc_error_723 = [ NaN ; NaN ; NaN ];
Tc_error_723  = [ NaN ; NaN ; NaN ];

%-- Image #724:
omc_724 = [ NaN ; NaN ; NaN ];
Tc_724  = [ NaN ; NaN ; NaN ];
omc_error_724 = [ NaN ; NaN ; NaN ];
Tc_error_724  = [ NaN ; NaN ; NaN ];

%-- Image #725:
omc_725 = [ NaN ; NaN ; NaN ];
Tc_725  = [ NaN ; NaN ; NaN ];
omc_error_725 = [ NaN ; NaN ; NaN ];
Tc_error_725  = [ NaN ; NaN ; NaN ];

%-- Image #726:
omc_726 = [ NaN ; NaN ; NaN ];
Tc_726  = [ NaN ; NaN ; NaN ];
omc_error_726 = [ NaN ; NaN ; NaN ];
Tc_error_726  = [ NaN ; NaN ; NaN ];

%-- Image #727:
omc_727 = [ NaN ; NaN ; NaN ];
Tc_727  = [ NaN ; NaN ; NaN ];
omc_error_727 = [ NaN ; NaN ; NaN ];
Tc_error_727  = [ NaN ; NaN ; NaN ];

%-- Image #728:
omc_728 = [ NaN ; NaN ; NaN ];
Tc_728  = [ NaN ; NaN ; NaN ];
omc_error_728 = [ NaN ; NaN ; NaN ];
Tc_error_728  = [ NaN ; NaN ; NaN ];

%-- Image #729:
omc_729 = [ NaN ; NaN ; NaN ];
Tc_729  = [ NaN ; NaN ; NaN ];
omc_error_729 = [ NaN ; NaN ; NaN ];
Tc_error_729  = [ NaN ; NaN ; NaN ];

%-- Image #730:
omc_730 = [ NaN ; NaN ; NaN ];
Tc_730  = [ NaN ; NaN ; NaN ];
omc_error_730 = [ NaN ; NaN ; NaN ];
Tc_error_730  = [ NaN ; NaN ; NaN ];

%-- Image #731:
omc_731 = [ NaN ; NaN ; NaN ];
Tc_731  = [ NaN ; NaN ; NaN ];
omc_error_731 = [ NaN ; NaN ; NaN ];
Tc_error_731  = [ NaN ; NaN ; NaN ];

%-- Image #732:
omc_732 = [ NaN ; NaN ; NaN ];
Tc_732  = [ NaN ; NaN ; NaN ];
omc_error_732 = [ NaN ; NaN ; NaN ];
Tc_error_732  = [ NaN ; NaN ; NaN ];

%-- Image #733:
omc_733 = [ NaN ; NaN ; NaN ];
Tc_733  = [ NaN ; NaN ; NaN ];
omc_error_733 = [ NaN ; NaN ; NaN ];
Tc_error_733  = [ NaN ; NaN ; NaN ];

%-- Image #734:
omc_734 = [ NaN ; NaN ; NaN ];
Tc_734  = [ NaN ; NaN ; NaN ];
omc_error_734 = [ NaN ; NaN ; NaN ];
Tc_error_734  = [ NaN ; NaN ; NaN ];

%-- Image #735:
omc_735 = [ NaN ; NaN ; NaN ];
Tc_735  = [ NaN ; NaN ; NaN ];
omc_error_735 = [ NaN ; NaN ; NaN ];
Tc_error_735  = [ NaN ; NaN ; NaN ];

%-- Image #736:
omc_736 = [ NaN ; NaN ; NaN ];
Tc_736  = [ NaN ; NaN ; NaN ];
omc_error_736 = [ NaN ; NaN ; NaN ];
Tc_error_736  = [ NaN ; NaN ; NaN ];

%-- Image #737:
omc_737 = [ NaN ; NaN ; NaN ];
Tc_737  = [ NaN ; NaN ; NaN ];
omc_error_737 = [ NaN ; NaN ; NaN ];
Tc_error_737  = [ NaN ; NaN ; NaN ];

%-- Image #738:
omc_738 = [ NaN ; NaN ; NaN ];
Tc_738  = [ NaN ; NaN ; NaN ];
omc_error_738 = [ NaN ; NaN ; NaN ];
Tc_error_738  = [ NaN ; NaN ; NaN ];

%-- Image #739:
omc_739 = [ NaN ; NaN ; NaN ];
Tc_739  = [ NaN ; NaN ; NaN ];
omc_error_739 = [ NaN ; NaN ; NaN ];
Tc_error_739  = [ NaN ; NaN ; NaN ];

%-- Image #740:
omc_740 = [ NaN ; NaN ; NaN ];
Tc_740  = [ NaN ; NaN ; NaN ];
omc_error_740 = [ NaN ; NaN ; NaN ];
Tc_error_740  = [ NaN ; NaN ; NaN ];

%-- Image #741:
omc_741 = [ NaN ; NaN ; NaN ];
Tc_741  = [ NaN ; NaN ; NaN ];
omc_error_741 = [ NaN ; NaN ; NaN ];
Tc_error_741  = [ NaN ; NaN ; NaN ];

%-- Image #742:
omc_742 = [ NaN ; NaN ; NaN ];
Tc_742  = [ NaN ; NaN ; NaN ];
omc_error_742 = [ NaN ; NaN ; NaN ];
Tc_error_742  = [ NaN ; NaN ; NaN ];

%-- Image #743:
omc_743 = [ NaN ; NaN ; NaN ];
Tc_743  = [ NaN ; NaN ; NaN ];
omc_error_743 = [ NaN ; NaN ; NaN ];
Tc_error_743  = [ NaN ; NaN ; NaN ];

%-- Image #744:
omc_744 = [ NaN ; NaN ; NaN ];
Tc_744  = [ NaN ; NaN ; NaN ];
omc_error_744 = [ NaN ; NaN ; NaN ];
Tc_error_744  = [ NaN ; NaN ; NaN ];

%-- Image #745:
omc_745 = [ NaN ; NaN ; NaN ];
Tc_745  = [ NaN ; NaN ; NaN ];
omc_error_745 = [ NaN ; NaN ; NaN ];
Tc_error_745  = [ NaN ; NaN ; NaN ];

%-- Image #746:
omc_746 = [ NaN ; NaN ; NaN ];
Tc_746  = [ NaN ; NaN ; NaN ];
omc_error_746 = [ NaN ; NaN ; NaN ];
Tc_error_746  = [ NaN ; NaN ; NaN ];

%-- Image #747:
omc_747 = [ NaN ; NaN ; NaN ];
Tc_747  = [ NaN ; NaN ; NaN ];
omc_error_747 = [ NaN ; NaN ; NaN ];
Tc_error_747  = [ NaN ; NaN ; NaN ];

%-- Image #748:
omc_748 = [ NaN ; NaN ; NaN ];
Tc_748  = [ NaN ; NaN ; NaN ];
omc_error_748 = [ NaN ; NaN ; NaN ];
Tc_error_748  = [ NaN ; NaN ; NaN ];

%-- Image #749:
omc_749 = [ NaN ; NaN ; NaN ];
Tc_749  = [ NaN ; NaN ; NaN ];
omc_error_749 = [ NaN ; NaN ; NaN ];
Tc_error_749  = [ NaN ; NaN ; NaN ];

%-- Image #750:
omc_750 = [ NaN ; NaN ; NaN ];
Tc_750  = [ NaN ; NaN ; NaN ];
omc_error_750 = [ NaN ; NaN ; NaN ];
Tc_error_750  = [ NaN ; NaN ; NaN ];

%-- Image #751:
omc_751 = [ NaN ; NaN ; NaN ];
Tc_751  = [ NaN ; NaN ; NaN ];
omc_error_751 = [ NaN ; NaN ; NaN ];
Tc_error_751  = [ NaN ; NaN ; NaN ];

%-- Image #752:
omc_752 = [ NaN ; NaN ; NaN ];
Tc_752  = [ NaN ; NaN ; NaN ];
omc_error_752 = [ NaN ; NaN ; NaN ];
Tc_error_752  = [ NaN ; NaN ; NaN ];

%-- Image #753:
omc_753 = [ NaN ; NaN ; NaN ];
Tc_753  = [ NaN ; NaN ; NaN ];
omc_error_753 = [ NaN ; NaN ; NaN ];
Tc_error_753  = [ NaN ; NaN ; NaN ];

%-- Image #754:
omc_754 = [ NaN ; NaN ; NaN ];
Tc_754  = [ NaN ; NaN ; NaN ];
omc_error_754 = [ NaN ; NaN ; NaN ];
Tc_error_754  = [ NaN ; NaN ; NaN ];

%-- Image #755:
omc_755 = [ NaN ; NaN ; NaN ];
Tc_755  = [ NaN ; NaN ; NaN ];
omc_error_755 = [ NaN ; NaN ; NaN ];
Tc_error_755  = [ NaN ; NaN ; NaN ];

%-- Image #756:
omc_756 = [ NaN ; NaN ; NaN ];
Tc_756  = [ NaN ; NaN ; NaN ];
omc_error_756 = [ NaN ; NaN ; NaN ];
Tc_error_756  = [ NaN ; NaN ; NaN ];

%-- Image #757:
omc_757 = [ NaN ; NaN ; NaN ];
Tc_757  = [ NaN ; NaN ; NaN ];
omc_error_757 = [ NaN ; NaN ; NaN ];
Tc_error_757  = [ NaN ; NaN ; NaN ];

%-- Image #758:
omc_758 = [ NaN ; NaN ; NaN ];
Tc_758  = [ NaN ; NaN ; NaN ];
omc_error_758 = [ NaN ; NaN ; NaN ];
Tc_error_758  = [ NaN ; NaN ; NaN ];

%-- Image #759:
omc_759 = [ NaN ; NaN ; NaN ];
Tc_759  = [ NaN ; NaN ; NaN ];
omc_error_759 = [ NaN ; NaN ; NaN ];
Tc_error_759  = [ NaN ; NaN ; NaN ];

%-- Image #760:
omc_760 = [ NaN ; NaN ; NaN ];
Tc_760  = [ NaN ; NaN ; NaN ];
omc_error_760 = [ NaN ; NaN ; NaN ];
Tc_error_760  = [ NaN ; NaN ; NaN ];

%-- Image #761:
omc_761 = [ NaN ; NaN ; NaN ];
Tc_761  = [ NaN ; NaN ; NaN ];
omc_error_761 = [ NaN ; NaN ; NaN ];
Tc_error_761  = [ NaN ; NaN ; NaN ];

%-- Image #762:
omc_762 = [ NaN ; NaN ; NaN ];
Tc_762  = [ NaN ; NaN ; NaN ];
omc_error_762 = [ NaN ; NaN ; NaN ];
Tc_error_762  = [ NaN ; NaN ; NaN ];

%-- Image #763:
omc_763 = [ NaN ; NaN ; NaN ];
Tc_763  = [ NaN ; NaN ; NaN ];
omc_error_763 = [ NaN ; NaN ; NaN ];
Tc_error_763  = [ NaN ; NaN ; NaN ];

%-- Image #764:
omc_764 = [ -2.057942e+000 ; -2.185683e+000 ; -2.446493e-001 ];
Tc_764  = [ -1.170402e+002 ; -9.812781e+001 ; 1.889140e+002 ];
omc_error_764 = [ 1.406541e-002 ; 1.394106e-002 ; 2.803772e-002 ];
Tc_error_764  = [ 2.654737e+000 ; 2.898725e+000 ; 4.346451e+000 ];

%-- Image #765:
omc_765 = [ NaN ; NaN ; NaN ];
Tc_765  = [ NaN ; NaN ; NaN ];
omc_error_765 = [ NaN ; NaN ; NaN ];
Tc_error_765  = [ NaN ; NaN ; NaN ];

%-- Image #766:
omc_766 = [ NaN ; NaN ; NaN ];
Tc_766  = [ NaN ; NaN ; NaN ];
omc_error_766 = [ NaN ; NaN ; NaN ];
Tc_error_766  = [ NaN ; NaN ; NaN ];

%-- Image #767:
omc_767 = [ NaN ; NaN ; NaN ];
Tc_767  = [ NaN ; NaN ; NaN ];
omc_error_767 = [ NaN ; NaN ; NaN ];
Tc_error_767  = [ NaN ; NaN ; NaN ];

%-- Image #768:
omc_768 = [ NaN ; NaN ; NaN ];
Tc_768  = [ NaN ; NaN ; NaN ];
omc_error_768 = [ NaN ; NaN ; NaN ];
Tc_error_768  = [ NaN ; NaN ; NaN ];

%-- Image #769:
omc_769 = [ NaN ; NaN ; NaN ];
Tc_769  = [ NaN ; NaN ; NaN ];
omc_error_769 = [ NaN ; NaN ; NaN ];
Tc_error_769  = [ NaN ; NaN ; NaN ];

%-- Image #770:
omc_770 = [ NaN ; NaN ; NaN ];
Tc_770  = [ NaN ; NaN ; NaN ];
omc_error_770 = [ NaN ; NaN ; NaN ];
Tc_error_770  = [ NaN ; NaN ; NaN ];

%-- Image #771:
omc_771 = [ NaN ; NaN ; NaN ];
Tc_771  = [ NaN ; NaN ; NaN ];
omc_error_771 = [ NaN ; NaN ; NaN ];
Tc_error_771  = [ NaN ; NaN ; NaN ];

%-- Image #772:
omc_772 = [ NaN ; NaN ; NaN ];
Tc_772  = [ NaN ; NaN ; NaN ];
omc_error_772 = [ NaN ; NaN ; NaN ];
Tc_error_772  = [ NaN ; NaN ; NaN ];

%-- Image #773:
omc_773 = [ NaN ; NaN ; NaN ];
Tc_773  = [ NaN ; NaN ; NaN ];
omc_error_773 = [ NaN ; NaN ; NaN ];
Tc_error_773  = [ NaN ; NaN ; NaN ];

%-- Image #774:
omc_774 = [ NaN ; NaN ; NaN ];
Tc_774  = [ NaN ; NaN ; NaN ];
omc_error_774 = [ NaN ; NaN ; NaN ];
Tc_error_774  = [ NaN ; NaN ; NaN ];

%-- Image #775:
omc_775 = [ NaN ; NaN ; NaN ];
Tc_775  = [ NaN ; NaN ; NaN ];
omc_error_775 = [ NaN ; NaN ; NaN ];
Tc_error_775  = [ NaN ; NaN ; NaN ];

%-- Image #776:
omc_776 = [ NaN ; NaN ; NaN ];
Tc_776  = [ NaN ; NaN ; NaN ];
omc_error_776 = [ NaN ; NaN ; NaN ];
Tc_error_776  = [ NaN ; NaN ; NaN ];

%-- Image #777:
omc_777 = [ NaN ; NaN ; NaN ];
Tc_777  = [ NaN ; NaN ; NaN ];
omc_error_777 = [ NaN ; NaN ; NaN ];
Tc_error_777  = [ NaN ; NaN ; NaN ];

%-- Image #778:
omc_778 = [ NaN ; NaN ; NaN ];
Tc_778  = [ NaN ; NaN ; NaN ];
omc_error_778 = [ NaN ; NaN ; NaN ];
Tc_error_778  = [ NaN ; NaN ; NaN ];

%-- Image #779:
omc_779 = [ NaN ; NaN ; NaN ];
Tc_779  = [ NaN ; NaN ; NaN ];
omc_error_779 = [ NaN ; NaN ; NaN ];
Tc_error_779  = [ NaN ; NaN ; NaN ];

%-- Image #780:
omc_780 = [ NaN ; NaN ; NaN ];
Tc_780  = [ NaN ; NaN ; NaN ];
omc_error_780 = [ NaN ; NaN ; NaN ];
Tc_error_780  = [ NaN ; NaN ; NaN ];

%-- Image #781:
omc_781 = [ NaN ; NaN ; NaN ];
Tc_781  = [ NaN ; NaN ; NaN ];
omc_error_781 = [ NaN ; NaN ; NaN ];
Tc_error_781  = [ NaN ; NaN ; NaN ];

%-- Image #782:
omc_782 = [ NaN ; NaN ; NaN ];
Tc_782  = [ NaN ; NaN ; NaN ];
omc_error_782 = [ NaN ; NaN ; NaN ];
Tc_error_782  = [ NaN ; NaN ; NaN ];

%-- Image #783:
omc_783 = [ NaN ; NaN ; NaN ];
Tc_783  = [ NaN ; NaN ; NaN ];
omc_error_783 = [ NaN ; NaN ; NaN ];
Tc_error_783  = [ NaN ; NaN ; NaN ];

%-- Image #784:
omc_784 = [ NaN ; NaN ; NaN ];
Tc_784  = [ NaN ; NaN ; NaN ];
omc_error_784 = [ NaN ; NaN ; NaN ];
Tc_error_784  = [ NaN ; NaN ; NaN ];

%-- Image #785:
omc_785 = [ NaN ; NaN ; NaN ];
Tc_785  = [ NaN ; NaN ; NaN ];
omc_error_785 = [ NaN ; NaN ; NaN ];
Tc_error_785  = [ NaN ; NaN ; NaN ];

%-- Image #786:
omc_786 = [ NaN ; NaN ; NaN ];
Tc_786  = [ NaN ; NaN ; NaN ];
omc_error_786 = [ NaN ; NaN ; NaN ];
Tc_error_786  = [ NaN ; NaN ; NaN ];

%-- Image #787:
omc_787 = [ NaN ; NaN ; NaN ];
Tc_787  = [ NaN ; NaN ; NaN ];
omc_error_787 = [ NaN ; NaN ; NaN ];
Tc_error_787  = [ NaN ; NaN ; NaN ];

%-- Image #788:
omc_788 = [ NaN ; NaN ; NaN ];
Tc_788  = [ NaN ; NaN ; NaN ];
omc_error_788 = [ NaN ; NaN ; NaN ];
Tc_error_788  = [ NaN ; NaN ; NaN ];

%-- Image #789:
omc_789 = [ NaN ; NaN ; NaN ];
Tc_789  = [ NaN ; NaN ; NaN ];
omc_error_789 = [ NaN ; NaN ; NaN ];
Tc_error_789  = [ NaN ; NaN ; NaN ];

%-- Image #790:
omc_790 = [ NaN ; NaN ; NaN ];
Tc_790  = [ NaN ; NaN ; NaN ];
omc_error_790 = [ NaN ; NaN ; NaN ];
Tc_error_790  = [ NaN ; NaN ; NaN ];

%-- Image #791:
omc_791 = [ NaN ; NaN ; NaN ];
Tc_791  = [ NaN ; NaN ; NaN ];
omc_error_791 = [ NaN ; NaN ; NaN ];
Tc_error_791  = [ NaN ; NaN ; NaN ];

%-- Image #792:
omc_792 = [ NaN ; NaN ; NaN ];
Tc_792  = [ NaN ; NaN ; NaN ];
omc_error_792 = [ NaN ; NaN ; NaN ];
Tc_error_792  = [ NaN ; NaN ; NaN ];

%-- Image #793:
omc_793 = [ NaN ; NaN ; NaN ];
Tc_793  = [ NaN ; NaN ; NaN ];
omc_error_793 = [ NaN ; NaN ; NaN ];
Tc_error_793  = [ NaN ; NaN ; NaN ];

%-- Image #794:
omc_794 = [ NaN ; NaN ; NaN ];
Tc_794  = [ NaN ; NaN ; NaN ];
omc_error_794 = [ NaN ; NaN ; NaN ];
Tc_error_794  = [ NaN ; NaN ; NaN ];

%-- Image #795:
omc_795 = [ NaN ; NaN ; NaN ];
Tc_795  = [ NaN ; NaN ; NaN ];
omc_error_795 = [ NaN ; NaN ; NaN ];
Tc_error_795  = [ NaN ; NaN ; NaN ];

%-- Image #796:
omc_796 = [ NaN ; NaN ; NaN ];
Tc_796  = [ NaN ; NaN ; NaN ];
omc_error_796 = [ NaN ; NaN ; NaN ];
Tc_error_796  = [ NaN ; NaN ; NaN ];

%-- Image #797:
omc_797 = [ NaN ; NaN ; NaN ];
Tc_797  = [ NaN ; NaN ; NaN ];
omc_error_797 = [ NaN ; NaN ; NaN ];
Tc_error_797  = [ NaN ; NaN ; NaN ];

%-- Image #798:
omc_798 = [ NaN ; NaN ; NaN ];
Tc_798  = [ NaN ; NaN ; NaN ];
omc_error_798 = [ NaN ; NaN ; NaN ];
Tc_error_798  = [ NaN ; NaN ; NaN ];

%-- Image #799:
omc_799 = [ NaN ; NaN ; NaN ];
Tc_799  = [ NaN ; NaN ; NaN ];
omc_error_799 = [ NaN ; NaN ; NaN ];
Tc_error_799  = [ NaN ; NaN ; NaN ];

%-- Image #800:
omc_800 = [ NaN ; NaN ; NaN ];
Tc_800  = [ NaN ; NaN ; NaN ];
omc_error_800 = [ NaN ; NaN ; NaN ];
Tc_error_800  = [ NaN ; NaN ; NaN ];

%-- Image #801:
omc_801 = [ NaN ; NaN ; NaN ];
Tc_801  = [ NaN ; NaN ; NaN ];
omc_error_801 = [ NaN ; NaN ; NaN ];
Tc_error_801  = [ NaN ; NaN ; NaN ];

%-- Image #802:
omc_802 = [ NaN ; NaN ; NaN ];
Tc_802  = [ NaN ; NaN ; NaN ];
omc_error_802 = [ NaN ; NaN ; NaN ];
Tc_error_802  = [ NaN ; NaN ; NaN ];

%-- Image #803:
omc_803 = [ NaN ; NaN ; NaN ];
Tc_803  = [ NaN ; NaN ; NaN ];
omc_error_803 = [ NaN ; NaN ; NaN ];
Tc_error_803  = [ NaN ; NaN ; NaN ];

%-- Image #804:
omc_804 = [ NaN ; NaN ; NaN ];
Tc_804  = [ NaN ; NaN ; NaN ];
omc_error_804 = [ NaN ; NaN ; NaN ];
Tc_error_804  = [ NaN ; NaN ; NaN ];

%-- Image #805:
omc_805 = [ NaN ; NaN ; NaN ];
Tc_805  = [ NaN ; NaN ; NaN ];
omc_error_805 = [ NaN ; NaN ; NaN ];
Tc_error_805  = [ NaN ; NaN ; NaN ];

%-- Image #806:
omc_806 = [ NaN ; NaN ; NaN ];
Tc_806  = [ NaN ; NaN ; NaN ];
omc_error_806 = [ NaN ; NaN ; NaN ];
Tc_error_806  = [ NaN ; NaN ; NaN ];

%-- Image #807:
omc_807 = [ NaN ; NaN ; NaN ];
Tc_807  = [ NaN ; NaN ; NaN ];
omc_error_807 = [ NaN ; NaN ; NaN ];
Tc_error_807  = [ NaN ; NaN ; NaN ];

%-- Image #808:
omc_808 = [ NaN ; NaN ; NaN ];
Tc_808  = [ NaN ; NaN ; NaN ];
omc_error_808 = [ NaN ; NaN ; NaN ];
Tc_error_808  = [ NaN ; NaN ; NaN ];

%-- Image #809:
omc_809 = [ NaN ; NaN ; NaN ];
Tc_809  = [ NaN ; NaN ; NaN ];
omc_error_809 = [ NaN ; NaN ; NaN ];
Tc_error_809  = [ NaN ; NaN ; NaN ];

%-- Image #810:
omc_810 = [ NaN ; NaN ; NaN ];
Tc_810  = [ NaN ; NaN ; NaN ];
omc_error_810 = [ NaN ; NaN ; NaN ];
Tc_error_810  = [ NaN ; NaN ; NaN ];

%-- Image #811:
omc_811 = [ NaN ; NaN ; NaN ];
Tc_811  = [ NaN ; NaN ; NaN ];
omc_error_811 = [ NaN ; NaN ; NaN ];
Tc_error_811  = [ NaN ; NaN ; NaN ];

%-- Image #812:
omc_812 = [ NaN ; NaN ; NaN ];
Tc_812  = [ NaN ; NaN ; NaN ];
omc_error_812 = [ NaN ; NaN ; NaN ];
Tc_error_812  = [ NaN ; NaN ; NaN ];

%-- Image #813:
omc_813 = [ NaN ; NaN ; NaN ];
Tc_813  = [ NaN ; NaN ; NaN ];
omc_error_813 = [ NaN ; NaN ; NaN ];
Tc_error_813  = [ NaN ; NaN ; NaN ];

%-- Image #814:
omc_814 = [ NaN ; NaN ; NaN ];
Tc_814  = [ NaN ; NaN ; NaN ];
omc_error_814 = [ NaN ; NaN ; NaN ];
Tc_error_814  = [ NaN ; NaN ; NaN ];

%-- Image #815:
omc_815 = [ NaN ; NaN ; NaN ];
Tc_815  = [ NaN ; NaN ; NaN ];
omc_error_815 = [ NaN ; NaN ; NaN ];
Tc_error_815  = [ NaN ; NaN ; NaN ];

%-- Image #816:
omc_816 = [ NaN ; NaN ; NaN ];
Tc_816  = [ NaN ; NaN ; NaN ];
omc_error_816 = [ NaN ; NaN ; NaN ];
Tc_error_816  = [ NaN ; NaN ; NaN ];

%-- Image #817:
omc_817 = [ NaN ; NaN ; NaN ];
Tc_817  = [ NaN ; NaN ; NaN ];
omc_error_817 = [ NaN ; NaN ; NaN ];
Tc_error_817  = [ NaN ; NaN ; NaN ];

%-- Image #818:
omc_818 = [ NaN ; NaN ; NaN ];
Tc_818  = [ NaN ; NaN ; NaN ];
omc_error_818 = [ NaN ; NaN ; NaN ];
Tc_error_818  = [ NaN ; NaN ; NaN ];

%-- Image #819:
omc_819 = [ NaN ; NaN ; NaN ];
Tc_819  = [ NaN ; NaN ; NaN ];
omc_error_819 = [ NaN ; NaN ; NaN ];
Tc_error_819  = [ NaN ; NaN ; NaN ];

%-- Image #820:
omc_820 = [ NaN ; NaN ; NaN ];
Tc_820  = [ NaN ; NaN ; NaN ];
omc_error_820 = [ NaN ; NaN ; NaN ];
Tc_error_820  = [ NaN ; NaN ; NaN ];

%-- Image #821:
omc_821 = [ -1.810632e+000 ; -1.956458e+000 ; -4.556138e-001 ];
Tc_821  = [ -1.161287e+002 ; -9.363267e+001 ; 1.896412e+002 ];
omc_error_821 = [ 1.334625e-002 ; 1.415249e-002 ; 2.348776e-002 ];
Tc_error_821  = [ 2.630010e+000 ; 2.966181e+000 ; 4.336704e+000 ];

%-- Image #822:
omc_822 = [ NaN ; NaN ; NaN ];
Tc_822  = [ NaN ; NaN ; NaN ];
omc_error_822 = [ NaN ; NaN ; NaN ];
Tc_error_822  = [ NaN ; NaN ; NaN ];

%-- Image #823:
omc_823 = [ NaN ; NaN ; NaN ];
Tc_823  = [ NaN ; NaN ; NaN ];
omc_error_823 = [ NaN ; NaN ; NaN ];
Tc_error_823  = [ NaN ; NaN ; NaN ];

%-- Image #824:
omc_824 = [ NaN ; NaN ; NaN ];
Tc_824  = [ NaN ; NaN ; NaN ];
omc_error_824 = [ NaN ; NaN ; NaN ];
Tc_error_824  = [ NaN ; NaN ; NaN ];

%-- Image #825:
omc_825 = [ NaN ; NaN ; NaN ];
Tc_825  = [ NaN ; NaN ; NaN ];
omc_error_825 = [ NaN ; NaN ; NaN ];
Tc_error_825  = [ NaN ; NaN ; NaN ];

%-- Image #826:
omc_826 = [ NaN ; NaN ; NaN ];
Tc_826  = [ NaN ; NaN ; NaN ];
omc_error_826 = [ NaN ; NaN ; NaN ];
Tc_error_826  = [ NaN ; NaN ; NaN ];

%-- Image #827:
omc_827 = [ NaN ; NaN ; NaN ];
Tc_827  = [ NaN ; NaN ; NaN ];
omc_error_827 = [ NaN ; NaN ; NaN ];
Tc_error_827  = [ NaN ; NaN ; NaN ];

%-- Image #828:
omc_828 = [ NaN ; NaN ; NaN ];
Tc_828  = [ NaN ; NaN ; NaN ];
omc_error_828 = [ NaN ; NaN ; NaN ];
Tc_error_828  = [ NaN ; NaN ; NaN ];

%-- Image #829:
omc_829 = [ NaN ; NaN ; NaN ];
Tc_829  = [ NaN ; NaN ; NaN ];
omc_error_829 = [ NaN ; NaN ; NaN ];
Tc_error_829  = [ NaN ; NaN ; NaN ];

%-- Image #830:
omc_830 = [ NaN ; NaN ; NaN ];
Tc_830  = [ NaN ; NaN ; NaN ];
omc_error_830 = [ NaN ; NaN ; NaN ];
Tc_error_830  = [ NaN ; NaN ; NaN ];

%-- Image #831:
omc_831 = [ NaN ; NaN ; NaN ];
Tc_831  = [ NaN ; NaN ; NaN ];
omc_error_831 = [ NaN ; NaN ; NaN ];
Tc_error_831  = [ NaN ; NaN ; NaN ];

%-- Image #832:
omc_832 = [ NaN ; NaN ; NaN ];
Tc_832  = [ NaN ; NaN ; NaN ];
omc_error_832 = [ NaN ; NaN ; NaN ];
Tc_error_832  = [ NaN ; NaN ; NaN ];

%-- Image #833:
omc_833 = [ NaN ; NaN ; NaN ];
Tc_833  = [ NaN ; NaN ; NaN ];
omc_error_833 = [ NaN ; NaN ; NaN ];
Tc_error_833  = [ NaN ; NaN ; NaN ];

%-- Image #834:
omc_834 = [ NaN ; NaN ; NaN ];
Tc_834  = [ NaN ; NaN ; NaN ];
omc_error_834 = [ NaN ; NaN ; NaN ];
Tc_error_834  = [ NaN ; NaN ; NaN ];

%-- Image #835:
omc_835 = [ NaN ; NaN ; NaN ];
Tc_835  = [ NaN ; NaN ; NaN ];
omc_error_835 = [ NaN ; NaN ; NaN ];
Tc_error_835  = [ NaN ; NaN ; NaN ];

%-- Image #836:
omc_836 = [ NaN ; NaN ; NaN ];
Tc_836  = [ NaN ; NaN ; NaN ];
omc_error_836 = [ NaN ; NaN ; NaN ];
Tc_error_836  = [ NaN ; NaN ; NaN ];

%-- Image #837:
omc_837 = [ NaN ; NaN ; NaN ];
Tc_837  = [ NaN ; NaN ; NaN ];
omc_error_837 = [ NaN ; NaN ; NaN ];
Tc_error_837  = [ NaN ; NaN ; NaN ];

%-- Image #838:
omc_838 = [ NaN ; NaN ; NaN ];
Tc_838  = [ NaN ; NaN ; NaN ];
omc_error_838 = [ NaN ; NaN ; NaN ];
Tc_error_838  = [ NaN ; NaN ; NaN ];

%-- Image #839:
omc_839 = [ NaN ; NaN ; NaN ];
Tc_839  = [ NaN ; NaN ; NaN ];
omc_error_839 = [ NaN ; NaN ; NaN ];
Tc_error_839  = [ NaN ; NaN ; NaN ];

%-- Image #840:
omc_840 = [ -1.736973e+000 ; -1.800384e+000 ; -1.490404e-001 ];
Tc_840  = [ -1.114402e+002 ; -1.068698e+002 ; 2.810925e+002 ];
omc_error_840 = [ 1.540039e-002 ; 1.505318e-002 ; 2.392231e-002 ];
Tc_error_840  = [ 3.750668e+000 ; 4.070443e+000 ; 5.546460e+000 ];

%-- Image #841:
omc_841 = [ NaN ; NaN ; NaN ];
Tc_841  = [ NaN ; NaN ; NaN ];
omc_error_841 = [ NaN ; NaN ; NaN ];
Tc_error_841  = [ NaN ; NaN ; NaN ];

%-- Image #842:
omc_842 = [ NaN ; NaN ; NaN ];
Tc_842  = [ NaN ; NaN ; NaN ];
omc_error_842 = [ NaN ; NaN ; NaN ];
Tc_error_842  = [ NaN ; NaN ; NaN ];

%-- Image #843:
omc_843 = [ NaN ; NaN ; NaN ];
Tc_843  = [ NaN ; NaN ; NaN ];
omc_error_843 = [ NaN ; NaN ; NaN ];
Tc_error_843  = [ NaN ; NaN ; NaN ];

%-- Image #844:
omc_844 = [ NaN ; NaN ; NaN ];
Tc_844  = [ NaN ; NaN ; NaN ];
omc_error_844 = [ NaN ; NaN ; NaN ];
Tc_error_844  = [ NaN ; NaN ; NaN ];

%-- Image #845:
omc_845 = [ NaN ; NaN ; NaN ];
Tc_845  = [ NaN ; NaN ; NaN ];
omc_error_845 = [ NaN ; NaN ; NaN ];
Tc_error_845  = [ NaN ; NaN ; NaN ];

%-- Image #846:
omc_846 = [ NaN ; NaN ; NaN ];
Tc_846  = [ NaN ; NaN ; NaN ];
omc_error_846 = [ NaN ; NaN ; NaN ];
Tc_error_846  = [ NaN ; NaN ; NaN ];

%-- Image #847:
omc_847 = [ NaN ; NaN ; NaN ];
Tc_847  = [ NaN ; NaN ; NaN ];
omc_error_847 = [ NaN ; NaN ; NaN ];
Tc_error_847  = [ NaN ; NaN ; NaN ];

%-- Image #848:
omc_848 = [ NaN ; NaN ; NaN ];
Tc_848  = [ NaN ; NaN ; NaN ];
omc_error_848 = [ NaN ; NaN ; NaN ];
Tc_error_848  = [ NaN ; NaN ; NaN ];

%-- Image #849:
omc_849 = [ NaN ; NaN ; NaN ];
Tc_849  = [ NaN ; NaN ; NaN ];
omc_error_849 = [ NaN ; NaN ; NaN ];
Tc_error_849  = [ NaN ; NaN ; NaN ];

%-- Image #850:
omc_850 = [ NaN ; NaN ; NaN ];
Tc_850  = [ NaN ; NaN ; NaN ];
omc_error_850 = [ NaN ; NaN ; NaN ];
Tc_error_850  = [ NaN ; NaN ; NaN ];

%-- Image #851:
omc_851 = [ NaN ; NaN ; NaN ];
Tc_851  = [ NaN ; NaN ; NaN ];
omc_error_851 = [ NaN ; NaN ; NaN ];
Tc_error_851  = [ NaN ; NaN ; NaN ];

%-- Image #852:
omc_852 = [ NaN ; NaN ; NaN ];
Tc_852  = [ NaN ; NaN ; NaN ];
omc_error_852 = [ NaN ; NaN ; NaN ];
Tc_error_852  = [ NaN ; NaN ; NaN ];

%-- Image #853:
omc_853 = [ NaN ; NaN ; NaN ];
Tc_853  = [ NaN ; NaN ; NaN ];
omc_error_853 = [ NaN ; NaN ; NaN ];
Tc_error_853  = [ NaN ; NaN ; NaN ];

%-- Image #854:
omc_854 = [ NaN ; NaN ; NaN ];
Tc_854  = [ NaN ; NaN ; NaN ];
omc_error_854 = [ NaN ; NaN ; NaN ];
Tc_error_854  = [ NaN ; NaN ; NaN ];

%-- Image #855:
omc_855 = [ NaN ; NaN ; NaN ];
Tc_855  = [ NaN ; NaN ; NaN ];
omc_error_855 = [ NaN ; NaN ; NaN ];
Tc_error_855  = [ NaN ; NaN ; NaN ];

%-- Image #856:
omc_856 = [ NaN ; NaN ; NaN ];
Tc_856  = [ NaN ; NaN ; NaN ];
omc_error_856 = [ NaN ; NaN ; NaN ];
Tc_error_856  = [ NaN ; NaN ; NaN ];

%-- Image #857:
omc_857 = [ NaN ; NaN ; NaN ];
Tc_857  = [ NaN ; NaN ; NaN ];
omc_error_857 = [ NaN ; NaN ; NaN ];
Tc_error_857  = [ NaN ; NaN ; NaN ];

%-- Image #858:
omc_858 = [ NaN ; NaN ; NaN ];
Tc_858  = [ NaN ; NaN ; NaN ];
omc_error_858 = [ NaN ; NaN ; NaN ];
Tc_error_858  = [ NaN ; NaN ; NaN ];

%-- Image #859:
omc_859 = [ NaN ; NaN ; NaN ];
Tc_859  = [ NaN ; NaN ; NaN ];
omc_error_859 = [ NaN ; NaN ; NaN ];
Tc_error_859  = [ NaN ; NaN ; NaN ];

%-- Image #860:
omc_860 = [ NaN ; NaN ; NaN ];
Tc_860  = [ NaN ; NaN ; NaN ];
omc_error_860 = [ NaN ; NaN ; NaN ];
Tc_error_860  = [ NaN ; NaN ; NaN ];

%-- Image #861:
omc_861 = [ NaN ; NaN ; NaN ];
Tc_861  = [ NaN ; NaN ; NaN ];
omc_error_861 = [ NaN ; NaN ; NaN ];
Tc_error_861  = [ NaN ; NaN ; NaN ];

%-- Image #862:
omc_862 = [ NaN ; NaN ; NaN ];
Tc_862  = [ NaN ; NaN ; NaN ];
omc_error_862 = [ NaN ; NaN ; NaN ];
Tc_error_862  = [ NaN ; NaN ; NaN ];

%-- Image #863:
omc_863 = [ NaN ; NaN ; NaN ];
Tc_863  = [ NaN ; NaN ; NaN ];
omc_error_863 = [ NaN ; NaN ; NaN ];
Tc_error_863  = [ NaN ; NaN ; NaN ];

%-- Image #864:
omc_864 = [ NaN ; NaN ; NaN ];
Tc_864  = [ NaN ; NaN ; NaN ];
omc_error_864 = [ NaN ; NaN ; NaN ];
Tc_error_864  = [ NaN ; NaN ; NaN ];

%-- Image #865:
omc_865 = [ NaN ; NaN ; NaN ];
Tc_865  = [ NaN ; NaN ; NaN ];
omc_error_865 = [ NaN ; NaN ; NaN ];
Tc_error_865  = [ NaN ; NaN ; NaN ];

%-- Image #866:
omc_866 = [ NaN ; NaN ; NaN ];
Tc_866  = [ NaN ; NaN ; NaN ];
omc_error_866 = [ NaN ; NaN ; NaN ];
Tc_error_866  = [ NaN ; NaN ; NaN ];

%-- Image #867:
omc_867 = [ NaN ; NaN ; NaN ];
Tc_867  = [ NaN ; NaN ; NaN ];
omc_error_867 = [ NaN ; NaN ; NaN ];
Tc_error_867  = [ NaN ; NaN ; NaN ];

%-- Image #868:
omc_868 = [ NaN ; NaN ; NaN ];
Tc_868  = [ NaN ; NaN ; NaN ];
omc_error_868 = [ NaN ; NaN ; NaN ];
Tc_error_868  = [ NaN ; NaN ; NaN ];

%-- Image #869:
omc_869 = [ NaN ; NaN ; NaN ];
Tc_869  = [ NaN ; NaN ; NaN ];
omc_error_869 = [ NaN ; NaN ; NaN ];
Tc_error_869  = [ NaN ; NaN ; NaN ];

%-- Image #870:
omc_870 = [ NaN ; NaN ; NaN ];
Tc_870  = [ NaN ; NaN ; NaN ];
omc_error_870 = [ NaN ; NaN ; NaN ];
Tc_error_870  = [ NaN ; NaN ; NaN ];

%-- Image #871:
omc_871 = [ NaN ; NaN ; NaN ];
Tc_871  = [ NaN ; NaN ; NaN ];
omc_error_871 = [ NaN ; NaN ; NaN ];
Tc_error_871  = [ NaN ; NaN ; NaN ];

%-- Image #872:
omc_872 = [ NaN ; NaN ; NaN ];
Tc_872  = [ NaN ; NaN ; NaN ];
omc_error_872 = [ NaN ; NaN ; NaN ];
Tc_error_872  = [ NaN ; NaN ; NaN ];

%-- Image #873:
omc_873 = [ NaN ; NaN ; NaN ];
Tc_873  = [ NaN ; NaN ; NaN ];
omc_error_873 = [ NaN ; NaN ; NaN ];
Tc_error_873  = [ NaN ; NaN ; NaN ];

%-- Image #874:
omc_874 = [ NaN ; NaN ; NaN ];
Tc_874  = [ NaN ; NaN ; NaN ];
omc_error_874 = [ NaN ; NaN ; NaN ];
Tc_error_874  = [ NaN ; NaN ; NaN ];

%-- Image #875:
omc_875 = [ NaN ; NaN ; NaN ];
Tc_875  = [ NaN ; NaN ; NaN ];
omc_error_875 = [ NaN ; NaN ; NaN ];
Tc_error_875  = [ NaN ; NaN ; NaN ];

%-- Image #876:
omc_876 = [ NaN ; NaN ; NaN ];
Tc_876  = [ NaN ; NaN ; NaN ];
omc_error_876 = [ NaN ; NaN ; NaN ];
Tc_error_876  = [ NaN ; NaN ; NaN ];

%-- Image #877:
omc_877 = [ NaN ; NaN ; NaN ];
Tc_877  = [ NaN ; NaN ; NaN ];
omc_error_877 = [ NaN ; NaN ; NaN ];
Tc_error_877  = [ NaN ; NaN ; NaN ];

%-- Image #878:
omc_878 = [ NaN ; NaN ; NaN ];
Tc_878  = [ NaN ; NaN ; NaN ];
omc_error_878 = [ NaN ; NaN ; NaN ];
Tc_error_878  = [ NaN ; NaN ; NaN ];

%-- Image #879:
omc_879 = [ NaN ; NaN ; NaN ];
Tc_879  = [ NaN ; NaN ; NaN ];
omc_error_879 = [ NaN ; NaN ; NaN ];
Tc_error_879  = [ NaN ; NaN ; NaN ];

%-- Image #880:
omc_880 = [ NaN ; NaN ; NaN ];
Tc_880  = [ NaN ; NaN ; NaN ];
omc_error_880 = [ NaN ; NaN ; NaN ];
Tc_error_880  = [ NaN ; NaN ; NaN ];

%-- Image #881:
omc_881 = [ NaN ; NaN ; NaN ];
Tc_881  = [ NaN ; NaN ; NaN ];
omc_error_881 = [ NaN ; NaN ; NaN ];
Tc_error_881  = [ NaN ; NaN ; NaN ];

%-- Image #882:
omc_882 = [ NaN ; NaN ; NaN ];
Tc_882  = [ NaN ; NaN ; NaN ];
omc_error_882 = [ NaN ; NaN ; NaN ];
Tc_error_882  = [ NaN ; NaN ; NaN ];

%-- Image #883:
omc_883 = [ NaN ; NaN ; NaN ];
Tc_883  = [ NaN ; NaN ; NaN ];
omc_error_883 = [ NaN ; NaN ; NaN ];
Tc_error_883  = [ NaN ; NaN ; NaN ];

%-- Image #884:
omc_884 = [ NaN ; NaN ; NaN ];
Tc_884  = [ NaN ; NaN ; NaN ];
omc_error_884 = [ NaN ; NaN ; NaN ];
Tc_error_884  = [ NaN ; NaN ; NaN ];

%-- Image #885:
omc_885 = [ NaN ; NaN ; NaN ];
Tc_885  = [ NaN ; NaN ; NaN ];
omc_error_885 = [ NaN ; NaN ; NaN ];
Tc_error_885  = [ NaN ; NaN ; NaN ];

%-- Image #886:
omc_886 = [ NaN ; NaN ; NaN ];
Tc_886  = [ NaN ; NaN ; NaN ];
omc_error_886 = [ NaN ; NaN ; NaN ];
Tc_error_886  = [ NaN ; NaN ; NaN ];

%-- Image #887:
omc_887 = [ NaN ; NaN ; NaN ];
Tc_887  = [ NaN ; NaN ; NaN ];
omc_error_887 = [ NaN ; NaN ; NaN ];
Tc_error_887  = [ NaN ; NaN ; NaN ];

%-- Image #888:
omc_888 = [ NaN ; NaN ; NaN ];
Tc_888  = [ NaN ; NaN ; NaN ];
omc_error_888 = [ NaN ; NaN ; NaN ];
Tc_error_888  = [ NaN ; NaN ; NaN ];

%-- Image #889:
omc_889 = [ NaN ; NaN ; NaN ];
Tc_889  = [ NaN ; NaN ; NaN ];
omc_error_889 = [ NaN ; NaN ; NaN ];
Tc_error_889  = [ NaN ; NaN ; NaN ];

%-- Image #890:
omc_890 = [ NaN ; NaN ; NaN ];
Tc_890  = [ NaN ; NaN ; NaN ];
omc_error_890 = [ NaN ; NaN ; NaN ];
Tc_error_890  = [ NaN ; NaN ; NaN ];

%-- Image #891:
omc_891 = [ NaN ; NaN ; NaN ];
Tc_891  = [ NaN ; NaN ; NaN ];
omc_error_891 = [ NaN ; NaN ; NaN ];
Tc_error_891  = [ NaN ; NaN ; NaN ];

%-- Image #892:
omc_892 = [ NaN ; NaN ; NaN ];
Tc_892  = [ NaN ; NaN ; NaN ];
omc_error_892 = [ NaN ; NaN ; NaN ];
Tc_error_892  = [ NaN ; NaN ; NaN ];

%-- Image #893:
omc_893 = [ NaN ; NaN ; NaN ];
Tc_893  = [ NaN ; NaN ; NaN ];
omc_error_893 = [ NaN ; NaN ; NaN ];
Tc_error_893  = [ NaN ; NaN ; NaN ];

%-- Image #894:
omc_894 = [ NaN ; NaN ; NaN ];
Tc_894  = [ NaN ; NaN ; NaN ];
omc_error_894 = [ NaN ; NaN ; NaN ];
Tc_error_894  = [ NaN ; NaN ; NaN ];

%-- Image #895:
omc_895 = [ NaN ; NaN ; NaN ];
Tc_895  = [ NaN ; NaN ; NaN ];
omc_error_895 = [ NaN ; NaN ; NaN ];
Tc_error_895  = [ NaN ; NaN ; NaN ];

%-- Image #896:
omc_896 = [ NaN ; NaN ; NaN ];
Tc_896  = [ NaN ; NaN ; NaN ];
omc_error_896 = [ NaN ; NaN ; NaN ];
Tc_error_896  = [ NaN ; NaN ; NaN ];

%-- Image #897:
omc_897 = [ NaN ; NaN ; NaN ];
Tc_897  = [ NaN ; NaN ; NaN ];
omc_error_897 = [ NaN ; NaN ; NaN ];
Tc_error_897  = [ NaN ; NaN ; NaN ];

%-- Image #898:
omc_898 = [ NaN ; NaN ; NaN ];
Tc_898  = [ NaN ; NaN ; NaN ];
omc_error_898 = [ NaN ; NaN ; NaN ];
Tc_error_898  = [ NaN ; NaN ; NaN ];

%-- Image #899:
omc_899 = [ NaN ; NaN ; NaN ];
Tc_899  = [ NaN ; NaN ; NaN ];
omc_error_899 = [ NaN ; NaN ; NaN ];
Tc_error_899  = [ NaN ; NaN ; NaN ];

%-- Image #900:
omc_900 = [ NaN ; NaN ; NaN ];
Tc_900  = [ NaN ; NaN ; NaN ];
omc_error_900 = [ NaN ; NaN ; NaN ];
Tc_error_900  = [ NaN ; NaN ; NaN ];

%-- Image #901:
omc_901 = [ NaN ; NaN ; NaN ];
Tc_901  = [ NaN ; NaN ; NaN ];
omc_error_901 = [ NaN ; NaN ; NaN ];
Tc_error_901  = [ NaN ; NaN ; NaN ];

%-- Image #902:
omc_902 = [ NaN ; NaN ; NaN ];
Tc_902  = [ NaN ; NaN ; NaN ];
omc_error_902 = [ NaN ; NaN ; NaN ];
Tc_error_902  = [ NaN ; NaN ; NaN ];

%-- Image #903:
omc_903 = [ NaN ; NaN ; NaN ];
Tc_903  = [ NaN ; NaN ; NaN ];
omc_error_903 = [ NaN ; NaN ; NaN ];
Tc_error_903  = [ NaN ; NaN ; NaN ];

%-- Image #904:
omc_904 = [ NaN ; NaN ; NaN ];
Tc_904  = [ NaN ; NaN ; NaN ];
omc_error_904 = [ NaN ; NaN ; NaN ];
Tc_error_904  = [ NaN ; NaN ; NaN ];

%-- Image #905:
omc_905 = [ NaN ; NaN ; NaN ];
Tc_905  = [ NaN ; NaN ; NaN ];
omc_error_905 = [ NaN ; NaN ; NaN ];
Tc_error_905  = [ NaN ; NaN ; NaN ];

%-- Image #906:
omc_906 = [ NaN ; NaN ; NaN ];
Tc_906  = [ NaN ; NaN ; NaN ];
omc_error_906 = [ NaN ; NaN ; NaN ];
Tc_error_906  = [ NaN ; NaN ; NaN ];

%-- Image #907:
omc_907 = [ NaN ; NaN ; NaN ];
Tc_907  = [ NaN ; NaN ; NaN ];
omc_error_907 = [ NaN ; NaN ; NaN ];
Tc_error_907  = [ NaN ; NaN ; NaN ];

%-- Image #908:
omc_908 = [ NaN ; NaN ; NaN ];
Tc_908  = [ NaN ; NaN ; NaN ];
omc_error_908 = [ NaN ; NaN ; NaN ];
Tc_error_908  = [ NaN ; NaN ; NaN ];

%-- Image #909:
omc_909 = [ NaN ; NaN ; NaN ];
Tc_909  = [ NaN ; NaN ; NaN ];
omc_error_909 = [ NaN ; NaN ; NaN ];
Tc_error_909  = [ NaN ; NaN ; NaN ];

%-- Image #910:
omc_910 = [ NaN ; NaN ; NaN ];
Tc_910  = [ NaN ; NaN ; NaN ];
omc_error_910 = [ NaN ; NaN ; NaN ];
Tc_error_910  = [ NaN ; NaN ; NaN ];

%-- Image #911:
omc_911 = [ NaN ; NaN ; NaN ];
Tc_911  = [ NaN ; NaN ; NaN ];
omc_error_911 = [ NaN ; NaN ; NaN ];
Tc_error_911  = [ NaN ; NaN ; NaN ];

%-- Image #912:
omc_912 = [ NaN ; NaN ; NaN ];
Tc_912  = [ NaN ; NaN ; NaN ];
omc_error_912 = [ NaN ; NaN ; NaN ];
Tc_error_912  = [ NaN ; NaN ; NaN ];

%-- Image #913:
omc_913 = [ NaN ; NaN ; NaN ];
Tc_913  = [ NaN ; NaN ; NaN ];
omc_error_913 = [ NaN ; NaN ; NaN ];
Tc_error_913  = [ NaN ; NaN ; NaN ];

%-- Image #914:
omc_914 = [ NaN ; NaN ; NaN ];
Tc_914  = [ NaN ; NaN ; NaN ];
omc_error_914 = [ NaN ; NaN ; NaN ];
Tc_error_914  = [ NaN ; NaN ; NaN ];

%-- Image #915:
omc_915 = [ NaN ; NaN ; NaN ];
Tc_915  = [ NaN ; NaN ; NaN ];
omc_error_915 = [ NaN ; NaN ; NaN ];
Tc_error_915  = [ NaN ; NaN ; NaN ];

%-- Image #916:
omc_916 = [ NaN ; NaN ; NaN ];
Tc_916  = [ NaN ; NaN ; NaN ];
omc_error_916 = [ NaN ; NaN ; NaN ];
Tc_error_916  = [ NaN ; NaN ; NaN ];

%-- Image #917:
omc_917 = [ NaN ; NaN ; NaN ];
Tc_917  = [ NaN ; NaN ; NaN ];
omc_error_917 = [ NaN ; NaN ; NaN ];
Tc_error_917  = [ NaN ; NaN ; NaN ];

%-- Image #918:
omc_918 = [ NaN ; NaN ; NaN ];
Tc_918  = [ NaN ; NaN ; NaN ];
omc_error_918 = [ NaN ; NaN ; NaN ];
Tc_error_918  = [ NaN ; NaN ; NaN ];

%-- Image #919:
omc_919 = [ NaN ; NaN ; NaN ];
Tc_919  = [ NaN ; NaN ; NaN ];
omc_error_919 = [ NaN ; NaN ; NaN ];
Tc_error_919  = [ NaN ; NaN ; NaN ];

%-- Image #920:
omc_920 = [ NaN ; NaN ; NaN ];
Tc_920  = [ NaN ; NaN ; NaN ];
omc_error_920 = [ NaN ; NaN ; NaN ];
Tc_error_920  = [ NaN ; NaN ; NaN ];

%-- Image #921:
omc_921 = [ NaN ; NaN ; NaN ];
Tc_921  = [ NaN ; NaN ; NaN ];
omc_error_921 = [ NaN ; NaN ; NaN ];
Tc_error_921  = [ NaN ; NaN ; NaN ];

%-- Image #922:
omc_922 = [ NaN ; NaN ; NaN ];
Tc_922  = [ NaN ; NaN ; NaN ];
omc_error_922 = [ NaN ; NaN ; NaN ];
Tc_error_922  = [ NaN ; NaN ; NaN ];

%-- Image #923:
omc_923 = [ NaN ; NaN ; NaN ];
Tc_923  = [ NaN ; NaN ; NaN ];
omc_error_923 = [ NaN ; NaN ; NaN ];
Tc_error_923  = [ NaN ; NaN ; NaN ];

%-- Image #924:
omc_924 = [ NaN ; NaN ; NaN ];
Tc_924  = [ NaN ; NaN ; NaN ];
omc_error_924 = [ NaN ; NaN ; NaN ];
Tc_error_924  = [ NaN ; NaN ; NaN ];

%-- Image #925:
omc_925 = [ NaN ; NaN ; NaN ];
Tc_925  = [ NaN ; NaN ; NaN ];
omc_error_925 = [ NaN ; NaN ; NaN ];
Tc_error_925  = [ NaN ; NaN ; NaN ];

%-- Image #926:
omc_926 = [ NaN ; NaN ; NaN ];
Tc_926  = [ NaN ; NaN ; NaN ];
omc_error_926 = [ NaN ; NaN ; NaN ];
Tc_error_926  = [ NaN ; NaN ; NaN ];

%-- Image #927:
omc_927 = [ NaN ; NaN ; NaN ];
Tc_927  = [ NaN ; NaN ; NaN ];
omc_error_927 = [ NaN ; NaN ; NaN ];
Tc_error_927  = [ NaN ; NaN ; NaN ];

%-- Image #928:
omc_928 = [ NaN ; NaN ; NaN ];
Tc_928  = [ NaN ; NaN ; NaN ];
omc_error_928 = [ NaN ; NaN ; NaN ];
Tc_error_928  = [ NaN ; NaN ; NaN ];

%-- Image #929:
omc_929 = [ NaN ; NaN ; NaN ];
Tc_929  = [ NaN ; NaN ; NaN ];
omc_error_929 = [ NaN ; NaN ; NaN ];
Tc_error_929  = [ NaN ; NaN ; NaN ];

%-- Image #930:
omc_930 = [ NaN ; NaN ; NaN ];
Tc_930  = [ NaN ; NaN ; NaN ];
omc_error_930 = [ NaN ; NaN ; NaN ];
Tc_error_930  = [ NaN ; NaN ; NaN ];

%-- Image #931:
omc_931 = [ NaN ; NaN ; NaN ];
Tc_931  = [ NaN ; NaN ; NaN ];
omc_error_931 = [ NaN ; NaN ; NaN ];
Tc_error_931  = [ NaN ; NaN ; NaN ];

%-- Image #932:
omc_932 = [ NaN ; NaN ; NaN ];
Tc_932  = [ NaN ; NaN ; NaN ];
omc_error_932 = [ NaN ; NaN ; NaN ];
Tc_error_932  = [ NaN ; NaN ; NaN ];

%-- Image #933:
omc_933 = [ NaN ; NaN ; NaN ];
Tc_933  = [ NaN ; NaN ; NaN ];
omc_error_933 = [ NaN ; NaN ; NaN ];
Tc_error_933  = [ NaN ; NaN ; NaN ];

%-- Image #934:
omc_934 = [ NaN ; NaN ; NaN ];
Tc_934  = [ NaN ; NaN ; NaN ];
omc_error_934 = [ NaN ; NaN ; NaN ];
Tc_error_934  = [ NaN ; NaN ; NaN ];

%-- Image #935:
omc_935 = [ NaN ; NaN ; NaN ];
Tc_935  = [ NaN ; NaN ; NaN ];
omc_error_935 = [ NaN ; NaN ; NaN ];
Tc_error_935  = [ NaN ; NaN ; NaN ];

%-- Image #936:
omc_936 = [ NaN ; NaN ; NaN ];
Tc_936  = [ NaN ; NaN ; NaN ];
omc_error_936 = [ NaN ; NaN ; NaN ];
Tc_error_936  = [ NaN ; NaN ; NaN ];

%-- Image #937:
omc_937 = [ NaN ; NaN ; NaN ];
Tc_937  = [ NaN ; NaN ; NaN ];
omc_error_937 = [ NaN ; NaN ; NaN ];
Tc_error_937  = [ NaN ; NaN ; NaN ];

%-- Image #938:
omc_938 = [ NaN ; NaN ; NaN ];
Tc_938  = [ NaN ; NaN ; NaN ];
omc_error_938 = [ NaN ; NaN ; NaN ];
Tc_error_938  = [ NaN ; NaN ; NaN ];

%-- Image #939:
omc_939 = [ NaN ; NaN ; NaN ];
Tc_939  = [ NaN ; NaN ; NaN ];
omc_error_939 = [ NaN ; NaN ; NaN ];
Tc_error_939  = [ NaN ; NaN ; NaN ];

%-- Image #940:
omc_940 = [ 2.011742e+000 ; 1.690993e+000 ; 1.652370e-001 ];
Tc_940  = [ -8.614120e+001 ; -8.355489e+001 ; 2.641538e+002 ];
omc_error_940 = [ 1.564242e-002 ; 1.291613e-002 ; 2.668684e-002 ];
Tc_error_940  = [ 3.567271e+000 ; 3.670192e+000 ; 5.250721e+000 ];

%-- Image #941:
omc_941 = [ NaN ; NaN ; NaN ];
Tc_941  = [ NaN ; NaN ; NaN ];
omc_error_941 = [ NaN ; NaN ; NaN ];
Tc_error_941  = [ NaN ; NaN ; NaN ];

%-- Image #942:
omc_942 = [ NaN ; NaN ; NaN ];
Tc_942  = [ NaN ; NaN ; NaN ];
omc_error_942 = [ NaN ; NaN ; NaN ];
Tc_error_942  = [ NaN ; NaN ; NaN ];

%-- Image #943:
omc_943 = [ NaN ; NaN ; NaN ];
Tc_943  = [ NaN ; NaN ; NaN ];
omc_error_943 = [ NaN ; NaN ; NaN ];
Tc_error_943  = [ NaN ; NaN ; NaN ];

%-- Image #944:
omc_944 = [ NaN ; NaN ; NaN ];
Tc_944  = [ NaN ; NaN ; NaN ];
omc_error_944 = [ NaN ; NaN ; NaN ];
Tc_error_944  = [ NaN ; NaN ; NaN ];

%-- Image #945:
omc_945 = [ NaN ; NaN ; NaN ];
Tc_945  = [ NaN ; NaN ; NaN ];
omc_error_945 = [ NaN ; NaN ; NaN ];
Tc_error_945  = [ NaN ; NaN ; NaN ];

%-- Image #946:
omc_946 = [ NaN ; NaN ; NaN ];
Tc_946  = [ NaN ; NaN ; NaN ];
omc_error_946 = [ NaN ; NaN ; NaN ];
Tc_error_946  = [ NaN ; NaN ; NaN ];

%-- Image #947:
omc_947 = [ NaN ; NaN ; NaN ];
Tc_947  = [ NaN ; NaN ; NaN ];
omc_error_947 = [ NaN ; NaN ; NaN ];
Tc_error_947  = [ NaN ; NaN ; NaN ];

%-- Image #948:
omc_948 = [ NaN ; NaN ; NaN ];
Tc_948  = [ NaN ; NaN ; NaN ];
omc_error_948 = [ NaN ; NaN ; NaN ];
Tc_error_948  = [ NaN ; NaN ; NaN ];

%-- Image #949:
omc_949 = [ NaN ; NaN ; NaN ];
Tc_949  = [ NaN ; NaN ; NaN ];
omc_error_949 = [ NaN ; NaN ; NaN ];
Tc_error_949  = [ NaN ; NaN ; NaN ];

%-- Image #950:
omc_950 = [ NaN ; NaN ; NaN ];
Tc_950  = [ NaN ; NaN ; NaN ];
omc_error_950 = [ NaN ; NaN ; NaN ];
Tc_error_950  = [ NaN ; NaN ; NaN ];

%-- Image #951:
omc_951 = [ NaN ; NaN ; NaN ];
Tc_951  = [ NaN ; NaN ; NaN ];
omc_error_951 = [ NaN ; NaN ; NaN ];
Tc_error_951  = [ NaN ; NaN ; NaN ];

%-- Image #952:
omc_952 = [ NaN ; NaN ; NaN ];
Tc_952  = [ NaN ; NaN ; NaN ];
omc_error_952 = [ NaN ; NaN ; NaN ];
Tc_error_952  = [ NaN ; NaN ; NaN ];

%-- Image #953:
omc_953 = [ NaN ; NaN ; NaN ];
Tc_953  = [ NaN ; NaN ; NaN ];
omc_error_953 = [ NaN ; NaN ; NaN ];
Tc_error_953  = [ NaN ; NaN ; NaN ];

%-- Image #954:
omc_954 = [ NaN ; NaN ; NaN ];
Tc_954  = [ NaN ; NaN ; NaN ];
omc_error_954 = [ NaN ; NaN ; NaN ];
Tc_error_954  = [ NaN ; NaN ; NaN ];

%-- Image #955:
omc_955 = [ NaN ; NaN ; NaN ];
Tc_955  = [ NaN ; NaN ; NaN ];
omc_error_955 = [ NaN ; NaN ; NaN ];
Tc_error_955  = [ NaN ; NaN ; NaN ];

%-- Image #956:
omc_956 = [ NaN ; NaN ; NaN ];
Tc_956  = [ NaN ; NaN ; NaN ];
omc_error_956 = [ NaN ; NaN ; NaN ];
Tc_error_956  = [ NaN ; NaN ; NaN ];

%-- Image #957:
omc_957 = [ NaN ; NaN ; NaN ];
Tc_957  = [ NaN ; NaN ; NaN ];
omc_error_957 = [ NaN ; NaN ; NaN ];
Tc_error_957  = [ NaN ; NaN ; NaN ];

%-- Image #958:
omc_958 = [ NaN ; NaN ; NaN ];
Tc_958  = [ NaN ; NaN ; NaN ];
omc_error_958 = [ NaN ; NaN ; NaN ];
Tc_error_958  = [ NaN ; NaN ; NaN ];

%-- Image #959:
omc_959 = [ NaN ; NaN ; NaN ];
Tc_959  = [ NaN ; NaN ; NaN ];
omc_error_959 = [ NaN ; NaN ; NaN ];
Tc_error_959  = [ NaN ; NaN ; NaN ];

%-- Image #960:
omc_960 = [ NaN ; NaN ; NaN ];
Tc_960  = [ NaN ; NaN ; NaN ];
omc_error_960 = [ NaN ; NaN ; NaN ];
Tc_error_960  = [ NaN ; NaN ; NaN ];

%-- Image #961:
omc_961 = [ NaN ; NaN ; NaN ];
Tc_961  = [ NaN ; NaN ; NaN ];
omc_error_961 = [ NaN ; NaN ; NaN ];
Tc_error_961  = [ NaN ; NaN ; NaN ];

%-- Image #962:
omc_962 = [ NaN ; NaN ; NaN ];
Tc_962  = [ NaN ; NaN ; NaN ];
omc_error_962 = [ NaN ; NaN ; NaN ];
Tc_error_962  = [ NaN ; NaN ; NaN ];

%-- Image #963:
omc_963 = [ NaN ; NaN ; NaN ];
Tc_963  = [ NaN ; NaN ; NaN ];
omc_error_963 = [ NaN ; NaN ; NaN ];
Tc_error_963  = [ NaN ; NaN ; NaN ];

%-- Image #964:
omc_964 = [ NaN ; NaN ; NaN ];
Tc_964  = [ NaN ; NaN ; NaN ];
omc_error_964 = [ NaN ; NaN ; NaN ];
Tc_error_964  = [ NaN ; NaN ; NaN ];

%-- Image #965:
omc_965 = [ 2.081514e+000 ; 1.876524e+000 ; 3.762270e-001 ];
Tc_965  = [ -7.048186e+001 ; -8.677834e+001 ; 2.347021e+002 ];
omc_error_965 = [ 1.574448e-002 ; 1.319606e-002 ; 2.922489e-002 ];
Tc_error_965  = [ 3.237484e+000 ; 3.331858e+000 ; 5.113007e+000 ];

%-- Image #966:
omc_966 = [ NaN ; NaN ; NaN ];
Tc_966  = [ NaN ; NaN ; NaN ];
omc_error_966 = [ NaN ; NaN ; NaN ];
Tc_error_966  = [ NaN ; NaN ; NaN ];

%-- Image #967:
omc_967 = [ NaN ; NaN ; NaN ];
Tc_967  = [ NaN ; NaN ; NaN ];
omc_error_967 = [ NaN ; NaN ; NaN ];
Tc_error_967  = [ NaN ; NaN ; NaN ];

%-- Image #968:
omc_968 = [ NaN ; NaN ; NaN ];
Tc_968  = [ NaN ; NaN ; NaN ];
omc_error_968 = [ NaN ; NaN ; NaN ];
Tc_error_968  = [ NaN ; NaN ; NaN ];

%-- Image #969:
omc_969 = [ NaN ; NaN ; NaN ];
Tc_969  = [ NaN ; NaN ; NaN ];
omc_error_969 = [ NaN ; NaN ; NaN ];
Tc_error_969  = [ NaN ; NaN ; NaN ];

%-- Image #970:
omc_970 = [ NaN ; NaN ; NaN ];
Tc_970  = [ NaN ; NaN ; NaN ];
omc_error_970 = [ NaN ; NaN ; NaN ];
Tc_error_970  = [ NaN ; NaN ; NaN ];

%-- Image #971:
omc_971 = [ NaN ; NaN ; NaN ];
Tc_971  = [ NaN ; NaN ; NaN ];
omc_error_971 = [ NaN ; NaN ; NaN ];
Tc_error_971  = [ NaN ; NaN ; NaN ];

%-- Image #972:
omc_972 = [ NaN ; NaN ; NaN ];
Tc_972  = [ NaN ; NaN ; NaN ];
omc_error_972 = [ NaN ; NaN ; NaN ];
Tc_error_972  = [ NaN ; NaN ; NaN ];

%-- Image #973:
omc_973 = [ NaN ; NaN ; NaN ];
Tc_973  = [ NaN ; NaN ; NaN ];
omc_error_973 = [ NaN ; NaN ; NaN ];
Tc_error_973  = [ NaN ; NaN ; NaN ];

%-- Image #974:
omc_974 = [ NaN ; NaN ; NaN ];
Tc_974  = [ NaN ; NaN ; NaN ];
omc_error_974 = [ NaN ; NaN ; NaN ];
Tc_error_974  = [ NaN ; NaN ; NaN ];

%-- Image #975:
omc_975 = [ NaN ; NaN ; NaN ];
Tc_975  = [ NaN ; NaN ; NaN ];
omc_error_975 = [ NaN ; NaN ; NaN ];
Tc_error_975  = [ NaN ; NaN ; NaN ];

%-- Image #976:
omc_976 = [ NaN ; NaN ; NaN ];
Tc_976  = [ NaN ; NaN ; NaN ];
omc_error_976 = [ NaN ; NaN ; NaN ];
Tc_error_976  = [ NaN ; NaN ; NaN ];

%-- Image #977:
omc_977 = [ NaN ; NaN ; NaN ];
Tc_977  = [ NaN ; NaN ; NaN ];
omc_error_977 = [ NaN ; NaN ; NaN ];
Tc_error_977  = [ NaN ; NaN ; NaN ];

%-- Image #978:
omc_978 = [ NaN ; NaN ; NaN ];
Tc_978  = [ NaN ; NaN ; NaN ];
omc_error_978 = [ NaN ; NaN ; NaN ];
Tc_error_978  = [ NaN ; NaN ; NaN ];

%-- Image #979:
omc_979 = [ NaN ; NaN ; NaN ];
Tc_979  = [ NaN ; NaN ; NaN ];
omc_error_979 = [ NaN ; NaN ; NaN ];
Tc_error_979  = [ NaN ; NaN ; NaN ];

%-- Image #980:
omc_980 = [ NaN ; NaN ; NaN ];
Tc_980  = [ NaN ; NaN ; NaN ];
omc_error_980 = [ NaN ; NaN ; NaN ];
Tc_error_980  = [ NaN ; NaN ; NaN ];

%-- Image #981:
omc_981 = [ NaN ; NaN ; NaN ];
Tc_981  = [ NaN ; NaN ; NaN ];
omc_error_981 = [ NaN ; NaN ; NaN ];
Tc_error_981  = [ NaN ; NaN ; NaN ];

%-- Image #982:
omc_982 = [ NaN ; NaN ; NaN ];
Tc_982  = [ NaN ; NaN ; NaN ];
omc_error_982 = [ NaN ; NaN ; NaN ];
Tc_error_982  = [ NaN ; NaN ; NaN ];

%-- Image #983:
omc_983 = [ NaN ; NaN ; NaN ];
Tc_983  = [ NaN ; NaN ; NaN ];
omc_error_983 = [ NaN ; NaN ; NaN ];
Tc_error_983  = [ NaN ; NaN ; NaN ];

%-- Image #984:
omc_984 = [ NaN ; NaN ; NaN ];
Tc_984  = [ NaN ; NaN ; NaN ];
omc_error_984 = [ NaN ; NaN ; NaN ];
Tc_error_984  = [ NaN ; NaN ; NaN ];

%-- Image #985:
omc_985 = [ NaN ; NaN ; NaN ];
Tc_985  = [ NaN ; NaN ; NaN ];
omc_error_985 = [ NaN ; NaN ; NaN ];
Tc_error_985  = [ NaN ; NaN ; NaN ];

%-- Image #986:
omc_986 = [ NaN ; NaN ; NaN ];
Tc_986  = [ NaN ; NaN ; NaN ];
omc_error_986 = [ NaN ; NaN ; NaN ];
Tc_error_986  = [ NaN ; NaN ; NaN ];

%-- Image #987:
omc_987 = [ NaN ; NaN ; NaN ];
Tc_987  = [ NaN ; NaN ; NaN ];
omc_error_987 = [ NaN ; NaN ; NaN ];
Tc_error_987  = [ NaN ; NaN ; NaN ];

%-- Image #988:
omc_988 = [ NaN ; NaN ; NaN ];
Tc_988  = [ NaN ; NaN ; NaN ];
omc_error_988 = [ NaN ; NaN ; NaN ];
Tc_error_988  = [ NaN ; NaN ; NaN ];

%-- Image #989:
omc_989 = [ NaN ; NaN ; NaN ];
Tc_989  = [ NaN ; NaN ; NaN ];
omc_error_989 = [ NaN ; NaN ; NaN ];
Tc_error_989  = [ NaN ; NaN ; NaN ];

%-- Image #990:
omc_990 = [ NaN ; NaN ; NaN ];
Tc_990  = [ NaN ; NaN ; NaN ];
omc_error_990 = [ NaN ; NaN ; NaN ];
Tc_error_990  = [ NaN ; NaN ; NaN ];

%-- Image #991:
omc_991 = [ NaN ; NaN ; NaN ];
Tc_991  = [ NaN ; NaN ; NaN ];
omc_error_991 = [ NaN ; NaN ; NaN ];
Tc_error_991  = [ NaN ; NaN ; NaN ];

%-- Image #992:
omc_992 = [ NaN ; NaN ; NaN ];
Tc_992  = [ NaN ; NaN ; NaN ];
omc_error_992 = [ NaN ; NaN ; NaN ];
Tc_error_992  = [ NaN ; NaN ; NaN ];

%-- Image #993:
omc_993 = [ 2.110485e+000 ; 2.212026e+000 ; 5.066059e-001 ];
Tc_993  = [ -5.308315e+001 ; -1.104077e+002 ; 2.294871e+002 ];
omc_error_993 = [ 1.630495e-002 ; 1.570615e-002 ; 3.391959e-002 ];
Tc_error_993  = [ 3.229300e+000 ; 3.409989e+000 ; 5.375045e+000 ];

%-- Image #994:
omc_994 = [ NaN ; NaN ; NaN ];
Tc_994  = [ NaN ; NaN ; NaN ];
omc_error_994 = [ NaN ; NaN ; NaN ];
Tc_error_994  = [ NaN ; NaN ; NaN ];

%-- Image #995:
omc_995 = [ NaN ; NaN ; NaN ];
Tc_995  = [ NaN ; NaN ; NaN ];
omc_error_995 = [ NaN ; NaN ; NaN ];
Tc_error_995  = [ NaN ; NaN ; NaN ];

%-- Image #996:
omc_996 = [ NaN ; NaN ; NaN ];
Tc_996  = [ NaN ; NaN ; NaN ];
omc_error_996 = [ NaN ; NaN ; NaN ];
Tc_error_996  = [ NaN ; NaN ; NaN ];

%-- Image #997:
omc_997 = [ NaN ; NaN ; NaN ];
Tc_997  = [ NaN ; NaN ; NaN ];
omc_error_997 = [ NaN ; NaN ; NaN ];
Tc_error_997  = [ NaN ; NaN ; NaN ];

%-- Image #998:
omc_998 = [ NaN ; NaN ; NaN ];
Tc_998  = [ NaN ; NaN ; NaN ];
omc_error_998 = [ NaN ; NaN ; NaN ];
Tc_error_998  = [ NaN ; NaN ; NaN ];

%-- Image #999:
omc_999 = [ NaN ; NaN ; NaN ];
Tc_999  = [ NaN ; NaN ; NaN ];
omc_error_999 = [ NaN ; NaN ; NaN ];
Tc_error_999  = [ NaN ; NaN ; NaN ];

%-- Image #1000:
omc_1000 = [ NaN ; NaN ; NaN ];
Tc_1000  = [ NaN ; NaN ; NaN ];
omc_error_1000 = [ NaN ; NaN ; NaN ];
Tc_error_1000  = [ NaN ; NaN ; NaN ];

%-- Image #1001:
omc_1001 = [ NaN ; NaN ; NaN ];
Tc_1001  = [ NaN ; NaN ; NaN ];
omc_error_1001 = [ NaN ; NaN ; NaN ];
Tc_error_1001  = [ NaN ; NaN ; NaN ];

%-- Image #1002:
omc_1002 = [ NaN ; NaN ; NaN ];
Tc_1002  = [ NaN ; NaN ; NaN ];
omc_error_1002 = [ NaN ; NaN ; NaN ];
Tc_error_1002  = [ NaN ; NaN ; NaN ];

%-- Image #1003:
omc_1003 = [ NaN ; NaN ; NaN ];
Tc_1003  = [ NaN ; NaN ; NaN ];
omc_error_1003 = [ NaN ; NaN ; NaN ];
Tc_error_1003  = [ NaN ; NaN ; NaN ];

%-- Image #1004:
omc_1004 = [ NaN ; NaN ; NaN ];
Tc_1004  = [ NaN ; NaN ; NaN ];
omc_error_1004 = [ NaN ; NaN ; NaN ];
Tc_error_1004  = [ NaN ; NaN ; NaN ];

%-- Image #1005:
omc_1005 = [ NaN ; NaN ; NaN ];
Tc_1005  = [ NaN ; NaN ; NaN ];
omc_error_1005 = [ NaN ; NaN ; NaN ];
Tc_error_1005  = [ NaN ; NaN ; NaN ];

%-- Image #1006:
omc_1006 = [ NaN ; NaN ; NaN ];
Tc_1006  = [ NaN ; NaN ; NaN ];
omc_error_1006 = [ NaN ; NaN ; NaN ];
Tc_error_1006  = [ NaN ; NaN ; NaN ];

%-- Image #1007:
omc_1007 = [ NaN ; NaN ; NaN ];
Tc_1007  = [ NaN ; NaN ; NaN ];
omc_error_1007 = [ NaN ; NaN ; NaN ];
Tc_error_1007  = [ NaN ; NaN ; NaN ];

%-- Image #1008:
omc_1008 = [ NaN ; NaN ; NaN ];
Tc_1008  = [ NaN ; NaN ; NaN ];
omc_error_1008 = [ NaN ; NaN ; NaN ];
Tc_error_1008  = [ NaN ; NaN ; NaN ];

%-- Image #1009:
omc_1009 = [ NaN ; NaN ; NaN ];
Tc_1009  = [ NaN ; NaN ; NaN ];
omc_error_1009 = [ NaN ; NaN ; NaN ];
Tc_error_1009  = [ NaN ; NaN ; NaN ];

%-- Image #1010:
omc_1010 = [ NaN ; NaN ; NaN ];
Tc_1010  = [ NaN ; NaN ; NaN ];
omc_error_1010 = [ NaN ; NaN ; NaN ];
Tc_error_1010  = [ NaN ; NaN ; NaN ];

%-- Image #1011:
omc_1011 = [ NaN ; NaN ; NaN ];
Tc_1011  = [ NaN ; NaN ; NaN ];
omc_error_1011 = [ NaN ; NaN ; NaN ];
Tc_error_1011  = [ NaN ; NaN ; NaN ];

%-- Image #1012:
omc_1012 = [ NaN ; NaN ; NaN ];
Tc_1012  = [ NaN ; NaN ; NaN ];
omc_error_1012 = [ NaN ; NaN ; NaN ];
Tc_error_1012  = [ NaN ; NaN ; NaN ];

%-- Image #1013:
omc_1013 = [ NaN ; NaN ; NaN ];
Tc_1013  = [ NaN ; NaN ; NaN ];
omc_error_1013 = [ NaN ; NaN ; NaN ];
Tc_error_1013  = [ NaN ; NaN ; NaN ];

%-- Image #1014:
omc_1014 = [ NaN ; NaN ; NaN ];
Tc_1014  = [ NaN ; NaN ; NaN ];
omc_error_1014 = [ NaN ; NaN ; NaN ];
Tc_error_1014  = [ NaN ; NaN ; NaN ];

%-- Image #1015:
omc_1015 = [ NaN ; NaN ; NaN ];
Tc_1015  = [ NaN ; NaN ; NaN ];
omc_error_1015 = [ NaN ; NaN ; NaN ];
Tc_error_1015  = [ NaN ; NaN ; NaN ];

%-- Image #1016:
omc_1016 = [ NaN ; NaN ; NaN ];
Tc_1016  = [ NaN ; NaN ; NaN ];
omc_error_1016 = [ NaN ; NaN ; NaN ];
Tc_error_1016  = [ NaN ; NaN ; NaN ];

%-- Image #1017:
omc_1017 = [ NaN ; NaN ; NaN ];
Tc_1017  = [ NaN ; NaN ; NaN ];
omc_error_1017 = [ NaN ; NaN ; NaN ];
Tc_error_1017  = [ NaN ; NaN ; NaN ];

%-- Image #1018:
omc_1018 = [ NaN ; NaN ; NaN ];
Tc_1018  = [ NaN ; NaN ; NaN ];
omc_error_1018 = [ NaN ; NaN ; NaN ];
Tc_error_1018  = [ NaN ; NaN ; NaN ];

%-- Image #1019:
omc_1019 = [ NaN ; NaN ; NaN ];
Tc_1019  = [ NaN ; NaN ; NaN ];
omc_error_1019 = [ NaN ; NaN ; NaN ];
Tc_error_1019  = [ NaN ; NaN ; NaN ];

%-- Image #1020:
omc_1020 = [ NaN ; NaN ; NaN ];
Tc_1020  = [ NaN ; NaN ; NaN ];
omc_error_1020 = [ NaN ; NaN ; NaN ];
Tc_error_1020  = [ NaN ; NaN ; NaN ];

%-- Image #1021:
omc_1021 = [ NaN ; NaN ; NaN ];
Tc_1021  = [ NaN ; NaN ; NaN ];
omc_error_1021 = [ NaN ; NaN ; NaN ];
Tc_error_1021  = [ NaN ; NaN ; NaN ];

%-- Image #1022:
omc_1022 = [ NaN ; NaN ; NaN ];
Tc_1022  = [ NaN ; NaN ; NaN ];
omc_error_1022 = [ NaN ; NaN ; NaN ];
Tc_error_1022  = [ NaN ; NaN ; NaN ];

%-- Image #1023:
omc_1023 = [ NaN ; NaN ; NaN ];
Tc_1023  = [ NaN ; NaN ; NaN ];
omc_error_1023 = [ NaN ; NaN ; NaN ];
Tc_error_1023  = [ NaN ; NaN ; NaN ];

%-- Image #1024:
omc_1024 = [ NaN ; NaN ; NaN ];
Tc_1024  = [ NaN ; NaN ; NaN ];
omc_error_1024 = [ NaN ; NaN ; NaN ];
Tc_error_1024  = [ NaN ; NaN ; NaN ];

%-- Image #1025:
omc_1025 = [ NaN ; NaN ; NaN ];
Tc_1025  = [ NaN ; NaN ; NaN ];
omc_error_1025 = [ NaN ; NaN ; NaN ];
Tc_error_1025  = [ NaN ; NaN ; NaN ];

%-- Image #1026:
omc_1026 = [ NaN ; NaN ; NaN ];
Tc_1026  = [ NaN ; NaN ; NaN ];
omc_error_1026 = [ NaN ; NaN ; NaN ];
Tc_error_1026  = [ NaN ; NaN ; NaN ];

%-- Image #1027:
omc_1027 = [ NaN ; NaN ; NaN ];
Tc_1027  = [ NaN ; NaN ; NaN ];
omc_error_1027 = [ NaN ; NaN ; NaN ];
Tc_error_1027  = [ NaN ; NaN ; NaN ];

%-- Image #1028:
omc_1028 = [ NaN ; NaN ; NaN ];
Tc_1028  = [ NaN ; NaN ; NaN ];
omc_error_1028 = [ NaN ; NaN ; NaN ];
Tc_error_1028  = [ NaN ; NaN ; NaN ];

%-- Image #1029:
omc_1029 = [ NaN ; NaN ; NaN ];
Tc_1029  = [ NaN ; NaN ; NaN ];
omc_error_1029 = [ NaN ; NaN ; NaN ];
Tc_error_1029  = [ NaN ; NaN ; NaN ];

%-- Image #1030:
omc_1030 = [ NaN ; NaN ; NaN ];
Tc_1030  = [ NaN ; NaN ; NaN ];
omc_error_1030 = [ NaN ; NaN ; NaN ];
Tc_error_1030  = [ NaN ; NaN ; NaN ];

%-- Image #1031:
omc_1031 = [ NaN ; NaN ; NaN ];
Tc_1031  = [ NaN ; NaN ; NaN ];
omc_error_1031 = [ NaN ; NaN ; NaN ];
Tc_error_1031  = [ NaN ; NaN ; NaN ];

%-- Image #1032:
omc_1032 = [ NaN ; NaN ; NaN ];
Tc_1032  = [ NaN ; NaN ; NaN ];
omc_error_1032 = [ NaN ; NaN ; NaN ];
Tc_error_1032  = [ NaN ; NaN ; NaN ];

%-- Image #1033:
omc_1033 = [ NaN ; NaN ; NaN ];
Tc_1033  = [ NaN ; NaN ; NaN ];
omc_error_1033 = [ NaN ; NaN ; NaN ];
Tc_error_1033  = [ NaN ; NaN ; NaN ];

%-- Image #1034:
omc_1034 = [ NaN ; NaN ; NaN ];
Tc_1034  = [ NaN ; NaN ; NaN ];
omc_error_1034 = [ NaN ; NaN ; NaN ];
Tc_error_1034  = [ NaN ; NaN ; NaN ];

%-- Image #1035:
omc_1035 = [ NaN ; NaN ; NaN ];
Tc_1035  = [ NaN ; NaN ; NaN ];
omc_error_1035 = [ NaN ; NaN ; NaN ];
Tc_error_1035  = [ NaN ; NaN ; NaN ];

%-- Image #1036:
omc_1036 = [ NaN ; NaN ; NaN ];
Tc_1036  = [ NaN ; NaN ; NaN ];
omc_error_1036 = [ NaN ; NaN ; NaN ];
Tc_error_1036  = [ NaN ; NaN ; NaN ];

%-- Image #1037:
omc_1037 = [ NaN ; NaN ; NaN ];
Tc_1037  = [ NaN ; NaN ; NaN ];
omc_error_1037 = [ NaN ; NaN ; NaN ];
Tc_error_1037  = [ NaN ; NaN ; NaN ];

%-- Image #1038:
omc_1038 = [ NaN ; NaN ; NaN ];
Tc_1038  = [ NaN ; NaN ; NaN ];
omc_error_1038 = [ NaN ; NaN ; NaN ];
Tc_error_1038  = [ NaN ; NaN ; NaN ];

%-- Image #1039:
omc_1039 = [ NaN ; NaN ; NaN ];
Tc_1039  = [ NaN ; NaN ; NaN ];
omc_error_1039 = [ NaN ; NaN ; NaN ];
Tc_error_1039  = [ NaN ; NaN ; NaN ];

%-- Image #1040:
omc_1040 = [ NaN ; NaN ; NaN ];
Tc_1040  = [ NaN ; NaN ; NaN ];
omc_error_1040 = [ NaN ; NaN ; NaN ];
Tc_error_1040  = [ NaN ; NaN ; NaN ];

%-- Image #1041:
omc_1041 = [ NaN ; NaN ; NaN ];
Tc_1041  = [ NaN ; NaN ; NaN ];
omc_error_1041 = [ NaN ; NaN ; NaN ];
Tc_error_1041  = [ NaN ; NaN ; NaN ];

%-- Image #1042:
omc_1042 = [ NaN ; NaN ; NaN ];
Tc_1042  = [ NaN ; NaN ; NaN ];
omc_error_1042 = [ NaN ; NaN ; NaN ];
Tc_error_1042  = [ NaN ; NaN ; NaN ];

%-- Image #1043:
omc_1043 = [ NaN ; NaN ; NaN ];
Tc_1043  = [ NaN ; NaN ; NaN ];
omc_error_1043 = [ NaN ; NaN ; NaN ];
Tc_error_1043  = [ NaN ; NaN ; NaN ];

%-- Image #1044:
omc_1044 = [ NaN ; NaN ; NaN ];
Tc_1044  = [ NaN ; NaN ; NaN ];
omc_error_1044 = [ NaN ; NaN ; NaN ];
Tc_error_1044  = [ NaN ; NaN ; NaN ];

%-- Image #1045:
omc_1045 = [ NaN ; NaN ; NaN ];
Tc_1045  = [ NaN ; NaN ; NaN ];
omc_error_1045 = [ NaN ; NaN ; NaN ];
Tc_error_1045  = [ NaN ; NaN ; NaN ];

%-- Image #1046:
omc_1046 = [ NaN ; NaN ; NaN ];
Tc_1046  = [ NaN ; NaN ; NaN ];
omc_error_1046 = [ NaN ; NaN ; NaN ];
Tc_error_1046  = [ NaN ; NaN ; NaN ];

%-- Image #1047:
omc_1047 = [ NaN ; NaN ; NaN ];
Tc_1047  = [ NaN ; NaN ; NaN ];
omc_error_1047 = [ NaN ; NaN ; NaN ];
Tc_error_1047  = [ NaN ; NaN ; NaN ];

%-- Image #1048:
omc_1048 = [ NaN ; NaN ; NaN ];
Tc_1048  = [ NaN ; NaN ; NaN ];
omc_error_1048 = [ NaN ; NaN ; NaN ];
Tc_error_1048  = [ NaN ; NaN ; NaN ];

%-- Image #1049:
omc_1049 = [ NaN ; NaN ; NaN ];
Tc_1049  = [ NaN ; NaN ; NaN ];
omc_error_1049 = [ NaN ; NaN ; NaN ];
Tc_error_1049  = [ NaN ; NaN ; NaN ];

%-- Image #1050:
omc_1050 = [ NaN ; NaN ; NaN ];
Tc_1050  = [ NaN ; NaN ; NaN ];
omc_error_1050 = [ NaN ; NaN ; NaN ];
Tc_error_1050  = [ NaN ; NaN ; NaN ];

%-- Image #1051:
omc_1051 = [ NaN ; NaN ; NaN ];
Tc_1051  = [ NaN ; NaN ; NaN ];
omc_error_1051 = [ NaN ; NaN ; NaN ];
Tc_error_1051  = [ NaN ; NaN ; NaN ];

%-- Image #1052:
omc_1052 = [ NaN ; NaN ; NaN ];
Tc_1052  = [ NaN ; NaN ; NaN ];
omc_error_1052 = [ NaN ; NaN ; NaN ];
Tc_error_1052  = [ NaN ; NaN ; NaN ];

%-- Image #1053:
omc_1053 = [ NaN ; NaN ; NaN ];
Tc_1053  = [ NaN ; NaN ; NaN ];
omc_error_1053 = [ NaN ; NaN ; NaN ];
Tc_error_1053  = [ NaN ; NaN ; NaN ];

%-- Image #1054:
omc_1054 = [ NaN ; NaN ; NaN ];
Tc_1054  = [ NaN ; NaN ; NaN ];
omc_error_1054 = [ NaN ; NaN ; NaN ];
Tc_error_1054  = [ NaN ; NaN ; NaN ];

%-- Image #1055:
omc_1055 = [ NaN ; NaN ; NaN ];
Tc_1055  = [ NaN ; NaN ; NaN ];
omc_error_1055 = [ NaN ; NaN ; NaN ];
Tc_error_1055  = [ NaN ; NaN ; NaN ];

%-- Image #1056:
omc_1056 = [ NaN ; NaN ; NaN ];
Tc_1056  = [ NaN ; NaN ; NaN ];
omc_error_1056 = [ NaN ; NaN ; NaN ];
Tc_error_1056  = [ NaN ; NaN ; NaN ];

%-- Image #1057:
omc_1057 = [ NaN ; NaN ; NaN ];
Tc_1057  = [ NaN ; NaN ; NaN ];
omc_error_1057 = [ NaN ; NaN ; NaN ];
Tc_error_1057  = [ NaN ; NaN ; NaN ];

%-- Image #1058:
omc_1058 = [ NaN ; NaN ; NaN ];
Tc_1058  = [ NaN ; NaN ; NaN ];
omc_error_1058 = [ NaN ; NaN ; NaN ];
Tc_error_1058  = [ NaN ; NaN ; NaN ];

%-- Image #1059:
omc_1059 = [ NaN ; NaN ; NaN ];
Tc_1059  = [ NaN ; NaN ; NaN ];
omc_error_1059 = [ NaN ; NaN ; NaN ];
Tc_error_1059  = [ NaN ; NaN ; NaN ];

%-- Image #1060:
omc_1060 = [ NaN ; NaN ; NaN ];
Tc_1060  = [ NaN ; NaN ; NaN ];
omc_error_1060 = [ NaN ; NaN ; NaN ];
Tc_error_1060  = [ NaN ; NaN ; NaN ];

%-- Image #1061:
omc_1061 = [ NaN ; NaN ; NaN ];
Tc_1061  = [ NaN ; NaN ; NaN ];
omc_error_1061 = [ NaN ; NaN ; NaN ];
Tc_error_1061  = [ NaN ; NaN ; NaN ];

%-- Image #1062:
omc_1062 = [ NaN ; NaN ; NaN ];
Tc_1062  = [ NaN ; NaN ; NaN ];
omc_error_1062 = [ NaN ; NaN ; NaN ];
Tc_error_1062  = [ NaN ; NaN ; NaN ];

%-- Image #1063:
omc_1063 = [ NaN ; NaN ; NaN ];
Tc_1063  = [ NaN ; NaN ; NaN ];
omc_error_1063 = [ NaN ; NaN ; NaN ];
Tc_error_1063  = [ NaN ; NaN ; NaN ];

%-- Image #1064:
omc_1064 = [ NaN ; NaN ; NaN ];
Tc_1064  = [ NaN ; NaN ; NaN ];
omc_error_1064 = [ NaN ; NaN ; NaN ];
Tc_error_1064  = [ NaN ; NaN ; NaN ];

%-- Image #1065:
omc_1065 = [ NaN ; NaN ; NaN ];
Tc_1065  = [ NaN ; NaN ; NaN ];
omc_error_1065 = [ NaN ; NaN ; NaN ];
Tc_error_1065  = [ NaN ; NaN ; NaN ];

%-- Image #1066:
omc_1066 = [ NaN ; NaN ; NaN ];
Tc_1066  = [ NaN ; NaN ; NaN ];
omc_error_1066 = [ NaN ; NaN ; NaN ];
Tc_error_1066  = [ NaN ; NaN ; NaN ];

%-- Image #1067:
omc_1067 = [ NaN ; NaN ; NaN ];
Tc_1067  = [ NaN ; NaN ; NaN ];
omc_error_1067 = [ NaN ; NaN ; NaN ];
Tc_error_1067  = [ NaN ; NaN ; NaN ];

%-- Image #1068:
omc_1068 = [ NaN ; NaN ; NaN ];
Tc_1068  = [ NaN ; NaN ; NaN ];
omc_error_1068 = [ NaN ; NaN ; NaN ];
Tc_error_1068  = [ NaN ; NaN ; NaN ];

%-- Image #1069:
omc_1069 = [ NaN ; NaN ; NaN ];
Tc_1069  = [ NaN ; NaN ; NaN ];
omc_error_1069 = [ NaN ; NaN ; NaN ];
Tc_error_1069  = [ NaN ; NaN ; NaN ];

%-- Image #1070:
omc_1070 = [ NaN ; NaN ; NaN ];
Tc_1070  = [ NaN ; NaN ; NaN ];
omc_error_1070 = [ NaN ; NaN ; NaN ];
Tc_error_1070  = [ NaN ; NaN ; NaN ];

%-- Image #1071:
omc_1071 = [ NaN ; NaN ; NaN ];
Tc_1071  = [ NaN ; NaN ; NaN ];
omc_error_1071 = [ NaN ; NaN ; NaN ];
Tc_error_1071  = [ NaN ; NaN ; NaN ];

%-- Image #1072:
omc_1072 = [ NaN ; NaN ; NaN ];
Tc_1072  = [ NaN ; NaN ; NaN ];
omc_error_1072 = [ NaN ; NaN ; NaN ];
Tc_error_1072  = [ NaN ; NaN ; NaN ];

%-- Image #1073:
omc_1073 = [ NaN ; NaN ; NaN ];
Tc_1073  = [ NaN ; NaN ; NaN ];
omc_error_1073 = [ NaN ; NaN ; NaN ];
Tc_error_1073  = [ NaN ; NaN ; NaN ];

%-- Image #1074:
omc_1074 = [ NaN ; NaN ; NaN ];
Tc_1074  = [ NaN ; NaN ; NaN ];
omc_error_1074 = [ NaN ; NaN ; NaN ];
Tc_error_1074  = [ NaN ; NaN ; NaN ];

%-- Image #1075:
omc_1075 = [ NaN ; NaN ; NaN ];
Tc_1075  = [ NaN ; NaN ; NaN ];
omc_error_1075 = [ NaN ; NaN ; NaN ];
Tc_error_1075  = [ NaN ; NaN ; NaN ];

%-- Image #1076:
omc_1076 = [ NaN ; NaN ; NaN ];
Tc_1076  = [ NaN ; NaN ; NaN ];
omc_error_1076 = [ NaN ; NaN ; NaN ];
Tc_error_1076  = [ NaN ; NaN ; NaN ];

%-- Image #1077:
omc_1077 = [ NaN ; NaN ; NaN ];
Tc_1077  = [ NaN ; NaN ; NaN ];
omc_error_1077 = [ NaN ; NaN ; NaN ];
Tc_error_1077  = [ NaN ; NaN ; NaN ];

%-- Image #1078:
omc_1078 = [ NaN ; NaN ; NaN ];
Tc_1078  = [ NaN ; NaN ; NaN ];
omc_error_1078 = [ NaN ; NaN ; NaN ];
Tc_error_1078  = [ NaN ; NaN ; NaN ];

%-- Image #1079:
omc_1079 = [ NaN ; NaN ; NaN ];
Tc_1079  = [ NaN ; NaN ; NaN ];
omc_error_1079 = [ NaN ; NaN ; NaN ];
Tc_error_1079  = [ NaN ; NaN ; NaN ];

%-- Image #1080:
omc_1080 = [ NaN ; NaN ; NaN ];
Tc_1080  = [ NaN ; NaN ; NaN ];
omc_error_1080 = [ NaN ; NaN ; NaN ];
Tc_error_1080  = [ NaN ; NaN ; NaN ];

%-- Image #1081:
omc_1081 = [ NaN ; NaN ; NaN ];
Tc_1081  = [ NaN ; NaN ; NaN ];
omc_error_1081 = [ NaN ; NaN ; NaN ];
Tc_error_1081  = [ NaN ; NaN ; NaN ];

%-- Image #1082:
omc_1082 = [ NaN ; NaN ; NaN ];
Tc_1082  = [ NaN ; NaN ; NaN ];
omc_error_1082 = [ NaN ; NaN ; NaN ];
Tc_error_1082  = [ NaN ; NaN ; NaN ];

%-- Image #1083:
omc_1083 = [ NaN ; NaN ; NaN ];
Tc_1083  = [ NaN ; NaN ; NaN ];
omc_error_1083 = [ NaN ; NaN ; NaN ];
Tc_error_1083  = [ NaN ; NaN ; NaN ];

%-- Image #1084:
omc_1084 = [ NaN ; NaN ; NaN ];
Tc_1084  = [ NaN ; NaN ; NaN ];
omc_error_1084 = [ NaN ; NaN ; NaN ];
Tc_error_1084  = [ NaN ; NaN ; NaN ];

%-- Image #1085:
omc_1085 = [ NaN ; NaN ; NaN ];
Tc_1085  = [ NaN ; NaN ; NaN ];
omc_error_1085 = [ NaN ; NaN ; NaN ];
Tc_error_1085  = [ NaN ; NaN ; NaN ];

%-- Image #1086:
omc_1086 = [ NaN ; NaN ; NaN ];
Tc_1086  = [ NaN ; NaN ; NaN ];
omc_error_1086 = [ NaN ; NaN ; NaN ];
Tc_error_1086  = [ NaN ; NaN ; NaN ];

%-- Image #1087:
omc_1087 = [ NaN ; NaN ; NaN ];
Tc_1087  = [ NaN ; NaN ; NaN ];
omc_error_1087 = [ NaN ; NaN ; NaN ];
Tc_error_1087  = [ NaN ; NaN ; NaN ];

%-- Image #1088:
omc_1088 = [ NaN ; NaN ; NaN ];
Tc_1088  = [ NaN ; NaN ; NaN ];
omc_error_1088 = [ NaN ; NaN ; NaN ];
Tc_error_1088  = [ NaN ; NaN ; NaN ];

%-- Image #1089:
omc_1089 = [ NaN ; NaN ; NaN ];
Tc_1089  = [ NaN ; NaN ; NaN ];
omc_error_1089 = [ NaN ; NaN ; NaN ];
Tc_error_1089  = [ NaN ; NaN ; NaN ];

%-- Image #1090:
omc_1090 = [ NaN ; NaN ; NaN ];
Tc_1090  = [ NaN ; NaN ; NaN ];
omc_error_1090 = [ NaN ; NaN ; NaN ];
Tc_error_1090  = [ NaN ; NaN ; NaN ];

%-- Image #1091:
omc_1091 = [ NaN ; NaN ; NaN ];
Tc_1091  = [ NaN ; NaN ; NaN ];
omc_error_1091 = [ NaN ; NaN ; NaN ];
Tc_error_1091  = [ NaN ; NaN ; NaN ];

%-- Image #1092:
omc_1092 = [ NaN ; NaN ; NaN ];
Tc_1092  = [ NaN ; NaN ; NaN ];
omc_error_1092 = [ NaN ; NaN ; NaN ];
Tc_error_1092  = [ NaN ; NaN ; NaN ];

%-- Image #1093:
omc_1093 = [ NaN ; NaN ; NaN ];
Tc_1093  = [ NaN ; NaN ; NaN ];
omc_error_1093 = [ NaN ; NaN ; NaN ];
Tc_error_1093  = [ NaN ; NaN ; NaN ];

%-- Image #1094:
omc_1094 = [ NaN ; NaN ; NaN ];
Tc_1094  = [ NaN ; NaN ; NaN ];
omc_error_1094 = [ NaN ; NaN ; NaN ];
Tc_error_1094  = [ NaN ; NaN ; NaN ];

%-- Image #1095:
omc_1095 = [ NaN ; NaN ; NaN ];
Tc_1095  = [ NaN ; NaN ; NaN ];
omc_error_1095 = [ NaN ; NaN ; NaN ];
Tc_error_1095  = [ NaN ; NaN ; NaN ];

%-- Image #1096:
omc_1096 = [ NaN ; NaN ; NaN ];
Tc_1096  = [ NaN ; NaN ; NaN ];
omc_error_1096 = [ NaN ; NaN ; NaN ];
Tc_error_1096  = [ NaN ; NaN ; NaN ];

%-- Image #1097:
omc_1097 = [ NaN ; NaN ; NaN ];
Tc_1097  = [ NaN ; NaN ; NaN ];
omc_error_1097 = [ NaN ; NaN ; NaN ];
Tc_error_1097  = [ NaN ; NaN ; NaN ];

%-- Image #1098:
omc_1098 = [ NaN ; NaN ; NaN ];
Tc_1098  = [ NaN ; NaN ; NaN ];
omc_error_1098 = [ NaN ; NaN ; NaN ];
Tc_error_1098  = [ NaN ; NaN ; NaN ];

%-- Image #1099:
omc_1099 = [ NaN ; NaN ; NaN ];
Tc_1099  = [ NaN ; NaN ; NaN ];
omc_error_1099 = [ NaN ; NaN ; NaN ];
Tc_error_1099  = [ NaN ; NaN ; NaN ];

%-- Image #1100:
omc_1100 = [ NaN ; NaN ; NaN ];
Tc_1100  = [ NaN ; NaN ; NaN ];
omc_error_1100 = [ NaN ; NaN ; NaN ];
Tc_error_1100  = [ NaN ; NaN ; NaN ];

%-- Image #1101:
omc_1101 = [ NaN ; NaN ; NaN ];
Tc_1101  = [ NaN ; NaN ; NaN ];
omc_error_1101 = [ NaN ; NaN ; NaN ];
Tc_error_1101  = [ NaN ; NaN ; NaN ];

%-- Image #1102:
omc_1102 = [ NaN ; NaN ; NaN ];
Tc_1102  = [ NaN ; NaN ; NaN ];
omc_error_1102 = [ NaN ; NaN ; NaN ];
Tc_error_1102  = [ NaN ; NaN ; NaN ];

%-- Image #1103:
omc_1103 = [ NaN ; NaN ; NaN ];
Tc_1103  = [ NaN ; NaN ; NaN ];
omc_error_1103 = [ NaN ; NaN ; NaN ];
Tc_error_1103  = [ NaN ; NaN ; NaN ];

%-- Image #1104:
omc_1104 = [ NaN ; NaN ; NaN ];
Tc_1104  = [ NaN ; NaN ; NaN ];
omc_error_1104 = [ NaN ; NaN ; NaN ];
Tc_error_1104  = [ NaN ; NaN ; NaN ];

%-- Image #1105:
omc_1105 = [ -1.695220e+000 ; -1.778526e+000 ; 4.214674e-001 ];
Tc_1105  = [ -3.879681e+001 ; -1.050011e+002 ; 3.354110e+002 ];
omc_error_1105 = [ 1.484503e-002 ; 1.528368e-002 ; 2.436318e-002 ];
Tc_error_1105  = [ 4.466805e+000 ; 4.564313e+000 ; 5.230852e+000 ];

%-- Image #1106:
omc_1106 = [ NaN ; NaN ; NaN ];
Tc_1106  = [ NaN ; NaN ; NaN ];
omc_error_1106 = [ NaN ; NaN ; NaN ];
Tc_error_1106  = [ NaN ; NaN ; NaN ];

%-- Image #1107:
omc_1107 = [ NaN ; NaN ; NaN ];
Tc_1107  = [ NaN ; NaN ; NaN ];
omc_error_1107 = [ NaN ; NaN ; NaN ];
Tc_error_1107  = [ NaN ; NaN ; NaN ];

%-- Image #1108:
omc_1108 = [ NaN ; NaN ; NaN ];
Tc_1108  = [ NaN ; NaN ; NaN ];
omc_error_1108 = [ NaN ; NaN ; NaN ];
Tc_error_1108  = [ NaN ; NaN ; NaN ];

%-- Image #1109:
omc_1109 = [ NaN ; NaN ; NaN ];
Tc_1109  = [ NaN ; NaN ; NaN ];
omc_error_1109 = [ NaN ; NaN ; NaN ];
Tc_error_1109  = [ NaN ; NaN ; NaN ];

%-- Image #1110:
omc_1110 = [ NaN ; NaN ; NaN ];
Tc_1110  = [ NaN ; NaN ; NaN ];
omc_error_1110 = [ NaN ; NaN ; NaN ];
Tc_error_1110  = [ NaN ; NaN ; NaN ];

%-- Image #1111:
omc_1111 = [ NaN ; NaN ; NaN ];
Tc_1111  = [ NaN ; NaN ; NaN ];
omc_error_1111 = [ NaN ; NaN ; NaN ];
Tc_error_1111  = [ NaN ; NaN ; NaN ];

%-- Image #1112:
omc_1112 = [ NaN ; NaN ; NaN ];
Tc_1112  = [ NaN ; NaN ; NaN ];
omc_error_1112 = [ NaN ; NaN ; NaN ];
Tc_error_1112  = [ NaN ; NaN ; NaN ];

%-- Image #1113:
omc_1113 = [ NaN ; NaN ; NaN ];
Tc_1113  = [ NaN ; NaN ; NaN ];
omc_error_1113 = [ NaN ; NaN ; NaN ];
Tc_error_1113  = [ NaN ; NaN ; NaN ];

%-- Image #1114:
omc_1114 = [ NaN ; NaN ; NaN ];
Tc_1114  = [ NaN ; NaN ; NaN ];
omc_error_1114 = [ NaN ; NaN ; NaN ];
Tc_error_1114  = [ NaN ; NaN ; NaN ];

%-- Image #1115:
omc_1115 = [ NaN ; NaN ; NaN ];
Tc_1115  = [ NaN ; NaN ; NaN ];
omc_error_1115 = [ NaN ; NaN ; NaN ];
Tc_error_1115  = [ NaN ; NaN ; NaN ];

%-- Image #1116:
omc_1116 = [ NaN ; NaN ; NaN ];
Tc_1116  = [ NaN ; NaN ; NaN ];
omc_error_1116 = [ NaN ; NaN ; NaN ];
Tc_error_1116  = [ NaN ; NaN ; NaN ];

%-- Image #1117:
omc_1117 = [ NaN ; NaN ; NaN ];
Tc_1117  = [ NaN ; NaN ; NaN ];
omc_error_1117 = [ NaN ; NaN ; NaN ];
Tc_error_1117  = [ NaN ; NaN ; NaN ];

%-- Image #1118:
omc_1118 = [ NaN ; NaN ; NaN ];
Tc_1118  = [ NaN ; NaN ; NaN ];
omc_error_1118 = [ NaN ; NaN ; NaN ];
Tc_error_1118  = [ NaN ; NaN ; NaN ];

%-- Image #1119:
omc_1119 = [ NaN ; NaN ; NaN ];
Tc_1119  = [ NaN ; NaN ; NaN ];
omc_error_1119 = [ NaN ; NaN ; NaN ];
Tc_error_1119  = [ NaN ; NaN ; NaN ];

%-- Image #1120:
omc_1120 = [ NaN ; NaN ; NaN ];
Tc_1120  = [ NaN ; NaN ; NaN ];
omc_error_1120 = [ NaN ; NaN ; NaN ];
Tc_error_1120  = [ NaN ; NaN ; NaN ];

%-- Image #1121:
omc_1121 = [ NaN ; NaN ; NaN ];
Tc_1121  = [ NaN ; NaN ; NaN ];
omc_error_1121 = [ NaN ; NaN ; NaN ];
Tc_error_1121  = [ NaN ; NaN ; NaN ];

%-- Image #1122:
omc_1122 = [ NaN ; NaN ; NaN ];
Tc_1122  = [ NaN ; NaN ; NaN ];
omc_error_1122 = [ NaN ; NaN ; NaN ];
Tc_error_1122  = [ NaN ; NaN ; NaN ];

%-- Image #1123:
omc_1123 = [ NaN ; NaN ; NaN ];
Tc_1123  = [ NaN ; NaN ; NaN ];
omc_error_1123 = [ NaN ; NaN ; NaN ];
Tc_error_1123  = [ NaN ; NaN ; NaN ];

%-- Image #1124:
omc_1124 = [ NaN ; NaN ; NaN ];
Tc_1124  = [ NaN ; NaN ; NaN ];
omc_error_1124 = [ NaN ; NaN ; NaN ];
Tc_error_1124  = [ NaN ; NaN ; NaN ];

%-- Image #1125:
omc_1125 = [ NaN ; NaN ; NaN ];
Tc_1125  = [ NaN ; NaN ; NaN ];
omc_error_1125 = [ NaN ; NaN ; NaN ];
Tc_error_1125  = [ NaN ; NaN ; NaN ];

%-- Image #1126:
omc_1126 = [ NaN ; NaN ; NaN ];
Tc_1126  = [ NaN ; NaN ; NaN ];
omc_error_1126 = [ NaN ; NaN ; NaN ];
Tc_error_1126  = [ NaN ; NaN ; NaN ];

%-- Image #1127:
omc_1127 = [ NaN ; NaN ; NaN ];
Tc_1127  = [ NaN ; NaN ; NaN ];
omc_error_1127 = [ NaN ; NaN ; NaN ];
Tc_error_1127  = [ NaN ; NaN ; NaN ];

%-- Image #1128:
omc_1128 = [ NaN ; NaN ; NaN ];
Tc_1128  = [ NaN ; NaN ; NaN ];
omc_error_1128 = [ NaN ; NaN ; NaN ];
Tc_error_1128  = [ NaN ; NaN ; NaN ];

%-- Image #1129:
omc_1129 = [ NaN ; NaN ; NaN ];
Tc_1129  = [ NaN ; NaN ; NaN ];
omc_error_1129 = [ NaN ; NaN ; NaN ];
Tc_error_1129  = [ NaN ; NaN ; NaN ];

%-- Image #1130:
omc_1130 = [ NaN ; NaN ; NaN ];
Tc_1130  = [ NaN ; NaN ; NaN ];
omc_error_1130 = [ NaN ; NaN ; NaN ];
Tc_error_1130  = [ NaN ; NaN ; NaN ];

%-- Image #1131:
omc_1131 = [ NaN ; NaN ; NaN ];
Tc_1131  = [ NaN ; NaN ; NaN ];
omc_error_1131 = [ NaN ; NaN ; NaN ];
Tc_error_1131  = [ NaN ; NaN ; NaN ];

%-- Image #1132:
omc_1132 = [ NaN ; NaN ; NaN ];
Tc_1132  = [ NaN ; NaN ; NaN ];
omc_error_1132 = [ NaN ; NaN ; NaN ];
Tc_error_1132  = [ NaN ; NaN ; NaN ];

%-- Image #1133:
omc_1133 = [ NaN ; NaN ; NaN ];
Tc_1133  = [ NaN ; NaN ; NaN ];
omc_error_1133 = [ NaN ; NaN ; NaN ];
Tc_error_1133  = [ NaN ; NaN ; NaN ];

%-- Image #1134:
omc_1134 = [ NaN ; NaN ; NaN ];
Tc_1134  = [ NaN ; NaN ; NaN ];
omc_error_1134 = [ NaN ; NaN ; NaN ];
Tc_error_1134  = [ NaN ; NaN ; NaN ];

%-- Image #1135:
omc_1135 = [ NaN ; NaN ; NaN ];
Tc_1135  = [ NaN ; NaN ; NaN ];
omc_error_1135 = [ NaN ; NaN ; NaN ];
Tc_error_1135  = [ NaN ; NaN ; NaN ];

%-- Image #1136:
omc_1136 = [ NaN ; NaN ; NaN ];
Tc_1136  = [ NaN ; NaN ; NaN ];
omc_error_1136 = [ NaN ; NaN ; NaN ];
Tc_error_1136  = [ NaN ; NaN ; NaN ];

%-- Image #1137:
omc_1137 = [ NaN ; NaN ; NaN ];
Tc_1137  = [ NaN ; NaN ; NaN ];
omc_error_1137 = [ NaN ; NaN ; NaN ];
Tc_error_1137  = [ NaN ; NaN ; NaN ];

%-- Image #1138:
omc_1138 = [ NaN ; NaN ; NaN ];
Tc_1138  = [ NaN ; NaN ; NaN ];
omc_error_1138 = [ NaN ; NaN ; NaN ];
Tc_error_1138  = [ NaN ; NaN ; NaN ];

%-- Image #1139:
omc_1139 = [ NaN ; NaN ; NaN ];
Tc_1139  = [ NaN ; NaN ; NaN ];
omc_error_1139 = [ NaN ; NaN ; NaN ];
Tc_error_1139  = [ NaN ; NaN ; NaN ];

%-- Image #1140:
omc_1140 = [ NaN ; NaN ; NaN ];
Tc_1140  = [ NaN ; NaN ; NaN ];
omc_error_1140 = [ NaN ; NaN ; NaN ];
Tc_error_1140  = [ NaN ; NaN ; NaN ];

%-- Image #1141:
omc_1141 = [ NaN ; NaN ; NaN ];
Tc_1141  = [ NaN ; NaN ; NaN ];
omc_error_1141 = [ NaN ; NaN ; NaN ];
Tc_error_1141  = [ NaN ; NaN ; NaN ];

%-- Image #1142:
omc_1142 = [ NaN ; NaN ; NaN ];
Tc_1142  = [ NaN ; NaN ; NaN ];
omc_error_1142 = [ NaN ; NaN ; NaN ];
Tc_error_1142  = [ NaN ; NaN ; NaN ];

%-- Image #1143:
omc_1143 = [ NaN ; NaN ; NaN ];
Tc_1143  = [ NaN ; NaN ; NaN ];
omc_error_1143 = [ NaN ; NaN ; NaN ];
Tc_error_1143  = [ NaN ; NaN ; NaN ];

%-- Image #1144:
omc_1144 = [ NaN ; NaN ; NaN ];
Tc_1144  = [ NaN ; NaN ; NaN ];
omc_error_1144 = [ NaN ; NaN ; NaN ];
Tc_error_1144  = [ NaN ; NaN ; NaN ];

%-- Image #1145:
omc_1145 = [ NaN ; NaN ; NaN ];
Tc_1145  = [ NaN ; NaN ; NaN ];
omc_error_1145 = [ NaN ; NaN ; NaN ];
Tc_error_1145  = [ NaN ; NaN ; NaN ];

%-- Image #1146:
omc_1146 = [ NaN ; NaN ; NaN ];
Tc_1146  = [ NaN ; NaN ; NaN ];
omc_error_1146 = [ NaN ; NaN ; NaN ];
Tc_error_1146  = [ NaN ; NaN ; NaN ];

%-- Image #1147:
omc_1147 = [ NaN ; NaN ; NaN ];
Tc_1147  = [ NaN ; NaN ; NaN ];
omc_error_1147 = [ NaN ; NaN ; NaN ];
Tc_error_1147  = [ NaN ; NaN ; NaN ];

%-- Image #1148:
omc_1148 = [ NaN ; NaN ; NaN ];
Tc_1148  = [ NaN ; NaN ; NaN ];
omc_error_1148 = [ NaN ; NaN ; NaN ];
Tc_error_1148  = [ NaN ; NaN ; NaN ];

%-- Image #1149:
omc_1149 = [ NaN ; NaN ; NaN ];
Tc_1149  = [ NaN ; NaN ; NaN ];
omc_error_1149 = [ NaN ; NaN ; NaN ];
Tc_error_1149  = [ NaN ; NaN ; NaN ];

%-- Image #1150:
omc_1150 = [ NaN ; NaN ; NaN ];
Tc_1150  = [ NaN ; NaN ; NaN ];
omc_error_1150 = [ NaN ; NaN ; NaN ];
Tc_error_1150  = [ NaN ; NaN ; NaN ];

%-- Image #1151:
omc_1151 = [ NaN ; NaN ; NaN ];
Tc_1151  = [ NaN ; NaN ; NaN ];
omc_error_1151 = [ NaN ; NaN ; NaN ];
Tc_error_1151  = [ NaN ; NaN ; NaN ];

%-- Image #1152:
omc_1152 = [ NaN ; NaN ; NaN ];
Tc_1152  = [ NaN ; NaN ; NaN ];
omc_error_1152 = [ NaN ; NaN ; NaN ];
Tc_error_1152  = [ NaN ; NaN ; NaN ];

%-- Image #1153:
omc_1153 = [ NaN ; NaN ; NaN ];
Tc_1153  = [ NaN ; NaN ; NaN ];
omc_error_1153 = [ NaN ; NaN ; NaN ];
Tc_error_1153  = [ NaN ; NaN ; NaN ];

%-- Image #1154:
omc_1154 = [ NaN ; NaN ; NaN ];
Tc_1154  = [ NaN ; NaN ; NaN ];
omc_error_1154 = [ NaN ; NaN ; NaN ];
Tc_error_1154  = [ NaN ; NaN ; NaN ];

%-- Image #1155:
omc_1155 = [ NaN ; NaN ; NaN ];
Tc_1155  = [ NaN ; NaN ; NaN ];
omc_error_1155 = [ NaN ; NaN ; NaN ];
Tc_error_1155  = [ NaN ; NaN ; NaN ];

%-- Image #1156:
omc_1156 = [ NaN ; NaN ; NaN ];
Tc_1156  = [ NaN ; NaN ; NaN ];
omc_error_1156 = [ NaN ; NaN ; NaN ];
Tc_error_1156  = [ NaN ; NaN ; NaN ];

%-- Image #1157:
omc_1157 = [ NaN ; NaN ; NaN ];
Tc_1157  = [ NaN ; NaN ; NaN ];
omc_error_1157 = [ NaN ; NaN ; NaN ];
Tc_error_1157  = [ NaN ; NaN ; NaN ];

%-- Image #1158:
omc_1158 = [ NaN ; NaN ; NaN ];
Tc_1158  = [ NaN ; NaN ; NaN ];
omc_error_1158 = [ NaN ; NaN ; NaN ];
Tc_error_1158  = [ NaN ; NaN ; NaN ];

%-- Image #1159:
omc_1159 = [ NaN ; NaN ; NaN ];
Tc_1159  = [ NaN ; NaN ; NaN ];
omc_error_1159 = [ NaN ; NaN ; NaN ];
Tc_error_1159  = [ NaN ; NaN ; NaN ];

%-- Image #1160:
omc_1160 = [ NaN ; NaN ; NaN ];
Tc_1160  = [ NaN ; NaN ; NaN ];
omc_error_1160 = [ NaN ; NaN ; NaN ];
Tc_error_1160  = [ NaN ; NaN ; NaN ];

%-- Image #1161:
omc_1161 = [ NaN ; NaN ; NaN ];
Tc_1161  = [ NaN ; NaN ; NaN ];
omc_error_1161 = [ NaN ; NaN ; NaN ];
Tc_error_1161  = [ NaN ; NaN ; NaN ];

%-- Image #1162:
omc_1162 = [ NaN ; NaN ; NaN ];
Tc_1162  = [ NaN ; NaN ; NaN ];
omc_error_1162 = [ NaN ; NaN ; NaN ];
Tc_error_1162  = [ NaN ; NaN ; NaN ];

%-- Image #1163:
omc_1163 = [ NaN ; NaN ; NaN ];
Tc_1163  = [ NaN ; NaN ; NaN ];
omc_error_1163 = [ NaN ; NaN ; NaN ];
Tc_error_1163  = [ NaN ; NaN ; NaN ];

%-- Image #1164:
omc_1164 = [ NaN ; NaN ; NaN ];
Tc_1164  = [ NaN ; NaN ; NaN ];
omc_error_1164 = [ NaN ; NaN ; NaN ];
Tc_error_1164  = [ NaN ; NaN ; NaN ];

%-- Image #1165:
omc_1165 = [ NaN ; NaN ; NaN ];
Tc_1165  = [ NaN ; NaN ; NaN ];
omc_error_1165 = [ NaN ; NaN ; NaN ];
Tc_error_1165  = [ NaN ; NaN ; NaN ];

%-- Image #1166:
omc_1166 = [ NaN ; NaN ; NaN ];
Tc_1166  = [ NaN ; NaN ; NaN ];
omc_error_1166 = [ NaN ; NaN ; NaN ];
Tc_error_1166  = [ NaN ; NaN ; NaN ];

%-- Image #1167:
omc_1167 = [ NaN ; NaN ; NaN ];
Tc_1167  = [ NaN ; NaN ; NaN ];
omc_error_1167 = [ NaN ; NaN ; NaN ];
Tc_error_1167  = [ NaN ; NaN ; NaN ];

%-- Image #1168:
omc_1168 = [ -1.886115e+000 ; -1.926277e+000 ; 1.711891e-001 ];
Tc_1168  = [ -7.199295e+001 ; -9.151633e+001 ; 2.710711e+002 ];
omc_error_1168 = [ 1.494862e-002 ; 1.567438e-002 ; 2.628556e-002 ];
Tc_error_1168  = [ 3.595156e+000 ; 3.774046e+000 ; 4.844060e+000 ];

%-- Image #1169:
omc_1169 = [ NaN ; NaN ; NaN ];
Tc_1169  = [ NaN ; NaN ; NaN ];
omc_error_1169 = [ NaN ; NaN ; NaN ];
Tc_error_1169  = [ NaN ; NaN ; NaN ];

%-- Image #1170:
omc_1170 = [ NaN ; NaN ; NaN ];
Tc_1170  = [ NaN ; NaN ; NaN ];
omc_error_1170 = [ NaN ; NaN ; NaN ];
Tc_error_1170  = [ NaN ; NaN ; NaN ];

%-- Image #1171:
omc_1171 = [ NaN ; NaN ; NaN ];
Tc_1171  = [ NaN ; NaN ; NaN ];
omc_error_1171 = [ NaN ; NaN ; NaN ];
Tc_error_1171  = [ NaN ; NaN ; NaN ];

%-- Image #1172:
omc_1172 = [ NaN ; NaN ; NaN ];
Tc_1172  = [ NaN ; NaN ; NaN ];
omc_error_1172 = [ NaN ; NaN ; NaN ];
Tc_error_1172  = [ NaN ; NaN ; NaN ];

%-- Image #1173:
omc_1173 = [ NaN ; NaN ; NaN ];
Tc_1173  = [ NaN ; NaN ; NaN ];
omc_error_1173 = [ NaN ; NaN ; NaN ];
Tc_error_1173  = [ NaN ; NaN ; NaN ];

%-- Image #1174:
omc_1174 = [ NaN ; NaN ; NaN ];
Tc_1174  = [ NaN ; NaN ; NaN ];
omc_error_1174 = [ NaN ; NaN ; NaN ];
Tc_error_1174  = [ NaN ; NaN ; NaN ];

%-- Image #1175:
omc_1175 = [ NaN ; NaN ; NaN ];
Tc_1175  = [ NaN ; NaN ; NaN ];
omc_error_1175 = [ NaN ; NaN ; NaN ];
Tc_error_1175  = [ NaN ; NaN ; NaN ];

%-- Image #1176:
omc_1176 = [ NaN ; NaN ; NaN ];
Tc_1176  = [ NaN ; NaN ; NaN ];
omc_error_1176 = [ NaN ; NaN ; NaN ];
Tc_error_1176  = [ NaN ; NaN ; NaN ];

%-- Image #1177:
omc_1177 = [ NaN ; NaN ; NaN ];
Tc_1177  = [ NaN ; NaN ; NaN ];
omc_error_1177 = [ NaN ; NaN ; NaN ];
Tc_error_1177  = [ NaN ; NaN ; NaN ];

%-- Image #1178:
omc_1178 = [ NaN ; NaN ; NaN ];
Tc_1178  = [ NaN ; NaN ; NaN ];
omc_error_1178 = [ NaN ; NaN ; NaN ];
Tc_error_1178  = [ NaN ; NaN ; NaN ];

%-- Image #1179:
omc_1179 = [ NaN ; NaN ; NaN ];
Tc_1179  = [ NaN ; NaN ; NaN ];
omc_error_1179 = [ NaN ; NaN ; NaN ];
Tc_error_1179  = [ NaN ; NaN ; NaN ];

%-- Image #1180:
omc_1180 = [ NaN ; NaN ; NaN ];
Tc_1180  = [ NaN ; NaN ; NaN ];
omc_error_1180 = [ NaN ; NaN ; NaN ];
Tc_error_1180  = [ NaN ; NaN ; NaN ];

%-- Image #1181:
omc_1181 = [ NaN ; NaN ; NaN ];
Tc_1181  = [ NaN ; NaN ; NaN ];
omc_error_1181 = [ NaN ; NaN ; NaN ];
Tc_error_1181  = [ NaN ; NaN ; NaN ];

%-- Image #1182:
omc_1182 = [ NaN ; NaN ; NaN ];
Tc_1182  = [ NaN ; NaN ; NaN ];
omc_error_1182 = [ NaN ; NaN ; NaN ];
Tc_error_1182  = [ NaN ; NaN ; NaN ];

%-- Image #1183:
omc_1183 = [ NaN ; NaN ; NaN ];
Tc_1183  = [ NaN ; NaN ; NaN ];
omc_error_1183 = [ NaN ; NaN ; NaN ];
Tc_error_1183  = [ NaN ; NaN ; NaN ];

%-- Image #1184:
omc_1184 = [ NaN ; NaN ; NaN ];
Tc_1184  = [ NaN ; NaN ; NaN ];
omc_error_1184 = [ NaN ; NaN ; NaN ];
Tc_error_1184  = [ NaN ; NaN ; NaN ];

%-- Image #1185:
omc_1185 = [ NaN ; NaN ; NaN ];
Tc_1185  = [ NaN ; NaN ; NaN ];
omc_error_1185 = [ NaN ; NaN ; NaN ];
Tc_error_1185  = [ NaN ; NaN ; NaN ];

%-- Image #1186:
omc_1186 = [ NaN ; NaN ; NaN ];
Tc_1186  = [ NaN ; NaN ; NaN ];
omc_error_1186 = [ NaN ; NaN ; NaN ];
Tc_error_1186  = [ NaN ; NaN ; NaN ];

%-- Image #1187:
omc_1187 = [ NaN ; NaN ; NaN ];
Tc_1187  = [ NaN ; NaN ; NaN ];
omc_error_1187 = [ NaN ; NaN ; NaN ];
Tc_error_1187  = [ NaN ; NaN ; NaN ];

%-- Image #1188:
omc_1188 = [ NaN ; NaN ; NaN ];
Tc_1188  = [ NaN ; NaN ; NaN ];
omc_error_1188 = [ NaN ; NaN ; NaN ];
Tc_error_1188  = [ NaN ; NaN ; NaN ];

%-- Image #1189:
omc_1189 = [ NaN ; NaN ; NaN ];
Tc_1189  = [ NaN ; NaN ; NaN ];
omc_error_1189 = [ NaN ; NaN ; NaN ];
Tc_error_1189  = [ NaN ; NaN ; NaN ];

%-- Image #1190:
omc_1190 = [ NaN ; NaN ; NaN ];
Tc_1190  = [ NaN ; NaN ; NaN ];
omc_error_1190 = [ NaN ; NaN ; NaN ];
Tc_error_1190  = [ NaN ; NaN ; NaN ];

%-- Image #1191:
omc_1191 = [ NaN ; NaN ; NaN ];
Tc_1191  = [ NaN ; NaN ; NaN ];
omc_error_1191 = [ NaN ; NaN ; NaN ];
Tc_error_1191  = [ NaN ; NaN ; NaN ];

%-- Image #1192:
omc_1192 = [ NaN ; NaN ; NaN ];
Tc_1192  = [ NaN ; NaN ; NaN ];
omc_error_1192 = [ NaN ; NaN ; NaN ];
Tc_error_1192  = [ NaN ; NaN ; NaN ];

%-- Image #1193:
omc_1193 = [ NaN ; NaN ; NaN ];
Tc_1193  = [ NaN ; NaN ; NaN ];
omc_error_1193 = [ NaN ; NaN ; NaN ];
Tc_error_1193  = [ NaN ; NaN ; NaN ];

%-- Image #1194:
omc_1194 = [ NaN ; NaN ; NaN ];
Tc_1194  = [ NaN ; NaN ; NaN ];
omc_error_1194 = [ NaN ; NaN ; NaN ];
Tc_error_1194  = [ NaN ; NaN ; NaN ];

%-- Image #1195:
omc_1195 = [ NaN ; NaN ; NaN ];
Tc_1195  = [ NaN ; NaN ; NaN ];
omc_error_1195 = [ NaN ; NaN ; NaN ];
Tc_error_1195  = [ NaN ; NaN ; NaN ];

%-- Image #1196:
omc_1196 = [ NaN ; NaN ; NaN ];
Tc_1196  = [ NaN ; NaN ; NaN ];
omc_error_1196 = [ NaN ; NaN ; NaN ];
Tc_error_1196  = [ NaN ; NaN ; NaN ];

%-- Image #1197:
omc_1197 = [ NaN ; NaN ; NaN ];
Tc_1197  = [ NaN ; NaN ; NaN ];
omc_error_1197 = [ NaN ; NaN ; NaN ];
Tc_error_1197  = [ NaN ; NaN ; NaN ];

%-- Image #1198:
omc_1198 = [ NaN ; NaN ; NaN ];
Tc_1198  = [ NaN ; NaN ; NaN ];
omc_error_1198 = [ NaN ; NaN ; NaN ];
Tc_error_1198  = [ NaN ; NaN ; NaN ];

%-- Image #1199:
omc_1199 = [ NaN ; NaN ; NaN ];
Tc_1199  = [ NaN ; NaN ; NaN ];
omc_error_1199 = [ NaN ; NaN ; NaN ];
Tc_error_1199  = [ NaN ; NaN ; NaN ];

%-- Image #1200:
omc_1200 = [ NaN ; NaN ; NaN ];
Tc_1200  = [ NaN ; NaN ; NaN ];
omc_error_1200 = [ NaN ; NaN ; NaN ];
Tc_error_1200  = [ NaN ; NaN ; NaN ];

%-- Image #1201:
omc_1201 = [ NaN ; NaN ; NaN ];
Tc_1201  = [ NaN ; NaN ; NaN ];
omc_error_1201 = [ NaN ; NaN ; NaN ];
Tc_error_1201  = [ NaN ; NaN ; NaN ];

%-- Image #1202:
omc_1202 = [ NaN ; NaN ; NaN ];
Tc_1202  = [ NaN ; NaN ; NaN ];
omc_error_1202 = [ NaN ; NaN ; NaN ];
Tc_error_1202  = [ NaN ; NaN ; NaN ];

%-- Image #1203:
omc_1203 = [ -2.083720e+000 ; -2.061369e+000 ; -2.235917e-001 ];
Tc_1203  = [ -8.306098e+001 ; -7.817514e+001 ; 2.064653e+002 ];
omc_error_1203 = [ 1.350392e-002 ; 1.504227e-002 ; 2.867941e-002 ];
Tc_error_1203  = [ 2.830311e+000 ; 3.038524e+000 ; 4.422725e+000 ];

%-- Image #1204:
omc_1204 = [ NaN ; NaN ; NaN ];
Tc_1204  = [ NaN ; NaN ; NaN ];
omc_error_1204 = [ NaN ; NaN ; NaN ];
Tc_error_1204  = [ NaN ; NaN ; NaN ];

%-- Image #1205:
omc_1205 = [ NaN ; NaN ; NaN ];
Tc_1205  = [ NaN ; NaN ; NaN ];
omc_error_1205 = [ NaN ; NaN ; NaN ];
Tc_error_1205  = [ NaN ; NaN ; NaN ];

%-- Image #1206:
omc_1206 = [ NaN ; NaN ; NaN ];
Tc_1206  = [ NaN ; NaN ; NaN ];
omc_error_1206 = [ NaN ; NaN ; NaN ];
Tc_error_1206  = [ NaN ; NaN ; NaN ];

%-- Image #1207:
omc_1207 = [ NaN ; NaN ; NaN ];
Tc_1207  = [ NaN ; NaN ; NaN ];
omc_error_1207 = [ NaN ; NaN ; NaN ];
Tc_error_1207  = [ NaN ; NaN ; NaN ];

%-- Image #1208:
omc_1208 = [ NaN ; NaN ; NaN ];
Tc_1208  = [ NaN ; NaN ; NaN ];
omc_error_1208 = [ NaN ; NaN ; NaN ];
Tc_error_1208  = [ NaN ; NaN ; NaN ];

%-- Image #1209:
omc_1209 = [ NaN ; NaN ; NaN ];
Tc_1209  = [ NaN ; NaN ; NaN ];
omc_error_1209 = [ NaN ; NaN ; NaN ];
Tc_error_1209  = [ NaN ; NaN ; NaN ];

%-- Image #1210:
omc_1210 = [ NaN ; NaN ; NaN ];
Tc_1210  = [ NaN ; NaN ; NaN ];
omc_error_1210 = [ NaN ; NaN ; NaN ];
Tc_error_1210  = [ NaN ; NaN ; NaN ];

%-- Image #1211:
omc_1211 = [ NaN ; NaN ; NaN ];
Tc_1211  = [ NaN ; NaN ; NaN ];
omc_error_1211 = [ NaN ; NaN ; NaN ];
Tc_error_1211  = [ NaN ; NaN ; NaN ];

%-- Image #1212:
omc_1212 = [ NaN ; NaN ; NaN ];
Tc_1212  = [ NaN ; NaN ; NaN ];
omc_error_1212 = [ NaN ; NaN ; NaN ];
Tc_error_1212  = [ NaN ; NaN ; NaN ];

%-- Image #1213:
omc_1213 = [ NaN ; NaN ; NaN ];
Tc_1213  = [ NaN ; NaN ; NaN ];
omc_error_1213 = [ NaN ; NaN ; NaN ];
Tc_error_1213  = [ NaN ; NaN ; NaN ];

%-- Image #1214:
omc_1214 = [ NaN ; NaN ; NaN ];
Tc_1214  = [ NaN ; NaN ; NaN ];
omc_error_1214 = [ NaN ; NaN ; NaN ];
Tc_error_1214  = [ NaN ; NaN ; NaN ];

%-- Image #1215:
omc_1215 = [ NaN ; NaN ; NaN ];
Tc_1215  = [ NaN ; NaN ; NaN ];
omc_error_1215 = [ NaN ; NaN ; NaN ];
Tc_error_1215  = [ NaN ; NaN ; NaN ];

%-- Image #1216:
omc_1216 = [ NaN ; NaN ; NaN ];
Tc_1216  = [ NaN ; NaN ; NaN ];
omc_error_1216 = [ NaN ; NaN ; NaN ];
Tc_error_1216  = [ NaN ; NaN ; NaN ];

%-- Image #1217:
omc_1217 = [ NaN ; NaN ; NaN ];
Tc_1217  = [ NaN ; NaN ; NaN ];
omc_error_1217 = [ NaN ; NaN ; NaN ];
Tc_error_1217  = [ NaN ; NaN ; NaN ];

%-- Image #1218:
omc_1218 = [ NaN ; NaN ; NaN ];
Tc_1218  = [ NaN ; NaN ; NaN ];
omc_error_1218 = [ NaN ; NaN ; NaN ];
Tc_error_1218  = [ NaN ; NaN ; NaN ];

%-- Image #1219:
omc_1219 = [ NaN ; NaN ; NaN ];
Tc_1219  = [ NaN ; NaN ; NaN ];
omc_error_1219 = [ NaN ; NaN ; NaN ];
Tc_error_1219  = [ NaN ; NaN ; NaN ];

%-- Image #1220:
omc_1220 = [ NaN ; NaN ; NaN ];
Tc_1220  = [ NaN ; NaN ; NaN ];
omc_error_1220 = [ NaN ; NaN ; NaN ];
Tc_error_1220  = [ NaN ; NaN ; NaN ];

%-- Image #1221:
omc_1221 = [ NaN ; NaN ; NaN ];
Tc_1221  = [ NaN ; NaN ; NaN ];
omc_error_1221 = [ NaN ; NaN ; NaN ];
Tc_error_1221  = [ NaN ; NaN ; NaN ];

%-- Image #1222:
omc_1222 = [ NaN ; NaN ; NaN ];
Tc_1222  = [ NaN ; NaN ; NaN ];
omc_error_1222 = [ NaN ; NaN ; NaN ];
Tc_error_1222  = [ NaN ; NaN ; NaN ];

%-- Image #1223:
omc_1223 = [ NaN ; NaN ; NaN ];
Tc_1223  = [ NaN ; NaN ; NaN ];
omc_error_1223 = [ NaN ; NaN ; NaN ];
Tc_error_1223  = [ NaN ; NaN ; NaN ];

%-- Image #1224:
omc_1224 = [ NaN ; NaN ; NaN ];
Tc_1224  = [ NaN ; NaN ; NaN ];
omc_error_1224 = [ NaN ; NaN ; NaN ];
Tc_error_1224  = [ NaN ; NaN ; NaN ];

%-- Image #1225:
omc_1225 = [ NaN ; NaN ; NaN ];
Tc_1225  = [ NaN ; NaN ; NaN ];
omc_error_1225 = [ NaN ; NaN ; NaN ];
Tc_error_1225  = [ NaN ; NaN ; NaN ];

%-- Image #1226:
omc_1226 = [ NaN ; NaN ; NaN ];
Tc_1226  = [ NaN ; NaN ; NaN ];
omc_error_1226 = [ NaN ; NaN ; NaN ];
Tc_error_1226  = [ NaN ; NaN ; NaN ];

%-- Image #1227:
omc_1227 = [ NaN ; NaN ; NaN ];
Tc_1227  = [ NaN ; NaN ; NaN ];
omc_error_1227 = [ NaN ; NaN ; NaN ];
Tc_error_1227  = [ NaN ; NaN ; NaN ];

%-- Image #1228:
omc_1228 = [ NaN ; NaN ; NaN ];
Tc_1228  = [ NaN ; NaN ; NaN ];
omc_error_1228 = [ NaN ; NaN ; NaN ];
Tc_error_1228  = [ NaN ; NaN ; NaN ];

%-- Image #1229:
omc_1229 = [ NaN ; NaN ; NaN ];
Tc_1229  = [ NaN ; NaN ; NaN ];
omc_error_1229 = [ NaN ; NaN ; NaN ];
Tc_error_1229  = [ NaN ; NaN ; NaN ];

%-- Image #1230:
omc_1230 = [ NaN ; NaN ; NaN ];
Tc_1230  = [ NaN ; NaN ; NaN ];
omc_error_1230 = [ NaN ; NaN ; NaN ];
Tc_error_1230  = [ NaN ; NaN ; NaN ];

%-- Image #1231:
omc_1231 = [ NaN ; NaN ; NaN ];
Tc_1231  = [ NaN ; NaN ; NaN ];
omc_error_1231 = [ NaN ; NaN ; NaN ];
Tc_error_1231  = [ NaN ; NaN ; NaN ];

%-- Image #1232:
omc_1232 = [ NaN ; NaN ; NaN ];
Tc_1232  = [ NaN ; NaN ; NaN ];
omc_error_1232 = [ NaN ; NaN ; NaN ];
Tc_error_1232  = [ NaN ; NaN ; NaN ];

%-- Image #1233:
omc_1233 = [ NaN ; NaN ; NaN ];
Tc_1233  = [ NaN ; NaN ; NaN ];
omc_error_1233 = [ NaN ; NaN ; NaN ];
Tc_error_1233  = [ NaN ; NaN ; NaN ];

%-- Image #1234:
omc_1234 = [ NaN ; NaN ; NaN ];
Tc_1234  = [ NaN ; NaN ; NaN ];
omc_error_1234 = [ NaN ; NaN ; NaN ];
Tc_error_1234  = [ NaN ; NaN ; NaN ];

%-- Image #1235:
omc_1235 = [ NaN ; NaN ; NaN ];
Tc_1235  = [ NaN ; NaN ; NaN ];
omc_error_1235 = [ NaN ; NaN ; NaN ];
Tc_error_1235  = [ NaN ; NaN ; NaN ];

%-- Image #1236:
omc_1236 = [ NaN ; NaN ; NaN ];
Tc_1236  = [ NaN ; NaN ; NaN ];
omc_error_1236 = [ NaN ; NaN ; NaN ];
Tc_error_1236  = [ NaN ; NaN ; NaN ];

%-- Image #1237:
omc_1237 = [ NaN ; NaN ; NaN ];
Tc_1237  = [ NaN ; NaN ; NaN ];
omc_error_1237 = [ NaN ; NaN ; NaN ];
Tc_error_1237  = [ NaN ; NaN ; NaN ];

%-- Image #1238:
omc_1238 = [ NaN ; NaN ; NaN ];
Tc_1238  = [ NaN ; NaN ; NaN ];
omc_error_1238 = [ NaN ; NaN ; NaN ];
Tc_error_1238  = [ NaN ; NaN ; NaN ];

%-- Image #1239:
omc_1239 = [ NaN ; NaN ; NaN ];
Tc_1239  = [ NaN ; NaN ; NaN ];
omc_error_1239 = [ NaN ; NaN ; NaN ];
Tc_error_1239  = [ NaN ; NaN ; NaN ];

%-- Image #1240:
omc_1240 = [ NaN ; NaN ; NaN ];
Tc_1240  = [ NaN ; NaN ; NaN ];
omc_error_1240 = [ NaN ; NaN ; NaN ];
Tc_error_1240  = [ NaN ; NaN ; NaN ];

%-- Image #1241:
omc_1241 = [ NaN ; NaN ; NaN ];
Tc_1241  = [ NaN ; NaN ; NaN ];
omc_error_1241 = [ NaN ; NaN ; NaN ];
Tc_error_1241  = [ NaN ; NaN ; NaN ];

%-- Image #1242:
omc_1242 = [ NaN ; NaN ; NaN ];
Tc_1242  = [ NaN ; NaN ; NaN ];
omc_error_1242 = [ NaN ; NaN ; NaN ];
Tc_error_1242  = [ NaN ; NaN ; NaN ];

%-- Image #1243:
omc_1243 = [ NaN ; NaN ; NaN ];
Tc_1243  = [ NaN ; NaN ; NaN ];
omc_error_1243 = [ NaN ; NaN ; NaN ];
Tc_error_1243  = [ NaN ; NaN ; NaN ];

%-- Image #1244:
omc_1244 = [ NaN ; NaN ; NaN ];
Tc_1244  = [ NaN ; NaN ; NaN ];
omc_error_1244 = [ NaN ; NaN ; NaN ];
Tc_error_1244  = [ NaN ; NaN ; NaN ];

%-- Image #1245:
omc_1245 = [ NaN ; NaN ; NaN ];
Tc_1245  = [ NaN ; NaN ; NaN ];
omc_error_1245 = [ NaN ; NaN ; NaN ];
Tc_error_1245  = [ NaN ; NaN ; NaN ];

%-- Image #1246:
omc_1246 = [ NaN ; NaN ; NaN ];
Tc_1246  = [ NaN ; NaN ; NaN ];
omc_error_1246 = [ NaN ; NaN ; NaN ];
Tc_error_1246  = [ NaN ; NaN ; NaN ];

%-- Image #1247:
omc_1247 = [ NaN ; NaN ; NaN ];
Tc_1247  = [ NaN ; NaN ; NaN ];
omc_error_1247 = [ NaN ; NaN ; NaN ];
Tc_error_1247  = [ NaN ; NaN ; NaN ];

%-- Image #1248:
omc_1248 = [ NaN ; NaN ; NaN ];
Tc_1248  = [ NaN ; NaN ; NaN ];
omc_error_1248 = [ NaN ; NaN ; NaN ];
Tc_error_1248  = [ NaN ; NaN ; NaN ];

%-- Image #1249:
omc_1249 = [ NaN ; NaN ; NaN ];
Tc_1249  = [ NaN ; NaN ; NaN ];
omc_error_1249 = [ NaN ; NaN ; NaN ];
Tc_error_1249  = [ NaN ; NaN ; NaN ];

%-- Image #1250:
omc_1250 = [ NaN ; NaN ; NaN ];
Tc_1250  = [ NaN ; NaN ; NaN ];
omc_error_1250 = [ NaN ; NaN ; NaN ];
Tc_error_1250  = [ NaN ; NaN ; NaN ];

%-- Image #1251:
omc_1251 = [ NaN ; NaN ; NaN ];
Tc_1251  = [ NaN ; NaN ; NaN ];
omc_error_1251 = [ NaN ; NaN ; NaN ];
Tc_error_1251  = [ NaN ; NaN ; NaN ];

%-- Image #1252:
omc_1252 = [ NaN ; NaN ; NaN ];
Tc_1252  = [ NaN ; NaN ; NaN ];
omc_error_1252 = [ NaN ; NaN ; NaN ];
Tc_error_1252  = [ NaN ; NaN ; NaN ];

%-- Image #1253:
omc_1253 = [ NaN ; NaN ; NaN ];
Tc_1253  = [ NaN ; NaN ; NaN ];
omc_error_1253 = [ NaN ; NaN ; NaN ];
Tc_error_1253  = [ NaN ; NaN ; NaN ];

%-- Image #1254:
omc_1254 = [ NaN ; NaN ; NaN ];
Tc_1254  = [ NaN ; NaN ; NaN ];
omc_error_1254 = [ NaN ; NaN ; NaN ];
Tc_error_1254  = [ NaN ; NaN ; NaN ];

%-- Image #1255:
omc_1255 = [ NaN ; NaN ; NaN ];
Tc_1255  = [ NaN ; NaN ; NaN ];
omc_error_1255 = [ NaN ; NaN ; NaN ];
Tc_error_1255  = [ NaN ; NaN ; NaN ];

%-- Image #1256:
omc_1256 = [ NaN ; NaN ; NaN ];
Tc_1256  = [ NaN ; NaN ; NaN ];
omc_error_1256 = [ NaN ; NaN ; NaN ];
Tc_error_1256  = [ NaN ; NaN ; NaN ];

%-- Image #1257:
omc_1257 = [ NaN ; NaN ; NaN ];
Tc_1257  = [ NaN ; NaN ; NaN ];
omc_error_1257 = [ NaN ; NaN ; NaN ];
Tc_error_1257  = [ NaN ; NaN ; NaN ];

%-- Image #1258:
omc_1258 = [ NaN ; NaN ; NaN ];
Tc_1258  = [ NaN ; NaN ; NaN ];
omc_error_1258 = [ NaN ; NaN ; NaN ];
Tc_error_1258  = [ NaN ; NaN ; NaN ];

%-- Image #1259:
omc_1259 = [ NaN ; NaN ; NaN ];
Tc_1259  = [ NaN ; NaN ; NaN ];
omc_error_1259 = [ NaN ; NaN ; NaN ];
Tc_error_1259  = [ NaN ; NaN ; NaN ];

%-- Image #1260:
omc_1260 = [ NaN ; NaN ; NaN ];
Tc_1260  = [ NaN ; NaN ; NaN ];
omc_error_1260 = [ NaN ; NaN ; NaN ];
Tc_error_1260  = [ NaN ; NaN ; NaN ];

%-- Image #1261:
omc_1261 = [ NaN ; NaN ; NaN ];
Tc_1261  = [ NaN ; NaN ; NaN ];
omc_error_1261 = [ NaN ; NaN ; NaN ];
Tc_error_1261  = [ NaN ; NaN ; NaN ];

%-- Image #1262:
omc_1262 = [ NaN ; NaN ; NaN ];
Tc_1262  = [ NaN ; NaN ; NaN ];
omc_error_1262 = [ NaN ; NaN ; NaN ];
Tc_error_1262  = [ NaN ; NaN ; NaN ];

%-- Image #1263:
omc_1263 = [ NaN ; NaN ; NaN ];
Tc_1263  = [ NaN ; NaN ; NaN ];
omc_error_1263 = [ NaN ; NaN ; NaN ];
Tc_error_1263  = [ NaN ; NaN ; NaN ];

%-- Image #1264:
omc_1264 = [ NaN ; NaN ; NaN ];
Tc_1264  = [ NaN ; NaN ; NaN ];
omc_error_1264 = [ NaN ; NaN ; NaN ];
Tc_error_1264  = [ NaN ; NaN ; NaN ];

%-- Image #1265:
omc_1265 = [ NaN ; NaN ; NaN ];
Tc_1265  = [ NaN ; NaN ; NaN ];
omc_error_1265 = [ NaN ; NaN ; NaN ];
Tc_error_1265  = [ NaN ; NaN ; NaN ];

%-- Image #1266:
omc_1266 = [ NaN ; NaN ; NaN ];
Tc_1266  = [ NaN ; NaN ; NaN ];
omc_error_1266 = [ NaN ; NaN ; NaN ];
Tc_error_1266  = [ NaN ; NaN ; NaN ];

%-- Image #1267:
omc_1267 = [ NaN ; NaN ; NaN ];
Tc_1267  = [ NaN ; NaN ; NaN ];
omc_error_1267 = [ NaN ; NaN ; NaN ];
Tc_error_1267  = [ NaN ; NaN ; NaN ];

%-- Image #1268:
omc_1268 = [ NaN ; NaN ; NaN ];
Tc_1268  = [ NaN ; NaN ; NaN ];
omc_error_1268 = [ NaN ; NaN ; NaN ];
Tc_error_1268  = [ NaN ; NaN ; NaN ];

%-- Image #1269:
omc_1269 = [ NaN ; NaN ; NaN ];
Tc_1269  = [ NaN ; NaN ; NaN ];
omc_error_1269 = [ NaN ; NaN ; NaN ];
Tc_error_1269  = [ NaN ; NaN ; NaN ];

%-- Image #1270:
omc_1270 = [ -1.992800e+000 ; -2.163230e+000 ; -1.427469e-001 ];
Tc_1270  = [ -8.612508e+001 ; -1.073378e+002 ; 2.092259e+002 ];
omc_error_1270 = [ 1.383698e-002 ; 1.482612e-002 ; 2.845405e-002 ];
Tc_error_1270  = [ 2.895292e+000 ; 3.101969e+000 ; 4.532948e+000 ];

%-- Image #1271:
omc_1271 = [ NaN ; NaN ; NaN ];
Tc_1271  = [ NaN ; NaN ; NaN ];
omc_error_1271 = [ NaN ; NaN ; NaN ];
Tc_error_1271  = [ NaN ; NaN ; NaN ];

%-- Image #1272:
omc_1272 = [ NaN ; NaN ; NaN ];
Tc_1272  = [ NaN ; NaN ; NaN ];
omc_error_1272 = [ NaN ; NaN ; NaN ];
Tc_error_1272  = [ NaN ; NaN ; NaN ];

%-- Image #1273:
omc_1273 = [ NaN ; NaN ; NaN ];
Tc_1273  = [ NaN ; NaN ; NaN ];
omc_error_1273 = [ NaN ; NaN ; NaN ];
Tc_error_1273  = [ NaN ; NaN ; NaN ];

%-- Image #1274:
omc_1274 = [ NaN ; NaN ; NaN ];
Tc_1274  = [ NaN ; NaN ; NaN ];
omc_error_1274 = [ NaN ; NaN ; NaN ];
Tc_error_1274  = [ NaN ; NaN ; NaN ];

%-- Image #1275:
omc_1275 = [ NaN ; NaN ; NaN ];
Tc_1275  = [ NaN ; NaN ; NaN ];
omc_error_1275 = [ NaN ; NaN ; NaN ];
Tc_error_1275  = [ NaN ; NaN ; NaN ];

%-- Image #1276:
omc_1276 = [ NaN ; NaN ; NaN ];
Tc_1276  = [ NaN ; NaN ; NaN ];
omc_error_1276 = [ NaN ; NaN ; NaN ];
Tc_error_1276  = [ NaN ; NaN ; NaN ];

%-- Image #1277:
omc_1277 = [ NaN ; NaN ; NaN ];
Tc_1277  = [ NaN ; NaN ; NaN ];
omc_error_1277 = [ NaN ; NaN ; NaN ];
Tc_error_1277  = [ NaN ; NaN ; NaN ];

%-- Image #1278:
omc_1278 = [ NaN ; NaN ; NaN ];
Tc_1278  = [ NaN ; NaN ; NaN ];
omc_error_1278 = [ NaN ; NaN ; NaN ];
Tc_error_1278  = [ NaN ; NaN ; NaN ];

%-- Image #1279:
omc_1279 = [ NaN ; NaN ; NaN ];
Tc_1279  = [ NaN ; NaN ; NaN ];
omc_error_1279 = [ NaN ; NaN ; NaN ];
Tc_error_1279  = [ NaN ; NaN ; NaN ];

%-- Image #1280:
omc_1280 = [ NaN ; NaN ; NaN ];
Tc_1280  = [ NaN ; NaN ; NaN ];
omc_error_1280 = [ NaN ; NaN ; NaN ];
Tc_error_1280  = [ NaN ; NaN ; NaN ];

%-- Image #1281:
omc_1281 = [ NaN ; NaN ; NaN ];
Tc_1281  = [ NaN ; NaN ; NaN ];
omc_error_1281 = [ NaN ; NaN ; NaN ];
Tc_error_1281  = [ NaN ; NaN ; NaN ];

%-- Image #1282:
omc_1282 = [ NaN ; NaN ; NaN ];
Tc_1282  = [ NaN ; NaN ; NaN ];
omc_error_1282 = [ NaN ; NaN ; NaN ];
Tc_error_1282  = [ NaN ; NaN ; NaN ];

%-- Image #1283:
omc_1283 = [ NaN ; NaN ; NaN ];
Tc_1283  = [ NaN ; NaN ; NaN ];
omc_error_1283 = [ NaN ; NaN ; NaN ];
Tc_error_1283  = [ NaN ; NaN ; NaN ];

%-- Image #1284:
omc_1284 = [ NaN ; NaN ; NaN ];
Tc_1284  = [ NaN ; NaN ; NaN ];
omc_error_1284 = [ NaN ; NaN ; NaN ];
Tc_error_1284  = [ NaN ; NaN ; NaN ];

%-- Image #1285:
omc_1285 = [ NaN ; NaN ; NaN ];
Tc_1285  = [ NaN ; NaN ; NaN ];
omc_error_1285 = [ NaN ; NaN ; NaN ];
Tc_error_1285  = [ NaN ; NaN ; NaN ];

%-- Image #1286:
omc_1286 = [ NaN ; NaN ; NaN ];
Tc_1286  = [ NaN ; NaN ; NaN ];
omc_error_1286 = [ NaN ; NaN ; NaN ];
Tc_error_1286  = [ NaN ; NaN ; NaN ];

%-- Image #1287:
omc_1287 = [ NaN ; NaN ; NaN ];
Tc_1287  = [ NaN ; NaN ; NaN ];
omc_error_1287 = [ NaN ; NaN ; NaN ];
Tc_error_1287  = [ NaN ; NaN ; NaN ];

%-- Image #1288:
omc_1288 = [ NaN ; NaN ; NaN ];
Tc_1288  = [ NaN ; NaN ; NaN ];
omc_error_1288 = [ NaN ; NaN ; NaN ];
Tc_error_1288  = [ NaN ; NaN ; NaN ];

%-- Image #1289:
omc_1289 = [ NaN ; NaN ; NaN ];
Tc_1289  = [ NaN ; NaN ; NaN ];
omc_error_1289 = [ NaN ; NaN ; NaN ];
Tc_error_1289  = [ NaN ; NaN ; NaN ];

%-- Image #1290:
omc_1290 = [ NaN ; NaN ; NaN ];
Tc_1290  = [ NaN ; NaN ; NaN ];
omc_error_1290 = [ NaN ; NaN ; NaN ];
Tc_error_1290  = [ NaN ; NaN ; NaN ];

%-- Image #1291:
omc_1291 = [ NaN ; NaN ; NaN ];
Tc_1291  = [ NaN ; NaN ; NaN ];
omc_error_1291 = [ NaN ; NaN ; NaN ];
Tc_error_1291  = [ NaN ; NaN ; NaN ];

%-- Image #1292:
omc_1292 = [ NaN ; NaN ; NaN ];
Tc_1292  = [ NaN ; NaN ; NaN ];
omc_error_1292 = [ NaN ; NaN ; NaN ];
Tc_error_1292  = [ NaN ; NaN ; NaN ];

%-- Image #1293:
omc_1293 = [ NaN ; NaN ; NaN ];
Tc_1293  = [ NaN ; NaN ; NaN ];
omc_error_1293 = [ NaN ; NaN ; NaN ];
Tc_error_1293  = [ NaN ; NaN ; NaN ];

%-- Image #1294:
omc_1294 = [ NaN ; NaN ; NaN ];
Tc_1294  = [ NaN ; NaN ; NaN ];
omc_error_1294 = [ NaN ; NaN ; NaN ];
Tc_error_1294  = [ NaN ; NaN ; NaN ];

%-- Image #1295:
omc_1295 = [ NaN ; NaN ; NaN ];
Tc_1295  = [ NaN ; NaN ; NaN ];
omc_error_1295 = [ NaN ; NaN ; NaN ];
Tc_error_1295  = [ NaN ; NaN ; NaN ];

%-- Image #1296:
omc_1296 = [ NaN ; NaN ; NaN ];
Tc_1296  = [ NaN ; NaN ; NaN ];
omc_error_1296 = [ NaN ; NaN ; NaN ];
Tc_error_1296  = [ NaN ; NaN ; NaN ];

%-- Image #1297:
omc_1297 = [ NaN ; NaN ; NaN ];
Tc_1297  = [ NaN ; NaN ; NaN ];
omc_error_1297 = [ NaN ; NaN ; NaN ];
Tc_error_1297  = [ NaN ; NaN ; NaN ];

%-- Image #1298:
omc_1298 = [ NaN ; NaN ; NaN ];
Tc_1298  = [ NaN ; NaN ; NaN ];
omc_error_1298 = [ NaN ; NaN ; NaN ];
Tc_error_1298  = [ NaN ; NaN ; NaN ];

%-- Image #1299:
omc_1299 = [ NaN ; NaN ; NaN ];
Tc_1299  = [ NaN ; NaN ; NaN ];
omc_error_1299 = [ NaN ; NaN ; NaN ];
Tc_error_1299  = [ NaN ; NaN ; NaN ];

%-- Image #1300:
omc_1300 = [ NaN ; NaN ; NaN ];
Tc_1300  = [ NaN ; NaN ; NaN ];
omc_error_1300 = [ NaN ; NaN ; NaN ];
Tc_error_1300  = [ NaN ; NaN ; NaN ];

%-- Image #1301:
omc_1301 = [ NaN ; NaN ; NaN ];
Tc_1301  = [ NaN ; NaN ; NaN ];
omc_error_1301 = [ NaN ; NaN ; NaN ];
Tc_error_1301  = [ NaN ; NaN ; NaN ];

%-- Image #1302:
omc_1302 = [ NaN ; NaN ; NaN ];
Tc_1302  = [ NaN ; NaN ; NaN ];
omc_error_1302 = [ NaN ; NaN ; NaN ];
Tc_error_1302  = [ NaN ; NaN ; NaN ];

%-- Image #1303:
omc_1303 = [ NaN ; NaN ; NaN ];
Tc_1303  = [ NaN ; NaN ; NaN ];
omc_error_1303 = [ NaN ; NaN ; NaN ];
Tc_error_1303  = [ NaN ; NaN ; NaN ];

%-- Image #1304:
omc_1304 = [ NaN ; NaN ; NaN ];
Tc_1304  = [ NaN ; NaN ; NaN ];
omc_error_1304 = [ NaN ; NaN ; NaN ];
Tc_error_1304  = [ NaN ; NaN ; NaN ];

%-- Image #1305:
omc_1305 = [ NaN ; NaN ; NaN ];
Tc_1305  = [ NaN ; NaN ; NaN ];
omc_error_1305 = [ NaN ; NaN ; NaN ];
Tc_error_1305  = [ NaN ; NaN ; NaN ];

%-- Image #1306:
omc_1306 = [ NaN ; NaN ; NaN ];
Tc_1306  = [ NaN ; NaN ; NaN ];
omc_error_1306 = [ NaN ; NaN ; NaN ];
Tc_error_1306  = [ NaN ; NaN ; NaN ];

%-- Image #1307:
omc_1307 = [ NaN ; NaN ; NaN ];
Tc_1307  = [ NaN ; NaN ; NaN ];
omc_error_1307 = [ NaN ; NaN ; NaN ];
Tc_error_1307  = [ NaN ; NaN ; NaN ];

%-- Image #1308:
omc_1308 = [ NaN ; NaN ; NaN ];
Tc_1308  = [ NaN ; NaN ; NaN ];
omc_error_1308 = [ NaN ; NaN ; NaN ];
Tc_error_1308  = [ NaN ; NaN ; NaN ];

%-- Image #1309:
omc_1309 = [ NaN ; NaN ; NaN ];
Tc_1309  = [ NaN ; NaN ; NaN ];
omc_error_1309 = [ NaN ; NaN ; NaN ];
Tc_error_1309  = [ NaN ; NaN ; NaN ];

%-- Image #1310:
omc_1310 = [ NaN ; NaN ; NaN ];
Tc_1310  = [ NaN ; NaN ; NaN ];
omc_error_1310 = [ NaN ; NaN ; NaN ];
Tc_error_1310  = [ NaN ; NaN ; NaN ];

%-- Image #1311:
omc_1311 = [ NaN ; NaN ; NaN ];
Tc_1311  = [ NaN ; NaN ; NaN ];
omc_error_1311 = [ NaN ; NaN ; NaN ];
Tc_error_1311  = [ NaN ; NaN ; NaN ];

%-- Image #1312:
omc_1312 = [ NaN ; NaN ; NaN ];
Tc_1312  = [ NaN ; NaN ; NaN ];
omc_error_1312 = [ NaN ; NaN ; NaN ];
Tc_error_1312  = [ NaN ; NaN ; NaN ];

%-- Image #1313:
omc_1313 = [ NaN ; NaN ; NaN ];
Tc_1313  = [ NaN ; NaN ; NaN ];
omc_error_1313 = [ NaN ; NaN ; NaN ];
Tc_error_1313  = [ NaN ; NaN ; NaN ];

%-- Image #1314:
omc_1314 = [ NaN ; NaN ; NaN ];
Tc_1314  = [ NaN ; NaN ; NaN ];
omc_error_1314 = [ NaN ; NaN ; NaN ];
Tc_error_1314  = [ NaN ; NaN ; NaN ];

%-- Image #1315:
omc_1315 = [ NaN ; NaN ; NaN ];
Tc_1315  = [ NaN ; NaN ; NaN ];
omc_error_1315 = [ NaN ; NaN ; NaN ];
Tc_error_1315  = [ NaN ; NaN ; NaN ];

%-- Image #1316:
omc_1316 = [ NaN ; NaN ; NaN ];
Tc_1316  = [ NaN ; NaN ; NaN ];
omc_error_1316 = [ NaN ; NaN ; NaN ];
Tc_error_1316  = [ NaN ; NaN ; NaN ];

%-- Image #1317:
omc_1317 = [ NaN ; NaN ; NaN ];
Tc_1317  = [ NaN ; NaN ; NaN ];
omc_error_1317 = [ NaN ; NaN ; NaN ];
Tc_error_1317  = [ NaN ; NaN ; NaN ];

%-- Image #1318:
omc_1318 = [ NaN ; NaN ; NaN ];
Tc_1318  = [ NaN ; NaN ; NaN ];
omc_error_1318 = [ NaN ; NaN ; NaN ];
Tc_error_1318  = [ NaN ; NaN ; NaN ];

%-- Image #1319:
omc_1319 = [ NaN ; NaN ; NaN ];
Tc_1319  = [ NaN ; NaN ; NaN ];
omc_error_1319 = [ NaN ; NaN ; NaN ];
Tc_error_1319  = [ NaN ; NaN ; NaN ];

%-- Image #1320:
omc_1320 = [ NaN ; NaN ; NaN ];
Tc_1320  = [ NaN ; NaN ; NaN ];
omc_error_1320 = [ NaN ; NaN ; NaN ];
Tc_error_1320  = [ NaN ; NaN ; NaN ];

%-- Image #1321:
omc_1321 = [ NaN ; NaN ; NaN ];
Tc_1321  = [ NaN ; NaN ; NaN ];
omc_error_1321 = [ NaN ; NaN ; NaN ];
Tc_error_1321  = [ NaN ; NaN ; NaN ];

%-- Image #1322:
omc_1322 = [ NaN ; NaN ; NaN ];
Tc_1322  = [ NaN ; NaN ; NaN ];
omc_error_1322 = [ NaN ; NaN ; NaN ];
Tc_error_1322  = [ NaN ; NaN ; NaN ];

%-- Image #1323:
omc_1323 = [ NaN ; NaN ; NaN ];
Tc_1323  = [ NaN ; NaN ; NaN ];
omc_error_1323 = [ NaN ; NaN ; NaN ];
Tc_error_1323  = [ NaN ; NaN ; NaN ];

%-- Image #1324:
omc_1324 = [ NaN ; NaN ; NaN ];
Tc_1324  = [ NaN ; NaN ; NaN ];
omc_error_1324 = [ NaN ; NaN ; NaN ];
Tc_error_1324  = [ NaN ; NaN ; NaN ];

%-- Image #1325:
omc_1325 = [ NaN ; NaN ; NaN ];
Tc_1325  = [ NaN ; NaN ; NaN ];
omc_error_1325 = [ NaN ; NaN ; NaN ];
Tc_error_1325  = [ NaN ; NaN ; NaN ];

%-- Image #1326:
omc_1326 = [ NaN ; NaN ; NaN ];
Tc_1326  = [ NaN ; NaN ; NaN ];
omc_error_1326 = [ NaN ; NaN ; NaN ];
Tc_error_1326  = [ NaN ; NaN ; NaN ];

%-- Image #1327:
omc_1327 = [ NaN ; NaN ; NaN ];
Tc_1327  = [ NaN ; NaN ; NaN ];
omc_error_1327 = [ NaN ; NaN ; NaN ];
Tc_error_1327  = [ NaN ; NaN ; NaN ];

%-- Image #1328:
omc_1328 = [ NaN ; NaN ; NaN ];
Tc_1328  = [ NaN ; NaN ; NaN ];
omc_error_1328 = [ NaN ; NaN ; NaN ];
Tc_error_1328  = [ NaN ; NaN ; NaN ];

%-- Image #1329:
omc_1329 = [ NaN ; NaN ; NaN ];
Tc_1329  = [ NaN ; NaN ; NaN ];
omc_error_1329 = [ NaN ; NaN ; NaN ];
Tc_error_1329  = [ NaN ; NaN ; NaN ];

%-- Image #1330:
omc_1330 = [ NaN ; NaN ; NaN ];
Tc_1330  = [ NaN ; NaN ; NaN ];
omc_error_1330 = [ NaN ; NaN ; NaN ];
Tc_error_1330  = [ NaN ; NaN ; NaN ];

%-- Image #1331:
omc_1331 = [ NaN ; NaN ; NaN ];
Tc_1331  = [ NaN ; NaN ; NaN ];
omc_error_1331 = [ NaN ; NaN ; NaN ];
Tc_error_1331  = [ NaN ; NaN ; NaN ];

%-- Image #1332:
omc_1332 = [ NaN ; NaN ; NaN ];
Tc_1332  = [ NaN ; NaN ; NaN ];
omc_error_1332 = [ NaN ; NaN ; NaN ];
Tc_error_1332  = [ NaN ; NaN ; NaN ];

%-- Image #1333:
omc_1333 = [ NaN ; NaN ; NaN ];
Tc_1333  = [ NaN ; NaN ; NaN ];
omc_error_1333 = [ NaN ; NaN ; NaN ];
Tc_error_1333  = [ NaN ; NaN ; NaN ];

%-- Image #1334:
omc_1334 = [ NaN ; NaN ; NaN ];
Tc_1334  = [ NaN ; NaN ; NaN ];
omc_error_1334 = [ NaN ; NaN ; NaN ];
Tc_error_1334  = [ NaN ; NaN ; NaN ];

%-- Image #1335:
omc_1335 = [ NaN ; NaN ; NaN ];
Tc_1335  = [ NaN ; NaN ; NaN ];
omc_error_1335 = [ NaN ; NaN ; NaN ];
Tc_error_1335  = [ NaN ; NaN ; NaN ];

%-- Image #1336:
omc_1336 = [ -1.846542e+000 ; -1.912149e+000 ; -4.324945e-001 ];
Tc_1336  = [ -9.980352e+001 ; -8.506020e+001 ; 2.075735e+002 ];
omc_error_1336 = [ 1.337373e-002 ; 1.469134e-002 ; 2.441650e-002 ];
Tc_error_1336  = [ 2.837901e+000 ; 3.139078e+000 ; 4.509633e+000 ];

%-- Image #1337:
omc_1337 = [ NaN ; NaN ; NaN ];
Tc_1337  = [ NaN ; NaN ; NaN ];
omc_error_1337 = [ NaN ; NaN ; NaN ];
Tc_error_1337  = [ NaN ; NaN ; NaN ];

%-- Image #1338:
omc_1338 = [ NaN ; NaN ; NaN ];
Tc_1338  = [ NaN ; NaN ; NaN ];
omc_error_1338 = [ NaN ; NaN ; NaN ];
Tc_error_1338  = [ NaN ; NaN ; NaN ];

%-- Image #1339:
omc_1339 = [ NaN ; NaN ; NaN ];
Tc_1339  = [ NaN ; NaN ; NaN ];
omc_error_1339 = [ NaN ; NaN ; NaN ];
Tc_error_1339  = [ NaN ; NaN ; NaN ];

%-- Image #1340:
omc_1340 = [ NaN ; NaN ; NaN ];
Tc_1340  = [ NaN ; NaN ; NaN ];
omc_error_1340 = [ NaN ; NaN ; NaN ];
Tc_error_1340  = [ NaN ; NaN ; NaN ];

%-- Image #1341:
omc_1341 = [ NaN ; NaN ; NaN ];
Tc_1341  = [ NaN ; NaN ; NaN ];
omc_error_1341 = [ NaN ; NaN ; NaN ];
Tc_error_1341  = [ NaN ; NaN ; NaN ];

%-- Image #1342:
omc_1342 = [ NaN ; NaN ; NaN ];
Tc_1342  = [ NaN ; NaN ; NaN ];
omc_error_1342 = [ NaN ; NaN ; NaN ];
Tc_error_1342  = [ NaN ; NaN ; NaN ];

%-- Image #1343:
omc_1343 = [ NaN ; NaN ; NaN ];
Tc_1343  = [ NaN ; NaN ; NaN ];
omc_error_1343 = [ NaN ; NaN ; NaN ];
Tc_error_1343  = [ NaN ; NaN ; NaN ];

%-- Image #1344:
omc_1344 = [ NaN ; NaN ; NaN ];
Tc_1344  = [ NaN ; NaN ; NaN ];
omc_error_1344 = [ NaN ; NaN ; NaN ];
Tc_error_1344  = [ NaN ; NaN ; NaN ];

%-- Image #1345:
omc_1345 = [ NaN ; NaN ; NaN ];
Tc_1345  = [ NaN ; NaN ; NaN ];
omc_error_1345 = [ NaN ; NaN ; NaN ];
Tc_error_1345  = [ NaN ; NaN ; NaN ];

%-- Image #1346:
omc_1346 = [ NaN ; NaN ; NaN ];
Tc_1346  = [ NaN ; NaN ; NaN ];
omc_error_1346 = [ NaN ; NaN ; NaN ];
Tc_error_1346  = [ NaN ; NaN ; NaN ];

%-- Image #1347:
omc_1347 = [ NaN ; NaN ; NaN ];
Tc_1347  = [ NaN ; NaN ; NaN ];
omc_error_1347 = [ NaN ; NaN ; NaN ];
Tc_error_1347  = [ NaN ; NaN ; NaN ];

%-- Image #1348:
omc_1348 = [ NaN ; NaN ; NaN ];
Tc_1348  = [ NaN ; NaN ; NaN ];
omc_error_1348 = [ NaN ; NaN ; NaN ];
Tc_error_1348  = [ NaN ; NaN ; NaN ];

%-- Image #1349:
omc_1349 = [ NaN ; NaN ; NaN ];
Tc_1349  = [ NaN ; NaN ; NaN ];
omc_error_1349 = [ NaN ; NaN ; NaN ];
Tc_error_1349  = [ NaN ; NaN ; NaN ];

%-- Image #1350:
omc_1350 = [ NaN ; NaN ; NaN ];
Tc_1350  = [ NaN ; NaN ; NaN ];
omc_error_1350 = [ NaN ; NaN ; NaN ];
Tc_error_1350  = [ NaN ; NaN ; NaN ];

%-- Image #1351:
omc_1351 = [ NaN ; NaN ; NaN ];
Tc_1351  = [ NaN ; NaN ; NaN ];
omc_error_1351 = [ NaN ; NaN ; NaN ];
Tc_error_1351  = [ NaN ; NaN ; NaN ];

%-- Image #1352:
omc_1352 = [ NaN ; NaN ; NaN ];
Tc_1352  = [ NaN ; NaN ; NaN ];
omc_error_1352 = [ NaN ; NaN ; NaN ];
Tc_error_1352  = [ NaN ; NaN ; NaN ];

%-- Image #1353:
omc_1353 = [ NaN ; NaN ; NaN ];
Tc_1353  = [ NaN ; NaN ; NaN ];
omc_error_1353 = [ NaN ; NaN ; NaN ];
Tc_error_1353  = [ NaN ; NaN ; NaN ];

%-- Image #1354:
omc_1354 = [ NaN ; NaN ; NaN ];
Tc_1354  = [ NaN ; NaN ; NaN ];
omc_error_1354 = [ NaN ; NaN ; NaN ];
Tc_error_1354  = [ NaN ; NaN ; NaN ];

%-- Image #1355:
omc_1355 = [ NaN ; NaN ; NaN ];
Tc_1355  = [ NaN ; NaN ; NaN ];
omc_error_1355 = [ NaN ; NaN ; NaN ];
Tc_error_1355  = [ NaN ; NaN ; NaN ];

%-- Image #1356:
omc_1356 = [ NaN ; NaN ; NaN ];
Tc_1356  = [ NaN ; NaN ; NaN ];
omc_error_1356 = [ NaN ; NaN ; NaN ];
Tc_error_1356  = [ NaN ; NaN ; NaN ];

%-- Image #1357:
omc_1357 = [ NaN ; NaN ; NaN ];
Tc_1357  = [ NaN ; NaN ; NaN ];
omc_error_1357 = [ NaN ; NaN ; NaN ];
Tc_error_1357  = [ NaN ; NaN ; NaN ];

%-- Image #1358:
omc_1358 = [ NaN ; NaN ; NaN ];
Tc_1358  = [ NaN ; NaN ; NaN ];
omc_error_1358 = [ NaN ; NaN ; NaN ];
Tc_error_1358  = [ NaN ; NaN ; NaN ];

%-- Image #1359:
omc_1359 = [ NaN ; NaN ; NaN ];
Tc_1359  = [ NaN ; NaN ; NaN ];
omc_error_1359 = [ NaN ; NaN ; NaN ];
Tc_error_1359  = [ NaN ; NaN ; NaN ];

%-- Image #1360:
omc_1360 = [ NaN ; NaN ; NaN ];
Tc_1360  = [ NaN ; NaN ; NaN ];
omc_error_1360 = [ NaN ; NaN ; NaN ];
Tc_error_1360  = [ NaN ; NaN ; NaN ];

%-- Image #1361:
omc_1361 = [ NaN ; NaN ; NaN ];
Tc_1361  = [ NaN ; NaN ; NaN ];
omc_error_1361 = [ NaN ; NaN ; NaN ];
Tc_error_1361  = [ NaN ; NaN ; NaN ];

%-- Image #1362:
omc_1362 = [ NaN ; NaN ; NaN ];
Tc_1362  = [ NaN ; NaN ; NaN ];
omc_error_1362 = [ NaN ; NaN ; NaN ];
Tc_error_1362  = [ NaN ; NaN ; NaN ];

%-- Image #1363:
omc_1363 = [ NaN ; NaN ; NaN ];
Tc_1363  = [ NaN ; NaN ; NaN ];
omc_error_1363 = [ NaN ; NaN ; NaN ];
Tc_error_1363  = [ NaN ; NaN ; NaN ];

%-- Image #1364:
omc_1364 = [ NaN ; NaN ; NaN ];
Tc_1364  = [ NaN ; NaN ; NaN ];
omc_error_1364 = [ NaN ; NaN ; NaN ];
Tc_error_1364  = [ NaN ; NaN ; NaN ];

%-- Image #1365:
omc_1365 = [ NaN ; NaN ; NaN ];
Tc_1365  = [ NaN ; NaN ; NaN ];
omc_error_1365 = [ NaN ; NaN ; NaN ];
Tc_error_1365  = [ NaN ; NaN ; NaN ];

%-- Image #1366:
omc_1366 = [ NaN ; NaN ; NaN ];
Tc_1366  = [ NaN ; NaN ; NaN ];
omc_error_1366 = [ NaN ; NaN ; NaN ];
Tc_error_1366  = [ NaN ; NaN ; NaN ];

%-- Image #1367:
omc_1367 = [ NaN ; NaN ; NaN ];
Tc_1367  = [ NaN ; NaN ; NaN ];
omc_error_1367 = [ NaN ; NaN ; NaN ];
Tc_error_1367  = [ NaN ; NaN ; NaN ];

%-- Image #1368:
omc_1368 = [ NaN ; NaN ; NaN ];
Tc_1368  = [ NaN ; NaN ; NaN ];
omc_error_1368 = [ NaN ; NaN ; NaN ];
Tc_error_1368  = [ NaN ; NaN ; NaN ];

%-- Image #1369:
omc_1369 = [ NaN ; NaN ; NaN ];
Tc_1369  = [ NaN ; NaN ; NaN ];
omc_error_1369 = [ NaN ; NaN ; NaN ];
Tc_error_1369  = [ NaN ; NaN ; NaN ];

%-- Image #1370:
omc_1370 = [ NaN ; NaN ; NaN ];
Tc_1370  = [ NaN ; NaN ; NaN ];
omc_error_1370 = [ NaN ; NaN ; NaN ];
Tc_error_1370  = [ NaN ; NaN ; NaN ];

%-- Image #1371:
omc_1371 = [ NaN ; NaN ; NaN ];
Tc_1371  = [ NaN ; NaN ; NaN ];
omc_error_1371 = [ NaN ; NaN ; NaN ];
Tc_error_1371  = [ NaN ; NaN ; NaN ];

%-- Image #1372:
omc_1372 = [ NaN ; NaN ; NaN ];
Tc_1372  = [ NaN ; NaN ; NaN ];
omc_error_1372 = [ NaN ; NaN ; NaN ];
Tc_error_1372  = [ NaN ; NaN ; NaN ];

%-- Image #1373:
omc_1373 = [ NaN ; NaN ; NaN ];
Tc_1373  = [ NaN ; NaN ; NaN ];
omc_error_1373 = [ NaN ; NaN ; NaN ];
Tc_error_1373  = [ NaN ; NaN ; NaN ];

%-- Image #1374:
omc_1374 = [ NaN ; NaN ; NaN ];
Tc_1374  = [ NaN ; NaN ; NaN ];
omc_error_1374 = [ NaN ; NaN ; NaN ];
Tc_error_1374  = [ NaN ; NaN ; NaN ];

%-- Image #1375:
omc_1375 = [ NaN ; NaN ; NaN ];
Tc_1375  = [ NaN ; NaN ; NaN ];
omc_error_1375 = [ NaN ; NaN ; NaN ];
Tc_error_1375  = [ NaN ; NaN ; NaN ];

%-- Image #1376:
omc_1376 = [ NaN ; NaN ; NaN ];
Tc_1376  = [ NaN ; NaN ; NaN ];
omc_error_1376 = [ NaN ; NaN ; NaN ];
Tc_error_1376  = [ NaN ; NaN ; NaN ];

%-- Image #1377:
omc_1377 = [ NaN ; NaN ; NaN ];
Tc_1377  = [ NaN ; NaN ; NaN ];
omc_error_1377 = [ NaN ; NaN ; NaN ];
Tc_error_1377  = [ NaN ; NaN ; NaN ];

%-- Image #1378:
omc_1378 = [ NaN ; NaN ; NaN ];
Tc_1378  = [ NaN ; NaN ; NaN ];
omc_error_1378 = [ NaN ; NaN ; NaN ];
Tc_error_1378  = [ NaN ; NaN ; NaN ];

%-- Image #1379:
omc_1379 = [ NaN ; NaN ; NaN ];
Tc_1379  = [ NaN ; NaN ; NaN ];
omc_error_1379 = [ NaN ; NaN ; NaN ];
Tc_error_1379  = [ NaN ; NaN ; NaN ];

%-- Image #1380:
omc_1380 = [ NaN ; NaN ; NaN ];
Tc_1380  = [ NaN ; NaN ; NaN ];
omc_error_1380 = [ NaN ; NaN ; NaN ];
Tc_error_1380  = [ NaN ; NaN ; NaN ];

%-- Image #1381:
omc_1381 = [ NaN ; NaN ; NaN ];
Tc_1381  = [ NaN ; NaN ; NaN ];
omc_error_1381 = [ NaN ; NaN ; NaN ];
Tc_error_1381  = [ NaN ; NaN ; NaN ];

%-- Image #1382:
omc_1382 = [ NaN ; NaN ; NaN ];
Tc_1382  = [ NaN ; NaN ; NaN ];
omc_error_1382 = [ NaN ; NaN ; NaN ];
Tc_error_1382  = [ NaN ; NaN ; NaN ];

%-- Image #1383:
omc_1383 = [ NaN ; NaN ; NaN ];
Tc_1383  = [ NaN ; NaN ; NaN ];
omc_error_1383 = [ NaN ; NaN ; NaN ];
Tc_error_1383  = [ NaN ; NaN ; NaN ];

%-- Image #1384:
omc_1384 = [ NaN ; NaN ; NaN ];
Tc_1384  = [ NaN ; NaN ; NaN ];
omc_error_1384 = [ NaN ; NaN ; NaN ];
Tc_error_1384  = [ NaN ; NaN ; NaN ];

%-- Image #1385:
omc_1385 = [ NaN ; NaN ; NaN ];
Tc_1385  = [ NaN ; NaN ; NaN ];
omc_error_1385 = [ NaN ; NaN ; NaN ];
Tc_error_1385  = [ NaN ; NaN ; NaN ];

%-- Image #1386:
omc_1386 = [ NaN ; NaN ; NaN ];
Tc_1386  = [ NaN ; NaN ; NaN ];
omc_error_1386 = [ NaN ; NaN ; NaN ];
Tc_error_1386  = [ NaN ; NaN ; NaN ];

%-- Image #1387:
omc_1387 = [ NaN ; NaN ; NaN ];
Tc_1387  = [ NaN ; NaN ; NaN ];
omc_error_1387 = [ NaN ; NaN ; NaN ];
Tc_error_1387  = [ NaN ; NaN ; NaN ];

%-- Image #1388:
omc_1388 = [ NaN ; NaN ; NaN ];
Tc_1388  = [ NaN ; NaN ; NaN ];
omc_error_1388 = [ NaN ; NaN ; NaN ];
Tc_error_1388  = [ NaN ; NaN ; NaN ];

%-- Image #1389:
omc_1389 = [ NaN ; NaN ; NaN ];
Tc_1389  = [ NaN ; NaN ; NaN ];
omc_error_1389 = [ NaN ; NaN ; NaN ];
Tc_error_1389  = [ NaN ; NaN ; NaN ];

%-- Image #1390:
omc_1390 = [ NaN ; NaN ; NaN ];
Tc_1390  = [ NaN ; NaN ; NaN ];
omc_error_1390 = [ NaN ; NaN ; NaN ];
Tc_error_1390  = [ NaN ; NaN ; NaN ];

%-- Image #1391:
omc_1391 = [ NaN ; NaN ; NaN ];
Tc_1391  = [ NaN ; NaN ; NaN ];
omc_error_1391 = [ NaN ; NaN ; NaN ];
Tc_error_1391  = [ NaN ; NaN ; NaN ];

%-- Image #1392:
omc_1392 = [ NaN ; NaN ; NaN ];
Tc_1392  = [ NaN ; NaN ; NaN ];
omc_error_1392 = [ NaN ; NaN ; NaN ];
Tc_error_1392  = [ NaN ; NaN ; NaN ];

%-- Image #1393:
omc_1393 = [ NaN ; NaN ; NaN ];
Tc_1393  = [ NaN ; NaN ; NaN ];
omc_error_1393 = [ NaN ; NaN ; NaN ];
Tc_error_1393  = [ NaN ; NaN ; NaN ];

%-- Image #1394:
omc_1394 = [ NaN ; NaN ; NaN ];
Tc_1394  = [ NaN ; NaN ; NaN ];
omc_error_1394 = [ NaN ; NaN ; NaN ];
Tc_error_1394  = [ NaN ; NaN ; NaN ];

%-- Image #1395:
omc_1395 = [ NaN ; NaN ; NaN ];
Tc_1395  = [ NaN ; NaN ; NaN ];
omc_error_1395 = [ NaN ; NaN ; NaN ];
Tc_error_1395  = [ NaN ; NaN ; NaN ];

%-- Image #1396:
omc_1396 = [ NaN ; NaN ; NaN ];
Tc_1396  = [ NaN ; NaN ; NaN ];
omc_error_1396 = [ NaN ; NaN ; NaN ];
Tc_error_1396  = [ NaN ; NaN ; NaN ];

%-- Image #1397:
omc_1397 = [ NaN ; NaN ; NaN ];
Tc_1397  = [ NaN ; NaN ; NaN ];
omc_error_1397 = [ NaN ; NaN ; NaN ];
Tc_error_1397  = [ NaN ; NaN ; NaN ];

%-- Image #1398:
omc_1398 = [ NaN ; NaN ; NaN ];
Tc_1398  = [ NaN ; NaN ; NaN ];
omc_error_1398 = [ NaN ; NaN ; NaN ];
Tc_error_1398  = [ NaN ; NaN ; NaN ];

%-- Image #1399:
omc_1399 = [ NaN ; NaN ; NaN ];
Tc_1399  = [ NaN ; NaN ; NaN ];
omc_error_1399 = [ NaN ; NaN ; NaN ];
Tc_error_1399  = [ NaN ; NaN ; NaN ];

%-- Image #1400:
omc_1400 = [ NaN ; NaN ; NaN ];
Tc_1400  = [ NaN ; NaN ; NaN ];
omc_error_1400 = [ NaN ; NaN ; NaN ];
Tc_error_1400  = [ NaN ; NaN ; NaN ];

%-- Image #1401:
omc_1401 = [ NaN ; NaN ; NaN ];
Tc_1401  = [ NaN ; NaN ; NaN ];
omc_error_1401 = [ NaN ; NaN ; NaN ];
Tc_error_1401  = [ NaN ; NaN ; NaN ];

%-- Image #1402:
omc_1402 = [ NaN ; NaN ; NaN ];
Tc_1402  = [ NaN ; NaN ; NaN ];
omc_error_1402 = [ NaN ; NaN ; NaN ];
Tc_error_1402  = [ NaN ; NaN ; NaN ];

%-- Image #1403:
omc_1403 = [ NaN ; NaN ; NaN ];
Tc_1403  = [ NaN ; NaN ; NaN ];
omc_error_1403 = [ NaN ; NaN ; NaN ];
Tc_error_1403  = [ NaN ; NaN ; NaN ];

%-- Image #1404:
omc_1404 = [ NaN ; NaN ; NaN ];
Tc_1404  = [ NaN ; NaN ; NaN ];
omc_error_1404 = [ NaN ; NaN ; NaN ];
Tc_error_1404  = [ NaN ; NaN ; NaN ];

%-- Image #1405:
omc_1405 = [ NaN ; NaN ; NaN ];
Tc_1405  = [ NaN ; NaN ; NaN ];
omc_error_1405 = [ NaN ; NaN ; NaN ];
Tc_error_1405  = [ NaN ; NaN ; NaN ];

%-- Image #1406:
omc_1406 = [ -1.859137e+000 ; -1.982853e+000 ; -6.659104e-001 ];
Tc_1406  = [ -8.661311e+001 ; -7.380207e+001 ; 1.492578e+002 ];
omc_error_1406 = [ 1.105800e-002 ; 1.376209e-002 ; 2.307650e-002 ];
Tc_error_1406  = [ 2.143983e+000 ; 2.365874e+000 ; 3.608070e+000 ];

