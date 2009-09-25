% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 445.181089163 ; 450.692508487 ];

%-- Principal point:
cc = [ 319.804961823 ; 279.25887652];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.341922218669375 ; 0.128174344140444 ; 0.001760433715565 ; -0.000802228218203 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.066885158812874 ; 2.126330551148473 ];

%-- Principal point uncertainty:
cc_error = [ 2.700223074115929 ; 2.269462632568912 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013359521248231 ; 0.018412306784426 ; 0.001514168683325 ; 0.001267481463478 ; 0.000000000000000 ];

%-- Image size:
nx = 320;
ny = 240;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 21;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.054421e+000 ; -1.983832e+000 ; 2.279354e-001 ];
Tc_1  = [ -1.579332e+002 ; -1.497049e+002 ; 3.795563e+002 ];
omc_error_1 = [ 1.313056e-002 ; 1.147441e-002 ; 2.465117e-002 ];
Tc_error_1  = [ 4.757609e+000 ; 3.956927e+000 ; 4.655480e+000 ];

%-- Image #2:
omc_2 = [ 1.845598e+000 ; 2.065138e+000 ; -1.001534e+000 ];
Tc_2  = [ -1.161520e+002 ; -8.593947e+001 ; 4.511083e+002 ];
omc_error_2 = [ 9.161584e-003 ; 1.257831e-002 ; 1.981463e-002 ];
Tc_error_2  = [ 5.501193e+000 ; 4.621756e+000 ; 3.723409e+000 ];

%-- Image #3:
omc_3 = [ 2.005838e+000 ; 2.282422e+000 ; -7.572759e-001 ];
Tc_3  = [ -1.368146e+002 ; -2.329663e+002 ; 4.290654e+002 ];
omc_error_3 = [ 9.045028e-003 ; 1.416897e-002 ; 2.274975e-002 ];
Tc_error_3  = [ 5.682954e+000 ; 4.544334e+000 ; 4.606596e+000 ];

%-- Image #4:
omc_4 = [ -1.831389e+000 ; -1.858806e+000 ; -1.797257e-001 ];
Tc_4  = [ -1.410552e+002 ; -2.011626e+002 ; 3.374299e+002 ];
omc_error_4 = [ 1.288310e-002 ; 1.268619e-002 ; 2.207262e-002 ];
Tc_error_4  = [ 4.434628e+000 ; 3.826904e+000 ; 4.733324e+000 ];

%-- Image #5:
omc_5 = [ -1.906022e+000 ; -1.917254e+000 ; -5.068172e-002 ];
Tc_5  = [ -1.426945e+002 ; -9.118861e+001 ; 3.140094e+002 ];
omc_error_5 = [ 1.016749e-002 ; 1.233954e-002 ; 1.929920e-002 ];
Tc_error_5  = [ 3.865573e+000 ; 3.306320e+000 ; 3.902154e+000 ];

%-- Image #6:
omc_6 = [ -1.838046e+000 ; -1.885039e+000 ; 4.380735e-001 ];
Tc_6  = [ -9.783038e+001 ; -1.435947e+002 ; 4.022849e+002 ];
omc_error_6 = [ 1.134137e-002 ; 1.120939e-002 ; 1.988876e-002 ];
Tc_error_6  = [ 5.006644e+000 ; 4.071134e+000 ; 4.042017e+000 ];

%-- Image #7:
omc_7 = [ -1.868388e+000 ; -1.896663e+000 ; 4.069336e-001 ];
Tc_7  = [ -2.463514e+002 ; -1.447342e+002 ; 4.046153e+002 ];
omc_error_7 = [ 1.280109e-002 ; 9.951816e-003 ; 1.922032e-002 ];
Tc_error_7  = [ 5.201604e+000 ; 4.445711e+000 ; 5.018290e+000 ];

%-- Image #8:
omc_8 = [ 1.881299e+000 ; 1.768992e+000 ; 6.587663e-001 ];
Tc_8  = [ -1.746618e+002 ; -1.405741e+002 ; 2.290094e+002 ];
omc_error_8 = [ 1.085353e-002 ; 1.175637e-002 ; 1.868322e-002 ];
Tc_error_8  = [ 3.443880e+000 ; 2.692632e+000 ; 4.029392e+000 ];

%-- Image #9:
omc_9 = [ 2.088274e+000 ; 1.964530e+000 ; 3.862094e-001 ];
Tc_9  = [ -6.961502e+001 ; -1.366259e+002 ; 2.343435e+002 ];
omc_error_9 = [ 1.194122e-002 ; 1.035676e-002 ; 2.013340e-002 ];
Tc_error_9  = [ 3.090369e+000 ; 2.449574e+000 ; 3.374781e+000 ];

%-- Image #10:
omc_10 = [ -1.555015e+000 ; -2.211826e+000 ; 6.389454e-001 ];
Tc_10  = [ -5.703558e+001 ; -2.069040e+002 ; 4.237166e+002 ];
omc_error_10 = [ 1.167305e-002 ; 1.138070e-002 ; 1.994578e-002 ];
Tc_error_10  = [ 5.434959e+000 ; 4.279019e+000 ; 4.166261e+000 ];

%-- Image #11:
omc_11 = [ 1.813221e+000 ; 2.398497e+000 ; 8.500962e-001 ];
Tc_11  = [ -8.199136e+001 ; -1.459855e+002 ; 2.187799e+002 ];
omc_error_11 = [ 1.174962e-002 ; 1.124942e-002 ; 2.128445e-002 ];
Tc_error_11  = [ 3.047625e+000 ; 2.594422e+000 ; 3.596186e+000 ];

%-- Image #12:
omc_12 = [ -1.816402e+000 ; -1.737337e+000 ; -3.373535e-001 ];
Tc_12  = [ -1.914420e+002 ; -1.772151e+002 ; 3.505030e+002 ];
omc_error_12 = [ 1.283575e-002 ; 1.217136e-002 ; 2.051164e-002 ];
Tc_error_12  = [ 4.518762e+000 ; 4.022932e+000 ; 5.087995e+000 ];

%-- Image #13:
omc_13 = [ -1.928498e+000 ; -1.969073e+000 ; 2.831063e-001 ];
Tc_13  = [ -1.628307e+002 ; -1.495319e+002 ; 3.476830e+002 ];
omc_error_13 = [ 1.145033e-002 ; 1.010925e-002 ; 2.007536e-002 ];
Tc_error_13  = [ 4.367934e+000 ; 3.640112e+000 ; 4.175721e+000 ];

%-- Image #14:
omc_14 = [ -1.835238e+000 ; -2.033640e+000 ; -6.628227e-001 ];
Tc_14  = [ -1.491049e+002 ; -1.400342e+002 ; 2.282945e+002 ];
omc_error_14 = [ 9.902579e-003 ; 1.189569e-002 ; 1.946184e-002 ];
Tc_error_14  = [ 3.143573e+000 ; 2.806721e+000 ; 3.876245e+000 ];

%-- Image #15:
omc_15 = [ 2.170521e+000 ; 1.649891e+000 ; -3.443005e-001 ];
Tc_15  = [ -1.876652e+002 ; -1.151981e+002 ; 3.347509e+002 ];
omc_error_15 = [ 9.986860e-003 ; 1.158871e-002 ; 1.946732e-002 ];
Tc_error_15  = [ 4.179439e+000 ; 3.541883e+000 ; 4.250778e+000 ];

%-- Image #16:
omc_16 = [ -1.979223e+000 ; -1.508133e+000 ; 1.958907e-001 ];
Tc_16  = [ -1.875256e+002 ; -7.121601e+001 ; 3.475166e+002 ];
omc_error_16 = [ 1.091584e-002 ; 9.434603e-003 ; 1.644447e-002 ];
Tc_error_16  = [ 4.255012e+000 ; 3.651208e+000 ; 3.844859e+000 ];

%-- Image #17:
omc_17 = [ -2.085271e+000 ; -2.050661e+000 ; 1.122347e-001 ];
Tc_17  = [ -1.765627e+002 ; -1.346303e+002 ; 2.963348e+002 ];
omc_error_17 = [ 1.133410e-002 ; 9.959718e-003 ; 2.133614e-002 ];
Tc_error_17  = [ 3.733987e+000 ; 3.190558e+000 ; 4.114582e+000 ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ 1.994796e+000 ; 1.794841e+000 ; -4.072708e-001 ];
Tc_19  = [ -2.092482e+002 ; -1.100021e+002 ; 3.507537e+002 ];
omc_error_19 = [ 8.936560e-003 ; 1.226666e-002 ; 1.936420e-002 ];
Tc_error_19  = [ 4.335667e+000 ; 3.782374e+000 ; 4.404917e+000 ];

%-- Image #20:
omc_20 = [ -2.037305e+000 ; -1.515218e+000 ; -2.748547e-001 ];
Tc_20  = [ -1.922975e+002 ; -7.631017e+001 ; 3.091327e+002 ];
omc_error_20 = [ 1.121098e-002 ; 1.049260e-002 ; 1.748326e-002 ];
Tc_error_20  = [ 3.845028e+000 ; 3.379833e+000 ; 4.116130e+000 ];

%-- Image #21:
omc_21 = [ -1.739741e+000 ; -2.430042e+000 ; 4.404561e-001 ];
Tc_21  = [ -9.255736e+001 ; -2.040269e+002 ; 3.714822e+002 ];
omc_error_21 = [ 1.186790e-002 ; 1.172946e-002 ; 2.308476e-002 ];
Tc_error_21  = [ 4.752845e+000 ; 3.793108e+000 ; 4.270414e+000 ];

