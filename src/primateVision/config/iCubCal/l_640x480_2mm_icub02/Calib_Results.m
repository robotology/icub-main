% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 440.617747678 ; 446.535418555 ];

%-- Principal point:
cc = [ 299.915788918 ; 236.355897057 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.352157545963597 ; 0.122923342799365 ; -0.002374670917328 ; -0.000510591319869 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.526723127326878 ; 1.582538709707412 ];

%-- Principal point uncertainty:
cc_error = [ 1.712367107063067 ; 1.424466950406060 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.009044414942705 ; 0.013726243319881 ; 0.000956930301037 ; 0.000938641118050 ; 0.000000000000000 ];

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
omc_1 = [ -2.089318e+000 ; -2.162603e+000 ; 3.125337e-001 ];
Tc_1  = [ -1.365810e+002 ; -1.156896e+002 ; 3.819066e+002 ];
omc_error_1 = [ 7.248038e-003 ; 7.156808e-003 ; 1.497516e-002 ];
Tc_error_1  = [ 2.989246e+000 ; 2.464155e+000 ; 3.258429e+000 ];

%-- Image #2:
omc_2 = [ -1.954805e+000 ; -1.928864e+000 ; -1.744277e-001 ];
Tc_2  = [ -1.352401e+002 ; -4.140331e+001 ; 3.676964e+002 ];
omc_error_2 = [ 6.207452e-003 ; 8.483979e-003 ; 1.303762e-002 ];
Tc_error_2  = [ 2.866437e+000 ; 2.418799e+000 ; 3.259781e+000 ];

%-- Image #3:
omc_3 = [ -2.083476e+000 ; -2.287173e+000 ; 4.794997e-001 ];
Tc_3  = [ -1.184343e+002 ; -1.852435e+002 ; 4.126808e+002 ];
omc_error_3 = [ 8.196281e-003 ; 6.174042e-003 ; 1.543477e-002 ];
Tc_error_3  = [ 3.321245e+000 ; 2.659192e+000 ; 3.425120e+000 ];

%-- Image #4:
omc_4 = [ -1.886531e+000 ; -1.916408e+000 ; 4.256987e-001 ];
Tc_4  = [ -6.557379e+001 ; -1.264901e+002 ; 4.279900e+002 ];
omc_error_4 = [ 7.015501e-003 ; 7.718353e-003 ; 1.326799e-002 ];
Tc_error_4  = [ 3.373273e+000 ; 2.715531e+000 ; 3.162447e+000 ];

%-- Image #5:
omc_5 = [ -2.039317e+000 ; -2.053386e+000 ; 2.042697e-001 ];
Tc_5  = [ -2.799743e+002 ; -1.083748e+002 ; 3.952750e+002 ];
omc_error_5 = [ 8.290512e-003 ; 6.923100e-003 ; 1.413324e-002 ];
Tc_error_5  = [ 3.168359e+000 ; 2.784351e+000 ; 3.996827e+000 ];

%-- Image #6:
omc_6 = [ 1.963297e+000 ; 2.023442e+000 ; 4.421586e-001 ];
Tc_6  = [ -1.927337e+002 ; -1.188935e+002 ; 3.287949e+002 ];
omc_error_6 = [ 7.689709e-003 ; 8.788795e-003 ; 1.605790e-002 ];
Tc_error_6  = [ 2.859987e+000 ; 2.306383e+000 ; 3.628651e+000 ];

%-- Image #7:
omc_7 = [ 2.113952e+000 ; 2.093787e+000 ; 1.744924e-001 ];
Tc_7  = [ -3.143755e+001 ; -1.163458e+002 ; 3.054669e+002 ];
omc_error_7 = [ 7.819934e-003 ; 6.447412e-003 ; 1.453661e-002 ];
Tc_error_7  = [ 2.488042e+000 ; 1.962203e+000 ; 2.788350e+000 ];

%-- Image #8:
omc_8 = [ 1.841056e+000 ; 1.875779e+000 ; -7.164049e-001 ];
Tc_8  = [ -1.303374e+002 ; -5.341388e+001 ; 4.019066e+002 ];
omc_error_8 = [ 5.510601e-003 ; 6.686483e-003 ; 1.127983e-002 ];
Tc_error_8  = [ 3.082799e+000 ; 2.592299e+000 ; 2.740246e+000 ];

%-- Image #9:
omc_9 = [ 1.891006e+000 ; 1.923365e+000 ; -6.973575e-001 ];
Tc_9  = [ -1.400830e+002 ; -1.642493e+002 ; 4.058402e+002 ];
omc_error_9 = [ 5.290738e-003 ; 7.444063e-003 ; 1.169445e-002 ];
Tc_error_9  = [ 3.250124e+000 ; 2.657210e+000 ; 3.073470e+000 ];

%-- Image #10:
omc_10 = [ -1.874025e+000 ; -1.852182e+000 ; -4.004869e-001 ];
Tc_10  = [ -1.276663e+002 ; -1.434355e+002 ; 3.107404e+002 ];
omc_error_10 = [ 6.274553e-003 ; 7.431671e-003 ; 1.200189e-002 ];
Tc_error_10  = [ 2.552578e+000 ; 2.182676e+000 ; 3.097223e+000 ];

%-- Image #11:
omc_11 = [ -1.937316e+000 ; -1.920297e+000 ; -2.695452e-001 ];
Tc_11  = [ -1.552846e+002 ; -6.269012e+001 ; 3.064915e+002 ];
omc_error_11 = [ 5.801989e-003 ; 7.615072e-003 ; 1.172875e-002 ];
Tc_error_11  = [ 2.407910e+000 ; 2.061477e+000 ; 2.910469e+000 ];

%-- Image #12:
omc_12 = [ -2.154356e+000 ; -2.153414e+000 ; 9.686461e-002 ];
Tc_12  = [ -1.413484e+002 ; -1.008892e+002 ; 3.942283e+002 ];
omc_error_12 = [ 8.314977e-003 ; 8.742428e-003 ; 1.782448e-002 ];
Tc_error_12  = [ 3.101344e+000 ; 2.579749e+000 ; 3.613503e+000 ];

%-- Image #13:
omc_13 = [ NaN ; NaN ; NaN ];
Tc_13  = [ NaN ; NaN ; NaN ];
omc_error_13 = [ NaN ; NaN ; NaN ];
Tc_error_13  = [ NaN ; NaN ; NaN ];

%-- Image #14:
omc_14 = [ -1.774198e+000 ; -1.616854e+000 ; -1.098415e-001 ];
Tc_14  = [ -1.465173e+002 ; -1.229812e+002 ; 3.596192e+002 ];
omc_error_14 = [ 6.166403e-003 ; 6.938721e-003 ; 1.005817e-002 ];
Tc_error_14  = [ 2.847280e+000 ; 2.382986e+000 ; 3.055899e+000 ];

%-- Image #15:
omc_15 = [ -1.653916e+000 ; -2.096213e+000 ; 5.574861e-001 ];
Tc_15  = [ -5.663239e+001 ; -1.343391e+002 ; 4.293152e+002 ];
omc_error_15 = [ 6.415011e-003 ; 7.678344e-003 ; 1.189164e-002 ];
Tc_error_15  = [ 3.368318e+000 ; 2.720531e+000 ; 2.995146e+000 ];

%-- Image #16:
omc_16 = [ -1.931527e+000 ; -2.216194e+000 ; -5.088626e-001 ];
Tc_16  = [ -1.684376e+002 ; -1.429732e+002 ; 2.423398e+002 ];
omc_error_16 = [ 6.152505e-003 ; 6.943100e-003 ; 1.314572e-002 ];
Tc_error_16  = [ 2.104966e+000 ; 1.837920e+000 ; 2.847501e+000 ];

%-- Image #17:
omc_17 = [ -1.781070e+000 ; -2.395436e+000 ; 3.766257e-001 ];
Tc_17  = [ -9.763695e+001 ; -1.647843e+002 ; 3.701559e+002 ];
omc_error_17 = [ 6.473184e-003 ; 7.278318e-003 ; 1.372044e-002 ];
Tc_error_17  = [ 2.934960e+000 ; 2.371419e+000 ; 3.110200e+000 ];

%-- Image #18:
omc_18 = [ 2.280132e+000 ; 1.908611e+000 ; -1.305002e-001 ];
Tc_18  = [ -2.066905e+002 ; -7.706351e+001 ; 3.282256e+002 ];
omc_error_18 = [ 6.922768e-003 ; 7.097634e-003 ; 1.447759e-002 ];
Tc_error_18  = [ 2.609800e+000 ; 2.216130e+000 ; 3.253393e+000 ];

%-- Image #19:
omc_19 = [ -2.089291e+000 ; -2.092421e+000 ; 1.297246e-001 ];
Tc_19  = [ -1.340819e+002 ; -1.029014e+002 ; 4.046629e+002 ];
omc_error_19 = [ 8.072738e-003 ; 8.761013e-003 ; 1.683191e-002 ];
Tc_error_19  = [ 3.171799e+000 ; 2.637204e+000 ; 3.588745e+000 ];

%-- Image #20:
omc_20 = [ 2.300577e+000 ; 2.092260e+000 ; 3.248152e-002 ];
Tc_20  = [ -2.307667e+002 ; -5.813886e+001 ; 3.738980e+002 ];
omc_error_20 = [ 9.180571e-003 ; 9.123789e-003 ; 1.882620e-002 ];
Tc_error_20  = [ 2.972858e+000 ; 2.566328e+000 ; 3.876589e+000 ];

%-- Image #21:
omc_21 = [ -1.836049e+000 ; -1.692573e+000 ; 3.780304e-001 ];
Tc_21  = [ -1.000118e+002 ; -1.097081e+002 ; 4.480969e+002 ];
omc_error_21 = [ 6.821274e-003 ; 7.071878e-003 ; 1.121183e-002 ];
Tc_error_21  = [ 3.506341e+000 ; 2.856889e+000 ; 3.144597e+000 ];

