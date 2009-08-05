% Kernel Machines for Matlab
% Version 1.1, October 26 2007
% Copyright (c) Francesco Orabona.
%
% Please refer to your Matlab documentation on how to add Kernel Machines
% to your Matlab search path. (One place to start is the path-command)
%
% Kernel Machines training algorithm:
%   PERC_TRAIN           - Kernel Perceptron
%   ALMA2_TRAIN          - Kernel Alma-2
%   KLS_TRAIN            - Kernel Least Square
%   RKLS_TRAIN           - Random Kernel Least Square
%   SKLS_TRAIN           - Sparse Kernel Least Square
%   RSVM_TRAIN           - Reduced SVM
%   OISVM_TRAIN          - On-line Independent SVM
%   SILK_TRAIN           - Sparse Implicit Online Learning with Kernels
%   FORGETRON_TRAIN      - On-line Forgetron Algorithm
%   PROJECTRON_TRAIN     - On-line Projectron Algoritm
%   PROJECTRON_TRAIN_NEW - On-line Projectron++ Algoritm
%
% Inizialization functions:
%   K_INIT               - General inizializiation function
%   SKLS_INIT            - Inizializiation function for SKLS_TRAIN
%   OISVM_INIT           - Inizializiation function for OISVM_TRAIN
%
% Test functions:
%   K_PREDICT            - General prediction function
%
% Miscellaneous:
%   COMPUTE_KERNEL       - Calculate the kernel matrix
%   KERNEL_ALIGNMENT     - Calculate the alignment between two kernel matrices
%   SHUFFLEDATA          - Shuffle input and output data
%   DEMO_CLASSIF         - Demo of many classification algorithms
%   DEMO_SINC            - Demo of many regression algorithms on the 'sinc'