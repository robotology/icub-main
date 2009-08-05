function model = perc_train(X,Y,model)
% PERC_TRAIN Perceptron with Kernels
%    model = perc_train(X,Y,model)

model.rho=0;
model.tau=0;
model.minC=1;
model.maxC=1;

model=silk_train(X,Y,model);