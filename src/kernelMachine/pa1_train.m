function model = pa1_train(X,Y,model)
% PA1_TRAIN Passive Aggressive 1
%    model = pa1_train(X,Y,model)

model.rho=1;
model.tau=0;
model.maxC=model.C;
model.minC=0;

model=silk_train(X,Y,model);