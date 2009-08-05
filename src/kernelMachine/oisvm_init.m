function model = oisvm_init(ker,kerparam,C)
%OISVM_INIT    create an empty model for training with OISVM_TRAIN
%   MODEL = OISVM_INIT(KERNEL_FUNCTION,KERNEL_PARAMS,C)

model=k_init(ker,kerparam,C);

model.K=zeros(100,100);
model.hess = 1e-5;
model.eta=0.01;

model.x=0;
model.sv=[];
model.ss=0;
model.KbInv=[];