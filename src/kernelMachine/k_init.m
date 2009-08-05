function model = k_init(ker,kerparam,C)
%K_INIT    create an empty model for training with KLS_TRAIN and RKLS_TRAIN
%   MODEL = K_INIT(KERNEL_FUNCTION,KERNEL_PARAMS,C)

model.K=[];
model.X=[];
model.Y=[];
model.S=[];
model.C=C;
model.ker=ker;
model.kerparam=kerparam;
model.opt=[];
