function model = skls_init(ker,kerparam,C)

model=k_init(ker,kerparam,C);

model.hess = 1e-5;
model.eta=0.01;