function [model,loo_pred,out] = rkls_train(X,Y,model)

model.X=X;
model.Y=Y;

n = length(Y);
d = length(model.S);

K_BI=feval(model.ker,X,model.S,1:n,model.kerparam);
K_BB=K_BI(:,model.S);

K_BI=[K_BI ; ones(1,n)];
K_BB=[K_BB , zeros(d,1); zeros(1,d+1)];

% the matrix could be singular so we add a small value to the elements on
% the diagonal
mat=pinv(1/model.C*K_BB+K_BI*K_BI')*K_BI;
x=mat*Y';

model.beta = x(1:end-1);
model.b = x(end);

out=x'*K_BI;
H=K_BI'*mat;
dH=1-diag(H)';
loo_pred=Y-(Y-out)./dH;