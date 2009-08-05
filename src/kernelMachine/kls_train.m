function [model, loo_err, loo_pred] = kls_train(X,Y,model,bias)
%KLS_TRAIN    Train a Kernel Least Square (LS-SVM)
%   MODEL = KLS_TRAIN(X,Y,MODEL)
%   [MODEL, LOO_ERR] = KLS_TRAIN(X,Y,MODEL)
%   [MODEL, LOO_ERR, LOO_PRED] = KLS_TRAIN(X,Y,MODEL)

if nargin<4
    bias = 1; 
end
n = length(Y);

model.X=X;
model.Y=Y;
model.S=[1:n];

model.K=feval(model.ker,X,1:n,1:n,model.kerparam);

if bias
	model.K=[model.K ones(n,1); ones(1,n) 0];
	
	if numel(model.C)==1
        id=1/model.C*eye(n+1); id(end,end)=0;
    else
        id=diag(1./model.C); id(end+1,end+1)=0;
    end
  	G=pinv(model.K+id);
  	x=G*[Y';0];
   
	model.beta = x(1:end-1);
	model.b = x(end);
	
	out=model.K(1:n,:)*x;
	err_cla=1-numel(find(Y'==sign(out)))/n;
	err=mean((Y'-out).^2);
	dd=diag(G);
	loo_pred=Y'-model.beta./dd(1:end-1);
else
	if numel(model.C)==1
        id=1/model.C*eye(n);
    else
        id=diag(1./model.C);
    end

    G=pinv(model.K+id);
	x=G*Y';
	
	model.beta = x;
	model.b = 0;
	
	out=model.K*x;
	err_cla=1-numel(find(Y'==sign(out)))/n;
	err=mean((Y'-out).^2);
	dd=diag(G);
	loo_pred=Y'-model.beta./dd;
end
err_cla_loo=1-numel(find( Y'==sign(loo_pred) ))/n;
err_loo=mean((Y'-loo_pred).^2);
fprintf('MSE on training set = %1.3f\tMSE LOO = %1.3f\n',err,err_loo);
fprintf('Cla. rate on traning set = %2.4f\tCla. rate LOO = %2.4f\n',(1-err_cla)*100,(1-err_cla_loo)*100);

loo_err(1)=err_loo;
loo_err(2)=err_cla_loo;

% for i=1:n
%     i
%     idx=[1:i-1 i+1:n+1];
%     xtmp=(model.K(idx,idx)+1/model.C*eye(n))\[Y([1:i-1 i+1:n]) ; 0];
%     outtmp(i)=model.K(i,idx)*xtmp;
% end
% outtmp