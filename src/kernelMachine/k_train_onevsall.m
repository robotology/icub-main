function models=k_train_onevsall(p,t,models,f_train)
% K_TRAIN_ONEVSALL Generic training algorithm for One-vs-All
%    models=k_train_onevsall(X,Y,models,f_train)

for i=1:numel(models)
    idxp=find(t==i);
    idxn=find(t~=i);
    tn=zeros(size(t));
    tn(idxp)=1;
    tn(idxn)=-1;
    models{i}=feval(f_train,p,tn,models{i});
end
