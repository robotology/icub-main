function [out_pred,out_var] = k_predict(x,model)
% K_PREDICT Generic Prediction function
%    out = k_predict(x,model)

if iscell(model)==1
    [out_pred,out_var] = k_predict_multi(model,x);
end

nt = size(x,2);
if isfield(model,'SV')
    K = feval(model.ker,x,model.SV, model.kerparam);
else
    n = length(model.Y);
    K = feval(model.ker,[model.X ,x],(1:nt)+n,model.S,model.kerparam);
end

out_pred = (K*model.beta+model.b)';

if size(model.beta,2)>1
    [out_var,out_pred]=max(out_pred);
elseif nargout>1
    Kbb=feval(model.ker,model.X,model.S,model.S,model.kerparam);
    for i=1:size(x,2)
        out_var(i) = feval(model.ker,x,i,i,model.kerparam);
    end
    out_var=out_var+1/model.C-diag(K*pinv(Kbb+1/model.C)*K')';
end