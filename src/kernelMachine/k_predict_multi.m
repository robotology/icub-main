function [out,vals] = k_predict_multi(models,x)
% K_PREDICT_MULTI Generic prediction for multiclass models
%    [out,vals] = k_predict_multi(models,X)

for i=1:numel(models)
    vals(i,:) = k_predict(x,models{i});
end

[tmp,out]=max(vals);