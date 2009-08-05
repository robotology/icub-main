function [idx_tr,idx_te]=make_cv_splits(num_el,num_folds)
%[idx_tr,idx_te]=make_cv_splits(NUM_EL,NUM_FOLDS)
% Cross Validation splits, with NUM_FOLDS folds and for NUM_EL elements.
% It returns a cell, with list of indexes
% NOTE: It is NOT stratified cross validation!

for j=1:num_folds
    idx_tr{j}=[];
    idx_te{j}=j:num_folds:num_el;
    for i=1:num_folds
        if i==j
            continue;
        end
        idx_tr{j}=[idx_tr{j} i:num_folds:num_el];
    end
end