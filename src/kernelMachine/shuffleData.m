function [p,t]=shuffleData(p,t)

numSample=size(p,2);

%shuffling
%rand('state',0); %to have always the same random permutation
rp=randperm(numSample);

p=p(:,rp);
t=t(:,rp);