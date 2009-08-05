% demo.m
% X = MxN matrix, input points
%   M = dimension input space
%   N = number of training points
% Y = Nx1 vector, outputs/label

% adult1
[t, p] = read_sparse('a1a.txt');
p=full(p);
p=p';
t=t';

nn=size(p,2);
idxperm=randperm(nn);
train_ratio=0.75;
ptrain=p(:,idxperm(1:round(nn*train_ratio)));
ttrain=t(idxperm(1:round(nn*train_ratio)));
ptest=p(:,idxperm(round(nn*train_ratio)+1:end));
ttest=t(idxperm(round(nn*train_ratio)+1:end));
[pn,meanp,stdp] = prestd(ptrain,ttrain);
[pn2] = trastd(ptest,meanp,stdp);
X=pn;
Y=ttrain;
Xtest=pn2;
Ytest=ttest;

d=size(X,1);
n=size(X,2);

% Parameters for the SVMs
C=1;
gamma=.1/d; %good parameter for the sinc
hp.type = 'rbf';
hp.gamma = gamma;

% Sparse LS-SVM Online
fprintf('-----------------------------------------------------\n');
model=skls_init(@compute_kernel,hp,C);
model.eta=0.01;
fprintf('Sparse LS-SVM Online\n');
fprintf('eta=%2.2f\n',model.eta);
tic;
for i=1:n
    model = skls_train(X(:,i),Y(i),model);
    fprintf('.');
    if (mod(i,100)==0)
        fprintf('\n%d (%d vectors in the base)\n',i,length(model.S));
    end
end
fprintf('\nTime = %2.2f sec\n',toc);
model
out=k_predict(Xtest,model);
fprintf('Cla. rate on test set = %2.4f\n',numel(find(sign(out)==Ytest))/numel(Ytest)*100);
fprintf('\n\n\n');

% LS-SVM Batch
fprintf('-----------------------------------------------------\n');
model2=k_init(@compute_kernel,hp,C);
fprintf('LS-SVM Batch\n');
tic;
model2 = kls_train(X,Y,model2);
fprintf('\nTime = %2.2f sec\n',toc);
model2
out2=k_predict(Xtest,model2);
fprintf('Cla. rate on test set = %2.4f\n',numel(find(sign(out2)==Ytest))/numel(Ytest)*100);
fprintf('\n\n\n');

% Random LS-SVM Batch
fprintf('-----------------------------------------------------\n');
model3=k_init(@compute_kernel,hp,C);
% Select as bases some random training points
rp=randperm(numel(Y));
model3.S=rp(1:numel(model.S));
fprintf('Random LS-SVM Batch\n');
tic;
model3 = rkls_train(X,Y,model3);
fprintf('\nTime = %2.2f sec\n',toc);
model3
out3=k_predict(Xtest,model3);
fprintf('Cla. rate on test set = %2.4f\n',numel(find(sign(out3)==Ytest))/numel(Ytest)*100);

% OISVM Batch
fprintf('-----------------------------------------------------\n');
model4=oisvm_init(@compute_kernel,hp,C);
fprintf('OISVM Batch\n');
tic;
model4 = oisvm_train(X,Y,model4);
fprintf('\nTime = %2.2f sec\n',toc);
model4
out4=k_predict(Xtest,model4);
fprintf('Cla. rate on test set = %2.4f\n',numel(find(sign(out4)==Ytest))/numel(Ytest)*100);

% RSVM Batch
fprintf('-----------------------------------------------------\n');
model5=k_init(@compute_kernel,hp,C);
% Select as bases some random training points
rp=randperm(numel(Y));
model5.S=rp(1:numel(model.S));
fprintf('Random SVM Batch\n');
tic;
model5 = rsvm_train(X,Y,model5);
fprintf('\nTime = %2.2f sec\n',toc);
model5
out5=k_predict(Xtest,model5);
fprintf('Cla. rate on test set = %2.4f\n',numel(find(sign(out5)==Ytest))/numel(Ytest)*100);