% demo.m
% X = MxN matrix, input points
%   M = dimension input space
%   N = number of training points
% Y = Nx1 vector, outputs/label

% sinc
p=-20:0.1:20;
t=sin(p+eps)./(p+eps)+.0*randn(size(p));

nn=numel(p);
idxperm=randperm(nn);
train_ratio=0.75;
ptrain=p(idxperm(1:round(nn*train_ratio)));
ttrain=t(idxperm(1:round(nn*train_ratio)));
ptest=p(idxperm(round(nn*train_ratio)+1:end));
ttest=t(idxperm(round(nn*train_ratio)+1:end));
[pn,meanp,stdp,tn,meant,stdt] = prestd(ptrain,ttrain);
[pn2] = trastd(ptest,meanp,stdp);
[tn2] = trastd(ttest,meant,stdt);
X=pn;
Y=tn;
Xtest=pn2;
Ytest=tn2;

d=size(X,1);
n=size(X,2);

% Parameters for the SVMs
C=1;
gamma=100; %good parameter for the sinc
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
end
fprintf('\nTime = %2.2f sec\n',toc);
model
out=k_predict(Xtest,model);
fprintf('MSE on test set = %2.4f\n',mean((out-Ytest).^2));
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
fprintf('MSE on test set = %2.4f\n',mean((out2-Ytest).^2));
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
fprintf('MSE on test set = %2.4f\n',mean((out3-Ytest).^2));