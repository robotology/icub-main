function model = rsvm_train_K(K_BI,Y,model)

iter_max=20;

model.Y=Y;

n = length(Y);
d = length(model.S);

%K_BI=feval(model.ker,X,model.S,1:n,model.kerparam);
K_BB=K_BI(:,model.S);

K_BI=[ones(1,n) ; K_BI];
K_BB=[zeros(1,d+1) ; zeros(d,1), K_BB];

sv=1:n;
old_sv=[];
iter=0;
while ~isempty(setxor(sv,old_sv)) & (iter<iter_max)
    old_sv = sv;

    % the matrix could be singular so we add a small value to the elements on
    % the diagonal
    K_tmp=K_BI(:,sv);
    beta=model.C*((K_BB+model.C*K_tmp*K_tmp'+1e-8*eye(size(K_BB)))\K_tmp*Y(sv)');
    
    out = Y .* (beta'*K_BI); 
    sv = find(out < 1);
    iter = iter + 1;
end
    
model.beta = beta(2:end);
model.b = beta(1);