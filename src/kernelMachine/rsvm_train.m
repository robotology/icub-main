function model = rsvm_train(X,Y,model,bias)

if nargin<4
    bias = 1; 
end

iter_max=20;

model.X=X;
model.Y=Y;

n = length(Y);
d = length(model.S);

K_BI=feval(model.ker,X,model.S,1:n,model.kerparam);
K_BB=K_BI(:,model.S);

if bias
	K_BI=[K_BI ; ones(1,n)];
	K_BB=[K_BB , zeros(d,1); zeros(1,d+1)];
end	

if numel(model.S)==0
	sv=1:n;
	old_sv=[];
	iter=0;
	while ~isempty(setxor(sv,old_sv)) & (iter<iter_max)
        old_sv = sv;
	
        % the matrix could be singular so we add a small value to the elements on
        % the diagonal
        K_tmp=K_BI(:,sv);
        %beta=model.C*((K_BB+model.C*K_tmp*K_tmp'+1e-5*eye(size(K_BB)))\K_tmp*Y(sv)');
        beta=model.C*(pinv(K_BB(sv,sv)+model.C*K_tmp*K_tmp')*K_tmp*Y(sv)');
	
        out = Y .* (beta'*K_BI); 
        sv = find(out < 1);
        iter = iter + 1;
	end
	
	if ~isempty(setxor(sv,old_sv))
        fprintf('Warning: Max number of iterations reached!\n');
	end
else
	sv=1:n;
	old_sv=[];
	iter=0;
	while ~isempty(setxor(sv,old_sv)) & (iter<iter_max)
        old_sv = sv;
	
        % the matrix could be singular so we add a small value to the elements on
        % the diagonal
        K_tmp=K_BI(:,sv);
        %beta=model.C*((K_BB+model.C*K_tmp*K_tmp'+1e-5*eye(size(K_BB)))\K_tmp*Y(sv)');
        beta=model.C*(pinv(K_BB+model.C*K_tmp*K_tmp')*K_tmp*Y(sv)');
	
        out = Y .* (beta'*K_BI); 
        sv = find(out < 1);
        iter = iter + 1;
	end
	
	if ~isempty(setxor(sv,old_sv))
        fprintf('Warning: Max number of iterations reached!\n');
	end
end
        
if bias
    model.beta = beta(1:end-1);
	model.b = beta(end);
else
    model.beta = beta;
	model.b = 0;
end
    