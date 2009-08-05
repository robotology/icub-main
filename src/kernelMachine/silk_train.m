function model = silk_train(X,Y,model)
% SILK_TRAIN Sparse Implicit Online Learning with Kernels
%    model = silk_train(X,Y,model)
%    Note that the hinge loss is used.
%    With proper parameters it can emulate Perceptron, NORMA and
%    Passive-Aggressive.

n = length(Y);

if isfield(model,'rho')==0
    model.rho=1;
end

if isfield(model,'eta0')==0
    model.eta0=1;
end

if isfield(model,'iter')==0
    model.iter=0;
    model.beta=[];
    model.beta2=[];
    model.SV=[];
    model.Y_S=[];
    model.errTot=0;
    model.numSV=[];
    model.aer=[];
    model.b=0;
end

if isfield(model,'tau')==0
    model.tau=model.eta0*model.lambda/(1+model.eta0*model.lambda);
end

if isfield(model,'maxSV')==0
    model.maxSV=inf;
end

for i=1:n
    %model.eta2=sqrt(model.eta/(model.eta+model.iter));
    %model.eta=model.eta0;
    model.iter=model.iter+1;
        
    if numel(model.S)>0
        K_f=feval(model.ker,X(:,i),model.SV,model.kerparam);
        val_f=K_f*model.beta;
    else
        val_f=0;
    end

    model.errTot=model.errTot+(sign(val_f)~=Y(i));
    model.aer(model.iter)=model.errTot/model.iter;
    
    model.beta=model.beta*(1-model.tau);
    if Y(i)*val_f<=model.rho
        %fprintf('.');
        Kii=feval(model.ker,X(:,i),X(:,i),model.kerparam);
        new_beta=(model.rho-(1-model.tau)*val_f*Y(i))/Kii;
        model.beta(end+1,1)=Y(i)*max(min(new_beta,(1-model.tau)*model.maxC),model.minC);
        model.S(end+1)=model.iter;
        model.SV(:,end+1)=X(:,i);
        model.Y_S(end+1)=Y(i);
        
        if model.maxSV==inf
            model.beta2(end+1,1)=0;
        end
        
        if size(model.SV,2)>model.maxSV
            [mn,mn_idx]=min(abs(model.beta));
            model.beta(mn_idx)=[];
            model.SV(:,mn_idx)=[];
            model.Y_S(mn_idx)=[];
            model.S(mn_idx)=[];
        end
    else
        %fprintf('P');
    end

    if model.maxSV==inf
        model.beta2=model.beta2+model.beta;
    end
    
    model.numSV(model.iter)=numel(model.S);
    
    if mod(i,1000)==0
        %fprintf('\n');
        fprintf('.');
    end
    
%     figure(1);
%     plot(model.aer);
%     figure(2);
%     plot(model.numSV);
%     drawnow;
    
    %(model.beta'*compute_kernel(model.SV,1:numel(model.S),1:numel(model.S),model.kerparam)*model.beta)/i
end
fprintf('\n');