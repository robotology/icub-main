function model = silk_train_multi_cost(X,Y,model)
% variant for Cost-Sensitive classification

n = length(Y);

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
    model.costTot=0;
    model.numSV=[];
    model.aer=[];
    model.aco=[];
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
    model.eta2=model.eta;
    model.iter=model.iter+1;
    
    if numel(model.S)>0
        K_x=feval(model.ker,X(:,i),model.SV,model.kerparam);
        val_f=K_x*model.beta;
    else
        val_f=zeros(1,model.n_cla);
    end
    
    [mx_f,mx_idx]=max(val_f([1:Y(i)-1 Y(i)+1:model.n_cla]));
    if mx_idx>=Y(i)
        mx_idx=mx_idx+1;
    end

    model.errTot=model.errTot+(mx_f>val_f(Y(i)));
    rho=model.matErr(mx_idx,Y(i));
    model.costTot=model.costTot+rho;
    model.aer(model.iter)=model.errTot/model.iter;
    model.aco(model.iter)=model.costTot/model.iter;

    loss=mx_f-val_f(Y(i))+sqrt(rho);
    
    if loss>0
        fprintf('.');
        Kii=feval(model.ker,X(:,i),X(:,i),model.kerparam);
        new_beta=(rho+(1-model.tau)*(mx_f-val_f(Y(i))))/(2*Kii);
        new_beta=max(min(new_beta,(1-model.tau)*model.maxC),model.minC);
        model.beta(end+1,:)=zeros(1,model.n_cla);
        model.beta(end,Y(i))=new_beta;
        model.beta(end,mx_idx)=-new_beta;
        
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
        fprintf('P');
    end

    if model.maxSV==inf
        model.beta2=model.beta2+model.beta;
    end
    
    model.numSV(model.iter)=numel(model.S);
    
    if mod(i,100)==0
        fprintf('\n');
        %fprintf('.');
    end
    
    figure(1);
    plot(model.aer);
    figure(2);
    plot(model.numSV);
    drawnow;
end
fprintf('\n');
