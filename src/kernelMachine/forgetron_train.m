function model = forgetron_train(X,Y,model)
% Forgetron_TRAIN Forgetron
%    model = forgetron_train(X,Y,model)

n = length(Y);

if isfield(model,'iter')==0
    model.iter=0;
    model.beta=[];
    model.SV=[];
    model.Y_S=[];
    model.errTot=0;
    model.numSV=[];
    model.aer=[];
    model.out=[];
    model.Q=0;
end

if isfield(model,'maxSV')==0
    model.maxSV=numel(Y)/10;
end

U=0.25*sqrt((model.maxSV+1)/log(model.maxSV+1));
phi0=(model.maxSV+1)^(-1/(2*(model.maxSV+1)));

for i=1:n
    model.iter=model.iter+1;
        
    if numel(model.S)>0
        K_f=feval(model.ker,X(:,i),model.SV,model.kerparam);
        val_f=K_f*model.beta;
    else
        val_f=0;
        K_f=[];
    end

    model.errTot=model.errTot+(sign(val_f)~=Y(i));
    model.aer(model.iter)=model.errTot/model.iter;
    
    if Y(i)*val_f<=0
        %fprintf('.');
        model.beta(end+1,1)=Y(i);
        model.S(end+1)=model.iter;
        model.SV(:,end+1)=X(:,i);
        model.Y_S(end+1)=Y(i);
        
        norma_w=sqrt(model.beta'*feval(model.ker,model.SV,model.SV,model.kerparam)*model.beta);
        phi=min(phi0,U/norma_w);

        model.beta=model.beta*phi;
        
        if numel(model.S) > model.maxSV
            model.beta(1)=[];
            model.S(1)=[];
            model.SV(:,1)=[];
            model.Y_S(1)=[];
        end
    else
        %fprintf('P');
    end
    
    model.numSV(model.iter)=numel(model.S);
    
    if mod(i,1000)==0
        %fprintf('\n');
        fprintf('.');
    end
    
    figure(1);
    plot(model.aer,'r');
    figure(2);
    plot(model.numSV,'r');
    drawnow;
    
    %(model.beta'*compute_kernel(model.SV,1:numel(model.S),1:numel(model.S),model.kerparam)*model.beta)/i
end
fprintf('\n');