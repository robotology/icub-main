function model = projectron_train_new(X,Y,model)
% PROJECTRON_TRAIN_NEW Projectron
%    model = projectron_train_new(X,Y,model)

% fissato \eta ottengo che la massima norma di u e' 0.25/sqrt(\eta)

n = length(Y);

if isfield(model,'eta')==0
    model.eta=.1;
end

if isfield(model,'iter')==0
    model.iter=0;
    model.beta=[];
    model.beta2=[];
    model.SV=[];
    model.errTot=0;
    model.errTotAv=0;
    model.numSV=[];
    model.aer=[];
    model.aerAv=[];
    model.Kinv=[];
    model.b=0;
end

if isfield(model,'U')==0
    model.U=0.25*sqrt((model.maxSV+1)/log(model.maxSV+1));
end

n_skip=0;
n_proj1=0;
n_proj2=0;
n_pred=0;
delta=inf;
delta_max=0;

for i=1:n
    model.iter=model.iter+1;
        
    if numel(model.S)>0
        K_f=feval(model.ker,X(:,i),model.SV,model.kerparam);
        val_f=K_f*model.beta;
        val_f2=K_f*model.beta2;
    else
        val_f=0;
        val_f2=0;
        K_f=[];
    end

    model.errTot=model.errTot+(sign(val_f)~=Y(i));
    model.aer(model.iter)=model.errTot/model.iter;

    model.errTotAv=model.errTotAv+(sign(val_f2)~=Y(i));
    model.aerAv(model.iter)=model.errTotAv/model.iter;
    

    if Y(i)*val_f<=1
        loss=(1-Y(i)*val_f);
        Kii=feval(model.ker,X(:,i),X(:,i),model.kerparam);
        tau=min(loss/Kii,1);

        if size(model.SV,2)>0
            coeff=K_f*model.Kinv;
            delta=Kii-coeff*K_f';
            norm_xt_square=Kii-delta;
            tau_p=min(loss/norm_xt_square,1);
            if Y(i)*val_f<=0
                delta_max=((2*loss*tau_p-tau_p^2*norm_xt_square-0.5)/(2*model.U))^2;
            else
                delta_max=((2*loss*tau_p-tau_p^2*norm_xt_square)/(2*model.U))^2;
            end
        end

        if delta<delta_max
            %fprintf('U');
            model.beta=model.beta+Y(i)*tau_p*coeff';
            if loss<1
                n_proj2=n_proj2+1;
            else
                n_proj1=n_proj1+1;
            end
        elseif loss>=1
            %fprintf('.');
            model.beta(end+1,1)=Y(i)*tau;
            model.S(end+1)=model.iter;

            model.SV(:,end+1)=X(:,i);

            model.beta2(end+1,1)=0;

            if numel(model.Kinv)~=0
                tmp=[model.Kinv, zeros(numel(model.S)-1,1);zeros(1,numel(model.S))];
                tmp=tmp+[coeff'; -1]*[coeff'; -1]'/delta;
            else
                tmp=feval(model.ker,model.SV,model.SV,model.kerparam)^-1;
            end
            model.Kinv=tmp;
        else
            n_skip=n_skip+1;
            %fprintf('s');
        end
    else
        n_pred=n_pred+1;
        %fprintf('P');
    end

    model.beta2=model.beta2+model.beta;
    
    model.numSV(model.iter)=numel(model.S);
    
    if mod(i,1000)==0
        fprintf('#%d SV:%g(%d)\tpred:%g\tskip:%g\tproj1:%g\tproj2:%g\tAER:%g\n', ...
            ceil(i/1000),numel(model.S)/i*100,numel(model.S),n_pred/i*100,n_skip/i*100,n_proj1/i*100,n_proj2/i*100,model.aer(end)*100);
        %fprintf('\n');
        %fprintf('.');
    end
    
%     figure(1);
%     plot(model.aer,'r');
%     figure(2);
%     plot(model.numSV,'r');
%     drawnow;
end
fprintf('\n');