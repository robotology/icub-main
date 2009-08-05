function model = projectron_train_multi(X,Y,model)
% PROJECTRON_TRAIN_MULTI Projectron
%    model = projectron_train_multi(X,Y,model)
%
% Projectron++ variant

n = length(Y);

if isfield(model,'U')==0
    model.U=1;
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
    for i=1:model.n_cla
        model.Kinv{i}=[];
    end
    model.Y_S=[];
    model.b=0;
end

n_skip=0;
n_proj1=0;
n_proj2=0;
n_pred=0;
idx_true=[];
idx_wrong=[];

for i=1:n
        
    model.iter=model.iter+1;
        
    if numel(model.S)>0
        K_f=feval(model.ker,X(:,i),model.SV,model.kerparam);
        val_f=full(K_f*model.beta);
        %val_f2=K_f*model.beta2;
    else
        val_f=zeros(1,model.n_cla);
        %val_f2=0;
        K_f=[];
    end

    Yi=Y(i);
    
    tmp=val_f; tmp(Yi)=-inf;
    [mx_val,idx_mx_val]=max(tmp);

    model.errTot=model.errTot+(val_f(Yi)<=mx_val);
    model.aer(model.iter)=model.errTot/model.iter;

    %model.errTotAv=model.errTotAv+(sign(val_f2)~=Y(i));
    %model.aerAv(model.iter)=model.errTotAv/model.iter;
    
    if val_f(Yi)<=mx_val+1 && val_f(Yi)>mx_val %Margin error
        loss=(1-val_f(Yi)+mx_val);
        Kii=full(feval(model.ker,X(:,i),X(:,i),model.kerparam));
        
        delta_true=Kii;
        delta_wrong=Kii;
        
        if numel(model.S)>0
            idx_true=find(model.beta(:,Yi)~=0);
            idx_wrong=find(model.beta(:,idx_mx_val)~=0);

            if numel(idx_true)>0
                coeff_true=K_f(idx_true)*model.Kinv{Yi};
                delta_true=max(Kii-coeff_true*K_f(idx_true)',0);
            end

            if numel(idx_wrong)>0
                coeff_wrong=K_f(idx_wrong)*model.Kinv{idx_mx_val};
                delta_wrong=max(Kii-coeff_wrong*K_f(idx_wrong)',0);
            end
        end
        
        tau=min(loss/(2*Kii-(delta_true+delta_wrong)),model.C);
        gain=tau*(2*loss-tau*(2*Kii-(delta_true+delta_wrong))-2*sqrt(delta_true+delta_wrong)*model.U);
        
        if gain>=0
            model.beta(idx_true,Yi)=model.beta(idx_true,Yi)+tau*coeff_true';
            model.beta(idx_wrong,idx_mx_val)=model.beta(idx_wrong,idx_mx_val)-tau*coeff_wrong';
            n_proj2=n_proj2+1;
        else
            n_skip=n_skip+1;
        end
    elseif val_f(Yi)<=mx_val %Mistake
        loss=(1-val_f(Yi)+mx_val);
        Kii=full(feval(model.ker,X(:,i),X(:,i),model.kerparam));
        
        delta_true=Kii;
        delta_wrong=Kii;
        vec=spalloc(1,model.n_cla,2);
        
        if numel(model.S)>0
            idx_true=find(model.beta(:,Yi)~=0);
            idx_wrong=find(model.beta(:,idx_mx_val)~=0);

            if numel(idx_true)>0
                coeff_true=K_f(idx_true)*model.Kinv{Yi};
                delta_true=max(Kii-coeff_true*K_f(idx_true)',0);
            end

            if numel(idx_wrong)>0
                coeff_wrong=K_f(idx_wrong)*model.Kinv{idx_mx_val};
                delta_wrong=max(Kii-coeff_wrong*K_f(idx_wrong)',0);
            end
        end

        % 3 different possibilities for the projection step:
        
        % double proj
        tau(1)=min(loss/(2*Kii-(delta_true+delta_wrong)),model.C);
        gain(1)=tau(1)*(2*loss-tau(1)*(2*Kii-(delta_true+delta_wrong))-2*sqrt(delta_true+delta_wrong)*model.U);
        
        % proj true only
        tau(2)=min(loss/(2*Kii-delta_true),model.C);
        gain(2)=tau(2)*(2*loss-tau(2)*(2*Kii-delta_true)-2*sqrt(delta_true)*model.U);
        
        % proj wrong only
        tau(3)=min(loss/(2*Kii-delta_wrong),model.C);
        gain(3)=tau(3)*(2*loss-tau(3)*(2*Kii-delta_wrong)-2*sqrt(delta_wrong)*model.U);
             
        [mx,idx]=max(gain); % take the projection step with better gain
        if mx>=0.5 %check if the gain it greater than 0.5
            tau_opt=tau(idx);
        else
            % do a normal update step
            idx=4;
            tau_opt=min(loss/(2*Kii),model.C);
        end
            
        if idx==1 || idx==2
            if numel(idx_true)>0
                model.beta(idx_true,Yi)=model.beta(idx_true,Yi)+tau_opt*coeff_true';
            end
        else
            vec(Yi)=tau_opt;
            if numel(model.Kinv{Yi})~=0
               tmp=[model.Kinv{Yi}, zeros(size(model.Kinv{Yi},1),1);zeros(1,size(model.Kinv{Yi},1)+1)];
               tmp=tmp+[coeff_true'; -1]*[coeff_true'; -1]'/delta_true;
            else
               tmp=full(Kii^-1);
            end
            model.Kinv{Yi}=tmp;
        end            

        if idx==1 || idx==3
            if numel(idx_wrong)>0
                model.beta(idx_wrong,idx_mx_val)=model.beta(idx_wrong,idx_mx_val)-tau_opt*coeff_wrong';
            end
        else
            vec(idx_mx_val)=-tau_opt;
            if numel(model.Kinv{idx_mx_val})~=0
               tmp=[model.Kinv{idx_mx_val}, zeros(size(model.Kinv{idx_mx_val},1),1);zeros(1,size(model.Kinv{idx_mx_val},1)+1)];
               tmp=tmp+[coeff_wrong'; -1]*[coeff_wrong'; -1]'/delta_wrong;
            else
               tmp=full(Kii^-1);
            end
            model.Kinv{idx_mx_val}=tmp;
        end
            
        if idx~=1
            model.Y_S(end+1)=Yi;
            model.beta(end+1,:)=vec;
            model.S(end+1)=model.iter;
            model.SV(:,end+1)=X(:,i);
            model.beta2(end+1,1)=0;
        else
            n_proj1=n_proj1+1;
        end
    else
        n_pred=n_pred+1;
    end
    
    model.beta2=model.beta2+model.beta;
    
    model.numSV(model.iter)=numel(model.S);
    
    if mod(i,1000)==0
        fprintf('#%d SV:%g(%d)\tpred:%g\tskip:%g\tproj1:%g\tproj2:%g\tAER:%g\n', ...
            ceil(i/1000),numel(model.S)/i*100,numel(model.S),n_pred/i*100,n_skip/i*100,n_proj1/i*100,n_proj2/i*100,model.aer(end)*100);
    end
end
fprintf('\n');