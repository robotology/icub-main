function model = skls_train(X,Y,model)
  
opt=model.opt;
if nargin<3
    opt = []; 
end;
n = length(Y);
  
% Set the parameters to their default value
if ~isfield(opt,'verb'),        opt.verb        = 1;            end; 
  
for curr=1:n
        
    model.X=[model.X , X(:,curr)];
    model.Y=[model.Y ; Y(curr)];
    %curr  
    %fprintf('.');
    
    ultimo=size(model.X,2);

    last_col=feval(model.ker,model.X,model.S,ultimo,model.kerparam);
    model.K=[model.K,[1 ; last_col]];
        
    model.hess = cholupdate(model.hess,sqrt(model.C)*model.K(1:length(model.hess),ultimo),'+');
        
    %mean((Y(1:size(K,1))-out)./(1-diag(G)))

    if (length(model.S)>0)
        colonna2=model.K(2:end,ultimo);
        %Kb=K(S,2:end);
        if (length(model.S)==1)
            model.KbInv=model.K(2:end,model.S)^-1;
        end
        amin=model.KbInv*colonna2;
        lasterr=feval(model.ker,X,curr,curr,model.kerparam)-colonna2'*amin;
        %err(curr)=lasterr;
    else
        lasterr=inf;
    end
        
    if (lasterr>model.eta)
        if (length(model.S)>0)
            model.KbInv=[model.KbInv, zeros(size(model.KbInv,1),1);zeros(1,size(model.KbInv,1)+1)];
            model.KbInv=model.KbInv+[amin; -1]*[amin; -1]'/lasterr;
        end

        model.S = [model.S ultimo];                
        riga=feval(model.ker,model.X,ultimo,1:size(model.K,2),model.kerparam);
        model.K=[model.K; riga];
        
        % Quando nn fa training però aggiunge sempre
        % quindi aggiorna + di una colonna di K
        d  = length(model.S);
        h = [0; model.K(d+1,model.S)'] + model.C * model.K(1:d+1,:) * model.K(d+1,:)';

        % The new Hessian would be [[old_hessian h2]; [h2' h3]]
        h2 = h(end,:);
        h2 = h2 + 1e-10*h2*eye(size(h,2)); % Ridge is only for numerical reason
        h3 = model.hess' \ h(1:d,:);
        h4 = sqrt(h2-h3'*h3);
        model.hess = [[model.hess h3]; [zeros(1,d) h4]];
        %fprintf('B');
    %else
        %fprintf('N');
    end
    
    %d = length(model.hess);
    %G = model.C*model.hess \ (model.hess' \ model.K(:,1:d)');
    G = model.C*model.hess \ (model.hess' \ model.K);
    x = G*model.Y(1:size(model.K,2));
    %out=model.K*x;
    %Hii=1./(1-diag(model.K*G));
    %err=(model.Y(1:size(model.K,1))-out);
    %mean(err.^2)
    %mean((err.*Hii).^2)

end

model.beta = x(2:end);
model.b = x(1);