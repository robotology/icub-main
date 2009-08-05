function [model] = oisvm_train(X,Y,model)
% in input: numero righe=num feature; num colonne=num sample
% versione incrementale vera!
% versione con memorizzazione del Kernel solo, senza prodotto con la Y
% eliminata la Y dall'hessiano
% aggiornamento con cholesky sempre

  opt=model.opt;  
  C=model.C;
  ker=model.ker;
  kerparam=model.kerparam;

  n = length(Y);
  
  % Set the parameters to their default value
  if ~isfield(opt,'maxiter'),     opt.maxiter     = 20;           end;
  if ~isfield(opt,'verb'),        opt.verb        = 1;            end; 
  if ~isfield(opt,'rows_mem'),     opt.rows_mem     = 100;          end; 
  if ~isfield(opt,'cols_mem'),     opt.cols_mem     = 100;          end;
  
  maxRow=size(model.K,1);
  maxCol=size(model.K,2);
  %K = zeros(opt.rows_mem,opt.cols_mem); 
  %K=model.K;
  %KbInv=model.KbInv;
  
  % hess is the Cholesky decompostion of the Hessian
  %hess = sqrt(C*n)*(1+1e-10);
  %hess = sqrt(C*1)*(1+1e-10);
  %hess = 1e-5;
  %hess=model.hess;
  
  %x = 0;
  %sv = [];
  %ss = 0;
  %x=model.x;
  %sv=model.sv;
  %ss=model.ss;
  
  %S = []; % Vettori selezionati finora come basi
  %S=model.S;
  %te = zeros(1,n);
  %time = zeros(1,n);
  %obj = zeros(1,n);
  %nsv = zeros(1,n);
  
  %pred=[];
    
  for curr=1:n
        
    dimS=length(model.S);
    
    model.X=[model.X , X(:,curr)];
    model.Y=[model.Y , Y(curr)];
    ultimo=size(model.X,2);

    if (mod(curr,100)==0)
        fprintf('\n%d (%d vectors in the base)\n',ultimo,length(model.S));
    end
    
    %last_col=[Y(curr); feval(ker,X,S,curr,kerparam) * Y(curr)];
    last_col=[Y(curr); feval(ker,model.X,model.S,ultimo,kerparam) * Y(curr)];
    %K(1:dimS+1,curr)=last_col;
    model.K(1:dimS+1,ultimo)=last_col;
    if ultimo==maxCol
        tmp=zeros([size(model.K,1) size(model.K,2)+opt.cols_mem]);
        tmp(1:dimS+1,1:ultimo)=model.K(1:dimS+1,1:ultimo);
        %tmp(:,1:curr)=K(:,1:curr); %copia anke gli zeri della base, con
        %memcpy forse è + veloce, se la matrice è memorizzata x colonne
        model.K=tmp;
        maxCol=size(model.K,2);
    end
    
    if (dimS>0)
        colonna2=model.K(2:dimS+1,ultimo)*Y(curr);
        %Kb=K(S,2:end);
        %amin=(Kb)^-1*colonna2';
        amin=model.KbInv*colonna2;
        lasterr=feval(ker,X,curr,curr,kerparam)-colonna2'*amin;
        
        if (lasterr>model.eta)
            model.KbInv=[model.KbInv, zeros(dimS,1);zeros(1,dimS+1)];
            model.KbInv=model.KbInv+[amin; -1]*[amin; -1]'/lasterr;

            model.S = [model.S ultimo];
            riga=feval(ker,model.X,ultimo,1:ultimo,kerparam).*model.Y(1:ultimo);
            model.K(dimS+2,1:ultimo)=riga;
            
            d  = length(model.S);
            h = [0; model.K(d+1,model.S)'.*model.Y(model.S)'] + C * model.K(1:d+1,model.sv) * model.K(d+1,model.sv)';
            h2 = h(end,:);
            h2 = h2 + 1e-10*h2*eye(size(h,2)); % Ridge is only for numerical reason
            h3 = model.hess' \ h(1:d,:);
            h4 = sqrt(h2-h3'*h3);
            model.hess = [[model.hess h3]; [zeros(1,d) h4]];
            
            model.ss=[model.ss;sum(riga(model.sv))];
            if dimS+2==maxRow
                tmp=zeros([size(model.K,1)+opt.rows_mem size(model.K,2)]);
                tmp(1:size(model.K,1),1:ultimo)=model.K(1:size(model.K,1),1:ultimo);
                model.K=tmp;
                maxRow=size(model.K,1);
            end
            %fprintf('B');
%         else
%             fprintf('N');
        end
    else
        model.S = ultimo;
        %riga=feval(ker,X,curr,1:dimS+1,kerparam).*Y(1:curr);
        riga=feval(ker,model.X,ultimo,1:ultimo,kerparam).*model.Y(1:ultimo)';
        model.K(2,1:ultimo)=riga;
        %update_hess2(S,Y,C);
        model.hess=[1e-5,0;0,sqrt(model.K(2,1)*Y(1))];
        model.ss=[model.ss;sum(riga(model.sv))];
        model.KbInv=(model.K(2,1)*Y(1))^-1;
    end
            
    outNew = model.K(1:length(model.x),ultimo)'*model.x;
    if (outNew<1)
      %pred=[pred 0];
      
      K2 = model.K(1:size(model.hess,1),1:ultimo);
      
      model.hess = cholupdate(model.hess,sqrt(C)*K2(:,ultimo),'+');
      model.ss = model.ss+K2(:,ultimo);
      
      iter = 0;

      model.sv = [model.sv,ultimo];
      sv_bool = zeros(1,ultimo);
      sv_bool(model.sv) = 1;

      while iter < opt.maxiter
        iter = iter + 1;
        % Take a few Newton step (no line search). By writing out the
        % equations, this simplifies to following equation:
        model.x = C*(model.hess \ (model.hess' \ model.ss));
        % qui serve l'intera matrice K2: questo è il passo critico!
        out = model.x'*K2;      % Recompute the outputs...

        new_sv_bool = (out<1);
        new_sv = find(new_sv_bool);

        % The set of errors has changed (and so the Hessian). We update the
        % Cholesky decomposition of the Hessian
        ch=0;
        for i=find(new_sv_bool>sv_bool)
          model.hess = cholupdate(model.hess,sqrt(C)*K2(:,i),'+');
          model.ss = model.ss+K2(:,i);
          ch=1;
        end;
        for i=find(sv_bool>new_sv_bool)
          model.hess = cholupdate(model.hess,sqrt(C)*K2(:,i),'-');
          model.ss = model.ss-K2(:,i);
          ch=1;
        end;
        % Compute the objective function (just for debugging)
        % obj = 0.5* (norm(hess*x)^2 - 2*C*sum(out(new_sv)) + C*length(new_sv));

        % if opt.verb>0
        %    fprintf(['\rNb basis = %d (%d), iter Newton = %d, Obj = %.2f, ' ...
        %        'Nb errors = %d   '],length(hess)-1,length(find(x))-1,iter,obj,length(sv));
        % end;
        
        if ch==0
            break;
        end
        model.sv = new_sv;
        sv_bool=new_sv_bool;
      end;

      %fprintf('%d',iter);
      fprintf('.');
      %fprintf('%d ',sumch);
      %fprintf('%d ',numel(sv));
      
      %obj = [obj obj2];
    else
      %fprintf('0');
      fprintf('P');
      %pred=[pred 1];
    end
    
    %time(curr) = toc;
    %nsv(curr) = numel(find(x))-1;    
  end

  fprintf('Nb basis = %d (%d), Nb errors = %d\n',length(model.hess)-1,length(find(model.x))-1,length(model.sv));
  
  model.beta = [model.x(2:end); zeros(numel(model.S)-(numel(model.x)-1),1)];
  model.b = model.x(1);
  %if opt.verb>0, fprintf('\n'); end;

  %model.K=K;
  %model.KbInv=KbInv;
  %model.hess=hess;
  %model.x=x;
  %model.sv=sv;
  %model.ss=ss;
  %model.S=S;
  
  
%   function update_hess2(S,Y,C)
%   global K hess sv
%   
%   d  = length(S);
% 
%   %h = [zeros(1,d-d0); K(S,d0+2:d+1).*repmat(Y(S),1,d-d0)] + ...
%   %     C * K(sv,1:d+1)' * K(sv,d0+2:d+1);
%   %h = [zeros(1,d-d0); K(S,d0+2:d+1)] + C * K(sv,1:d+1)' * K(sv,d0+2:d+1);
%   h = [0; K(d+1,S)'.*Y(S)] + C * K(1:d+1,sv) * K(d+1,sv)';
% 
%   % The new Hessian would be [[old_hessian h2]; [h2' h3]]
%   %h2 = h(d0+2:end,:);
%   h2 = h(end,:);
%   %h2 = h2 + 1e-10*mean(diag(h2))*eye(size(h,2)); % Ridge is only for numerical reason
%   h2 = h2 + 1e-10*h2*eye(size(h,2)); % Ridge is only for numerical reason
%   %h3 = hess' \ h(1:d0+1,:);
%   h3 = hess' \ h(1:d,:);
%   %h4 = chol(h2-h3'*h3);
%   h4 = sqrt(h2-h3'*h3);
%   % New Cholesky decomposition of the augmented Hessian
%   %hess = [[hess h3]; [zeros(d-d0,d0+1) h4]];
%   hess = [[hess h3]; [zeros(1,d) h4]];
