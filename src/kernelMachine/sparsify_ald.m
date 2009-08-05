function model = sparsify_ald(X,Y,model)
%SPARSIFY_ALD    Sparsification using approximate linear dependency
%   MODEL = SPARSIFY_ALD(X,Y,MODEL)
  
n = length(Y);
  
for curr=1:n
        
    model.X=[model.X , X(:,curr)];
    %model.Y=[model.Y ; Y(curr)];
    %curr  
    %fprintf('.');
    
    ultimo=size(model.X,2);
    
    if (mod(curr,100)==0)
        fprintf('%d (%d bases)\n',curr,numel(model.S));
    end

    last_col=feval(model.ker,model.X,model.S,ultimo,model.kerparam);
    model.K=[model.K,[1 ; last_col]];
        
    if (length(model.S)>0)
        colonna2=model.K(2:end,ultimo);
        %Kb=K(S,2:end);
        if (length(model.S)==1)
            model.KbInv=model.K(2:end,model.S)^-1;
        end
        amin=model.KbInv*colonna2;
        lasterr=feval(model.ker,X,curr,curr,model.kerparam)-colonna2'*amin;
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
        %fprintf('B');
    %else
        %fprintf('.');
    end    
end