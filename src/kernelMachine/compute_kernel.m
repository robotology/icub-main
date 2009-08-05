function K = compute_kernel(in1,in2,in3,in4)
%COMPUTE_KERNEL    Calculates the kernel matrix
%   K = COMPUTE_KERNEL(X,IND1,IND2,KERNEL_PARAMS)
%   K = COMPUTE_KERNEL(X1,X2,KERNEL_PARAMS)

if nargin==4
  X=in1;
  ind1=in2;
  ind2=in3;
  hp=in4;
  X1=X(:,ind1);
  X2=X(:,ind2);
else
  X1=in1;
  X2=in2;
  hp=in3;
end

if size(X1,2)==0
    K = [];
    return;
end

switch hp.type
   case 'linear'
    K = X1'*X2;
    
   case 'poly'
    K = (hp.gamma*X1'*X2+hp.coef0).^hp.degree;
      
   case 'rbf'
    normX = sum(X1.^2,1);
    normY = sum(X2.^2,1);
    K = exp(-hp.gamma*(repmat(normX' ,1,size(X2,2)) + ...
                           repmat(normY,size(X1,2),1) - ...
                           2*X1'*X2));
                       
   case 'rbf_b'
    normX = sum(X1.^2,1);
    normY = sum(X2.^2,1);
    K = exp(-hp.gamma*(repmat(normX' ,1,size(X2,2)) + ...
                           repmat(normY,size(X1,2),1) - ...
                           2*X1'*X2))+hp.coef0;
                       
   case 'forest'
    for i=1:size(X1,2)
       normX(i) = forest_lin(X1(:,i),X1(:,i));
    end
    for i=1:size(X2,2)
       normY(i) = forest_lin(X2(:,i),X2(:,i));
    end
    K = exp(-(repmat(normX' ,1,size(X2,2)) + ...
                           repmat(normY,size(X1,2),1) - ...
                           2*forest_lin(X1,X2)));
    
   case 'triangular'
    normX = sum(X1.^2,1);
    normY = sum(X2.^2,1);
    K = -sqrt(repmat(normX',1,size(X2,2)) + repmat(normY,size(X1,2),1) - 2*X1'*X2);

   case 'sigmoid'
    K = tanh(hp.gamma*X1'*X2+hp.coef0);
    
   case 'expchi2'
    K=zeros(size(X1,2),size(X2,2));
    for i=1:size(X1,2)
        h1=X1(:,i);
        for j=1:size(X2,2)
           %K(i,j) = exp(-hp.gamma*sum(((X1(i,:)-X2(j,:)).^2)./(X1(i,:)+X2(j,:)+eps),2) );
           h2=X2(:,j);
           tmp=chisquare_sparse(h1,h2);
           %if tmp~=sum(((X1(i,:)-X2(j,:)).^2)./(X1(i,:)+X2(j,:)+eps))
           %    disp(tmp)
           %    disp(sum(((X1(i,:)-X2(j,:)).^2)./(X1(i,:)+X2(j,:)+eps)))
           %end
           K(i,j) = exp(-hp.gamma*tmp );
       end
    end
 
   case 'intersection'
    K=zeros(size(X1,2),size(X2,2));
    for i=1:size(X1,2)
        h1=X1(:,i);
        for j=1:size(X2,2)
           h2=X2(:,j);
           K(i,j) = sum(min(h1,h2));
        end
    end

   case 'w_intersection'
    K=zeros(size(X1,2),size(X2,2));
    for i=1:size(X1,2)
        h1=X1(:,i);
        for j=1:size(X2,2)
           h2=X2(:,j);
           K(i,j) = sum(hp.w.*min(h1,h2));
        end
    end

   case 'g_intersection'
    K=zeros(size(X1,2),size(X2,2));
    for i=1:size(X1,2)
        h1=X1(:,i);
        for j=1:size(X2,2)
           h2=X2(:,j);
           K(i,j) = sum(min(h1.^hp.degree,h2.^hp.degree));
        end
    end

   case 'same'
    K=zeros(size(X1,2),size(X2,2));
    for i=1:size(X1,2)
        for j=1:size(X2,2)
           K(i,j) = (X1(:,i)==X2(:,j));
       end
    end
    K=(1+1/(hp.n_class-1))*K-1/(hp.n_class-1);
    
   otherwise
    error('Unknown kernel');
end

function out=forest_lin(a,b)
for j=1:size(a,2)
    for i=1:size(b,2)
        out(j,i)=a(1:10,j)'*b(1:10,i);
        if a(11,j)==b(11,i)
            out(j,i)=out(j,i)+2;
        end
        if a(12,j)==b(12,i)
            out(j,i)=out(j,i)+2;
        end
    end
end