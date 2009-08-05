function k=local_kernel(X1,X2,gamma,div)

num1=size(X1,2);
num2=size(X2,2);

normX = sum(X1.^2,1);
normY = sum(X2.^2,1);
mat=repmat(normX',1,num2) + repmat(normY,num1,1) - 2*X1'*X2;

[mn,pos_vec]=sort(mat(:));

j=0;
dim_m2=min(min(num1,num2),div);
size1mat=size(mat,1);
marker_mat=zeros(size(mat));
mn2=zeros(1,dim_m2);

for i=1:dim_m2
    j=j+1;
    while marker_mat(pos_vec(j))==Inf
        j=j+1;
    end
    mn2(i)=mn(j);
    pos=pos_vec(j);

    %[row,col]=ind2sub(size(mat),pos);
    col=ceil(pos/size1mat);
    row=pos-(col-1)*size1mat;
    
    marker_mat(row,:)=Inf;
    marker_mat(:,col)=Inf;
end

% for i=1:min(min(num1,num2),div)
%     [mn3(i),pos]=min(mat(:));
%     %[row,col]=ind2sub(size(mat),pos);
%     col=ceil(pos/size(mat,1));
%     row=pos-(col-1)*size(mat,1);
%     
%     mat(row,:)=[];
%     mat(:,col)=[];
% end

%if mn==zeros(size(mn))
%    k=1;
%else
    k=sum(exp(-gamma*mn2))/div;
%end