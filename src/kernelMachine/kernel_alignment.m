function ka=kernel_alignment(A,B,mode)
%KERNEL_ALIGNMENT    Kernel alignment between two Gram matrices
%   KA = KERNEL_ALIGNMENT(A,B)

if nargin<3
    mode=0;
end

%standard
if mode==0
    p=A.*B;
    n1=norm(A,'fro');
    n2=norm(B,'fro');
    ka=sum(p(:))/(n1*n2);
elseif mode==1 % no diagonal
    A=A-diag(diag(A));
    B=B-diag(diag(B));
    p=A.*B;
    n1=norm(A,'fro');
    n2=norm(B,'fro');
    ka=sum(p(:))/(n1*n2);
elseif mode==2
    A=A-mean(A(:));
    B=B-mean(B(:));
    p=A.*B;
    n1=norm(A,'fro');
    n2=norm(B,'fro');
    ka=sum(p(:))/(n1*n2);
end