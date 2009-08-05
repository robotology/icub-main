function K = compute_kernel_set(X,ind1,ind2,hp)
  switch hp.type

   case 'local'
    if isempty(ind1)
      K = [];
      return;
    end;
    K=zeros(numel(ind1),numel(ind2));
    for i=1:numel(ind1)
        h1=X{ind1(i)};
        for j=1:numel(ind2)
           h2=X{ind2(j)};
           K(i,j) = local_kernel(h1,h2,hp.gamma,hp.div);
       end
    end
 
   otherwise
    error('Unknown kernel');
  end