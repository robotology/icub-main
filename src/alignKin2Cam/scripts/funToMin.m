function F=funToMin(p)
% The function to be minimized returns the mean distance computed over
% the whole work set between each point (whose location depends also on
% the unknown rototranslation matrix we're seeking for, which is given by
% the parameters vector p) and the center of mass of the complete set.
% Minimization runs on both eyes data at the same time.
%
% Author: Ugo Pattacini

Hl=evalin('base','Hl');
Hr=evalin('base','Hr');
Hle=evalin('base','Hle');
Hre=evalin('base','Hre');
a=evalin('base','a');
b=evalin('base','b');

% retrieve actual parameters values
p=(p-b)./a;

% compute the aligning rototranslation matrix for left eye
Hlx=axis2dcm(p(1:3));
Hlx(1:3,4)=p(4:6);

% compute the aligning rototranslation matrix for right eye
Hrx=axis2dcm(p(7:9));
Hrx(1:3,4)=p(10:12);

len=length(Hl);

% compute coordinates in the root reference
xl=cell(len,1);
xr=cell(len,1);
for i=1:len
    Tl=Hl{i}*Hlx*Hle{i};
    Tr=Hr{i}*Hrx*Hre{i};
    xl{i}=Tl(1:3,4);
    xr{i}=Tr(1:3,4);
end

% compute centroid
xm=mean([xl{:} xr{:}],2);

% compute distances
d=zeros(len,2);
for i=1:len
    d(i,1)=norm(xl{i}-xm);
    d(i,2)=norm(xr{i}-xm);
end

% return the function value
F=mean(reshape(d,2*len,1));

