function runMin(inFileName,varargin)
% The main routine to be launched by the user
% who has to provide the input and output files name.
% By default output file name is alignKin2Cam.ini
%
% Author: Ugo Pattacini
%#ok<*NASGU>

prepareWS(inFileName);

% 1D max focal displacement [m]
mf=1e-6;
% 1D max rotation [rad]
mal=1e-6*pi/180;
mar=100*pi/180;

% Impose lower and upper bounds as cubic volume;
% these bounds are then normalized in order to lie between -1 and 1.
% The vector we're searching for is of the form [pl tl pr tr],
% where pl is the 3 components rotation axis for left eye,
% tl is the 3 components translation vector for left eye and
% [pr, tr] are for the right eye.
lb=[-mal*ones(1,3) -mf*ones(1,3) -mar*ones(1,3) -mf*ones(1,3)]';
ub=[ mal*ones(1,3)  mf*ones(1,3)  mar*ones(1,3)  mf*ones(1,3)]';
a=2./(ub-lb);
b=-(ub+lb)./(ub-lb);
lb=lb.*a+b;
ub=ub.*a+b;

assignin('base','a',a);
assignin('base','b',b);

% first optimization
fprintf('\n\nAligning axes...\n');
% we use the interior point algorithm
options=optimset('Algorithm','interior-point',...
                 'DiffMaxChange',1e-6,'DiffMinChange',1e-8,...
                 'MaxIter',inf,'MaxFunEvals',inf,...
                 'TolFun',1e-9,'TolX',1e-6,'TolCon',1e-9,...
                 'PlotFcns',{@optimplotx @optimplotfval});

% initial guess
p0=zeros(12,1);

p=fmincon(@funToMin,p0,[],[],[],[],lb,ub,[],options);
p=((p-b)./a);

% display results
show(false,true,p);

% second optimization which finds the two virtual D-H links
% corresponding to the rototranslation for both eyes
linkLB=[-inf -inf -pi -pi];
linkUB=[ inf  inf  pi  pi];

global Hd;

% initial guess
% 2 links have to be found:
% 2 x {A,D,alpha,theta}, offset=0
links0=zeros(8,1);

% final matrix for left eye
Hd=axis2dcm(p(1:3));
Hd(1:3,4)=p(4:6);
fprintf('\n\nSearching for left D-H parameters...\n');
leftLinks=fmincon(@funDH,links0,[],[],[],[],linkLB,linkUB,[],options);

% desired matrix for right eye
Hd=axis2dcm(p(7:9));
Hd(1:3,4)=p(10:12);
fprintf('\n\nSearching for right D-H parameters...\n');
rightLinks=fmincon(@funDH,links0,[],[],[],[],linkLB,linkUB,[],options);

% write the output parameters file
if isempty(varargin)
    fid=fopen('alignKin2Cam.ini','wt');
else
    fid=fopen(varargin{1},'wt');
end

fprintf(fid,'// This file specifies the rototranslation matrices\n');
fprintf(fid,'// which align the eye z axis to the optical axis.\n\n');
fprintf(fid,'[LEFT]\n');
fprintf(fid,'R      %g %g %g\n',p(1),p(2),p(3));
fprintf(fid,'T      %g %g %g\n',p(4),p(5),p(6));
fprintf(fid,'length %g %g\n',leftLinks(1),leftLinks(5));
fprintf(fid,'offset %g %g\n',leftLinks(2),leftLinks(6));
fprintf(fid,'twist  %g %g\n',leftLinks(3),leftLinks(7));
fprintf(fid,'joint  %g %g\n',leftLinks(4),leftLinks(8));

fprintf(fid,'\n');

fprintf(fid,'[RIGHT]\n');
fprintf(fid,'R      %g %g %g\n',p(7),p(8),p(9));
fprintf(fid,'T      %g %g %g\n',p(10),p(11),p(12));
fprintf(fid,'length %g %g\n',rightLinks(1),rightLinks(5));
fprintf(fid,'offset %g %g\n',rightLinks(2),rightLinks(6));
fprintf(fid,'twist  %g %g\n',rightLinks(3),rightLinks(7));
fprintf(fid,'joint  %g %g\n',rightLinks(4),rightLinks(8));

fclose(fid);


%--------------------------------------------------------------------------
function F=funDH(links)

global Hd;

P{1}.A=links(1); P{1}.D=links(2); P{1}.alpha=links(3); P{1}.offset=0;
P{2}.A=links(5); P{2}.D=links(6); P{2}.alpha=links(7); P{2}.offset=0;
T=DH(P,1,links(4))*DH(P,2,links(8));

F=Hd-T;
F=0.5*sum(sum(F.*F));

