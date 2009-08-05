function prepareWS(fileName)
% read data from file and prepare them
% for optimization process
%
% Author: Ugo Pattacini

M=dlmread(fileName);
sz=size(M);
len=sz(1);
Hl=cell(len,1);
Hr=cell(len,1);
Hle=cell(len,1);
Hre=cell(len,1);

workSet=1:1:len;
trashSet=setdiff(1:len,workSet);   % if you wanna reduce working set already here

% retrieve all the required homogeneous transformations
for i=workSet
    Hl{i}=fkin('l',M(i,1:9));       % kin. transform from left eye to root
    Hr{i}=fkin('r',M(i,1:9));       % kin. transform from right eye to root
    Hle{i}=axis2dcm(M(i,10:12));    
    Hle{i}(1:3,4)=M(i,13:15)';      % transform from observed point to left eye
    Hre{i}=axis2dcm(M(i,16:18));    
    Hre{i}(1:3,4)=M(i,19:21)';      % transform from observed point to right eye
end

% handle outliers here by properly increasing trashSet

% remove elements you dont'wanna to work on
Hl(trashSet)=[];
Hr(trashSet)=[];
Hle(trashSet)=[];
Hre(trashSet)=[];

% finally assign variables in the workspace
assignin('base','Hl',Hl);
assignin('base','Hr',Hr);
assignin('base','Hle',Hle);
assignin('base','Hre',Hre);


