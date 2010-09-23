function [in,out]=acquireData(dataLogFileLeft,dataLogFileRight)
% This function returns the formatted data to learn the network upon
%#ok<*FNDSB>

% import raw data from log files
dataL=dlmread(dataLogFileLeft);
dataR=dlmread(dataLogFileRight);

% remove the first column
dataL(:,1)=[];
dataR(:,1)=[];

% harmonize the starting acquisition time
t0=min([dataL(1,1) dataR(1,1)]);
dataL(:,1)=dataL(:,1)-t0;
dataR(:,1)=dataR(:,1)-t0;

% remove the likelihood column (not used)
dataL(:,5)=[];
dataR(:,5)=[];

% select only the rows corresponding
% to object in visibility
idxL=find(dataL(:,7)~=0);
idxR=find(dataR(:,7)~=0);
dataL=dataL(idxL,1:6);
dataR=dataR(idxR,1:6);

% the data format is now as:
% 1  2  3  4  5  6
% t  x  y  z  u  v

% go on with the left/right data synchronization
% use the left data as pivot
tL=dataL(:,1);
tR=dataR(:,1);
L=length(tL);

% form input data as:
% 1  2  3
% ul ur v
in=zeros(L,3);

% form the output data as:
% 1  2  3
% x  y  z
out=dataL(:,2:4);

% fill the output data
for i=1:L    
    [~,j]=min(abs(tR-tL(i)));    
    in(i,1)=dataL(i,5);
    in(i,2)=dataR(j,5);
    in(i,3)=mean([dataL(i,6),dataR(j,6)]);
end

