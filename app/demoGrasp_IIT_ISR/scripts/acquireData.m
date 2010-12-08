function [in,out]=acquireData(dataLogFileLeft,dataLogFileRight,dataLogFileHead)
% This function returns the formatted data to learn the network upon.
% The three input files are the ones logged through the dataDumper module.

% import raw data from log files
dataL=dlmread(dataLogFileLeft);
dataR=dlmread(dataLogFileRight);
dataH=dlmread(dataLogFileHead);

% remove the first column
dataL(:,1)=[];
dataR(:,1)=[];
dataH(:,1)=[];

% harmonize the starting acquisition time
t0=min([dataL(1,1) dataR(1,1) dataH(1,1)]);
dataL(:,1)=dataL(:,1)-t0;
dataR(:,1)=dataR(:,1)-t0;
dataH(:,1)=dataH(:,1)-t0;

% remove the likelihood column (not used)
dataL(:,5)=[];
dataR(:,5)=[];

% select only the rows corresponding
% to object in visibility
dataL=dataL(dataL(:,7)~=0,1:6);
dataR=dataR(dataR(:,7)~=0,1:6);

% the data format is now as:
% 1  2  3  4  5  6
% t  x  y  z  u  v

% go on with the left/right data synchronization
% use the left data as pivot
tL=dataL(:,1);
tR=dataR(:,1);
tH=dataH(:,1);
L=length(tL);

% form input data as:
% 1    2    3    4    5    6    7
% tilt pan  ver  ul   vl   ur   vr
in=zeros(L,7);

% form the output data as:
% 1  2  3
% x  y  z
out=dataL(:,2:4);

% fill the input data
for i=1:L    
    [~,jR]=min(abs(tR-tL(i)));
    [~,jH]=min(abs(tH-tL(i)));
    in(i,1:3)=dataH(jH,5:7);
    in(i,4)=dataL(i,5);
    in(i,5)=dataL(i,6);
    in(i,6)=dataR(jR,5);
    in(i,7)=dataR(jR,6);
end

figure;
subplot(521),stairs(in(:,1)),title('tilt');
subplot(523),stairs(in(:,2)),title('pan');
subplot(525),stairs(in(:,3)),title('ver');
subplot(527),stairs(in(:,4:5)),title('[ul ur]');
subplot(529),stairs(in(:,6:7)),title('[vl vr]');

subplot(524),stairs(out(:,1)),title('eye-x');
subplot(526),stairs(out(:,2)),title('eye-y');
subplot(528),stairs(out(:,3)),title('eye-z');


