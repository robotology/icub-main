function [in,out]=acquireData(dataLogFileLeft,dataLogFileRight,dataLogFileHead)
% This function returns the formatted data to learn the network upon.
% The three input files are the ones logged through yarpdatadumper.

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

figure('Color','w');
global ha;

subplot(521),hold('on'),grid,stairs(in(:,1));
ha{1}=gca;title('tilt');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(523),hold('on'),grid,stairs(in(:,2));
ha{2}=gca;title('pan');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(525),hold('on'),grid,stairs(in(:,3));
ha{3}=gca;title('ver');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(527),hold('on'),grid,stairs(in(:,[4 6]));
ha{4}=gca;title('[ul ur]');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(529),hold('on'),grid,stairs(in(:,[5 7]));
ha{5}=gca;title('[vl vr]');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(524),hold('on'),grid,stairs(out(:,1));
ha{6}=gca;title('eye-x');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(526),hold('on'),grid,stairs(out(:,2));
ha{7}=gca;title('eye-y');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);

subplot(528),hold('on'),grid,stairs(out(:,3));
ha{8}=gca;title('eye-z');
set(zoom,'ActionPostCallback',@mypostcallback);
set(pan,'ActionPostCallback',@mypostcallback);


%--------------------------------------------------------------------------
function mypostcallback(obj,evd) %#ok<INUSL>

global ha;

newLim=get(evd.Axes,'XLim');

for i=1:length(ha)
    xlim(ha{i},newLim);
end


