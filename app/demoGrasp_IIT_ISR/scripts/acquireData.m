function [in,out,in_f,out_f]=acquireData(dataLogFileLeft,dataLogFileRight)
% This function returns the formatted data to learn the network upon.
% The two input files are the ones logged through the dataDumper module.

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
dataL=dataL(dataL(:,7)~=0,1:6);
dataR=dataR(dataR(:,7)~=0,1:6);

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

% filter out ripple
h=fdesign.lowpass('Fp,Fst,Ap,Ast',0.1,0.15,1,40);
d=design(h,'butter');
d=convert(d,'df1');

in_ofs=in;
in_ofs(:,1)=in(1,1);
in_ofs(:,2)=in(1,2);
in_ofs(:,3)=in(1,3);

out_ofs=out;
out_ofs(:,1)=out(1,1);
out_ofs(:,2)=out(1,2);
out_ofs(:,3)=out(1,3);

in_f=filtfilt(d.Numerator,d.Denominator,in-in_ofs)+in_ofs;
out_f=filtfilt(d.Numerator,d.Denominator,out-out_ofs)+out_ofs;

figure;
subplot(321),stairs([in(:,1),in_f(:,1)]),title('ul');
subplot(323),stairs([in(:,2),in_f(:,2)]),title('ur');
subplot(325),stairs([in(:,3),in_f(:,3)]),title('v');

subplot(322),stairs([out(:,1),out_f(:,1)]),title('eye-x');
subplot(324),stairs([out(:,2),out_f(:,2)]),title('eye-y');
subplot(326),stairs([out(:,3),out_f(:,3)]),title('eye-z');


