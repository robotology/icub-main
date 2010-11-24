function [in,out,in_f,out_f]=acquireData(dataLogFileLeft,dataLogFileRight,dataLogFileHead)
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
% 1   2   3   4   5
% ul  ur  v   pan ver
in=zeros(L,5);

% form the output data as:
% 1  2  3
% x  y  z
out=dataL(:,2:4);

% fill the input data
for i=1:L    
    [~,jR]=min(abs(tR-tL(i)));
    [~,jH]=min(abs(tH-tL(i)));
    in(i,1)=dataL(i,5);
    in(i,2)=dataR(jR,5);
    in(i,3)=mean([dataL(i,6),dataR(jR,6)]);
    in(i,4:5)=dataH(jH,6:7);
end

% filter out ripple
h=fdesign.lowpass('Fp,Fst,Ap,Ast',0.1,0.15,1,40);
d=design(h,'butter');
d=convert(d,'df1');

in_ofs=in;
in_ofs(:,1)=in(1,1);
in_ofs(:,2)=in(1,2);
in_ofs(:,3)=in(1,3);
in_ofs(:,4)=in(1,4);
in_ofs(:,5)=in(1,5);

out_ofs=out;
out_ofs(:,1)=out(1,1);
out_ofs(:,2)=out(1,2);
out_ofs(:,3)=out(1,3);

in_f=filtfilt(d.Numerator,d.Denominator,in-in_ofs)+in_ofs;
out_f=filtfilt(d.Numerator,d.Denominator,out-out_ofs)+out_ofs;

figure;
subplot(521),stairs([in(:,1),in_f(:,1)]),title('ul');
subplot(523),stairs([in(:,2),in_f(:,2)]),title('ur');
subplot(525),stairs([in(:,3),in_f(:,3)]),title('v');
subplot(527),stairs([in(:,4),in_f(:,4)]),title('pan');
subplot(528),stairs([in(:,5),in_f(:,5)]),title('ver');

subplot(524),stairs([out(:,1),out_f(:,1)]),title('eye-x');
subplot(526),stairs([out(:,2),out_f(:,2)]),title('eye-y');
subplot(528),stairs([out(:,3),out_f(:,3)]),title('eye-z');


