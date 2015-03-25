%% this file is to test motor encoders and joint encoders
% the file to be loaded is just a log of the stateExt:o port
% it can be obtained with the following command:
% yarp read ... /icub/left_arm/stateExt:o > logarm.txt
robotname='heid';
load_ll;
load_rl;
load_to;
% load_la;
% load_ra;


%%
logdata=log_ll;
%logdata=log_rl;
%logdata=log_to;
% logdata=log_la;
% logdata=log_la;

% the following variable defines the number of joints logged by the port
number_of_joints = 6;
number_of_joints_to_plot = 6;

i=0;
jp = logdata(:,i*number_of_joints+1:i*number_of_joints+number_of_joints);i=i+1;i*number_of_joints % 0
jv = logdata(:,i*number_of_joints+1:i*number_of_joints+number_of_joints);i=i+1;i*number_of_joints % 16
ja = logdata(:,i*number_of_joints+1:i*number_of_joints+number_of_joints);i=i+1;i*number_of_joints % 32
mp = logdata(:,i*number_of_joints+1:i*number_of_joints+number_of_joints);i=i+1;i*number_of_joints % 48
mv = logdata(:,i*number_of_joints+1:i*number_of_joints+number_of_joints);i=i+1;i*number_of_joints % 64
ma = logdata(:,i*number_of_joints+1:i*number_of_joints+number_of_joints);i=i+1;i*number_of_joints % 80

%% COUPLING MATRICES

if (number_of_joints==16) %% shoulder joints
matrix = eye(16);
matrix (1,1) = 1;
matrix (2,1) = -1.6455;
matrix (2,2) = 1.6455;
matrix (3,1) = -1.6455;
matrix (3,2) = 1.6455;
matrix (3,3) = 1.6455;
elseif (number_of_joints==3) %% torso joints
matrix = eye(3);
r=0.022*1000;
R=0.04*1000;
matrix (1,1) = R/r;
matrix (2,1) = 0;
matrix (3,1) = 0;
matrix (1,2) = -1;
matrix (2,2) = 1;
matrix (3,2) = 1;
matrix (1,3) = 0;
matrix (2,3) = -1;
matrix (3,3) = 1;
elseif (number_of_joints==6) %%leg joints
matrix = eye(number_of_joints);
matrix (1,1) = 60/40; %%check me
else
matrix = eye(number_of_joints); 
end
inv_matrix =inv(matrix);

jp2m = ((matrix*jp')*100)';
jv2m = ((matrix*jv')*100)';
ja2m = ((matrix*ja')*100)';

%%  JOINT POSITIONS vs MOTOR POSITIONS  -> VERIFIED
close all

for j=1:1:number_of_joints_to_plot
figure(j)
hold on
grid on
stairs (jp2m(:,j)-jp2m(1,j),'b')
stairs (mp(:,j)-mp(1,j),'r')
%xlim ([0 2000])
end

%% JOINT VELOCITES vs MOTOR VELOCITIES
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j=1:1:number_of_joints_to_plot
figure(10+j)
hold on
grid on
stairs (jv2m(:,j)-jv2m(1,j),'b')
stairs (mv(:,j)-mv(1,j),'r')
%xlim ([0 2000])
end

%% JOINT POSITIONS(DERIVED) vs JOINT SPEED
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j=1:1:number_of_joints_to_plot
figure(30+j)
hold on
grid on
stairs (diff(jp(:,j)/0.010),'b')
stairs (jv(:,j),'r')
%xlim ([0 2000])
end

%% MOTOR POSITIONS(DERIVED) vs MOTOR SPEED -> VERIFIED
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j=1:1:number_of_joints_to_plot
figure(40+j)
hold on
grid on
stairs (diff(mp(:,j)/0.010),'b')
stairs (mv(:,j),'r')
%xlim ([0 2000])
end 
