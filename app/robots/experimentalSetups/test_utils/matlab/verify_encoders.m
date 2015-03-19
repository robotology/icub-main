%% this file is to test motor encoders and joint encoders
% the file to be loaded is just a log of the stateExt:o port
% it can be obtained with the following command:
% yarp read ... /icub/left_arm/stateExt:o > logarm.txt
logdata=logarmsign2;

% the following variable defines the number of joints logged by the port
number_of_joints = 16;

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
else
matrix = eye(number_of_joints);
end
inv_matrix =inv(matrix);

jp2m = ((matrix*jp')*100)';
jv2m = ((matrix*jv')*100)';
ja2m = ((matrix*ja')*100)';

%%  JOINT POSITIONS vs MOTOR POSITIONS  -> VERIFIED
close all

figure(1)
hold on
grid on
stairs (jp2m(:,1)-jp2m(1,1),'b')
stairs (mp(:,1)-mp(1,1),'r')
xlim ([0 2000])


figure(2)
hold on
grid on
stairs (jp2m(:,2)-jp2m(1,2),'b')
stairs (mp(:,2)-mp(1,2),'r')
xlim ([0 2000])


figure(3)
hold on
grid on
stairs (jp2m(:,3)-jp2m(1,3),'b')
stairs (mp(:,3)-mp(1,3),'r')
xlim ([0 2000])


figure(4)
hold on
grid on
stairs (jp2m(:,4)-jp2m(1,4),'b')
stairs (mp(:,4)-mp(1,4),'r')
xlim ([0 2000])


%% JOINT VELOCITES vs MOTOR VELOCITIES
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(11)
hold on
grid on
stairs (jv2m(:,1)-jv2m(1,1),'b')
stairs (mv(:,1)-mv(1,1),'r')
xlim ([0 2000])


figure(12)
hold on
grid on
stairs (jv2m(:,2)-jv2m(1,2),'b')
stairs (mv(:,2)-mv(1,2),'r')
xlim ([0 2000])


figure(13)
hold on
grid on
stairs (jv2m(:,3)-jv2m(1,3),'b')
stairs (mv(:,3)-mv(1,3),'r')
xlim ([0 2000])


figure(14)
hold on
grid on
stairs (jv2m(:,4)-jv2m(1,4),'b')
stairs (mv(:,4)-mv(1,4),'r')
xlim ([0 2000])


%% JOINT POSITIONS(DERIVED) vs JOINT SPEED
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(31)
hold on
grid on
stairs (diff(jp(:,1)/0.010),'b')
stairs (jv(:,1),'r')
xlim ([0 2000])

figure(32)
hold on
grid on
stairs (diff(jp(:,2)/0.010),'b')
stairs (jv(:,2),'r')
xlim ([0 2000])

figure(33)
hold on
grid on
stairs (diff(jp(:,3)/0.010),'b')
stairs (jv(:,3),'r')
xlim ([0 2000])

figure(34)
hold on
grid on
stairs (diff(jp(:,4)/0.010),'b')
stairs (jv(:,4),'r')
xlim ([0 2000])

%% MOTOR POSITIONS(DERIVED) vs MOTOR SPEED -> VERIFIED
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(41)
hold on
grid on
stairs (diff(mp(:,1)/0.010),'b')
stairs (mv(:,1),'r')
xlim ([0 2000])

figure(42)
hold on
grid on
stairs (diff(mp(:,2)/0.010),'b')
stairs (mv(:,2),'r')
xlim ([0 2000])

figure(43)
hold on
grid on
stairs (diff(mp(:,3)/0.010),'b')
stairs (mv(:,3),'r')
xlim ([0 2000])

figure(44)
hold on
grid on
stairs (diff(mp(:,4)/0.010),'b')
stairs (mv(:,4),'r')
xlim ([0 2000])
