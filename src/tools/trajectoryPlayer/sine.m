sampling_period = 20; %ms
frequency = 1; %hertz
amplitude = 10; %degrees
offset = 0; %degrees
time = [0: 0.020: 10];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
out = amplitude*sin(time*2*pi*frequency)+offset;

figure(1)
hold on
grid on
plot (time, out)

fileID = fopen('trajectory.txt','w');
%A = [time; out];
%fprintf(fileID,'%6.3f %6.3f\n',A);
A = [out];
fprintf(fileID,'%6.3f\n',A);
fclose(fileID);