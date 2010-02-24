clear all
close all
clc
sweepCount = 1;
SAMPLER_RATE=10;
SWEEP_TIME=5000;
w0=0;
f=1;
A=100;
count=0;
for i=0:1000
    if(sweepCount<=SWEEP_TIME/SAMPLER_RATE)
        sweepCount=sweepCount+1
        wt=2*pi*f*sweepCount*(SAMPLER_RATE/1000)+w0;
    else
        f=f+1;
        sweepCount=1;
        w0=wt;
        wt=2*pi*f*sweepCount*(SAMPLER_RATE/1000)+w0;
    end
    count=count+1;
    tau(i+1)=A*sin(wt);
    time(i+1)=count*SAMPLER_RATE/1000;
end
plot(time,tau)

load ftSweepData_leftarm.dat
ftSweepData_leftarm=ftSweepData_leftarm(2:8000,:);
dt=ftSweepData_leftarm(2:end,1);
tauIn=ftSweepData_leftarm(:,2);
tauOut=ftSweepData_leftarm(:,3);
figure
plot(dt)
dt=mean(dt)
time=[0:dt:dt*(length(tauIn)-1)];
figure
plot(time,[tauIn 500*tauOut])
%%
Fs = 1/dt;                    % Sampling frequency
L = length(tauIn);                     % Length of signal
t = (0:L-1)*dt;                % Time vector
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
TAUIN = fft(tauIn,NFFT)/L;
TAUOUT = fft(tauOut,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);
H=TAUOUT./TAUIN;
% Plot single-sided amplitude spectrum.
figure
plot(f,2*abs(TAUIN(1:NFFT/2+1))),hold on 
plot(f,2*500*abs(TAUOUT(1:NFFT/2+1)),'r') 
title('Single-Sided Amplitude Spectrum of H')
xlabel('Frequency (Hz)')
ylabel('|H(f)|')
figure
semilogx(f,20*log10(2*abs(H(1:NFFT/2+1)))),axis([1 20 -90 0])




