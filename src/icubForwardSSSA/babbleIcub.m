function babbleIcub
global vel;
global theta;
global y;
global yd;
global J;
global simstep;
global simstep0;
global q;


numtrnvecs = 60;%600;
numtstvecs = 12;%120;
%numtrnvecs = 3000;
%numtstvecs = 600;

maxangularvel = 100; %radians per second
period = 1/30;
maxdelta = maxangularvel * period;


cycles = numtrnvecs + numtstvecs;

q = theta;
jointAngles = [];
endEffectorImgPts = [];

timestep  = 0.005;  %simulation time step
framerate = 60;     %50 frames per second
fratetimesteps = (1/framerate)/timestep;


for i=1 : cycles
    
    simstep0 = simstep;
    rand_delta = (0.5 - rand(7, 1)')* maxdelta * 2;
    q = q + rand_delta'
    simstep
    simstep0
    
   %wait for a 5 simulation step
   while (simstep <= simstep0 + fratetimesteps)
        pause(0.005); 
   end
   
   simstep
   simstep0
   fratetimesteps
   

    jointAngles = [jointAngles; q'];
    endEffectorImgPts = [endEffectorImgPts; y'];
end


save jointAngles.mat jointAngles;
save endEffectorImgPts.mat endEffectorImgPts;
