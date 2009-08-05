clear all
load GO.dat;
T= GO;

tt=1:length(T(:,1));

figure(1)
plot(tt , T(:,1), tt , T(:,2) ,tt , T(:,3) ,tt , T(:,4) )
title('target positions - right arm')


