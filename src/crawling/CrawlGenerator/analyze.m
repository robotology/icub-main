load left_arm_target_position.dat;
load right_arm_target_position.dat;
load right_leg_target_position.dat;
load left_leg_target_position.dat;

t1 = left_arm_target_position(:,1);
t2 = right_arm_target_position(:,1);
t3 = left_leg_target_position(:,1);
t4 = right_leg_target_position(:,1);

t_base = min([t1;t2;t3;t4]);

t1 = t1-t_base;
t2 = t2-t_base;
t3 = t3-t_base;
t4 = t4-t_base;



figure;
plot(t1,left_arm_target_position(:,2));
hold on;
plot(t2,right_arm_target_position(:,2),'k');
plot(t3,left_leg_target_position(:,2),'g');
plot(t4,right_leg_target_position(:,2),'r');


figure;

%subplot(4,1,1);
plot(t1,left_arm_target_position(:,2:5),'x');
xlim([t1(1) t1(end)]);

%subplot(4,1,2);
figure;
plot(t2,right_arm_target_position(:,2:5),'x');
xlim([t1(1) t1(end)]);

%subplot(4,1,3);
figure;
plot(t3,left_leg_target_position(:,2:5),'x');
xlim([t1(1) t1(end)]);

%subplot(4,1,4);
figure;
plot(t4,right_leg_target_position(:,2:5),'x');
xlim([t1(1) t1(end)]);
