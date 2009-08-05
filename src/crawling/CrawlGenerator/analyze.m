load left_arm_target_position.dat;
load right_arm_target_position.dat;
load right_leg_target_position.dat;
load left_leg_target_position.dat;

t1 = left_arm_target_position(:,5);
t2 = right_arm_target_position(:,5);
t3 = left_leg_target_position(:,5);
t4 = right_leg_target_position(:,5);

subplot(4,1,1);
plot(t1,left_arm_target_position(:,1:4));
xlim([t1(1) t1(end)]);

subplot(4,1,2);
plot(t2,right_arm_target_position(:,1:4));
xlim([t1(1) t1(end)]);

subplot(4,1,3);
plot(t3,left_leg_target_position(:,1:4));
xlim([t1(1) t1(end)]);

subplot(4,1,4);
plot(t4,right_leg_target_position(:,1:4));
xlim([t1(1) t1(end)]);
