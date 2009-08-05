load left_arm_target_position.dat;
load right_arm_target_position.dat;
load right_leg_target_position.dat;
load left_leg_target_position.dat;

load left_arm_encoders.dat;
load right_arm_encoders.dat;
load left_leg_encoders.dat;
load right_leg_encoders.dat;

t1 = left_arm_target_position(:,5);
t2 = right_arm_target_position(:,5);
t3 = left_leg_target_position(:,5);
t4 = right_leg_target_position(:,5);


t12 = left_arm_encoders(:,1);
t22 = right_arm_encoders(:,1);
t32 = left_leg_encoders(:,1);
t42 = right_leg_encoders(:,1);

t_base = min([t1;t2;t3;t4]);

t1 = t1-t_base;
t2 = t2-t_base;
t3 = t3-t_base;
t4 = t4-t_base;

figure;

%subplot(4,1,1);
plot(t1,left_arm_target_position(:,1:4),'--');
hold on;
plot(t12-t_base,left_arm_encoders(:,2:5),'x');
xlim([t1(1) t1(end)]);

%subplot(4,1,2);
figure;
plot(t2,right_arm_target_position(:,1:4),'--');
hold on;
plot(t22-t_base,right_arm_encoders(:,2:end),'x');
xlim([t1(1) t1(end)]);

%subplot(4,1,3);
figure;
plot(t3,left_leg_target_position(:,1:4),'--');
hold on;
plot(t32-t_base,left_leg_encoders(:,2:end),'x');
xlim([t1(1) t1(end)]);

%subplot(4,1,4);
figure;
plot(t4,right_leg_target_position(:,1:4),'--');
hold on;
plot(t42-t_base,right_leg_encoders(:,2:end),'x');
xlim([t1(1) t1(end)]);
