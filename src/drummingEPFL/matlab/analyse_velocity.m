part = 'left_arm';


parameters = load(sprintf('../DrumGeneratorModule2/%s_parameters.dat',part));
target_position = load(sprintf('../DrumGeneratorModule2/%s_target_position.dat',part));
target_speed = load(sprintf('../../velocityControl/%s_target_speed.dat',part));
encoders = load(sprintf('../DrumGeneratorModule2/%s_encoders.dat',part));
current_speed = load(sprintf('../../velocityControl/%s_current_speed.dat',part));

joint= {'Flex/Ext','Add/Abd','Rot','Elbow'};

figure;
for i = 1:4
  subplot(2,2,i);
  time = target_position(:,end);
  %plot(time,target_position(:,i),'linewidth',2);
  %hold on;
  %plot(time,encoders(end-length(time)+1:end,i),'--','linewidth',2);
  plot(time,target_position(:,i)-encoders(end-length(time)+1:end,i),'linewidth',2);
  set(gca,'fontsize',20);
  xlabel('Time [s]');
  ylabel('Position [Deg]');
  title(joint{i},'fontsize',30);
  %xlim([10 100]);
end

figure;
for i = 1:4
  subplot(2,2,i);
  time = target_speed(:,end);
  plot(time,target_speed(:,i),'linewidth',2);
  hold on;
  speed = diff(current_speed(:,i))./diff(current_speed(:,end));
  plot(current_speed(2:end,end),speed,'--','linewidth',2);
  set(gca,'fontsize',20);
  xlabel('Time [s]');
  ylabel('Speed [Deg/s]');
  title(joint{i},'fontsize',30);
  %xlim([90 100]);
end
