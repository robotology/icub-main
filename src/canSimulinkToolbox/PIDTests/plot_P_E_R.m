figure;
subplot(2,1,1);
hold on;
plot(knee_position.time,knee_position.signals.values,'blue')
plot(knee_position.time,knee_reference.signals.values,'r')
hold off;
subplot(2,1,2);
plot(knee_position.time,knee_error.signals.values,'g')