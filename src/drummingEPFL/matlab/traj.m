clear all

part='right_arm';

target =load(sprintf('%s_target_position.dat',part));
t=1:length(target(:,1));

encoders=load(sprintf('%s_encoders.dat',part));
rt=1:length(encoders(:,1));

feed_manager=load('../DrumManager/feedback_time.dat');


feed_generator=load(sprintf('%s_feedback.dat',part));

figure(1)
plot(t, target(:,1), rt, encoders(:,1), t, target(:,4), rt, encoders(:,4))
legend('shoulder 0 target', 'shoudler 0 real', 'elbow target', 'elbow real') 
title('left arm position - rhythmic')

figure(2)
plot(t, target(:,2), rt, encoders(:,2), t, target(:,3), rt, encoders(:,3))
legend('shoulder 1 target', 'shoudler 1 real', 'shoulder 2 target', 'shoulder 2 real') 
title('left arm position - discrete')

%figure(2)
%plot(feed_manager)
%hold on
%plot(feed_generator)

%plot(target(:,5)-meanT,target(:,1))
%hold on
%plot(
%a=size(feed_generator);

%figure(2)
%plot(feed_generator-meanT, -40*ones(size(feed_generator)), 'r.')
%plot(target(:,5)-meanT, -50*ones(size(target(:,5))), 'g.');
%hold on
%plot(feed_generator,'.')
