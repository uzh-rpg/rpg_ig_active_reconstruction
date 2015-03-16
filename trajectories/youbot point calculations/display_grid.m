close all;
% data = importdata('youbot_arm_grid_50pts_per_m.txt',' ');
% data2 = importdata('youbot_arm_grid_100pts_per_m.txt',' ');
data3 = importdata('youbot_arm_grid_60pts_per_m.txt',' ');

% figure;
% grid on
% scatter( data(:,2),data(:,1),'filled' )
% figure;
% grid on
% scatter( data2(:,1),data2(:,2),'filled' )
figure;
grid on
scatter( data3(:,1),data3(:,2),'filled' )
title('reachable space of youbot arm');
xlabel('horizontal distance x from arm base [m]');
ylabel('vertical distance y from arm base [m]');