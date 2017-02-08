% plot af data fra vigga 37 - balance
clear
close all
%%
dataCV = load('square_CV.txt');
dataCCV = load('square_CCV.txt');
% Vigga (37)
%  1    time 0.009 sec
%  2  3  4   (mission 0), state 2, thread 1, line 1
%  5  6 Wheel velocity [m/s] left, right: 0.0000 0.0000
%  7  8  9 10 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 0.460118
% 11    Battery voltage [V]: 11.77
% plot path
figure(102)
plot(dataCV(:,7), dataCV(:,8), 'b')
hold on
plot(dataCCV(:,7), dataCCV(:,8), 'r')
set(gca,'FontSize',12)
grid on
title('Robot Vigga (37), Square - no controller')
xlabel('X [m]')
ylabel('Y [m]')
legend('CV square', 'CCV square')
axis equal
%%
figure(110)
hold off
plot(dataCV(:,1), dataCV(:,5), 'b')
hold on
plot(dataCV(:,1), dataCV(:,6), '.-r')
%plot(dataCCV(:,1), dataCCV(:,5), 'r')
%plot(dataCCV(:,1), dataCCV(:,6), '.-r')
set(gca,'FontSize',12)
grid on
title('Robot Vigga (37), Square - no controller')
xlabel('time [s]')
ylabel('vel [m/s]')
legend('CV square left', 'CV square right', 'Location','southeast')
