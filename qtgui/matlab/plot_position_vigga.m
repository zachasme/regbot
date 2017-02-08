% plot af data fra vigga 37 - balance
clear
close all
%% plot ballance - hand held - partly on floor
dd = load('../log_position-vigga37_a.txt');
% Vigga (37)
%  1    time 0.002 sec
%  2  3  4 Gyro x,y,z [deg/s]: -0.0610352 0.0610352 -0.0305176
%  5  6 Motor velocity ref left, right: 1.64 1.64
%  7  8 Motor voltage [V] left, right: 9.0 9.0
%  9 10 Motor current left, right [A]: -0.661 -0.321
% 11 12 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 13 14 15 16 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 0.456026
% 17 26 ctrl head, ref=0, m=0, m2=0, uf=0, r2=0, ep=0,up=0, ui=0, u1=0, u=0
%%
figure(10)
hold off
plot(dd(:,1), dd(:,5),'r')
grid on
hold on
plot(dd(:,1), dd(:,7)/10,'-.r')
%plot(dd(:,1), dd(:,3)/100,'g')
plot(dd(:,1), dd(:,9),'b')
plot(dd(:,1), dd(:,13),'m')
legend('left ref','motor/10','current left', 'x')
title('topos')
