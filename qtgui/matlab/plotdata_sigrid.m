% plot af data fra vigga 37 - balance
clear
close all
%% plot ballance - hand held - partly on floor
dd = load('log_current-sigrid58_b.txt');
% Vigga (37)
% Sigrid (58)
%  1    time 0.004 sec
%  2  3  4   (mission 0), state 2, thread 3, line 0
%  5  6  7 Gyro x,y,z [deg/s]: 0.335693 0.305176 -0.183105
%  8  9 Motor voltage [V] left, right: 4.0 4.0
% 10 11 Motor current left, right [A]: -0.521 -0.338
% 12 13 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 14 15 16 17 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 1.39987
% 18    Battery voltage [V]: 12.12
%%
figure(10)
hold off
plot(dd(:,1), dd(:,8),'r')
grid on
hold on
plot(dd(:,1), dd(:,9),'-.r')
plot(dd(:,1), dd(:,10),'g')
plot(dd(:,1), dd(:,11),'b')
%plot(dd(:,1), dd(:,16),'m')
legend('motor left','motor right','current left','current right', 'tilt')
title('robobot test')
%%
figure(11)
hold off
plot(dd(:,1), dd(:,10),'r')
grid on
hold on
plot(dd(:,1), dd(:,12),'g')
plot(dd(:,1), dd(:,6),'-.r')
legend('current right','velocity right','motor voltage right')
title('right')
%%
%% plot ballance - hand held - partly on floor
dd = load('../regbot_karla_10_mfg.txt');
% Karla (10)
%  1    time 0.000 sec
%  2  3  4   (mission 0), state 2, thread 1, line 0
%  5  6 Motor velocity ref left, right: 0.00 0.00
%  7  8 Motor voltage [V] left, right: 0.0 0.0
%  9 10 Motor current left, right [A]: -0.031 -0.014
% 11 12 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 13    Battery voltage [V]: 13.14
% 14 15 Get data time [us]: 110 +ctrl 510
%%
figure(35)
hold off
plot(dd(:,1), dd(:,9),'r')
grid on
hold on
plot(dd(:,1), dd(:,11),'g')
plot(dd(:,1), dd(:,5),'-.m')
plot(dd(:,1), dd(:,7),'-.b')
legend('current [A]','velocity [m/s]','motor ref [m/s]','motor volt [V]')
title('left')
%
figure(36)
hold off
plot(dd(:,1), dd(:,10),'r')
grid on
hold on
plot(dd(:,1), dd(:,12),'g')
plot(dd(:,1), dd(:,6),'-.m')
plot(dd(:,1), dd(:,8),'-.b')
legend('current [A]','velocity [m/s]','motor ref [m/s]','motor volt [V]')
title('right')

%%
figure(2)
hold off
plot(dd(:,1), dd(:,7),'r')
hold on
plot(dd(:,1), dd(:,8),'b')
grid on
legend('motor voltage left','motor voltage right')
%
dd2 = dd(240:350,:);
mvolt= [mean(dd2(:,7:8)) std(dd2(:,7:8))]
wvel = [mean(dd2(:,11:12)) std(dd2(:,11:12))]
%%
%plot((draw(:,1)-draw(1,1))/3600,(47800 - draw(:,9))/7,'b')
%legend('netatmo','(47800-raw)/7','Location','north')
%xlabel('hour')
%ylabel('CO2 ppm')