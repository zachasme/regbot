% plot af data dra regbot
clear
close all
%% plot - velocity control
dd = load('../log_control_03.txt'); % velocity
% Karla (10)
%  1    time 0.001 sec
%  2  3  4   (mission 0), state 2, thread 1, line 0
%  5  6 Motor velocity ref left, right: 0.50 0.50
%  7  8 Motor voltage [V] left, right: 7.0 7.0
%  9 10 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 11    Battery voltage [V]: 11.15
% 12 13 Get data time [us]: 110 +ctrl 1120
% 14 23 ctrl left , ref=0.5, m=0, m2=0, uf=1.5, r2=0.5, rp=7.5,up=7.5, ui=0.075, u1=9.075, u=7
% 24 33 ctrl right, ref=0.5, m=0, m2=0, uf=1.5, r2=0.5, rp=7.5,up=7.5, ui=0.075, u1=9.075, u=7
% 34 43 ctrl head, ref=0, m=0, m2=0, uf=0, r2=0, ep=0,up=0, ui=0, u1=0, u=0
figure(10)
hold off
plot(dd(:,1), dd(:,5),'r')
grid on
hold on
plot(dd(:,1), dd(:,14),'-.b')
plot(dd(:,1), dd(:,15),'-.g')
plot(dd(:,1), dd(:,16),'b')
plot(dd(:,1), dd(:,17),'m')
plot(dd(:,1), dd(:,18),'.-g')
%plot(dd(:,1), dd(:,19),'k')
plot(dd(:,1), dd(:,20),'m')
plot(dd(:,1), dd(:,21),'c')
plot(dd(:,1), dd(:,7),'.-b')
plot(dd(:,1), dd(:,9),'.-k')
legend('ref left','ctrl ref', 'ctrl m', 'm2', 'uf', 'up', 'ui', 'u1','mot V','vel')
title('left')
%
figure(11)
hold off
plot(dd(:,1), dd(:,6),'r')
grid on
hold on
plot(dd(:,1), dd(:,24),'-.r')
plot(dd(:,1), dd(:,25),'-.g')
plot(dd(:,1), dd(:,26),'b')
plot(dd(:,1), dd(:,27),'m')
plot(dd(:,1), dd(:,28),'.-g')
plot(dd(:,1), dd(:,30),'m')
plot(dd(:,1), dd(:,31),'c')
plot(dd(:,1), dd(:,8),'.-b')
plot(dd(:,1), dd(:,10),'.-k')
legend('ref left','ctrl ref', 'ctrl m', 'm2', 'ff', 'up', 'ui', 'u1','mot V', 'vel')
title('right')
%
figure(12)
hold off
plot(dd(:,1), dd(:,34),'r')
grid on
hold on
%plot(dd(:,1), dd(:,24),'-.r')
plot(dd(:,1), dd(:,35),'-.g')
plot(dd(:,1), dd(:,36),'b')
plot(dd(:,1), dd(:,37),'m')
plot(dd(:,1), dd(:,38),'.-g')
plot(dd(:,1), dd(:,40),'m')
plot(dd(:,1), dd(:,41),'c')
%plot(dd(:,1), dd(:,8),'.-b')
%plot(dd(:,1), dd(:,10),'.-k')
legend('ctrl ref', 'ctrl m', 'm2', 'ff', 'up', 'ui', 'u1','mot V', 'vel')
title('heading')
%% plot - velocity control
dd = load('../log_control_04.txt'); % velocity
% Karla (10)
%  1    time 0.001 sec
%  2  3  4   (mission 0), state 2, thread 1, line 0
%  5  6 Motor velocity ref left, right: 0.30 0.30
%  7  8 Motor voltage [V] left, right: 9.0 9.0
%  9 10 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 11    Turnrate [r/s]: 0.0000
% 12 13 14 15 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 -2.98383
% 16    Battery voltage [V]: 10.98
% 17 18 Get data time [us]: 100 +ctrl 1130
% 19 28 ctrl head, ref=0, m=0, m2=0, uf=0, r2=0, ep=0,up=0, ui=0, u1=0, u=0%
figure(22)
hold off
plot(dd(:,1), dd(:,19),'r')
grid on
hold on
%plot(dd(:,1), dd(:,24),'-.r')
plot(dd(:,1), dd(:,20),'-.g')
plot(dd(:,1), dd(:,21),'.-b')
%plot(dd(:,1), dd(:,22),'m')
plot(dd(:,1), dd(:,24),'.-g')
plot(dd(:,1), dd(:,26),'m')
plot(dd(:,1), dd(:,28),'c')
plot(dd(:,1), dd(:,11)/10,'.-b')
plot(dd(:,1), dd(:,14),'-.k')
plot(dd(:,1), dd(:,9),'--r')
plot(dd(:,1), dd(:,10),'--b')
plot(dd(:,1), dd(:,5),'--g')
legend('ctrl ref', 'ctrl m', 'm2', 'up', 'ui', 'u','turnrate/10', 'heading','vel left','vel right', 'ref left')
title('heading control')
%
figure(23)
hold off
plot(dd(:,1), dd(:,9),'r')
grid on
hold on
plot(dd(:,1), dd(:,10),'b')
plot(dd(:,1), dd(:,5),'-.r')
plot(dd(:,1), dd(:,6),'-.b')
legend('vel left','vel right', 'ref left','ref right')
title('rate limit')
