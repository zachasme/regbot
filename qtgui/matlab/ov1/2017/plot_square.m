% plot af data fra vigga 37 - balance
clear
close all
%% plot ballance - hand held - partly on floor
dd1tr = load('square_tr.txt');
dd1ttr = load('square_time_tr.txt');
dd1 = load('square_1.txt');
dd12 = load('square_2.txt');
dd2 = load('square_gulv.txt');
dd3 = load('square_gulv_2.txt');
% logfile from robot Birte (60)
% Birte (60)
%  1    time 0.001 sec
%  2  3  4   (mission 0), state 2, thread 2, line 0
%  5  6 Motor voltage [V] left, right: 0.0 0.0
%  7  8 Motor current left, right [A]: 0.048 0.055
%  9 10 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 11 12 13 14 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 3.12417
% 15    Battery voltage [V]: 12.31

%%
dd = dd1ttr;
figure(2)
hold off
plot(dd(:,1), dd(:,5)/10,'r')
grid on
hold on
plot(dd(:,1), dd(:,6)/10,'b')
plot(dd(:,1), dd(:,7),'-.r')
plot(dd(:,1), dd(:,8),'-.b')
plot(dd(:,1), dd(:,9),'g')
plot(dd(:,1), dd(:,10),'c')
legend('left volt/10','right volt/10','left-cur  [A]','right cur [A]', 'left vel','right vel','Location','northwest')
title('Initial test new board')
%%
figure(3)
hold off
plot(dd(:,11),dd(:,12),'r')
hold on
%plot(dd12(:,11),dd12(:,12),'b')
set(gca,'FontSize',14)
grid on
axis equal
title('Robot Birte (60), Square - no controller')
xlabel('X [m]')
ylabel('Y [m]')
legend('CV square')
print('squareplot','-dpng')
