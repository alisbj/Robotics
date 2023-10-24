clc
close all
tau = TAU';

figure
plot(0:Ts_M:tf , tau(:,1))
hold on
plot(torque.time , torque.Data(:,1))
legend('f_1','f1_{sim}')
title('F1 comparison between dynamic model and simulink')
print('dynamic_model_validation','-depsc')
xlabel('t')
ylabel('F1 (N)')


figure
plot(0:Ts_M:tf , tau(:,2))
hold on
plot(torque.time , torque.Data(:,2))
legend('tau_2','tau2_{sim}')
title('\tau2 comparison between dynamic model and simulink')
print('dynamic_model_validation','-depsc')
xlabel('t')
ylabel('\tau (N.m)')

figure
plot(0:Ts_M:tf , tau(:,3))
hold on
plot(torque.time , torque.Data(:,3))
legend('tau_3','tau3_{sim}')
title('\tau3 comparison between dynamic model and simulink')
print('dynamic_model_validation','-depsc')
xlabel('t')
ylabel('\tau (N.m)')

figure
plot(0:Ts_M:tf , tau(:,4))
hold on
plot(torque.time , torque.Data(:,4))
legend('f_4','f4_{sim}')
title('F4 comparison between dynamic model and simulink')
print('dynamic_model_validation','-depsc')
xlabel('t')
ylabel('F4 (N)')

