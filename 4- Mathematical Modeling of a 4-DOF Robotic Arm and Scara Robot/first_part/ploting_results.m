%%
% I plot all tau in one figure

close all

figure
plot(0:Ts_M:tf , TAU')
hold on
plot(torque.time , torque.Data)
legend('tau_1' , 'tau_2','tau_3','tau_4','tau1_{sim}' , 'tau2_{sim}','tau3_{sim}','tau4_{sim}')
print('dynamic_model_validation','-depsc')
title('joints torque using simscape and dynamic model')
xlabel('t')
ylabel('\tau (N.m)')

