clc

close all
%% trapezoidal trajectory
wpts = [0 1];
[s,s_prime,s_second,tvec,pp] = trapveltraj(wpts,100001);

%% initial and final joint angles (considering the bias)
BETA = atan(48/9);
theta_i = [0 ; 0 ; 0 ; 0 ] ;
theta_f = [-2*pi/3 ; pi/3-BETA ; -pi/3+BETA ; pi/4 ];
%%
tf = 1;
Ts_M = 0.00001;
Ts = 0.001;
T = 0 : Ts_M : tf;
%%
p=0;
for j = 0 : Ts_M : tf
    p = p + 1;
    t_n = j/tf;

   
    theta = theta_i + (theta_f - theta_i) * s(p);
    theta = (theta - round( theta /2 / pi ) *2*pi);
    theta_dot = ((theta_f - theta_i) * s_prime(p) / tf);
    theta_ddot = ((theta_f - theta_i) * s_second(p) / tf^2);   
    
    theta1_num(p) = theta(1);
    theta2_num(p) = theta(2);
    theta3_num(p) = theta(3);
    theta4_num(p) = theta(4);
    
    theta1_dot_num(p) = theta_dot(1);
    theta2_dot_num(p) = theta_dot(2);
    theta3_dot_num(p) = theta_dot(3);
    theta4_dot_num(p) = theta_dot(4);
    
    theta1_ddot_num(p) = theta_ddot(1);
    theta2_ddot_num(p) = theta_ddot(2);
    theta3_ddot_num(p) = theta_ddot(3);
    theta4_ddot_num(p) = theta_ddot(4);
  
  

end
%%

theta1_timeseries = timeseries(theta1_num,T);
theta2_timeseries = timeseries(theta2_num,T);
theta3_timeseries = timeseries(theta3_num,T);
theta4_timeseries = timeseries(theta4_num,T);


%%
close all
markers = {'default','-.','--',':'};
figure()
hold on
plot(0 : Ts_M : tf , s / max(s) ,'LineStyle',markers{1},'LineWidth',2)
plot(0 : Ts_M : tf , s_prime/max(s_prime) ,'LineStyle',markers{3},'LineWidth',2)
plot(0 : Ts_M : tf , s_second/max(s_second) ,'LineStyle',markers{4},'LineWidth',2)
axis( [0 1 -1.5 2])
xlabel( '$t_n$(s)','interpreter','latex')
legend('$s$','$s''/s''_{max}$','$s''''/s''''_{max}$','Interpreter','latex')
grid('on')