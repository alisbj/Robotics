% tau = dynamic_model()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
beta = atan(48/9)
bias =[ 0 ; beta ; -beta; 0 ] ;
theta_i_m = theta_i+bias;
theta_f_m = theta_f+bias;
p = 0;
Ts_M = Ts_M
TAU = zeros(4, round(1 + tf / Ts_M));

%% trapezoidal 
wpts = [0 1];
[s,s_prime,s_second,tvec,pp] = trapveltraj(wpts,100001);
for j = 0 : Ts_M : tf
    p = p + 1;
    
    t_n = j/tf;
 
    theta = theta_i_m + (theta_f_m - theta_i_m) * s(p);
    theta = theta - round( theta /2 / pi ) *2*pi;
    theta_dot = (theta_f_m - theta_i_m) * s_prime(p) / tf;
    theta_ddot = (theta_f_m - theta_i_m) * s_second(p) / tf^2;
    

    
    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    theta4 = theta(4);

    
    theta1_dot = theta_dot(1);
    theta2_dot = theta_dot(2);
    theta3_dot = theta_dot(3);
    theta4_dot = theta_dot(4);

    theta1_ddot = theta_ddot(1);
    theta2_ddot = theta_ddot(2);
    theta3_ddot = theta_ddot(3);
    theta4_ddot = theta_ddot(4);


    TAU(:,p) = OpenMan_torques(theta1,...
       theta2,theta3,theta4,...
       theta1_dot,theta2_dot,theta3_dot,...
       theta4_dot,theta1_ddot,theta2_ddot,...
       theta3_ddot,theta4_ddot);
    

end
