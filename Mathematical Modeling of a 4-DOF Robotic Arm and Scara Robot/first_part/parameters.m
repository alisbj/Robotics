clc
clear all
close all

%%
Laser_length = 3;
quad  = [ -30 30;-2.5 30;-2.5 70;2.5,70;2.5,30;30 ,30;...
    30,2.5;70,2.5;70,-2.5;30,-2.5;30,-30;...
    2.5,-30;2.5,-70;-2.5,-70;-2.5,-30;-30,-30;...
    -30,-2.5;-70,-2.5;-70,2.5;-30,2.5]*0.5;

quad = flipud(quad);

quad_l = 3;
%%
plot(quad(:,1),quad(:,2))
%%
close all
propeller = 8;
prop  = [ -propeller -0.5; -propeller 0.5 ; propeller 0.5 ; propeller -0.5];
prop = flipud(prop);
prop_l = 0.2;
prop_dist = 33;
% plot(prop(:,1),prop(:,2))

%%%
foot_size = 20;
quad_foot = [ -foot_size -1; -foot_size 1 ; foot_size 1; foot_size -1];
quad_foot = flipud(quad_foot);
foot_h = 30;

%%
mass_link1 = 7.9119962e-02;

com_link1_ros = [3.0876154e-04 0.0000000e+00 -1.2176461e-04]';

I1_ros = [1.2505234e-05 , 0.0           , -1.7855208e-07
            0.0         ,  2.1898364e-05, 0.0
         -1.7855208e-07 ,  0.0          , 1.9267361e-05];
 
%% 
mass_link2 = 9.8406837e-02;

com_link2_ros = [-3.0184870*10^-4, 5.4043684*10^-4, 0.018 + 2.9433464*10^-02]';

I2_ros = [3.4543422e-05 , -1.6031095e-08, -3.8375155e-07
     -1.6031095e-08,  3.2689329e-05, 2.8511935e-08
     -3.8375155e-07,  2.8511935e-08, 1.8850320e-05];
%%
mass_link3 = 1.3850917e-01;

com_link3_ros = [1.0308393e-02, 3.7743363e-04, 1.0170197e-01]';

I3_ros = [3.3055381e-04, -9.7940978e-08, -3.8505711e-05
      -9.7940978e-08, 3.4290447e-04, -1.5717516e-06
     -3.8505711e-05, -1.5717516e-06, 6.0346498e-05];
%%

mass_link4 = 1.3274562e-01;

com_link4_ros = [9.0909590e-02, 3.8929816e-04, 2.2413279e-04]';

I4_ros = [3.0654178e-05, -1.2764155e-06, -2.6874417e-07
         -1.2764155e-06,  2.4230292e-04,  1.1559550e-08
         -2.6874417e-07,  1.1559550e-08,  2.5155057e-04];

%%     
mass_link5 = 1.4327573e-01;

com_link5_ros = [4.4206755e-02, 3.6839985e-07, 8.9142216e-03]';

I5_ros = [8.0870749e-05, 0.0          , -1.0157896e-06
          0.0          , 7.5980465e-05,  0.0
      -1.0157896e-6   , 0.0          , 9.3127351e-05];
%%
mass_subgripper_l = 1.0e-03;

com_subgripper_l_ros = [0 0 0]';

I_subgripper_l_ros = [1.0e-03   ,      0.0       ,  0.0
                      0.0       ,  1.0e-03       ,	0.0
                      0.0       ,      0.0       ,  1.0e-03];
%%
mass_subgripper_r = 1.0e-03;

com_subgripper_r_ros = [0 0 0]';

I_subgripper_r_ros =[1.0e-03   ,      0.0       ,  0.0
                      0.0       ,  1.0e-03       ,	0.0
                      0.0       ,      0.0       ,  1.0e-03];


%%

I1 = I1_ros;
I1_mom = [I1(1,1) , I1(2,2) , I1(3,3)];
I1_prod = [I1(2,3) , I1(1,3) , I1(1,2)];
com1 = com_link1_ros;




%%

I2 = I2_ros ;
I2_mom = [I2(1,1) , I2(2,2) , I2(3,3)];
I2_prod = [I2(2,3) , I2(1,3) , I2(1,2)];
com2 =  com_link2_ros;



%%

I3 =  I3_ros;
I3_mom = [I3(1,1) , I3(2,2) , I3(3,3)];
I3_prod = [I3(2,3) , I3(1,3) , I3(1,2)];
com3 = com_link3_ros;




%%

I4 =  I4_ros ;
I4_mom = [I4(1,1) , I4(2,2) , I4(3,3)];
I4_prod = [I4(2,3) , I4(1,3) , I4(1,2)];
com4 =  com_link4_ros;


%%

I5 = I5_ros;
I5_mom = [I5(1,1) , I5(2,2) , I5(3,3)];
I5_prod = [I5(2,3) , I5(1,3) , I5(1,2)];
com5 =  com_link5_ros;



%%

I_gripper_l = I_subgripper_l_ros;
I_gripper_l_mom = [I_gripper_l(1,1) , I_gripper_l(2,2) , I_gripper_l(3,3)];
I_gripper_l_prod = [I_gripper_l(2,3) , I_gripper_l(1,3) , I_gripper_l(1,2)];
com_gripper_l =  com_subgripper_l_ros;

%%

I_gripper_r = I_subgripper_r_ros;
I_gripper_r_mom = [I_gripper_r(1,1) , I_gripper_r(2,2) , I_gripper_r(3,3)];
I_gripper_r_prod = [I_gripper_r(2,3) , I_gripper_r(1,3) , I_gripper_r(1,2)];
com_gripper_r =  com_subgripper_r_ros;

%%
beta = atan(128/24);



QQ1 = eye(3);
bb1 = [0 ; 0 ; -0.017-0.0195 ];
T1 = [QQ1 bb1
      0 0 0 1] ;
  
invT1  = [QQ1' -bb1
      0 0 0 1] ;
com1_dh = invT1 * [com2;1];
com1_dh = com1_dh(1:3);
I1_dh = QQ1' * I2 * QQ1;




QQ2 = Qx(pi/2);

com2_dh = QQ2'  *com3;
I2_dh = QQ2' * I3 * QQ2;




beta = atan(128/24);

QQ3 = Qx(pi/2)*Qz(beta);
com3_dh = QQ3' * com4;
I3_dh = QQ3' * I4 * QQ3;

%%
% end effector
QQ4 = Qx(pi/2);

mass_EE = mass_link5 + mass_subgripper_r + mass_subgripper_l;

com4_dh = QQ4' * (com5 * mass_link5 + (com_gripper_r + [0.0817;-0.021;0] ) * mass_subgripper_r ...
    + (com_gripper_l + [0.0817;0.021;0]) *mass_subgripper_l )/mass_EE;

R_EE = com5 - com4_dh;
R_r = com_gripper_r +[0.0817;-0.021;0]- com4_dh;
R_l = com_gripper_l + [0.0817;0.021;0]- com4_dh;
%%
J_EE = I5 + mass_link5*(R_EE'*R_EE*eye(3)- R_EE*R_EE');
J_gripper_r = I_gripper_r + mass_subgripper_r*(R_r'*R_r*eye(3)- R_r*R_r');
J_gripper_l = I_gripper_l + mass_subgripper_l*(R_l'*R_l*eye(3)- R_l*R_l');
% I4_dh = QQ4' * (I5 + I_gripper_l + I_gripper_r )* QQ4;
I4_dh = QQ4' * (J_EE+ J_gripper_r + J_gripper_l )* QQ4;

%%
QQ0 = [-1 0 0
    0 -1 0
    0 0 1];

com0_dh = QQ0' * com1;
I0_dh = QQ0' * I1 * QQ0;
%%










