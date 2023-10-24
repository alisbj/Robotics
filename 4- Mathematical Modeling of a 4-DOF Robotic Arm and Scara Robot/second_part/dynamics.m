syms b1  theta2  theta3  b4 ...
    b1_dot  theta2_dot theta3_dot  b4_dot ...
    b1_ddot  theta2_ddot theta3_ddot  b4_ddot real

% variables
theta = [ b1 ; theta2 ; theta3 ; b4 ];
theta_dot = [ b1_dot ; theta2_dot; theta3_dot ; b4_dot];
theta_ddot = [ b1_ddot ; theta2_ddot; theta3_ddot ; b4_ddot];

%% dh params

 a1=0;
 a2=0.3971;
 a3=0.2508;
 a4=0;

 b2=0.121;
 b3=0.08;

 alpha1=pi/2;
 alpha2=0;
 alpha3=0;
 alpha4=0;

 theta1=0;
 theta4=0;

% all in WORLD frame
zero = [ 0 ; 0 ; 0];
z = [ 0 ; 0 ; 1];
Q0=[1 0 0 ; 0 0 1 ; 0 -1 0]

%% params
m1=0.8;
com1 = [0.15;0.08;0.01 ;1];
I1= [2000 2 160 ; 2 3000 1 ; 160 1 2000] * 1e-6;
T1 = [1 0 0 -0.2 ; 0 0 1 0 ; 0 -1 0 0 ; 0 0 0 1]
Q1 = [1 0 0 ; 0 0 1 ; 0 -1 0]
com1_DH=T1*com1;
com1_DH=com1_DH(1:3)
I1_DH=Q1*I1*Q1'

m2=1.2;
com2=[0.35;0.1;0.25;1];
I2=[1600 -0.25 -1200 ; -0.25 30000 0.05;-1200 0.05 30000] * 1e-6;
T2=[1 0 0 -0.073 ; 0 1 0 -0.083 ; 0 0 1 -0.079 ; 0 0 0 1]
Q2=eye(3,3)
com2_DH=T2*com2;
com2_DH=com2_DH(1:3)
I2_DH=Q2*I2*Q2'

m3=0.85;
com3=[0.45;0.1;0.3;1];
I3=[6000 -0.3 1300 ; -0.3 15000 -0.3 ; 1300 -0.3 12000] * 1e-6;
T3=[1 0 0 -0.4 ; 0 1 0 -0.09 ; 0 0 1 -0.2 ; 0 0 0 1]
Q3=eye(3,3)
com3_DH=T3*com3;
com3_DH=com3_DH(1:3)
I3_DH=Q3*I3*Q3'


m4=0.15;
com4=[0.65;0.02;0.4;1];
I4=[3000 0.07 0.04 ; 0.07 3000 0.1 ; 0.04 0.1 20] * 1e-6;
T4=[1 0 0 -0.65 ; 0 1 0 0 ; 0 0 1 -0.345 ; 0 0 0 1]
Q4=eye(3,3)
com4_DH=T4*com4;
com4_DH=com4_DH(1:3)
I4_DH=Q4*I4*Q4'

%%%%%%%%%%%dynamic

 Q1=dh_q(alpha1 , theta1);
 Q2=dh_q(alpha2 , theta2);
 Q3=dh_q(alpha3 , theta3);
 Q4=dh_q(alpha4 , theta4);

e1 = Q0*z;
e2 = Q0*Q1*z ;
e3 = Q0*Q1*Q2*z ;
e4 = Q0*Q1*Q2*Q3*z ;

b1_0 = 0.08;
theta2_0 = 0;
theta3_0 = 0;
b4_0 = 0;

 av1=dh_a(a1 , b1 , theta1);
 av2=dh_a(a2 , b2 , theta2);
 av3=dh_a(a3 , b3 , theta3);
 av4=dh_a(a4 , b4 , theta4);

theta1_0 = theta1;
theta4_0 = theta4;

r11 = Q0*(com1_DH+[0;0;b1] );

r12 =   Q0*(av1 + Q1*Qz( theta2 -theta2_0 )*(com2_DH) );
r22 =   Q0*Q1*Qz( theta2 - theta2_0)*(com2_DH) ;

r13 =   Q0*(av1 + Q1*av2 + Q1*Q2*Qz( theta3 - theta3_0)*(com3_DH) ) ;
r23 =   Q0*(Q1*av2 + Q1*Q2*Qz( theta3 - theta3_0)*(com3_DH)) ;
r33 =   Q0*(Q1*Q2*Qz( theta3 - theta3_0)*(com3_DH)) ;

r14 =   Q0*(av1 + Q1*av2 + Q1*Q2*av3 + Q1*Q2*Q3*(com4_DH+[0;0;b4])) ;
r24 =   Q0*(Q1*av2 + Q1*Q2*av3 + Q1*Q2*Q3*(com4_DH+[0;0;b4])) ;
r34 =   Q0*(Q1*Q2*av3 + Q1*Q2*Q3*(com4_DH+[0;0;b4]) );
r44 =   Q0*(Q1*Q2*Q3*(com4_DH+[0;0;b4]) ) ;

% Ni in world frame
N1 = [e1 zero zero zero   ];
N2 = [e1  cross(e2,r22)  zero            zero          ];
N3 = [e1  cross(e2,r23)  cross(e3,r33)   zero          ];
N4 = [e1  cross(e2,r24)  cross(e3,r34)   e4 ];

% Wi in i th frame <=== Ii in i th frame
W1 = [zero     zero               zero     zero];
W2 = [zero     z                     zero     zero];
W3 = [zero     Q2'*z             z           zero];
W4 = [zero     Q3'*Q2'*z     Q3'*z   zero];

M1 = m1 * N1'*N1 + W1'*I1_DH*W1;
M2 = m2 * N2'*N2 + W2'*Qz(theta2 - theta2_0)*I2_DH*Qz(theta2 - theta2_0)'*W2;
M3 = m3 * N3'*N3 + W3'*Qz(theta3 - theta3_0)*I3_DH*Qz(theta3 - theta3_0)'*W3;
M4 = m4 * N4'*N4 + W4'*I4_DH*W4;

M = M1 + M2 + M3 + M4  ;
T = 0.5*theta_dot'*M*theta_dot;

%hi all in world frame
h1 =  (Q0*(com1_DH+[0;0;b1]))'*z;
h2 =   (Q0*(av1 + Q1*Qz(theta2 - theta2_0)*com2_DH ))'*z;
h3 =   (Q0*(av1 + Q1*av2 + Q1*Q2*Qz(theta3 - theta3_0)*com3_DH))'*z;
h4 =   (Q0*(av1 + Q1*av2 + Q1*Q2*av3 + Q1*Q2*Q3*(com4_DH+[0;0;b4])))'*z;

g = 9.81;
v1 = m1 *g*h1;
v2 = m2 *g*h2;
v3 = m3 *g*h3;
v4 = m4 *g*h4;

V = v1 + v2 + v3 + v4 ;

M_dot = M_dot_generator(M,theta,theta_dot);
n = M_dot*theta_dot ...
    -jacobian(T , theta)' + jacobian(V,theta)';

tau = (M*theta_ddot + n);
tau=simplify(tau , 'Steps' , 100);

disp('Dynamic Equations :')
disp('//////////////////')
disp('f1 = ')
disp(vpa(tau(1) , 2 ))
disp('---------------------')

disp('tau2 = ')
disp(vpa(tau(2) , 2 ))
disp('---------------------')


disp('tau3 = ')
disp(vpa(tau(3) , 2 ))
disp('---------------------')

disp('f4 = ')
disp(vpa(tau(4) , 2 ))
disp('---------------------')


function [Q]=Qz(theta);
    Q=[cos(theta) , -sin(theta) , 0;
        sin(theta), cos(theta) , 0;
        0 , 0 ,1];
end

function [Q]=dh_q(alpha , theta);
    Q=[cos(theta) , -cos(alpha)*sin(theta) , sin(alpha)*sin(theta);
        sin(theta), cos(alpha)*cos(theta) , -sin(alpha)*cos(theta);
        0 , sin(alpha) , cos(alpha)];
end

function [a_i]=dh_a(a , b , theta);
    a_i = [a*cos(theta) ; a*sin(theta) ; b ];
end

function y = M_dot_generator(M,theta,theta_dot)
[m n] = size(M);

    for i = 1:m
        for j = 1:n
            y(i,j) =   jacobian(M(i,j),theta) * theta_dot;
        end
    end

y = simplify(y);
end