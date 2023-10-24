close all;
clear all;
clc
syms b1 theta2 theta3 b4
a1 = [0;0;b1];
a2 = [0.3971*cos(theta2);0.3971*sin(theta2);0.121];
a3 = [0.2508*cos(theta3);0.2508*sin(theta3);0.08];
a4 = [0;0;b4];
Q1 = [1 0 0;0 0 -1;0 1 0];
Q2 = [cos(theta2) -sin(theta2) 0;sin(theta2) cos(theta2) 0;0 0 1];
Q3 = [cos(theta3) -sin(theta3) 0;sin(theta3) cos(theta3) 0;0 0 1];
Q4 = [1 0 0;0 1 0;0 0 1];
p = vpa(simplify(a1 + Q1*a2 + Q1*Q2*a3 + Q1*Q2*Q3*a4))
Q = vpa(simplify(Q1*Q2*Q3*Q4))
