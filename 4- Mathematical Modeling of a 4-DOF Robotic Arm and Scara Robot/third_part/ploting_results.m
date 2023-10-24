clc
close all

time_dim = reference.time;


figure
subplot(4,1,1)
plot(time_dim , output.Data(:,1))
hold on
plot(time_dim , reference.Data(:,1))
legend('b1 refrence','b1 2th robot')

subplot(4,1,2)
plot(time_dim , output.Data(:,2))
hold on
plot(time_dim , reference.Data(:,2))
legend('theta2 refrence','theta2 2th robot')
xlabel('t')

subplot(4,1,3)
plot(time_dim , output.Data(:,3))
hold on
plot(time_dim , reference.Data(:,3))
legend('theta3 refrence','theta3 2th robot')
xlabel('t')

subplot(4,1,4)
plot(time_dim , output.Data(:,4))
hold on
plot(time_dim , reference.Data(:,4))
legend('b4 refrence','b4 2th robot')
xlabel('t')
