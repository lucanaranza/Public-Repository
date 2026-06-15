clc
clear
close all

data = readmatrix('data.txt');

time = data(54801:end,9);
setpoint = data(54801:end,2);
roll = data(54801:end,18);
servo = -data(54801:end,21).*180./pi;

figure(1)
xlabel("Time [s]");
ylabel("Angles [°]");
plot(time,setpoint, 'g');
hold on 
plot(time,roll, 'b');
plot(time,servo, 'r');
legend ("setpoint","roll","servoangle");