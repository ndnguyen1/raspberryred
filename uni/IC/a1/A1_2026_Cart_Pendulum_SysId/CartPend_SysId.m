%% Assignment 1 - Q1. System Identification
% 48580 Intelligent Control Studio
% System Identification of a Cart-Pendulum System
%         
% University of Technology Sydney, Australia
% Autumn 2026
%
% Coordinator: A/Prof Ricardo P. Aguilera
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
format long
s = tf('s');

%% reading data
load('Teensy41_test_data_01.mat')
logsout

Nini = 1;
time = logsout{1}.Values.Time(Nini:end-1);
time = time - time(1);
Vm = logsout{1}.Values.Data(Nini:end-1);
alpha(:,1) = logsout{2}.Values.Data(Nini:end-1);
xc = logsout{3}.Values.Data(Nini:end-1);

alpha_deg = rad2deg(alpha);

%% sampling time from Teensy 4.1
ts = time(2);

%% plots saved data
figure(101)
subplot(311)
plot(time,Vm,'LineWidth',2)
title('SysId Data')
ylim([-8,8])
yticks([-6 0 6])
grid
ylabel('$u(t)$~[V]','FontSize',20,'Interpreter','latex')

subplot(312)
plot(time,xc,'LineWidth',2)
grid
ylabel('$x_c(t)$~[m]','FontSize',20,'Interpreter','latex')

subplot(313)
plot(time,alpha_deg,'LineWidth',2)
grid
ylabel('$\alpha (t)$~[deg]','FontSize',20,'Interpreter','latex')
xlabel('Time (s)','FontSize',20,'Interpreter','latex')

%% System Identification
%  Add your Least Square code here
% 


