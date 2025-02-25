%% Simple Car Example 
% 2/17/25
% Tolemy N

clear all; clc; close all; format long; format compact;

%% State Space Equations 

% x_dot = A*x + B*u; % State Equation 
% y = C*x + D*u; % Output Equation 

% A is the system matrix 
% B is the input matrix
% C is the output matrix
% D is the feedforward matrix 

% x is the state vector 
% y is the output vector 

% u is the input vector 

%% System Parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Parameters 
m = 1; % Pendulum Mass, kg
b = 1; % Damping Coefficient (k), N/m
L = 1; % length of the surface; 
k = 1; % Spring Constant

x0 = 0; % [m/s]
% x0 = 0; % [m/s]

x0_2 = [0 0]'; % [m, m/s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control Input Signal, u

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 0.1; 
start_time = 0; % sec
stop_time = 50; % sec
% stop_time = 10; % sec

Total_points = (stop_time - start_time + dt)/dt;
t = start_time:dt:stop_time; 

% Or
% % % n = 2001; 
% % % dt_points = (stop_time - start_time)/(n-1);
% % % % n = (stop_time - start_time)/dt_points + 1 
% % % t = linspace(start_time, stop_time, n);

Start = 1; % Change the starting time (t_s). Start = 1 is equivalent to t>=0

% u = 1*ones(length(t), 1); % Unit step (1, t>=0 | 0, t<0)
u = zeros(length(t), 1);
u(Start:end) = 1*ones(length(t)-Start+1, 1); % Unit step (1, t>=0 | 0, t<0)

% % % u = t; % Ramp (t, t>=0 | 0, t<0)
% u = zeros(length(t), 1);
% u(Start:end) = t(1:end-Start+1); % Ramp (t, t>=0 | 0, t<0)

% % % u = t.^2/2; % Ramp (t, t>=0 | 0, t<0)
% u = zeros(length(t), 1);
% u(Start:end) = t(1:end-Start+1).^2./2; % Parabola (t.^2/2, t>=0 | 0, t<0)

% % % Impulse Response 
% u = zeros(length(t), 1); 
% u(51) = 1;  

% % % Sinusoidal Input
% Amp = 1;
% Omega = 1;
% u = zeros(length(t), 1);
% u(Start:end) = Amp*sin(Omega*t(Start:end)); % Parabola (A*sin(omega*t), t>=0 | 0, t<0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simple Car (1st order velocity)

% Equation of motion (EoM)

% m*x_dot_dot + b*x_dot = F(t)
% m*x_dot_dot + b*x_dot = u

% x_dot = v & x_dot_dot = v_dot
% m*v_dot + b*v = u
% v_dot = (-b/m)*v + (1/m)*u

A = -b/m;
B = 1/m;
% C = [1 0]; % track velocity, v(t) = x_dot(t)
C = 1; % track velocity, v(t) = x_dot(t)
D = 0;

% % Car Velocity over time
msd_sys = ss(A, B, C, D);
[yout, tout] = lsim(msd_sys, u, t, x0);
Sim_info_1 = lsiminfo(yout, tout);

v_sim = yout(:, 1);
 
figure(1)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, v_sim, 'LineWidth', 2)
legend("u, Signal", "x(t)")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$v(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Velocity, $v(t)$ of the car over time', 'Interpreter', 'latex', 'FontSize', 14)

%% Simple Car (2nd order x instead of v = x_dot)

% Equation of motion (EoM)

% m*x_dot_dot + b*x_dot = F(t)
% m*x_dot_dot + b*x_dot = u
% x_dot_dot = (-b/m)*x_dot = u

A = [0 1; 0 -b/m];
B = [0; 1/m];
% C = [1 0]; % track postion, x(t)
C = [1 0; 0 1]; % track postion, x(t) and velocity, v(t) = x_dot(t)
D = 0;

msd_sys = ss(A, B, C, D);
[yout, tout] = lsim(msd_sys, u, t, x0_2);
Sim_info_1 = lsiminfo(yout, tout);

x_sim = yout(:, 1);
v_sim = yout(:, 2);

% % Car Position and Velocity over time
figure(2)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, x_sim, 'LineWidth', 2)
legend("u, Signal", "x(t)", Location="best")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$x(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Displacement, $x(t)$ of the car over time', 'Interpreter', 'latex', 'FontSize', 14)

figure(3)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, v_sim, 'LineWidth', 2)
legend("u, Signal", "v(t)")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$v(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Velocity, $v(t)$ of the car over time', 'Interpreter', 'latex', 'FontSize', 14)