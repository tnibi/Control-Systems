%% Mass Spring Dampers With Two Applied Forces 
% 2/17/25 - Present 
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

%% Two Mass Spring Dampers all parameters 

% Case numbers 
% Case_Num = 1; % 1 for both carts with all parameters.  

% Case_Num = 2; % 2 for damper only on cart 1 (c2 = 0)
% Case_Num = 3; % 3 for damper only on cart 2 (c1 = 0)
% Case_Num = 4; % 4 for no damping (c1 = 0 and c2 = 0)

% Case_Num = 5; % 5 for spring only on cart 1 (k2 = 0)
% Case_Num = 6; % 6 for spring only on cart 2 (k1 = 0)
% Case_Num = 7; % 7 for no springs (k1 = 0 and k2 = 0)

% Case_Num = 8; % 8 for cart 1 with (c1 = 0 and k1 = 0) and cart 2 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0_2_DOF = [0 0 0 0]'; % Initial Conditions (ICs)
Case_Num_double_cart = 1; % edit this for the cases
Applied_Force = 3; % 1 for force at cart 1 and 2 for force at cart 2, 
% 3 for forces on both carts the control input, u = [u1 u2]

Double_Param.m_1 = 1; % Cart Mass, kg (Cart 1)
Double_Param.m_2 = 1; % Cart Mass, kg (Cart 2)

Double_Param.k_1 = 1; % Spring Coefficient, N/m
Double_Param.k_2 = 1; % Spring Coefficient, N/m

Double_Param.c_1 = 1; % Damper Coefficient, N*s/m or kg/s
Double_Param.c_2 = 1; % Damper Coefficient, N*s/m or kg/s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[m_1, m_2, k_1, k_2, c_1, c_2] = ...
    Double_cart_system_parameters(Case_Num_double_cart, Double_Param);

%% Control Input Signal, u

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 0.1; 
start_time = 0; % sec
stop_time = 50; % sec

% % % Total_points = (stop_time - start_time + dt)/dt 
% % t = start_time:dt:stop_time; 

% Or
n = 501; 
dt_points = (stop_time - start_time)/(n-1);
% n = (stop_time - start_time)/dt_points + 1 
t = linspace(start_time, stop_time, n);

Start = 1; % Change the starting time (t_s). Start = 1 is equivalent to t>=0

% % % u = 1*ones(length(t), 1); % Unit step (1, t>=0 | 0, t<0)
% % u = zeros(length(t), 1);
% % u(Start:end) = 1*ones(length(t)-Start+1, 1); % Unit step (1, t>=0 | 0, t<0)
% % u = [u u];

u1 = zeros(length(t), 1);
u1(Start:end) = 1*ones(length(t)-Start+1, 1); % Unit step (1, t>=0 | 0, t<0)

u2 = zeros(length(t), 1);
u2(Start:end) = 1*ones(length(t)-Start+1, 1); % Unit step (1, t>=0 | 0, t<0)
u = [u1 u2];

% % % u = t; % Ramp (t, t>=0 | 0, t<0)
% % u = zeros(length(t), 1);
% % u(Start:end) = t(1:end-Start+1); % Ramp (t, t>=0 | 0, t<0)
% % u = [u u];

% u1 = zeros(length(t), 1);
% u1(Start:end) = 1*t(1:end-Start+1); % Ramp (t, t>=0 | 0, t<0)
% 
% u2 = zeros(length(t), 1);
% u2(Start:end) = 1*t(1:end-Start+1); % Ramp (t, t>=0 | 0, t<0)
% u = [u1 u2];

% % % u = t.^2/2; % Ramp (t, t>=0 | 0, t<0)
% % u = zeros(length(t), 1);
% % u(Start:end) = 1*t(1:end-Start+1).^2./2; % Parabola (t.^2/2, t>=0 | 0, t<0)
% % u = [u u];

% u1 = zeros(length(t), 1);
% u1(Start:end) = 1*t(1:end-Start+1).^2./2; % Parabola (t.^2/2, t>=0 | 0, t<0)

% u2 = zeros(length(t), 1);
% u2(Start:end) = 1*t(1:end-Start+1).^2./2; % Parabola (t.^2/2, t>=0 | 0, t<0)
% u = [u1 u2];

% % % Impulse Response 
% % u = zeros(length(t), 1); 
% % u(51) = 1;  
% % u = [u1 u2];

% u1 = zeros(length(t), 1); 
% u1(51) = 1;  

% u2 = zeros(length(t), 1); 
% u2(51) = 1;  
% u = [u1 u2];

% % % Sinusoidal Input
% % Amp = 1;
% % Omega = 1;
% % u = zeros(length(t), 1);
% % u(Start:end) = Amp*sin(Omega*t(Start:end)); % Parabola (A*sin(omega*t), t>=0 | 0, t<0)
% % u = [u1 u2];

% % Amp1 = 1;
% % Omega1 = 1;
% u1 = zeros(length(t), 1);
% u1(Start:end) = Amp1*sin(Omega1*t(Start:end)); % Parabola (A*sin(omega*t), t>=0 | 0, t<0)

% % Amp2 = 1;
% % Omega2 = 1;
% u2 = zeros(length(t), 1);
% u2(Start:end) = Amp2*sin(Omega2*t(Start:end)); % Parabola (A*sin(omega*t), t>=0 | 0, t<0)
% u = [u1 u2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Two Mass Spring Dampers

A = [0 1 0 0; ...
    -(k_1 + k_2)/m_1 -(c_1 + c_2)/m_1 k_2/m_1 c_2/m_1; ...
    0 0 0 1; ...
    k_2/m_2 c_2/m_2 -k_2/m_2 -c_2/m_2];

if Applied_Force == 1
% B = [0; 1/m_1; 0; 0]; % control input, u (applied force) at cart # 1
B = [0 0; 1/m_1 0 ; 0 0; 0 0]; % control input, u (applied force) at both carts

elseif Applied_Force == 2
% B = [0; 0; 0; 1/m_2]; % control input, u (applied force) at cart # 2
B = [0 0 ; 0 0 ; 0 0; 0 1/m_2]; % control input, u (applied force) at both carts

elseif Applied_Force == 3
% B = [0; 1/m_1; 0; 1/m_2]; % control input, u (applied force) at both carts
B = [0 0 ; 1/m_1 0 ; 0 0; 0 1/m_2]; % control input, u (applied force) at both carts
end

C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; % Track all states 

D = 0;

% P = [B A*B A^2*B A^(n-1)*B];
P = ctrb(A, B);

fprintf('Two Mass Spring Dampers\n')
if rank(P) ~= size(A, 1)
fprintf('The system is uncontrollable because rank(P) = %g \n', rank(P))
elseif rank(P) == size(A, 1)
fprintf('The system is controllable because rank(P) = %g \n', rank(P))
end

% Q = [C; C*A; C*A^2; ... ; C*A^(n-1)]
Q = obsv(A, C);
if rank(Q) ~= size(A, 1)
fprintf('The system is unobservability because rank(Q) = %g \n\n', rank(Q))
elseif rank(Q) == size(A, 1)
fprintf('The system is observability because rank(Q) = %g \n\n', rank(Q))
end

%% Displacement and Velocity: Two Mass Spring Dampers

msd_sys = ss(A, B, C, D);
[yout, tout] = lsim(msd_sys, u, t, x0_2_DOF);
Sim_info_2 = lsiminfo(yout, tout);

figure(11)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, yout(:, 1), 'LineWidth', 2)
legend("u, Signal", "x_1(t)")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$x(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Displacement, $x_1(t)$ of cart 1 over time', 'Interpreter', 'latex', 'FontSize', 14)

figure(12)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, yout(:, 3), 'LineWidth', 2)
legend("u, Signal", "x_2(t)")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$x(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Displacement, $x_2(t)$ of cart 2 over time', 'Interpreter', 'latex', 'FontSize', 14)

figure(13)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, yout(:, [1 3]), 'LineWidth', 2)
legend("u, Signal", "x_1(t)", "x_2(t)")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$x(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Displacement, $x(t)$ of the carts over time', 'Interpreter', 'latex', 'FontSize', 14)

figure(14)
clf
plot(t, u, 'LineWidth', 1)
hold on
plot(tout, yout(:, [2 4]), 'LineWidth', 2)
legend("u, Signal", "v_1(t)", "v_2(t)")
grid on

xlabel('$t(s)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$v(t)$', 'Interpreter', 'latex', 'FontSize', 14)
title('Velocity, $v(t)$ of the cart over time', 'Interpreter', 'latex', 'FontSize', 14)

% x vs v
figure(15)
clf
% hold on
% comet(theta, omega)
% plot(yout(:, [1 3]), yout(:, [2 4]), 'LineWidth', 2)
plot(yout(:, 1), yout(:, 2), 'LineWidth', 2)
hold on
plot(yout(:, 3), yout(:, 4), 'r--', 'LineWidth', 2)
grid on

xlabel("$x(t)$", 'Interpreter', 'latex', 'FontSize', 14)
ylabel("$v(t)$", 'Interpreter', 'latex', 'FontSize', 14)
title('$x(t)$ vs $v(t)$ of the Carts over time', 'Interpreter', 'latex', 'FontSize', 14)
legend('Cart 1', 'Cart 2')

%% Functions 

function [m_1, m_2, k_1, k_2, c_1, c_2] = Double_cart_system_parameters(Case_Num, Double_Param)

m_1 = Double_Param.m_1;
m_2 = Double_Param.m_2;
if Case_Num <= 0 || Case_Num > 9
error("Need an integer between 1 and 8")

% Case_Num = 1; % 1 for both carts with all parameters.  
elseif Case_Num == 1
k_1 = Double_Param.k_1; % Spring Coefficient, N/m (Cart 1)
k_2 = Double_Param.k_2; % Spring Coefficient, N/m (Cart 2)

c_1 = Double_Param.c_1; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = Double_Param.c_1; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 2; % 2 for damper only on cart 1 (c2 = 0)
elseif Case_Num == 2
k_1 = Double_Param.k_1; % Spring Coefficient, N/m (Cart 1)
k_2 = Double_Param.k_2; % Spring Coefficient, N/m (Cart 2)

c_1 = Double_Param.c_1; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = 0; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 3; % 3 for damper only on cart 2 (c1 = 0)
elseif Case_Num == 3
k_1 = Double_Param.k_1; % Spring Coefficient, N/m (Cart 1)
k_2 = Double_Param.k_2; % Spring Coefficient, N/m (Cart 2)

c_1 = 0; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = Double_Param.c_2; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 4; % 4 for no damping (c1 = 0 and c2 = 0)
elseif Case_Num == 4
k_1 = Double_Param.k_1; % Spring Coefficient, N/m (Cart 1)
k_2 = Double_Param.k_2; % Spring Coefficient, N/m (Cart 2)

c_1 = 0; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = 0; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 5; % 5 for spring only on cart 1 (k2 = 0)
elseif Case_Num == 5
k_1 = Double_Param.k_1; % Spring Coefficient, N/m (Cart 1)
k_2 = 0; % Spring Coefficient, N/m (Cart 2)

c_1 = Double_Param.c_1; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = Double_Param.c_2; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 6; % 6 for spring only on cart 2 (k1 = 0)
elseif Case_Num == 6
k_1 = 0; % Spring Coefficient, N/m (Cart 1)
k_2 = Double_Param.k_2; % Spring Coefficient, N/m (Cart 2)

c_1 = Double_Param.c_1; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = Double_Param.c_2; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 7; % 7 for no springs (k1 = 0 and k2 = 0)
elseif Case_Num == 7
k_1 = 0; % Spring Coefficient, N/m (Cart 1)
k_2 = 0; % Spring Coefficient, N/m (Cart 2)

c_1 = Double_Param.c_1; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = Double_Param.c_2; % Damper Coefficient, N*s/m or kg/s (Cart 2)

% Case_Num = 8; % 8 for cart 1 with (c1 = 0 and k1 = 0) and cart 2 
elseif Case_Num == 8
k_1 = 0; % Spring Coefficient, N/m (Cart 1)
k_2 = Double_Param.k_2; % Spring Coefficient, N/m (Cart 2)

c_1 = 0; % Damper Coefficient, N*s/m or kg/s (Cart 1)
c_2 = Double_Param.c_2; % Damper Coefficient, N*s/m or kg/s (Cart 2)

end
end