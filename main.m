%% Solves polishing robot's dynamics and finds joint torques
% This script solves the polishing robot dynamics by using ODE45 and
% determines the joint torques required in order to confine the polishing
% head to the panel's surface
%
% Filename: main.m
% Written by: Muhammad Saud Ul Hassan
% Dated: 23-Apr-2020
% External Dependencies: odefun.m


clear, clc;

%% Defining system parameters
m1 = 1;  % Mass (kg) of Link 1
m2 = 2;  % Mass (kg) of Link 2
L = 1;  % Length (m) of Link 1 (Link 2's length is simply 2*L)
g = 0;  % Gravitational acceleration (m/s^2)
psi = deg2rad(60);  % The angle of inclination of the panel surface
gamma = deg2rad(60);  % The angle at which the polishing head applies force

% Packing all the parameters into a params vector
params = [m1; m2; L; g; psi; gamma];

%% Initial state vector (initial conditions)
theta1_0 = deg2rad(50);  % Initial angle of Link 1 (rad)
dtheta1_0 = 0;  % Initial angular velocity of Link 1 (rad/s)
theta2_0 = deg2rad(25.8);  % Initial angle of Link 2 (rad)
dtheta2_0 = 0;  % Initial angular velocity of Link 2 (rad/s)

% Packing all the initial conditions into a vector
x_0 = [theta1_0; dtheta1_0; theta2_0; dtheta2_0];

%% Time interval
t_initial = 0; 
t_final = 10;
dt = 0.001;   %sample interval (s)
t = t_initial:dt:t_final; 

%% Solving the system using ODE45
odefunc_handle = @(t,x) odefunc(t, x, params);
[t, results] = ode45(odefunc_handle, t, x_0);

%% Extracting angle and angular velocity data from results
theta_1 = results(:, 1);
dtheta_1 = results(:, 2);
theta_2 = results(:, 3);
dtheta_2 = results(:,4);

%% Computing the joint torques and total energy of the system
for i=1:length(t)
    
    % Calculating the magnitude of user force at each time point
    A = 1;
    B = 2*pi/1.5;
    F_mag = A*t(i)*sin(B*t(i));
    Fmag(i) = F_mag;
    
    % Finding states at current time point and storing into dummy variables
    theta1 = theta_1(i);
    dtheta1 = dtheta_1(i);
    theta2 = theta_2(i);
    dtheta2 = dtheta_2(i);
    
    % Evaluating the system dynamics at each time points
    T_ddtheta1 = (m1/3 + m2)*L^2;
    T_ddtheta2 = m2*L^2*cos(theta1-theta2);
    T_lambda1 = L*cos(theta1-psi);
    T_0 = m2*L^2*sin(theta1-theta2)*dtheta2^2 + (m1/2 + m2)*L*g*cos(theta1) - F_mag*L*sin(gamma-theta1);
    
    Q_ddtheta1 = m2*L^2*cos(theta1-theta2);
    Q_ddtheta2 = (4/3)*m2*L^2;
    Q_lambda1 = 2*L*cos(theta2-psi);
    Q_0 = -m2*L^2*sin(theta1-theta2)*dtheta1^2 + m2*g*L*cos(theta2) - 2*F_mag*L*sin(gamma-theta2);
    
    R_ddtheta1 = L*cos(theta1-psi);
    R_ddtheta2 = 2*L*cos(theta2-psi);
    R_lambda1 = 0;
    R_0 = -L*sin(theta1-psi)*dtheta1^2 - 2*L*sin(theta2-psi)*dtheta2^2;
    
    A = [T_ddtheta1, T_ddtheta2, T_lambda1;
         Q_ddtheta1, Q_ddtheta2, Q_lambda1;
         R_ddtheta1, R_ddtheta2, R_lambda1];    
     
    y = A\[-T_0; -Q_0; -R_0];
        
    ddtheta_1(i) = y(1);
    ddtheta_2(i) = y(2);
    lambda(i) = y(3);
    
    % Inverse Dynamics for Calculating Joint Torques at each time point
    J = [-L*sin(theta1), -2*L*sin(theta2); L*cos(theta1), 2*L*cos(theta2)];
    T(:,i) = J' * -lambda(i)*[-sin(psi); cos(psi)];
    
    % Calculating the total energy of the system at each time point
    KE(i) = (1/6)*m1*L^2*dtheta1^2 + (1/2)*m2*L^2*dtheta1^2 + (2/3)*m2*L^2*dtheta2^2 + m2*L^2*cos(theta1-theta2)*dtheta1*dtheta2;
    PE(i) = (0.5*m1*L*g + m2*L*g)*sin(theta1) + m2*L*g*sin(theta2);
    TE(i) = KE(i) + PE(i);
    
end
    

%% Plots
% Angle vs time plot
figure;
plot(t, rad2deg(theta_1));
hold on
plot(t, rad2deg(theta_2));
legend("$\theta_1$", "$\theta_2$", "Interpreter", "Latex");
axis([0 10 0 100]);
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("Angle (degrees)", "Interpreter", "Latex");
grid on
hold off

% Angular velocity vs time plot
figure;
plot(t, dtheta_1);
hold on
plot(t, dtheta_2);
legend("$\dot{\theta_1}$", "$\dot{\theta_2}$", "Interpreter", "Latex");
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("Anglar Velocity ($\frac{rad}{s}$)", "Interpreter", "Latex");
axis([0 10 -5 5]);
grid on
hold off

% Angular acceleration vs time plot
figure;
plot(t, ddtheta_1);
hold on
plot(t, ddtheta_2);
legend("$\ddot{\theta_1}$", "$\ddot{\theta_2}$", "Interpreter", "Latex");
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("Anglar Acceleration ($\frac{rad}{s^2}$)", "Interpreter", "Latex");
axis([0 10 -10 10]);
grid on
hold off

% Constraint force vs time plot
figure;
plot(t, lambda);
legend("Constraint Force ($\lambda_1$)", "Interpreter", "Latex");
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("$\lambda_1$ (N)", "Interpreter", "Latex");
grid on
hold off

% External force vs time plot
figure;
plot(t, Fmag);
legend("User Force (F)", "Interpreter", "Latex");
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("$User Force (N)$", "Interpreter", "Latex");
grid on
hold off

% Joint torques vs time plot
figure;
plot(t, T(1,:));
hold on
plot(t, T(2,:));
legend("$\tau_1$", "$\tau_2$", "Interpreter", "Latex");
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("Joint Torques (N-m)", "Interpreter", "Latex");
grid on
hold off

% Energy for time plot
figure;
plot(t, KE);
hold on
plot(t, PE);
plot(t, TE);
legend("Kinetic Energy", "Potential Energy", "Total Energy");
grid on
xlabel("$t (s)$", "Interpreter", "Latex");
ylabel("Energy (J)");
title({"Please note that the external force is not zero", "in this case, and hence the changing", "Total Energy"});