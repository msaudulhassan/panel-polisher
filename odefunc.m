% Filename: odefunc.m
% Written By: Muhammad Saud Ul Hassan
% External Dependencies: None

function x_dot = odefunc(t, x, params)
    %% Function defining the system dynamics
    % Arguments:
    % ----------
    % t: time (s)
    % x: state vector
    % params: vector of system parameters
    %
    % Outputs:
    % --------
    % x_dot: vector of derivatives of states
    
    m1 = params(1);  % Mass of Link 1
    m2 = params(2);  % Mass of Link 2
    L = params(3);  % Length of Link 1 (Link 2's length is simply 2*L)
    g = params(4);  % Gravitational Acceleration
    psi = params(5);  % Angle of Inclination of the panel
    gamma = params(6);  % Angle at which external force acts
    
    % Evaluating the magnitude of external force at current time point
    A = 1;
    B = 2*pi/1.5;
    F_mag = A*t*sin(B*t);
    
    % Unpacking the state vector
    theta1 = x(1);
    dtheta1 = x(2);
    theta2 = x(3);
    dtheta2 = x(4);
    
    % System dynamics
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
    
    % Packing state derivatives into the vector x_dot
    x_dot(1) = dtheta1;
    x_dot(2) = y(1);
    x_dot(3) = dtheta2;
    x_dot(4) = y(2);

    % Converting x_dot from a row vector to a column vector
    x_dot = x_dot';
    
end