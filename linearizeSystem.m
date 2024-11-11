% Define the parameters based on inertia values
Parameters


p1 = (Iyy - Izz) / Ixx;
p2 = 1 / Ixx;
p3 = (Ixx - Iyy) / Iyy;
p4 = 1 / Iyy;
p5 = (Ixx - Iyy) / Izz;
p6 = 1 / Izz;

% Define symbolic variables for states and inputs
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 real % State variables
syms U1 U2 U3 U4 real % Control inputs and constants

% Define the system of equations
dx1 = x2;
dx2 = p1 * x4 * x6 + p2 * U2;
dx3 = x4;
dx4 = p3 * x2 * x6 + p4 * U3;
dx5 = x6;
dx6 = p5 * x2 * x4 + p6 * U4;
dx7 = x8;
dx8 = ((cos(x1) * sin(x3) * cos(x5) + sin(x1) * sin(x5)) / m) * U1;
dx9 = x10;
dx10 = ((-cos(x1) * sin(x3) * sin(x5) + sin(x1) * cos(x5)) / m) * U1;
dx11 = x12;
dx12 = -g + ((cos(x3) * cos(x1)) / m) * U1;

% Group the equations into a vector for easy handling
dx = [dx1; dx2; dx3; dx4; dx5; dx6; dx7; dx8; dx9; dx10; dx11; dx12];
z = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12];
u = [U1 U2 U3 U4];
% Display the system of equations
disp('System of first-order differential equations:');
disp(dx);


%% Linearize

% Compute Jacobians
A = jacobian(dx, z); % Jacobian with respect to state variables
B = jacobian(dx, u); % Jacobian with respect to control inputs

% Display the Jacobian matrices
disp('Jacobian A (df/dx):');
disp(A);

disp('Jacobian B (df/du):');
disp(B);

% Substitute equilibrium point into A and B
A_eq = subs(A, [x1, x4, x6, x3, x5, x7, x9, x11, U1], [0, 0,0, 0, 0, 4, 4, 0,m*g]);
B_eq = subs(B, [x1, x4, x6, x3, x5, x7, x9, x11, U1], [0, 0,0, 0, 0, 4, 4, 0,m*g]);

A_eq1 = double(A_eq);
B_eq1 = double(B_eq);

disp('Linearized Jacobian A at equilibrium:');
disp(A_eq);

disp('Linearized Jacobian B at equilibrium:');
disp(B_eq);
%% Calculate tranfer matrix
% we consider as system outputs x y z and psi
C = diag([0 0 0 0 1 0 1 0 1 0 1 0]);
D = zeros([12, 4]);

sys = ss(A_eq1, B_eq1, C, D);
systf = tf(sys);
syms s
tfz = poly2sym(sym(systf.Numerator{11,1}), s) / poly2sym(sym(systf.Denominator{11,1}), s);
tfy = poly2sym(sym(systf.Numerator{9,2}), s) / poly2sym(sym(systf.Denominator{9,2}), s);
tfx = poly2sym(sym(systf.Numerator{7,3}), s) / poly2sym(sym(systf.Denominator{7,3}), s);
tfpsi = poly2sym(sym(systf.Numerator{5,4}), s) / poly2sym(sym(systf.Denominator{5,4}), s);

%% Design LQR controller 

q_phi   = 3;
q_th    = 3;
q_psi   = 3;
q_x = 1;
q_y = 1;
q_z = 15;

q_phi_dot   = 0.01;
q_th_dot    = 0.01;
q_psi_dot   = 0.1;
q_x_dot = 0.2;
q_y_dot = 0.2;
q_z_dot = 1;

qDiagElements = [q_phi q_phi_dot q_th q_th_dot q_psi q_psi_dot q_x q_x_dot q_y q_y_dot q_z q_z_dot];
r = [1 0.1 0.1 0.1];

Q = diag(qDiagElements);
R = diag(r);

[~, K, ~] = icare(A_eq1, B_eq1, Q, R);
K(abs(K)<0.01) = 0;