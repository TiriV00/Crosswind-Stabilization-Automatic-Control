clc
close all
clear

% symbolic definitions
syms mass psi rho Sx Cf g weight_part delta PbyI PbxI lx ly omega 
syms Sy Cs VbxI VbyI C1 C2 C3 C4 epsilon v_ang wheel_rad WxI WyI

sign_approx = @(x) tanh(1000*x); % Sostituzione di sign con tanh
abs_approx = @(x) sqrt(x^2 + epsilon);
norm_approx = @(x) sqrt(x(1)^2 + x(2)^2);

% plant formulas
Rbi = [cos(psi) sin(psi) 0;     % inertial-body
       -sin(psi) cos(psi) 0;
       0 0 1];

Rwb1 = [cos(delta) sin(delta) 0;  % body-wheel front wheels
       -sin(delta) cos(delta) 0;
       0 0 1];

Rwb3 = eye(3);  % wheel-body rear wheels
    
% calcolo dei mu

Pw1B = [lx ; ly ; 0];      % position vector of wheel i
Pw2B = [lx ; -ly ; 0];
Pw3B = [-lx ; -ly ; 0];
Pw4B = [-lx ; ly ; 0];
Pw1_2B = [lx; 0; 0];      % bycicle model positions
Pw3_4B = [-lx; 0; 0];

Rbi_der = [-sin(psi) cos(psi) 0;    % first derivative of Rbi
           -cos(psi) -sin(psi) 0;
           0 0 0];

VbI = [VbxI; VbyI; 0];  % intertial body velocity

Vw1I = VbI + Rbi_der' * Pw1_2B * omega; % wheel speed in intertial
Vw3I = VbI + Rbi_der' * Pw3_4B * omega;

Vw1W1 = Rwb1 * Rbi * Vw1I;    % wheel x speed in wheel
Vw3W3 = Rwb3 * Rbi * Vw3I;

% mu calculation
%subtraction1 = (v_ang * wheel_rad) - Vw1W1(1);
%maximum1 = (v_ang * wheel_rad) * (1 + sign(subtraction1)) * 0.5 +  Vw1W1(1) * (1-sign(subtraction1))*0.5;
lambda1 = 0; %(v_ang * wheel_rad - Vw1W1(1)) / (epsilon + maximum1 );

modulo1 = sqrt(Vw1W1(1)^2 + Vw1W1(2)^2);
beta1 = asin(Vw1W1(2)/modulo1);

mux1 = sign_approx(lambda1) * C1 * (1 - exp(-C2 * abs_approx(lambda1))) - C3 * lambda1;
muy1 = sign_approx(-beta1/(pi/2)) * C1 * (1 - exp(-C2 * abs_approx(-beta1/(pi/2)))) - C3 * (-beta1/(pi/2));

mux2 = mux1;
muy2 = muy1;

% electronic differential
%delta_omega = sign(psi) * omega * ly/wheel_rad;
%omega3 = v_ang + delta_omega;
%subtraction3 = (omega3 * wheel_rad) - Vw3W3(1);
%maximum3 = (omega3 * wheel_rad) * (1 + sign(subtraction3)) * 0.5 +  Vw3W3(1) * (1-sign(subtraction3))*0.5;

lambda3 = (v_ang * wheel_rad - Vw3W3(1)) / (epsilon + v_ang * wheel_rad);
modulo3 = sqrt(Vw3W3(1)^2 + Vw3W3(2)^2);
beta3 = asin(Vw3W3(2)/modulo3);
mux3 = sign_approx(lambda3) * C1 * (1 - exp(-C2 * abs_approx(lambda3))) - C3 * lambda3;
muy3 = sign_approx(-beta3/(pi/2)) * C1 * (1 - exp(-C2 * abs_approx(-beta3/(pi/2)))) - C3 * (-beta3/(pi/2));

%omega4 = v_ang - delta_omega;
%subtraction4 = (omega4 * wheel_rad) - Vw3W3(1);
%maximum4 = (omega4 * wheel_rad) * (1 + sign(subtraction4)) * 0.5 +  Vw3W3(1) * (1-sign(subtraction4))*0.5;
lambda4 = (v_ang * wheel_rad - Vw3W3(1)) / (epsilon + v_ang * wheel_rad);
mux4 = sign_approx(lambda4) * C1 * (1 - exp(-C2 * abs_approx(lambda4))) - C3 * lambda4;
muy4 = muy3;

% normal forces
N1 = mass * g * weight_part * 0.5;
N2 = N1;

N3 = mass * g * (1 - weight_part) * 0.5;
N4 = N3;

% force on wheels in wheel coordinates
Fw1xW1 = N1 * mux1;
Fw1yW1 = N1 * muy1;
Fw1W1 = [Fw1xW1; Fw1yW1; 0];

Fw2xW2 = N2 * mux2;
Fw2yW2 = N2 * muy2;
Fw2W2 = [Fw2xW2; Fw2yW2; 0];

Fw3xW3 = N3 * mux3;
Fw3yW3 = N3 * muy3;
Fw3W3 = [Fw3xW3; Fw3yW3; 0];

Fw4xW4 = N4 * mux4;   % wheel x forces
Fw4yW4 = N4 * muy4;   % wheel y forces
Fw4W4 = [Fw4xW4; Fw4yW4; 0];  % matrices of forces on each wheel

% force due to wind disturbance
%WxI = abs(W) * cos(csi);
%WyI = abs(W) * sin(csi);
WI = [WxI; WyI; 0];  % inertial wind speed

VaI = VbI + WI;  % intertial total velocity

%VaB = Rbi * VaI; % speed aero in body

%VaB_module = norm_approx(VaB);

Va_angle = acos(VaI(1)/norm_approx(VaI));
csi = acos(WxI/norm_approx(WI));

beta_aero = csi - Va_angle;

%FaxI = 0.5 * rho * Sx * Cf * sign(VaI(1)) * norm(VaI(1))^2;  % aerodynamic x forces body
%FayI = 0.5 * rho * Sy * Cs * sign(VaI(2)) * norm(VaI(2))^2;  % aerodynamic y forces body
FaxI = 0.5 * rho * Sx * Cf *  norm_approx(VaI)^2 * cos(beta_aero); % aerodynamic x forces body
FayI = 0.5 * rho * Sy * Cs * norm_approx(VaI)^2 * sin(beta_aero); % aerodynamic y forces body
FaI = [FaxI; FayI; 0];   % total forces in inertial coord
FaB = Rbi * FaI;        % total forces in body coord

FwB = Rwb1.' *(Fw1W1) + Rwb1.' *(Fw2W2) + Rwb3.' *(Fw3W3) + Rwb3.' *(Fw4W4);   % wheel forces in body coord
FwI = Rbi.' * FwB;  % wheel forces in inertial coord
abI = (FaI + FwI)/mass;     % x and y total acc in inertial coord

% yaw moment

IzB = 275458;      % yaw moment of inertia

laB = [lx; ly; 0];      % vector of body lenghts

MaB = cross(laB, FaB);   % aerodynamic torque on body

Mw1B = cross(Pw1B, (Rwb1.')*Fw1W1);   % torque on wheel i on body

Mw2B = cross(Pw2B, (Rwb1.')*Fw2W2);

Mw3B = cross(Pw3B, (Rwb3.')*Fw3W3);

Mw4B = cross(Pw4B, (Rwb3.')*Fw4W4);

a_psi = (Mw1B + Mw2B + Mw3B + Mw4B + MaB)/ IzB; % acceleration due to inertial


% SYMBOLIC MODEL

y = sym('y', [6 1]);
r = sym('r', [1 1]);
nu = sym('nu', [6 1]);

x = [PbxI;    % x pos
    PbyI;    % y pos
    VbI(1);    % x speed
    VbI(2);      % y speed
    psi;      % yaw angle
    omega];  % yaw rate

u = [delta;   % steering angle
    v_ang];   % wheel angular velocity

d = [WxI;     % wind speed
    WyI];    % wind incident angle

w = [d; nu; r];

f = [x(3);
    x(4); 
    abI(1); 
    abI(2); 
    x(6); 
    a_psi(3)];

h = [x(1) + nu(1); % GPS x position
     x(2) + nu(2); % visual system y position
     x(3) + nu(3); % GPS x speed
     x(4) + nu(4); % accelerometer/depth sensor y speed
     x(5) + nu(5); % magnetic angular position sensor yaw angle
     x(6) + nu(6)]; % gyroscope yaw rate

he = PbyI - r(1);
    

% matrices creation

A = jacobian(f, x);
B1 = jacobian(f, u);
B2 = jacobian(f, w);
C = jacobian(h, x);
D1 = jacobian(h, u);
D2 = jacobian(h, w);
Ce = jacobian(he, x);
D1e = jacobian(he, u);
D2e = jacobian(he, w);
matlabFunction(A, 'File', 'Amatrix')
matlabFunction(B1, 'File', 'B1matrix')
matlabFunction(B2, 'File', 'B2matrix')
matlabFunction(C, 'File', 'Cmatrix')
matlabFunction(D1, 'File', 'D1matrix')
matlabFunction(D2, 'File', 'D2matrix')
matlabFunction(Ce, 'File', 'Cematrix')
matlabFunction(D1e, 'File', 'D1ematrix')
matlabFunction(D2e, 'File', 'D2ematrix')



