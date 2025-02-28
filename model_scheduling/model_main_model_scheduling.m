clc
close all
clear

% RUN symbolic_model_v8 BEFORE THIS ONE

% simulation params                                                 
simtime = 20; % [s] simulation time span
DT = 1e-3; % [s] sample time fixed-step integration
SELECTOR = 0; % 1 = non-linearized, 0 = linearized

% vector dimensions
n = 6; % state vector -> x
p = 2; % control vector -> u
q = 6; % measurement vector -> y,ν
m = 1; % regulated output vector -> r
nd = 2; % disturbances vector -> d
r = 9; % exogenous vector -> w

% plant params for bus case
C1 = 1.2801; % [-] -> road fricition coefficients
C2 = 23.990;
C3 = 0.5200;
Cf = 1; % [-] -> aerodynamic drag coefficient front
Cs = 1.35; % [-] -> aerodynamic drag coefficient side
Sx = 7.5; % [m^2] -> longitunal projection area of the bus
Sy = 36; % [m^2] -> lateral projection area of the bus
g = 9.81; % [m/s^2] -> acceleration of gravity
lx = 5; % [m] -> distance from cg to wheel x coord
ly = 1.25; % [m] -> distance from cg to wheel y coord
mass = 22240; % [kg] -> mass of the bus
rho = 1.225; % [kg/m^3] -> air density
weight_part = 0.5; % [-] -> bus weight partition
epsilon = 1e-8; % [-] -> non 0/0 coeff
wheel_rad = 0.25; % [m] -> radius bus wheel


% linearization point - star
x0 = [0;                 % x trajectory
      0;                 % y pos
      25;              % x speed
      0;                 % y speed
      0;                 % yaw angle
      0];                % yaw rate

u0 = [0; 100]; % control input as steering angle and v_ang
d0 = [25; pi/2]; % Linearization disturbance [wind]
r0 = 0; % reference y position in the middle of the road and wheel angular speed
nu0 = [0; 0; 0; 0; 0; 0]; % errors
y0 = [x0(1); x0(2); x0(3); x0(4); x0(5); x0(6)];
e0 = 0;
w0 = [d0; nu0; r0];


% LINEAR PLANT
A = Amatrix(C1, C2, C3, Cf, Cs, Sx, Sy, x0(3), x0(4), w0(1), w0(2), u0(1), epsilon, g, lx, ly, mass, x0(6), x0(5), rho, u0(2), weight_part, wheel_rad);
B1 = B1matrix(C1, C2, C3, x0(3), x0(4), u0(1), epsilon, g, lx, mass, x0(6), x0(5), u0(2), weight_part, wheel_rad);
B2 = B2matrix(Cf, Cs, Sx, Sy, x0(3), x0(4), w0(1), w0(2), epsilon, lx, ly, mass, x0(5), rho);
C = Cmatrix;
D1 = D1matrix;
D2 = D2matrix;
Ce = Cematrix;
D1e = D1ematrix;
D2e = D2ematrix;

B = [B1 B2];
D = [D1 D2];
De = [D1e D2e];

disp("end section 1")

%% inverted wind section

% linearization point - star
d0_inv = [25; -pi/2]; % Linearization disturbance [wind]
w0_inv = [d0_inv; nu0; r0];

% LINEAR PLANT
A_inv = Amatrix(C1, C2, C3, Cf, Cs, Sx, Sy, x0(3), x0(4), w0_inv(1), w0_inv(2), u0(1), epsilon, g, lx, ly, mass, x0(6), x0(5), rho, u0(2), weight_part, wheel_rad);
B1_inv = B1matrix(C1, C2, C3, x0(3), x0(4), u0(1), epsilon, g, lx, mass, x0(6), x0(5), u0(2), weight_part, wheel_rad);
B2_inv = B2matrix(Cf, Cs, Sx, Sy, x0(3), x0(4), w0_inv(1), w0_inv(2), epsilon, lx, ly, mass, x0(5), rho);
C_inv = Cmatrix;
D1_inv = D1matrix;
D2_inv = D2matrix;
Ce_inv = Cematrix;
D1e_inv = D1ematrix;
D2e_inv = D2ematrix;

B_inv = [B1 B2];
D_inv = [D1 D2];
De_inv = [D1e D2e];

disp("end section 1 inv")

%% observability & reachability

% observability
O = obsv(A, C);
n_o = length(A);
if rank(O) == n_o
disp('(A,C) FULLY OBSERVABLE')
else
disp('(A,C) NOT FULLY OBSERVABLE')
end

% controllability
Reach = ctrb(A, B1); % Reach is the reachabilty matrix
lenA = length(A);
if rank(Reach) == lenA
disp('(A,B1) FULLY REACHABLE')
else
disp('(A,B1) NOT FULLY REACHABLE')
end

disp ("end section 2")

%% design of integral action
%
Aext = [A zeros(6, 1); % extended matrix A, multiplied by [dot_x, dot_eta]
        Ce zeros(1 ,1)];
B1ext = [B1;            % extended matrix B1, multiplied by u
         D1e];
Cext = Ce;
Dext = D1e;

qe_x_pos = 1;
qe_y_pos = 15;
qe_x_speed = 1;
qe_y_speed = 5;
qe_yaw_angle = 1;
qe_yaw_rate = 1;
qe_error_steer = 10;

re_steer = 10;

Qe = diag([qe_x_pos qe_y_pos qe_x_speed qe_y_speed qe_yaw_angle qe_yaw_rate qe_error_steer]);
Re = re_steer;

% call the LQR for the augmented system
[Ke, ~, poles] = lqr(Aext, B1ext, Qe, Re);

% Split Ke into K and Ki
K = Ke(:, 1:lenA);       
KI = Ke(1, lenA+1:end); 

disp(poles)

disp("end section 3")
%}

%% integral action inverted wind

%
Aext_inv = [A_inv zeros(6, 1); % extended matrix A, multiplied by [dot_x, dot_eta]
        Ce_inv zeros(1 ,1)];
B1ext_inv = [B1_inv;            % extended matrix B1, multiplied by u
         D1e_inv];
Cext = Ce_inv;
Dext = D1e_inv;

qe_x_pos = 1;
qe_y_pos = 15;
qe_x_speed = 1;
qe_y_speed = 5;
qe_yaw_angle = 1;
qe_yaw_rate = 1;
qe_error_steer = 10;

re_steer = 10;

Qe = diag([qe_x_pos qe_y_pos qe_x_speed qe_y_speed qe_yaw_angle qe_yaw_rate qe_error_steer]);
Re = re_steer;

% call the LQR for the augmented system
[Ke_inv, ~, poles] = lqr(Aext_inv, B1ext_inv, Qe, Re);

% Split Ke into K and Ki
K_inv = Ke_inv(:, 1:lenA);       
KI_inv = Ke_inv(1, lenA+1:end); 

disp(poles)

disp("end section 3 inv")
%% simulink and plot
clc
% simulation initial conditions

% variazione rispetto alla condizione iniziale del sistema lineare
x_init = [0;           % x pos
          0;           % y pos
          0;          % x speed
          0;           % y speed
          0;           % yaw angle
          0];          % yaw rate

u_init = [0; 0]; % control input as steering angle and v_ang
d_init = [0; 0]; % Linearization disturbance [wind]
r_init = 0; % reference y position in the middle of the road and v_ang
nu_init = [0; 0; 0; 0; 0; 0]; % errors
w_init = [d_init; nu_init; r_init];
y_init = [x_init(1); x_init(2); x_init(3); x_init(4); x_init(5); x_init(6)];
e_init = y_init(2) - r_init(1);
%}

%disp("end section 4")

%
close all
out = sim('test_sim_model_scheduling_v1.slx', simtime);

if out.check.data(1,1)
figure
plot(out.y_pos, 'Linewidth', 2)
hold on
grid on
%plot(out.x_speed, 'Linewidth', 2)
grid on
plot(out.y_speed, 'Linewidth', 2)
hold on
plot(out.psi, 'Linewidth', 2)
hold on
%plot(out.omega, 'Linewidth', 2)
hold on
ylabel('vehicle')
title('vehicle displacement')
legend('y pos (m)','y speed (m/s)','yaw rate (deg/s)')

figure
plot(out.x_speed, 'Linewidth', 2)
hold on
grid on
plot(out.y_speed, 'Linewidth', 2)
hold on
plot(out.wind_speed, 'Linewidth', 2)
ylabel('m/s')
title('vehicle and wind speed')
legend('x speed','y speed', 'wind speed')


figure
plot(out.y_ref, 'Linewidth', 2)
hold on
grid on
plot(out.steer, 'Linewidth', 2)
hold on
plot(out.y_pos, 'Linewidth', 2)
title('steering effect')
ylabel('vehicle')
legend('y ref (m)','steering angle (deg)','y pos (m)')
end

if ~out.check.data(1,1)
figure
plot(out.y_pos_inv, 'Linewidth', 2)
hold on
grid on
%plot(out.x_speed, 'Linewidth', 2)
grid on
plot(out.y_speed_inv, 'Linewidth', 2)
hold on
plot(out.psi_inv, 'Linewidth', 2)
hold on
%plot(out.omega, 'Linewidth', 2)
hold on
ylabel('vehicle')
title('vehicle displacement')
legend('y pos (m)','y speed (m/s)','yaw rate (deg/s)')

figure
plot(out.x_speed_inv, 'Linewidth', 2)
hold on
grid on
plot(out.y_speed_inv, 'Linewidth', 2)
hold on
plot(out.wind_speed, 'Linewidth', 2)
ylabel('m/s')
title('vehicle and wind speed')
legend('x speed','y speed', 'wind speed')


figure
plot(out.y_ref, 'Linewidth', 2)
hold on
grid on
plot(out.steer_inv, 'Linewidth', 2)
hold on
plot(out.y_pos_inv, 'Linewidth', 2)
title('steering effect')
ylabel('vehicle')
legend('y ref (m)','steering angle (deg)','y pos (m)')
end

%{
jerry = out.omega.Data(1,:);
filtered = smoothdata(jerry,"gaussian",150);

figure
plot(jerry, 'Linewidth', 2)
hold on
grid on
plot(filtered, 'Linewidth', 4)
hold on
title('filtering')
ylabel('yaw rate')
legend('yaw rate','filtered')
%}

%{
figure
plot(out.noise1)
grid on
legend('GNSS x position noise')

figure
plot(out.noise2,'Color','r')
grid on
legend('camera system y position noise')

figure
plot(out.noise3, 'Color','g')
grid on
legend('GNSS x speed noise')

figure
plot(out.noise5, 'Color','b')
grid on
legend('GNSS magnetometer noise')

figure
plot(out.noise6, 'Color','m')
grid on
legend('Pulse-40 gyroscope noise')
%}
%{

figure
plot(lamb1, m1);
grid on
%}