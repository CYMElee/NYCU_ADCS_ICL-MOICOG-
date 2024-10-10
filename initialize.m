%% Simulation setting
Sim_Time=30; %units:sec
dt = 0.001; % Time step
t=1:1:Sim_Time/dt;

%% Platform dynamics
g = [0,0,-9.81]'; %units:kg/m^2
m_sys = 74.33; %units:kg
RW_MOI = 2.9788e-3;
MOI = [7.13,0,0 ;...
       0,7.09,0 ;...
       0,0,7.13];
CoG = [1,2,3]'*10^-3; %units:m

amplitude = 0.35;
frequency = 1;

% Allocation Matrix
A_w = [1/2 -1/2 -1/2 1/2;...
        1/2 1/2 -1/2 -1/2;...
        1/(2)^0.5 1/(2)^0.5 1/(2)^0.5 1/(2)^0.5];

H_w = [1/2 -1/2 -1/2 1/2;...
       1/2 1/2 -1/2 -1/2;...
       1/(2)^0.5 1/(2)^0.5 1/(2)^0.5 1/(2)^0.5;...
       1 -1 1 -1];

% Measure(System)
R = [0,0,0]'; % Attitude:units rad .Represent by Euler angle(Order:"ZYX")
Rd = [0,0,0]';
Rd_zyx = [0,0,0];
R_prev = [0,0,0]';
R_prev_zyx = [0,0,0];
Omega_sys = [0,0,0]';
Omega_sys_prev = [0,0,0]';
Omega_sys_dot = [0,0,0]';
Omega_sys_dot_prev = [0,0,0]';
% Measure(R.W)
Omega_mo = [0,0,0,0]';
Omega_mo_init = [0,0,0,0]';
Omega_dot_mo = [0,0,0,0]';
Omega_mo_prev = [0,0,0,0]';

% Tracking error
eR = [0,0,0]';
eW = [0,0,0]';

% Desire
Rd = [0,0,0]';
Omegad = [0,0,0]';
Omegad_dot = [0,0,0]';

% Estimate Param
Theta_sys = [CoG(1),CoG(2),CoG(3),MOI(1,1),MOI(2,2),MOI(3,3)]'; % Ground Truth Value
Theta_hat_sys = [0,0,0,0,0,0]';
Theta_hat_dot_sys = [0,0,0,0,0,0]';

% Regression matrix
Y_CoG = zeros(3,3);
Y_J = zeros(3,3);
Y_sys = [Y_CoG,Y_J];






% Record
record_R = zeros(length(t),3);
record_Omega = zeros(length(t),3);
record_MOI = zeros(length(t),3); % a length(t)*3 vector using store estimated MOI
record_CoG = zeros(length(t),3);
record_M = zeros(length(t),3);
record_m = zeros(length(t),3);
record_y_cl = zeros(3,3,length(t));






%% Controller
%Controller control gain
Kr=15;
Kw=15;


%%Estimater
gamma = 2;
kcl=5;


