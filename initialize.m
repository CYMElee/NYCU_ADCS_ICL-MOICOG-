%% Simulation setting
Sim_Time=30; %units:sec
dt = 0.001; % Time step
t=1:1:Sim_Time/dt;

%% Platform dynamics
g = [0,0,-9.81]'; %units:kg/m^2
m_sys = 74.33; %units:kg
RW_MOI = 2.9788e-3;

%System's MoI

MOI = [7.13,0,0;...
       0,7.09,0;...
       0,0,7.13];
% CoG offect
CoG = [-0.001,0.001,-0.05]'; %units:m

% use to constrain the trajectory amplitude 
amplitude = 1;
% use to constrain the trajectory frequency 
frequency_x =0.2;
frequency_y = 0.6;
frequency_z = 0.5;

 

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
r = [0,0,0]';
q0 = [1,0,0,0]';
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
Rd = [0,0,0]'; %Desire attitude
Omegad = [0,0,0]'; %Desire angular velocity
Omegad_dot = [0,0,0]'; %Desire angular acceletare


% Estimate Param
Theta_sys = [CoG(1),CoG(2),CoG(3),MOI(1,1),MOI(2,2),MOI(3,3)]'; % Ground Truth Value


Theta_hat_sys =(Theta_sys*0); % Estimate from zero
Theta_hat_sys_prev = Theta_hat_sys;



% Regression matrix
Y_CoG = zeros(3,3);
Y_J = zeros(3,3);
Y_sys = [Y_CoG,Y_J];



M=[0,0,0]';




% Record
record_R = zeros(length(t),3);
record_Rd = zeros(length(t),3);
record_Omega = zeros(length(t),3);
record_MOI = zeros(length(t),3); % a length(t)*3 vector using store estimated MOI
record_CoG = zeros(length(t),3);
record_M = zeros(length(t),3);
record_m = zeros(length(t),3);
record_y_cl = zeros(3,6,length(t));
record_m_temp=zeros(length(t),3);
record_y_cl_temp=zeros(3,6,length(t));

record_theta = zeros(6,length(t));
record_theta_hat = zeros(6,length(t));

record_ICL_term = zeros(6,length(t));





%% Controller
%Controller control gain
Kr=25*eye(3);
Kw=20*eye(3);
C1=3.5;

%%Estimater
gamma = diag([0.0000005,0.0000005,0.0000005,0.0000000005,0.000000005,0.0000000005]);
kcl=diag([5000,5000,5000,500000,5000000,500000]);

