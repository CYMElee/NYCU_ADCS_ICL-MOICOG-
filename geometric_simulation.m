clear;
close all;

addpath('geometry-toolbox')
%% set drone parameters
% simulation time
dt = 1/1000;
sim_t =60;

platform1 = platform_dynamic;
platform1.dt = dt;            %delta t
platform1.sim_t = sim_t;      %whole 
platform1.t = 0:dt:sim_t;     %every time stamps


platform1.m = 74.33;
platform1.J = [7.13, 0, 0;...
               0, 7.09, 0;...
               0, 0, 7.13];

platform1.J_RW=2.9788*10^-3; % units:kg*m^2


%use to trans motor torque to platform 

platform1.allocation_matrix_AW =[     1/2,     -1/2,      -1/2,     1/2;...
                                     1/2,      1/2,      -1/2,     -1/2;...
                                    cosd(45), cosd(45),cosd(45),  cosd(45)];
platform1.allocation_matrix_HW =[     1/2,     -1/2,      -1/2,     1/2;...
                                     1/2,      1/2,      -1/2,     -1/2;...
                                    cosd(45), cosd(45),cosd(45),  cosd(45);...
                                    1           -1          1           -1];

platform1.allocation_matrix_HW_inv =  inv(platform1.allocation_matrix_HW);  
                             
%% create states array
zero_for_compare = zeros(1,length(platform1.t));

platform1.R = zeros(9, length(platform1.t));
platform1.W = zeros(3, length(platform1.t));
platform1.W_dot = zeros(3, length(platform1.t));

platform1.eR = zeros(3, length(platform1.t));
platform1.eW = zeros(3, length(platform1.t));

real_theta_array_platform1 = zeros(9, length(platform1.t));
theta_array_platform1 = zeros(9, length(platform1.t));
theta_hat_dot_array_platform1 = zeros(9, length(platform1.t));
contorl_output_array_platform1 = zeros(3, length(platform1.t));
dX_platform1 = zeros(12, 1);


%% create attitude array use to plot
platform1.dR = zeros(3, length(platform1.t));
platform1.R_Euler = zeros(3, length(platform1.t));


%% create R.W array use ODE45 and plot

platform1.Omega = zeros(4,length(platform1.t));



%% initial state
platform1.R(:, 1) = [1; 0; 0; 0; 1; 0; 0; 0; 1];  %initial attitude
platform1.W(:, 1) = [0; 0; 0]; %initial angular velocity


%% create controller
control_platform1 = controller;   


control_platform1.sigma_M_hat_array = zeros(3,control_platform1.N);

control_platform1.sigma_y_array = zeros(3,9,control_platform1.N);


%% create trajectory
traj = trajectory;

%% create allocation matrix
    
   
rw = "close";       
        
platform1.pc_2_mc = [0.005;-0.005;-0.005]; % distance between center of rotation and center of mass

control_platform1.theta = 0.5*[platform1.J(1,1);platform1.J(2,2);platform1.J(3,3);platform1.J(1,2);platform1.J(1,3);platform1.J(2,3);platform1.pc_2_mc(1);platform1.pc_2_mc(2);platform1.pc_2_mc(3)];
   

              
 %% start iteration

traj_type = "twist";   %"circle","position"
controller_type = "ICL";   %"origin","EMK","adaptive","ICL"

control_output_platform1  = zeros(3,1);



for i = 2:length(platform1.t)
    disp(i)
    t_now = platform1.t(i);
    platform1.Euler_Matrix = platform1.get_euler_matrix(i);
    desired = traj.traj_generate(t_now,i,traj_type,platform1);

    % calculate control force
    [control_output_platform1, platform1.eR(:, i), platform1.eW(:, i),control_platform1] = control_platform1.geometric_tracking_ctrl(i,platform1,desired,controller_type);
   

    
    real_theta_array_platform1(:,i) = [platform1.J(1,1),platform1.J(2,2),platform1.J(3,3),platform1.J(1,2),platform1.J(1,3),platform1.J(2,3),platform1.pc_2_mc(1),platform1.pc_2_mc(2),platform1.pc_2_mc(3)];
    theta_hat_dot_array_platform1(:,i) =control_platform1.theta_hat_dot;
    
    theta_array_platform1(:,i) = control_platform1.theta;



    
    if rw == "OPEN"


    platform1.M_p = [control_output_platform1(1);control_output_platform1(2);control_output_platform1(3);0] ;
    platform1.Omega_dot = -(platform1.allocation_matrix_HW_inv/platform1.J_RW)*platform1.M_p;
    
    % get the R.W control torque
    X0_platform1_RW =platform1.Omega(:,i-1);

    platform1.Omega(:,i) = X0_platform1_RW + platform1.Omega_dot*platform1.dt;
    real_control_torque_platform1 = -(platform1.allocation_matrix_AW*platform1.J_RW*platform1.Omega_dot+cross(platform1.W(:, i-1),platform1.allocation_matrix_AW*platform1.J_RW*platform1.Omega(:,i)));
    
    % store the real control input using for ICL
    control_platform1.M_real = real_control_torque_platform1;
        
    else
    real_control_torque_platform1 = control_output_platform1;
    control_platform1.M_real = control_output_platform1;
    end


    %% update states
    X0_platform1 = [...
        reshape(reshape(platform1.R(:, i-1), 3, 3), 9, 1);...
        platform1.W(:, i-1)];
    
   
    % get the Euler Metrix

    

    T_ext=cross(platform1.pc_2_mc,platform1.m*platform1.Euler_Matrix*[0;0;-9.81]);
    disp(T_ext);
    disp("control input");
   % disp(real_control_torque_platform1);


    [T_platform1, X_new_platform1] = ode45(@(t, x) platform1.dynamics( x, real_control_torque_platform1,T_ext), [0, dt], X0_platform1);
    
    dX_platform1 = platform1.dynamics(X0_platform1 , real_control_torque_platform1,T_ext);
    
    
    % Save the states 

    platform1.R(:, i) = X_new_platform1(end, 1:9);
    platform1.W(:, i) = X_new_platform1(end, 10:12);
    platform1.W_dot(:, i) = dX_platform1(10:12)';

    % Save date use to plot
    platform1.dR(:,i) = desired(:,1);
    platform1.R_Euler(:,i) = rotm2eul(reshape(platform1.R(:,i),3,3),"XYZ");

    
end

%% show the result




%% rotation

%% Attitude

font =24
figure('Name','rotation result');


subplot(3,2,1);

plot(platform1.t(2:end),platform1.R_Euler(1,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.dR(1,2:end),'--',"LineWidth",2);
title('$Attitude\ in\ Roll(x)\ Direction$','interpreter','latex', 'FontSize', font);
ylabel(' $Roll[rad]$','interpreter','latex','FontSize',font);
axis([-inf inf -2 2])
legend('$Roll$','$Roll_d$','interpreter','latex','FontSize',font);
subplot(3,2,2);
plot(platform1.t(2:end),platform1.eR(1,2:end),"LineWidth",2);
grid on;
title('$Attitude\ Error\ in\ Roll(x)\ Direction$','interpreter','latex', 'FontSize', font);
ylabel(' $e_{R_x}[rad]$','interpreter','latex','FontSize',font);
axis([-inf inf -0.5 0.5])

subplot(3,2,3);
plot(platform1.t(2:end),platform1.R_Euler(2,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.dR(2,2:end),'--',"LineWidth",2);
title('$Attitude\ in\ Pitch(y)\ Direction$','interpreter','latex', 'FontSize', font);
ylabel(' $Pitch[rad]$','interpreter','latex','FontSize',font);
axis([-inf inf -2 2])
legend('$Pitch$','$Pitch_d$','interpreter','latex','FontSize',font);

subplot(3,2,4);
plot(platform1.t(2:end),platform1.eR(2,2:end),"LineWidth",2);
grid on;
title('$Attitude\ Error\ in\ Pitch(y)\ Direction$','interpreter','latex', 'FontSize', font);
ylabel(' $e_{R_y}[rad]$','interpreter','latex','FontSize',font);
axis([-inf inf -0.5 0.5])



subplot(3,2,5);
plot(platform1.t(2:end),platform1.R_Euler(3,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.dR(3,2:end),'--',"LineWidth",2);
title('$Attitude\ in\ yaw(z)\ Direction$','interpreter','latex', 'FontSize', font);
ylabel(' $Yaw[rad]$','interpreter','latex','FontSize',font);
axis([-inf inf -2 2])
legend('$Yaw$','$Yaw_d$','interpreter','latex','FontSize',font);

subplot(3,2,6);
plot(platform1.t(2:end),platform1.eR(3,2:end),"LineWidth",2);
grid on;
title('$Attitude\ Error\ in\ Yaw(z)\ Direction$','interpreter','latex', 'FontSize', font);
ylabel(' $e_{R_z}[rad]$','interpreter','latex','FontSize',font);
axis([-inf inf -0.5 0.5])
















%% angular velocity
figure('Name','rotation result');

subplot(3,1,1);
plot(platform1.t(2:end),platform1.eR(1,2:end),"LineWidth",2);
grid on;
ylabel(' $e_{\Omega_x}[rad/s]$','interpreter','latex','FontSize',24);
title('$Angular\ velocity\ Errors$','interpreter','latex', 'FontSize', 24);
axis([-inf inf -1 1])
subplot(3,1,2);
plot(platform1.t(2:end),platform1.eR(2,2:end),"LineWidth",2);
grid on;
ylabel(' $e_{\Omega_y}[rad/s]$','interpreter','latex','FontSize',24);
axis([-inf inf -1 1])
subplot(3,1,3);
plot(platform1.t(2:end),platform1.eR(3,2:end),"LineWidth",2);
grid on;
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
ylabel(' $e_{\Omega_z}[rad/s]$','interpreter','latex','FontSize',24);
axis([-inf inf -1 1])
%% theta inertial

figure('Name','inertia estimation result');

subplot(6,1,1);
plot(platform1.t(2:end), theta_array_platform1(1,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{J,xx}[kgm^2]$','interpreter','latex','FontSize',16);
title('$Estimation\ Errors\ of\ the\ MoI$','interpreter','latex', 'FontSize', 24);
axis([-inf inf 4 8])


subplot(6,1,2);
plot(platform1.t(2:end), theta_array_platform1(2,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{J,yy}[kgm^2]$','interpreter','latex','FontSize',16);
axis([-inf inf 4 8])

subplot(6,1,3);
plot(platform1.t(2:end), theta_array_platform1(3,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{J,zz}[kgm^2]$','interpreter','latex','FontSize',16);
axis([-inf inf 4 8])

subplot(6,1,4);
plot(platform1.t(2:end), theta_array_platform1(4,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{J,xy}[kgm^2]$','interpreter','latex','FontSize',16);
axis([-inf inf -0.1 0.1])

subplot(6,1,5);
plot(platform1.t(2:end), theta_array_platform1(5,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{J,xz}[kgm^2]$','interpreter','latex','FontSize',16);
axis([-inf inf -0.1 0.1])

subplot(6,1,6);
plot(platform1.t(2:end), theta_array_platform1(6,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{J,yz}[kgm^2]$','interpreter','latex','FontSize',16);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
axis([-inf inf -0.1 0.1])

%% theta CoG
figure('Name','CoG estimation result');
subplot(3,1,1);
plot(platform1.t(2:end), theta_array_platform1(7,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{r_{M/B_{x}}}[kgm^2]$','interpreter','latex','FontSize',24);
title('$Estimation\ Errors\ of\ the\ CoG$','interpreter','latex', 'FontSize', 24);
axis([-inf inf -0.1 0.1])

subplot(3,1,2);
plot(platform1.t(2:end), theta_array_platform1(8,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{r_{M/B_{y}}}[kgm^2]$','interpreter','latex','FontSize',24);


subplot(3,1,3);
plot(platform1.t(2:end), theta_array_platform1(9,2:end),"LineWidth",2,'Color',[0 0.5 1]);
grid on;
ylabel(' $\tilde{\theta}_{r_{M/B_{z}}}[kgm^2]$','interpreter','latex','FontSize',24);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
axis([-inf inf -0.1 0.1])



