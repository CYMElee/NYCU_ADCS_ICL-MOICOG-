clear;
close all;

addpath('geometry-toolbox')
%% set drone parameters
% simulation time
dt = 1/1000;
sim_t =180;

platform1 = platform_dynamic;
platform1.dt = dt;            %delta t
platform1.sim_t = sim_t;      %whole 
platform1.t = 0:dt:sim_t;     %every time stamps


platform1.m = 69.85;
platform1.J = [6.77, 0, 0;...
               0, 6.76, 0;...
               0, 0, 6.70];


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


%% create states arra(R.W.)
platform1.Omega_dot = zeros(4, length(platform1.t));
platform1.Omega = zeros(4, length(platform1.t));


%% create attitude array use to plot
platform1.R_Euler = zeros(3, length(platform1.t));
platform1.Rd_Euler = zeros(3, length(platform1.t));

%% create singular velue matrix

platform1.y_sys_icl_singular_value = zeros(3,9,length(platform1.t));

y_sys_icl_11 =  zeros(length(platform1.t),1);
y_sys_icl_22 =  zeros(length(platform1.t),1);
y_sys_icl_33 =  zeros(length(platform1.t),1);
y_sys_icl_rank = zeros(length(platform1.t),1);



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
    
   
       
        
 platform1.pc_2_mc = [0.0005;0.0005;-0.002]; % distance between center of rotation and center of mass


   

              
 %% start iteration

traj_type = "twist";   %"circle","position"
controller_type = "ICL_RW";   %"origin","EMK","adaptive","ICL"

control_output_platform1  = zeros(3,1);
control_platform1.theta = 1*[platform1.J(1,1);platform1.J(2,2);platform1.J(3,3);platform1.J(1,2);platform1.J(1,3);platform1.J(2,3);platform1.pc_2_mc(1);platform1.pc_2_mc(2);platform1.pc_2_mc(3)];


for i = 2:length(platform1.t)
    disp(i)
    t_now = platform1.t(i);
    platform1.Euler_Matrix = platform1.get_euler_matrix(i);
    desired = traj.traj_generate(t_now,i,traj_type,platform1);
    platform1.Rd_Euler(:,i)=desired(:,1);
    % calculate control force
    [control_output_platform1, platform1.eR(:, i), platform1.eW(:, i),control_platform1,platform1.y_sys_icl_singular_value(:,:,i)] = control_platform1.geometric_tracking_ctrl(i,platform1,desired,controller_type);
    y_sys_icl_11(i) = platform1.y_sys_icl_singular_value(1,1,i);
    y_sys_icl_22(i) = platform1.y_sys_icl_singular_value(2,2,i);
    y_sys_icl_33(i) = platform1.y_sys_icl_singular_value(3,3,i);
    y_sys_icl_rank(i)=rank(platform1.y_sys_icl_singular_value(:,:,i));
    
    real_theta_array_platform1(:,i) = [platform1.J(1,1),platform1.J(2,2),platform1.J(3,3),platform1.J(1,2),platform1.J(1,3),platform1.J(2,3),platform1.pc_2_mc(1),platform1.pc_2_mc(2),platform1.pc_2_mc(3)];
    theta_hat_dot_array_platform1(:,i) =control_platform1.theta_hat_dot;
    
    theta_array_platform1(:,i) = control_platform1.theta;








  
    real_control_torque_platform1 = [control_output_platform1(1);control_output_platform1(2);control_output_platform1(3)];

  



        
    %% update states
    X0_platform1 = [...
        reshape(reshape(platform1.R(:, i-1), 3, 3), 9, 1);...
        platform1.W(:, i-1)];
    
   
    % get the Euler Metrix



    T_ext=cross(platform1.pc_2_mc,platform1.m*platform1.Euler_Matrix*[0;0;-9.81]);
    %disp(T_ext);
  %  disp("control input");
   % disp(real_control_torque_platform1);


    [T_platform1, X_new_platform1] = ode45(@(t, x) platform1.dynamics( x, real_control_torque_platform1,T_ext), [0, dt], X0_platform1);
    
    dX_platform1 = platform1.dynamics(X0_platform1 , real_control_torque_platform1,T_ext);
    
    
    % Save the states 

    platform1.R(:, i) = X_new_platform1(end, 1:9);
    platform1.W(:, i) = X_new_platform1(end, 10:12);
    platform1.W_dot(:, i) = dX_platform1(10:12)';

    platform1.R_Euler(:,i)= rotm2eul(reshape(platform1.R(:, i),3,3),"XYZ");

    
end

%% show the result

font = 20;


%% Attitude
figure('Name','Attitude result');
subplot(3,2,1);
plot(platform1.t(2:end),platform1.R_Euler(1,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.Rd_Euler(1,2:end),"LineWidth",2);
ylabel(' $Roll[rad]$','interpreter','latex','FontSize',font);
legend('$Roll$','$Roll_d$','interpreter','latex','FontSize',font);
title('$Attitude\ in\ Roll\ Direction$','interpreter','latex', 'FontSize', font);
axis([-inf inf -1 1])



subplot(3,2,3);
plot(platform1.t(2:end),platform1.R_Euler(2,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.Rd_Euler(2,2:end),"LineWidth",2);
ylabel(' $Pitch[rad]$','interpreter','latex','FontSize',font);
legend('$Pitch$','$Pitch_d$','interpreter','latex','FontSize',font);
title('$Attitude\ in\ Pitch\ Direction$','interpreter','latex', 'FontSize', font);
axis([-inf inf -1 1])


subplot(3,2,5);
plot(platform1.t(2:end),platform1.R_Euler(3,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.Rd_Euler(3,2:end),"LineWidth",2);
ylabel(' $Yaw[rad]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',font);
legend('$Yaw$','$Yaw_d$','interpreter','latex','FontSize',font);
title('$Attitude\ in\ Yaw\ Direction$','interpreter','latex', 'FontSize', font);
axis([-inf inf -1 1])



subplot(3,2,2);
plot(platform1.t(2:end),platform1.eR(1,2:end),"LineWidth",2);
grid on;
ylabel(' $e_{R_x}[rad]$','interpreter','latex','FontSize',font);
title('$Attitude\ Error\ in\ Roll\ Direction$','interpreter','latex', 'FontSize',font);
axis([-inf inf -1 1])

subplot(3,2,4);

plot(platform1.t(2:end),platform1.eR(2,2:end),"LineWidth",2);
grid on;
ylabel(' $e_{R_y}[rad]$','interpreter','latex','FontSize',font);
title('$Attitude\ Error\ in\ Pitch\ Direction$','interpreter','latex', 'FontSize', font);
axis([-inf inf -1 1])
subplot(3,2,6);

plot(platform1.t(2:end),platform1.eR(3,2:end),"LineWidth",2);
grid on;
xlabel(' $t[s]$','interpreter','latex','FontSize',font);
ylabel(' $e_{R_z}[rad]$','interpreter','latex','FontSize',font);
title('$Attitude\ Error\ in\ Yaw\ Direction$','interpreter','latex', 'FontSize',font);
axis([-inf inf -1 1])
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

%MoI 
figure('Name','Estimation of the MoI');

subplot(3,1,1);
plot(platform1.t(2:end),real_theta_array_platform1(1,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(1,2:end),"LineWidth",2);
ylabel(' $J_{xx}[kgm^2]$','interpreter','latex','FontSize',font);
legend('$J_{xx}$','$J_{xx}estimate$','interpreter','latex','FontSize',font);
title('$Estimation\ of\ the\ MoI$','interpreter','latex', 'FontSize', font);
axis([-inf inf 4 8])


subplot(3,1,2);
plot(platform1.t(2:end),real_theta_array_platform1(2,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(2,2:end),"LineWidth",2);
ylabel(' $J_{yy}[kgm^2]$','interpreter','latex','FontSize',font);
legend('$J_{yy}$','$J_{yy}estimate$','interpreter','latex','FontSize',font);
axis([-inf inf 4 8])

subplot(3,1,3);
plot(platform1.t(2:end),real_theta_array_platform1(3,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(3,2:end),"LineWidth",2);
ylabel(' $J_{zz}[kgm^2]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
legend('$J_{zz}$','$J_{zz}estimate$','interpreter','latex','FontSize',font);
axis([-inf inf 4 8])



figure('Name','Estimation error of the MoI');
subplot(3,1,1);
plot(platform1.t(2:end),real_theta_array_platform1(1,2:end)-theta_array_platform1(1,2:end),"LineWidth",2);
grid on;
ylabel(' $\tilde{\theta}_{J,xx}[kgm^2]$','interpreter','latex','FontSize',font);
title('$Estimation\ error\ of\ the\ MoI$','interpreter','latex', 'FontSize', font);
axis([-inf inf -1 1])

subplot(3,1,2);
plot(platform1.t(2:end),real_theta_array_platform1(2,2:end)-theta_array_platform1(2,2:end),"LineWidth",2);
grid on;
ylabel('$\tilde{\theta}_{J,yy}[kgm^2]$','interpreter','latex','FontSize',font);
axis([-inf inf -1 1])

subplot(3,1,3);
plot(platform1.t(2:end),real_theta_array_platform1(3,2:end)-theta_array_platform1(3,2:end),"LineWidth",2);
grid on;
ylabel('$\tilde{\theta}_{J,zz}[kgm^2]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
axis([-inf inf -1 1])



%PoI 
figure('Name','Estimation of the PoI');

subplot(3,1,1);
plot(platform1.t(2:end),real_theta_array_platform1(4,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(4,2:end),"LineWidth",2);
ylabel(' $J_{xy}[kgm^2]$','interpreter','latex','FontSize',font);
legend('$J_{xy}$','$J_{xy}estimate$','interpreter','latex','FontSize',font);
title('$Estimation\ of\ the\ PoI$','interpreter','latex', 'FontSize', font);
axis([-inf inf -0.1 0.1])


subplot(3,1,2);
plot(platform1.t(2:end),real_theta_array_platform1(5,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(5,2:end),"LineWidth",2);
ylabel(' $J_{xz}[kgm^2]$','interpreter','latex','FontSize',font);
legend('$J_{xz}$','$J_{xz}estimate$','interpreter','latex','FontSize',font);
axis([-inf inf -0.1 0.1])

subplot(3,1,3);
plot(platform1.t(2:end),real_theta_array_platform1(6,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(6,2:end),"LineWidth",2);
ylabel(' $J_{yz}[kgm^2]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
legend('$J_{yz}$','$J_{yz}estimate$','interpreter','latex','FontSize',font);
axis([-inf inf -0.1 0.1])









figure('Name','Estimation error of the PoI');
subplot(3,1,1);
plot(platform1.t(2:end),real_theta_array_platform1(4,2:end)-theta_array_platform1(4,2:end),"LineWidth",2);
grid on;
ylabel(' $\tilde{\theta}_{J,xy}[kgm^2]$','interpreter','latex','FontSize',font);
title('$Estimation\ error\ of\ the\ PoI$','interpreter','latex', 'FontSize', font);
axis([-inf inf -0.1 0.1])

subplot(3,1,2);
plot(platform1.t(2:end),real_theta_array_platform1(5,2:end)-theta_array_platform1(5,2:end),"LineWidth",2);
grid on;
ylabel('$\tilde{\theta}_{J,xz}[kgm^2]$','interpreter','latex','FontSize',font);
axis([-inf inf -0.1 0.1])

subplot(3,1,3);
plot(platform1.t(2:end),real_theta_array_platform1(6,2:end)-theta_array_platform1(6,2:end),"LineWidth",2);
grid on;
ylabel('$\tilde{\theta}_{J,yz}[kgm^2]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
axis([-inf inf -0.1 0.1])






%CoG
figure('Name','Estimation of the CoG');

subplot(3,1,1);
plot(platform1.t(2:end),real_theta_array_platform1(7,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(7,2:end),"LineWidth",2);
ylabel(' $CoG_{x}[m]$','interpreter','latex','FontSize',font);
legend('$CoG_{x}$','$CoG_{x}estimate$','interpreter','latex','FontSize',font);
title('$Estimation\ of\ the\ CoG$','interpreter','latex', 'FontSize', font);
axis([-inf inf -0.01 0.01])


subplot(3,1,2);
plot(platform1.t(2:end),real_theta_array_platform1(8,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(8,2:end),"LineWidth",2);
ylabel(' $CoG_{y}[m]$','interpreter','latex','FontSize',font);
legend('$CoG_{y}$','$CoG_{y}estimate$','interpreter','latex','FontSize',font);
axis([-inf inf -0.01 0.01])

subplot(3,1,3);
plot(platform1.t(2:end),real_theta_array_platform1(9,2:end),'--',"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),theta_array_platform1(9,2:end),"LineWidth",2);
ylabel('$CoG_{z}[m]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
legend('$CoG_{z}$','$CoG_{z}estimate$','interpreter','latex','FontSize',font);
axis([-inf inf -0.01 0.01])






figure('Name','Estimation error of the CoG');
subplot(3,1,1);
plot(platform1.t(2:end),real_theta_array_platform1(7,2:end)-theta_array_platform1(7,2:end),"LineWidth",2);
grid on;
ylabel(' $\tilde{\theta}_{CoG,x}[m]$','interpreter','latex','FontSize',font);
title('$Estimation\ error\ of\ the\ CoG$','interpreter','latex', 'FontSize', font);
axis([-inf inf -0.01 0.01])

subplot(3,1,2);
plot(platform1.t(2:end),real_theta_array_platform1(8,2:end)-theta_array_platform1(8,2:end),"LineWidth",2);
grid on;
ylabel(' $\tilde{\theta}_{CoG,y}[m]$','interpreter','latex','FontSize',font);
axis([-inf inf -0.01 0.01])

subplot(3,1,3);
plot(platform1.t(2:end),real_theta_array_platform1(9,2:end)-theta_array_platform1(9,2:end),"LineWidth",2);
grid on;
ylabel(' $\tilde{\theta}_{CoG,z}[m]$','interpreter','latex','FontSize',font);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
axis([-inf inf -0.01 0.01])



% plot the y_sys_icl Singular value

figure('Name','Singular value of y_sys_icl');
subplot(3,1,1);
plot(platform1.t(2:end), y_sys_icl_11(2:end),"LineWidth",2);
title('$Singular\ value\ of\ y_{sys}^{icl} (1,1)$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf 0 90])


subplot(3,1,2);
plot(platform1.t(2:end), y_sys_icl_22(2:end),"LineWidth",2);
title('$Singular\ value\ of\ y_{sys}^{icl} (2,2)$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf 0 90])


subplot(3,1,3);
plot(platform1.t(2:end), y_sys_icl_33(2:end),"LineWidth",2);
title('$Singular\ value\ of\ y_{sys}^{icl} (3,3)$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])





figure('Name','The rank of the singular of y_sys_icl');
plot(platform1.t(2:end),y_sys_icl_rank(2:end),"LineWidth",2);
title('$The\ rank\ of\ the\ singular\ value\ of\ y_{sys}^{icl}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf 0 4])


