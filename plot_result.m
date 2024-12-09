%% show the result
close all;
font = 20;


%%A.B,R.W(Angular velocity)
figure('Name','A.B,R.W(Omega)')
subplot(4,1,1);
plot(platform1.t(2:end),platform1.Omega(1,2:end),"LineWidth",2);
subplot(4,1,2);
plot(platform1.t(2:end),platform1.Omega(2,2:end),"LineWidth",2);
subplot(4,1,3);
plot(platform1.t(2:end),platform1.Omega(3,2:end),"LineWidth",2);
subplot(4,1,4);
plot(platform1.t(2:end),platform1.Omega(4,2:end),"LineWidth",2);



%%A.B,R.W(Torque)
figure('Name','A.B,R.W(Torque)')
subplot(4,1,1);
plot(platform1.t(2:end),(platform1.Omega_dot(1,2:end))*platform1.J_RW,"LineWidth",2);
subplot(4,1,2);
plot(platform1.t(2:end),(platform1.Omega_dot(2,2:end))*platform1.J_RW,"LineWidth",2);
subplot(4,1,3);
plot(platform1.t(2:end),(platform1.Omega_dot(3,2:end))*platform1.J_RW,"LineWidth",2);
subplot(4,1,4);
plot(platform1.t(2:end),(platform1.Omega_dot(4,2:end))*platform1.J_RW,"LineWidth",2);




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

%% Attitude 
figure('Name','Platform attitude');
subplot(3,1,1);
plot(platform1.t(2:end),platform1.R_Euler(1,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.Attitude_upper_limit(1,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.Attitude_lower_limit(1,2:end),'r--',"LineWidth",2);
ylabel(' $Roll[rad]$','interpreter','latex','FontSize',24);
title('$Platfrom\ attitude$','interpreter','latex', 'FontSize', 24);
legend('$Roll$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.78 0.78])
subplot(3,1,2);
plot(platform1.t(2:end),platform1.R_Euler(2,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.Attitude_upper_limit(2,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.Attitude_lower_limit(2,2:end),'r--',"LineWidth",2);
ylabel(' $Pitch[rad]$','interpreter','latex','FontSize',24);
legend('$Pitch$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.78 0.78])
subplot(3,1,3);
plot(platform1.t(2:end),platform1.R_Euler(3,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.Attitude_upper_limit(3,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.Attitude_lower_limit(3,2:end),'r--',"LineWidth",2);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
ylabel(' $Yaw[rad]$','interpreter','latex','FontSize',24);
legend('$Yaw$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -3.17 3.17])









%% angular velocity
figure('Name','Angular_velocity');
subplot(3,1,1);
plot(platform1.t(2:end),platform1.W(1,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.omega_upper_limit(1,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.omega_lower_limit(1,2:end),'r--',"LineWidth",2);
ylabel(' $\omega_{x}[rad/s]$','interpreter','latex','FontSize',24);
title('$Angular\ velocity$','interpreter','latex', 'FontSize', 24);
legend('$\omega_{x}$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.05 0.05])
subplot(3,1,2);
plot(platform1.t(2:end),platform1.W(2,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.omega_upper_limit(2,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.omega_lower_limit(2,2:end),'r--',"LineWidth",2);
ylabel(' $\omega_{y}[rad/s]$','interpreter','latex','FontSize',24);
legend('$\omega_{y}$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.05 0.05])
subplot(3,1,3);
plot(platform1.t(2:end),platform1.W(3,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.omega_upper_limit(3,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.omega_lower_limit(3,2:end),'r--',"LineWidth",2);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
ylabel(' $\omega_{z}[rad/s]$','interpreter','latex','FontSize',24);
legend('$\omega_{z}$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.05 0.05])







%% angular velocity tracking error
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

%% angular acceleration
figure('Name','Angular_acceleration');
subplot(3,1,1);
plot(platform1.t(2:end),platform1.W_dot(1,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.omega_dot_upper_limit(1,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.omega_dot_lower_limit(1,2:end),'r--',"LineWidth",2);
ylabel(' $\dot{\omega}_{x}[rad/s^{2}]$','interpreter','latex','FontSize',24);
title('$Angular\ accelerate$','interpreter','latex', 'FontSize', 24);
legend('$\dot{\omega}_{x}$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.05 0.05])
subplot(3,1,2);
plot(platform1.t(2:end),platform1.W_dot(2,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.omega_dot_upper_limit(2,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.omega_dot_lower_limit(2,2:end),'r--',"LineWidth",2);
ylabel(' $\dot{\omega}_{y}[rad/s^{2}]$','interpreter','latex','FontSize',24);
legend('$\dot{\omega}_{y}$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.05 0.05])
subplot(3,1,3);
plot(platform1.t(2:end),platform1.W_dot(3,2:end),"LineWidth",2);
grid on;
hold on;
plot(platform1.t(2:end),platform1.omega_dot_upper_limit(3,2:end),'r--',"LineWidth",2);
plot(platform1.t(2:end),platform1.omega_dot_lower_limit(3,2:end),'r--',"LineWidth",2);
xlabel(' $t[s]$','interpreter','latex','FontSize',24);
ylabel(' $\dot{\omega}_{z}[rad/s^{2}]$','interpreter','latex','FontSize',24);
legend('$\dot{\omega}_{z}$','$upper$','$lower$','interpreter','latex','FontSize',font);
axis([-inf inf -0.05 0.05])



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
subplot(3,9,1);
plot(platform1.t(2:end), y_sys_icl_11(2:end),"LineWidth",2);
title('$S_{1,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-0 180 0 6000])


subplot(3,9,2);
plot(platform1.t(2:end), y_sys_icl_12(2:end),"LineWidth",2);
title('$S_{1,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,3);
plot(platform1.t(2:end), y_sys_icl_13(2:end),"LineWidth",2);
title('$S_{1,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,4);
plot(platform1.t(2:end), y_sys_icl_14(2:end),"LineWidth",2);
title('$S_{1,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,5);
plot(platform1.t(2:end), y_sys_icl_15(2:end),"LineWidth",2);
title('$S_{1,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,6);
plot(platform1.t(2:end), y_sys_icl_16(2:end),"LineWidth",2);
title('$S_{1,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])
subplot(3,9,7);
plot(platform1.t(2:end), y_sys_icl_17(2:end),"LineWidth",2);
title('$S_{1,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,8);
plot(platform1.t(2:end), y_sys_icl_18(2:end),"LineWidth",2);
title('$S_{1,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,9);
plot(platform1.t(2:end), y_sys_icl_19(2:end),"LineWidth",2);
title('$S_{1,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])





















subplot(3,9,10);
plot(platform1.t(2:end), y_sys_icl_21(2:end),"LineWidth",2);
title('$S_{2,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,11);
plot(platform1.t(2:end), y_sys_icl_22(2:end),"LineWidth",2);
title('$S_{2,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-0 180 0 6000])


subplot(3,9,12);
plot(platform1.t(2:end), y_sys_icl_23(2:end),"LineWidth",2);
title('$S_{2,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,13);
plot(platform1.t(2:end), y_sys_icl_24(2:end),"LineWidth",2);
title('$S_{2,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,14);
plot(platform1.t(2:end), y_sys_icl_25(2:end),"LineWidth",2);
title('$S_{2,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,15);
plot(platform1.t(2:end), y_sys_icl_26(2:end),"LineWidth",2);
title('$S_{2,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])



subplot(3,9,16);
plot(platform1.t(2:end), y_sys_icl_27(2:end),"LineWidth",2);
title('$S_{2,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,17);
plot(platform1.t(2:end), y_sys_icl_28(2:end),"LineWidth",2);
title('$S_{2,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,18);
plot(platform1.t(2:end), y_sys_icl_29(2:end),"LineWidth",2);
title('$S_{2,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])















subplot(3,9,19);
plot(platform1.t(2:end), y_sys_icl_31(2:end),"LineWidth",2);
title('$S_{3,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,20);
plot(platform1.t(2:end), y_sys_icl_32(2:end),"LineWidth",2);
title('$S_{3,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,21);
plot(platform1.t(2:end), y_sys_icl_33(2:end),"LineWidth",2);
title('$S_{3,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-0 180 0 0.5])


subplot(3,9,22);
plot(platform1.t(2:end), y_sys_icl_34(2:end),"LineWidth",2);
title('$S_{3,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,23);
plot(platform1.t(2:end), y_sys_icl_35(2:end),"LineWidth",2);
title('$S_{3,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,24);
plot(platform1.t(2:end), y_sys_icl_36(2:end),"LineWidth",2);
title('$S_{3,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,25);
plot(platform1.t(2:end), y_sys_icl_37(2:end),"LineWidth",2);
title('$S_{3,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,26);
plot(platform1.t(2:end), y_sys_icl_38(2:end),"LineWidth",2);
title('$S_{3,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,9,27);
plot(platform1.t(2:end), y_sys_icl_39(2:end),"LineWidth",2);
title('$S_{3,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])






















figure('Name','The rank of the singular of y_sys_icl');
plot(platform1.t(2:end),y_sys_icl_rank(2:end),"LineWidth",2);
title('$The\ rank\ of\ the\ singular\ value\ of\ y_{sys}^{icl}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf 0 4])


% plot the y_sys_icl left singular value



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



% plot the y_sys_icl LEFT singular value



figure('Name','Left singular vectors of y_sys_icl');
subplot(3,3,1);
plot(platform1.t(2:end),y_sys_icl_left_11(2:end),"LineWidth",2);
title('$U_{1,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,2);
plot(platform1.t(2:end), y_sys_icl_left_12(2:end),"LineWidth",2);
title('$U_{1,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,3);
plot(platform1.t(2:end), y_sys_icl_left_13(2:end),"LineWidth",2);
title('$U_{1,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,4);
plot(platform1.t(2:end), y_sys_icl_left_21(2:end),"LineWidth",2);
title('$U_{2,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,5);
plot(platform1.t(2:end), y_sys_icl_left_22(2:end),"LineWidth",2);
title('$U_{2,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,6);
plot(platform1.t(2:end), y_sys_icl_left_23(2:end),"LineWidth",2);
title('$U_{2,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])



subplot(3,3,7);
plot(platform1.t(2:end), y_sys_icl_left_31(2:end),"LineWidth",2);
title('$U_{3,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,8);
plot(platform1.t(2:end),y_sys_icl_left_32(2:end),"LineWidth",2);
title('$U_{3,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


subplot(3,3,9);
plot(platform1.t(2:end), y_sys_icl_left_33(2:end),"LineWidth",2);
title('$U_{3,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.1 0.1])


% plot the y_sys_icl RIGHT singular value



figure('Name','Right singular vectors of y_sys_icl');
subplot(9,9,1);
plot(platform1.t(2:end),y_sys_icl_right_11(2:end),"LineWidth",2);
title('$V_{1,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,2);
plot(platform1.t(2:end), y_sys_icl_right_12(2:end),"LineWidth",2);
title('$V_{1,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,3);
plot(platform1.t(2:end),y_sys_icl_right_13(2:end),"LineWidth",2);
title('$V_{1,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,4);
plot(platform1.t(2:end), y_sys_icl_right_14(2:end),"LineWidth",2);
title('$V_{1,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,5);
plot(platform1.t(2:end),y_sys_icl_right_15(2:end),"LineWidth",2);
title('$V_{1,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,6);
plot(platform1.t(2:end), y_sys_icl_right_16(2:end),"LineWidth",2);
title('$V_{1,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,7);
plot(platform1.t(2:end), y_sys_icl_right_17(2:end),"LineWidth",2);
title('$V_{1,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,8);
plot(platform1.t(2:end),y_sys_icl_right_18(2:end),"LineWidth",2);
title('$V_{1,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,9);
plot(platform1.t(2:end),y_sys_icl_right_19(2:end),"LineWidth",2);
title('$V_{1,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])









subplot(9,9,10);
plot(platform1.t(2:end),y_sys_icl_right_21(2:end),"LineWidth",2);
title('$V_{2,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,11);
plot(platform1.t(2:end), y_sys_icl_right_22(2:end),"LineWidth",2);
title('$V_{2,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,12);
plot(platform1.t(2:end),y_sys_icl_right_23(2:end),"LineWidth",2);
title('$V_{2,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,13);
plot(platform1.t(2:end), y_sys_icl_right_24(2:end),"LineWidth",2);
title('$V_{2,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,14);
plot(platform1.t(2:end),y_sys_icl_right_25(2:end),"LineWidth",2);
title('$V_{2,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,15);
plot(platform1.t(2:end), y_sys_icl_right_26(2:end),"LineWidth",2);
title('$V_{2,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,16);
plot(platform1.t(2:end), y_sys_icl_right_27(2:end),"LineWidth",2);
title('$V_{2,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,17);
plot(platform1.t(2:end),y_sys_icl_right_28(2:end),"LineWidth",2);
title('$V_{2,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,18);
plot(platform1.t(2:end),y_sys_icl_right_29(2:end),"LineWidth",2);
title('$V_{2,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])










subplot(9,9,19);
plot(platform1.t(2:end),y_sys_icl_right_31(2:end),"LineWidth",2);
title('$V_{3,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,20);
plot(platform1.t(2:end), y_sys_icl_right_32(2:end),"LineWidth",2);
title('$V_{3,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,21);
plot(platform1.t(2:end),y_sys_icl_right_33(2:end),"LineWidth",2);
title('$V_{3,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,22);
plot(platform1.t(2:end), y_sys_icl_right_34(2:end),"LineWidth",2);
title('$V_{3,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,23);
plot(platform1.t(2:end),y_sys_icl_right_35(2:end),"LineWidth",2);
title('$V_{3,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,24);
plot(platform1.t(2:end), y_sys_icl_right_36(2:end),"LineWidth",2);
title('$V_{3,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,25);
plot(platform1.t(2:end), y_sys_icl_right_37(2:end),"LineWidth",2);
title('$V_{3,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,26);
plot(platform1.t(2:end),y_sys_icl_right_38(2:end),"LineWidth",2);
title('$V_{3,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,27);
plot(platform1.t(2:end),y_sys_icl_right_39(2:end),"LineWidth",2);
title('$V_{3,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])










subplot(9,9,28);
plot(platform1.t(2:end),y_sys_icl_right_41(2:end),"LineWidth",2);
title('$V_{4,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,29);
plot(platform1.t(2:end), y_sys_icl_right_42(2:end),"LineWidth",2);
title('$V_{4,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,30);
plot(platform1.t(2:end),y_sys_icl_right_43(2:end),"LineWidth",2);
title('$V_{4,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,31);
plot(platform1.t(2:end), y_sys_icl_right_44(2:end),"LineWidth",2);
title('$V_{4,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,32);
plot(platform1.t(2:end),y_sys_icl_right_45(2:end),"LineWidth",2);
title('$V_{4,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,33);
plot(platform1.t(2:end), y_sys_icl_right_46(2:end),"LineWidth",2);
title('$V_{4,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,34);
plot(platform1.t(2:end), y_sys_icl_right_47(2:end),"LineWidth",2);
title('$V_{4,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,35);
plot(platform1.t(2:end),y_sys_icl_right_48(2:end),"LineWidth",2);
title('$V_{4,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,36);
plot(platform1.t(2:end),y_sys_icl_right_49(2:end),"LineWidth",2);
title('$V_{4,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])
















subplot(9,9,37);
plot(platform1.t(2:end),y_sys_icl_right_51(2:end),"LineWidth",2);
title('$V_{5,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,38);
plot(platform1.t(2:end), y_sys_icl_right_52(2:end),"LineWidth",2);
title('$V_{5,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,39);
plot(platform1.t(2:end),y_sys_icl_right_53(2:end),"LineWidth",2);
title('$V_{5,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,40);
plot(platform1.t(2:end), y_sys_icl_right_54(2:end),"LineWidth",2);
title('$V_{5,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,41);
plot(platform1.t(2:end),y_sys_icl_right_55(2:end),"LineWidth",2);
title('$V_{5,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,42);
plot(platform1.t(2:end), y_sys_icl_right_56(2:end),"LineWidth",2);
title('$V_{5,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,43);
plot(platform1.t(2:end), y_sys_icl_right_57(2:end),"LineWidth",2);
title('$V_{5,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,44);
plot(platform1.t(2:end),y_sys_icl_right_58(2:end),"LineWidth",2);
title('$V_{5,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,45);
plot(platform1.t(2:end),y_sys_icl_right_59(2:end),"LineWidth",2);
title('$V_{5,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])
















subplot(9,9,46);
plot(platform1.t(2:end),y_sys_icl_right_61(2:end),"LineWidth",2);
title('$V_{6,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,47);
plot(platform1.t(2:end), y_sys_icl_right_62(2:end),"LineWidth",2);
title('$V_{6,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,48);
plot(platform1.t(2:end),y_sys_icl_right_63(2:end),"LineWidth",2);
title('$V_{6,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,49);
plot(platform1.t(2:end), y_sys_icl_right_64(2:end),"LineWidth",2);
title('$V_{6,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,50);
plot(platform1.t(2:end),y_sys_icl_right_65(2:end),"LineWidth",2);
title('$V_{6,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,51);
plot(platform1.t(2:end), y_sys_icl_right_66(2:end),"LineWidth",2);
title('$V_{6,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,52);
plot(platform1.t(2:end), y_sys_icl_right_67(2:end),"LineWidth",2);
title('$V_{6,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,53);
plot(platform1.t(2:end),y_sys_icl_right_68(2:end),"LineWidth",2);
title('$V_{6,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,54);
plot(platform1.t(2:end),y_sys_icl_right_69(2:end),"LineWidth",2);
title('$V_{6,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


















subplot(9,9,55);
plot(platform1.t(2:end),y_sys_icl_right_71(2:end),"LineWidth",2);
title('$V_{7,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,56);
plot(platform1.t(2:end), y_sys_icl_right_72(2:end),"LineWidth",2);
title('$V_{7,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,57);
plot(platform1.t(2:end),y_sys_icl_right_73(2:end),"LineWidth",2);
title('$V_{7,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,58);
plot(platform1.t(2:end), y_sys_icl_right_74(2:end),"LineWidth",2);
title('$V_{7,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,59);
plot(platform1.t(2:end),y_sys_icl_right_75(2:end),"LineWidth",2);
title('$V_{7,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,60);
plot(platform1.t(2:end), y_sys_icl_right_76(2:end),"LineWidth",2);
title('$V_{7,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,61);
plot(platform1.t(2:end), y_sys_icl_right_77(2:end),"LineWidth",2);
title('$V_{7,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,62);
plot(platform1.t(2:end),y_sys_icl_right_78(2:end),"LineWidth",2);
title('$V_{7,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,63);
plot(platform1.t(2:end),y_sys_icl_right_79(2:end),"LineWidth",2);
title('$V_{7,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])














subplot(9,9,64);
plot(platform1.t(2:end),y_sys_icl_right_81(2:end),"LineWidth",2);
title('$V_{8,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,65);
plot(platform1.t(2:end), y_sys_icl_right_82(2:end),"LineWidth",2);
title('$V_{8,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,66);
plot(platform1.t(2:end),y_sys_icl_right_83(2:end),"LineWidth",2);
title('$V_{8,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,67);
plot(platform1.t(2:end), y_sys_icl_right_84(2:end),"LineWidth",2);
title('$V_{8,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,68);
plot(platform1.t(2:end),y_sys_icl_right_85(2:end),"LineWidth",2);
title('$V_{8,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,69);
plot(platform1.t(2:end), y_sys_icl_right_86(2:end),"LineWidth",2);
title('$V_{8,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,70);
plot(platform1.t(2:end), y_sys_icl_right_87(2:end),"LineWidth",2);
title('$V_{8,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,71);
plot(platform1.t(2:end),y_sys_icl_right_88(2:end),"LineWidth",2);
title('$V_{8,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,72);
plot(platform1.t(2:end),y_sys_icl_right_89(2:end),"LineWidth",2);
title('$V_{8,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])














subplot(9,9,73);
plot(platform1.t(2:end),y_sys_icl_right_91(2:end),"LineWidth",2);
title('$V_{9,1}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,74);
plot(platform1.t(2:end), y_sys_icl_right_92(2:end),"LineWidth",2);
title('$V_{9,2}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,75);
plot(platform1.t(2:end),y_sys_icl_right_93(2:end),"LineWidth",2);
title('$V_{9,3}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,76);
plot(platform1.t(2:end), y_sys_icl_right_94(2:end),"LineWidth",2);
title('$V_{9,4}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,77);
plot(platform1.t(2:end),y_sys_icl_right_95(2:end),"LineWidth",2);
title('$V_{9,5}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,78);
plot(platform1.t(2:end), y_sys_icl_right_96(2:end),"LineWidth",2);
title('$V_{9,6}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])



subplot(9,9,79);
plot(platform1.t(2:end), y_sys_icl_right_97(2:end),"LineWidth",2);
title('$V_{9,7}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,80);
plot(platform1.t(2:end),y_sys_icl_right_98(2:end),"LineWidth",2);
title('$V_{9,8}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])


subplot(9,9,81);
plot(platform1.t(2:end),y_sys_icl_right_99(2:end),"LineWidth",2);
title('$V_{9,9}$','interpreter','latex', 'FontSize', font);
grid on;
axis([-inf inf -0.5 0.5])








