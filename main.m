clear all, close all;
addpath('./tools/')
initialize;
%% Simulate

for i=1:length(t)
    % Trajectory

   
  

    Rd = amplitude*[sin(frequency_x*i*dt)...
                    ,cos(frequency_y*i*dt),...
                    -sin(frequency_z*i*dt)]';


   % Rd = amplitude*[0,...
                 %  0,...
                  %  -sin(frequency_z*i*dt)]';
    Rd_zyx = [Rd(3),Rd(2),Rd(1)];
    

    Euler_Matrix = [1,0,-sin(R_prev(2));...
                    0,cos(R_prev(1)),sin(R_prev(1))*cos(R_prev(2));...
                    0,-sin(R_prev(1)),cos(R_prev(1))*cos(R_prev(2))];
   
   %  Omegad = Euler_Matrix*amplitude*[0,...
                                   % 0,...
                                   % -frequency_z*cos(frequency_z*i*dt)]';

   Omegad = Euler_Matrix*amplitude*[ frequency_x*cos(frequency_x*i*dt),...
                                    -frequency_y*sin(frequency_y*i*dt),...
                                    - frequency_z*cos(frequency_z*i*dt)]';
   Omegad_dot = Euler_Matrix*amplitude*[-(frequency_x^2)*sin(frequency_x*i*dt),...
                                        -(frequency_y^2)*cos(frequency_y*i*dt),...
                                      (frequency_z^2)*sin(frequency_z*i*dt)]';
  % Omegad_dot = Euler_Matrix*amplitude*[0,...
                                      %   0,...
                                      %  (frequency_z^2)*sin(frequency_z*i*dt)]';
    
   record_theta(:,i)=Theta_sys;

    % Tracking Error
    eR = Attitude_Error(R_prev_zyx,Rd_zyx);
    eW = Omega_Error(R_prev_zyx,Rd_zyx,Omegad,Omega_sys_prev);


    % The g vector in body frame will change
    g_body = Euler_Matrix*g;
   


    % Regression matrix
    g_skew = hat_map(g_body);
    Y_CoG = m_sys *g_skew;
    Omega_hat_map = hat_map(Omega_sys_prev);
    R_rot = eul2rotm(R_prev_zyx);
    Rd_rot = eul2rotm(Rd_zyx);
    Omega_bar = Omega_hat_map* R_rot'*Rd_rot-R_rot'*Rd_rot*Omegad_dot;...
    Y_J =[Omega_bar(1),Omega_sys_prev(2)*Omega_sys_prev(3),-Omega_sys_prev(2)*Omega_sys_prev(3);...
          -Omega_sys_prev(1)*Omega_sys_prev(3),Omega_bar(2),Omega_sys_prev(1)*Omega_sys_prev(3);...
            Omega_sys_prev(1)*Omega_sys_prev(2),-Omega_sys_prev(1)*Omega_sys_prev(2),Omega_bar(3)];
    Y_sys=[-Y_CoG,Y_J];





    % Regression matrix use for Estimate
    Y_J_cl = [Omega_sys_dot_prev(1),Omega_sys_prev(2)*Omega_sys_prev(3),Omega_sys_prev(2)*Omega_sys_prev(3) ;...
              -Omega_sys_prev(1)*Omega_sys_prev(3) ,Omega_sys_dot_prev(2),-Omega_sys_prev(1)*Omega_sys_prev(3);...
              Omega_sys_prev(1)*Omega_sys_prev(2) ,-Omega_sys_prev(1)*Omega_sys_prev(2),Omega_sys_dot_prev(3)];
    Y_sys_cl = [Y_CoG,Y_J_cl];
    % integral y_sys_cl from delta t to t
    
  

    record_y_cl(:,:,i) = (Y_sys_cl*dt);
    record_m(i,:) = (M*dt);
    

   % record_m(i,:) = (- A_w*RW_MOI*Omega_dot_mo -cross(Omega_sys_prev,A_w*RW_MOI*Omega_mo_prev))*dt;
    ICL_term = [0,0,0,0,0,0]';
    ICL_term_prev = [0,0,0,0,0,0]';
    
    if i>100
        for j=1:5
            ICL_term = ICL_term_prev + record_y_cl(:,:,i-j)'*(record_m(i-j,:)'-(record_y_cl(:,:,i-j)*Theta_hat_sys_prev));
            ICL_term_prev = ICL_term;
        end
    end
    %ICL_term = [0,0,0,0,0,0]';
    Theta_hat_dot_sys = gamma*Y_sys'*(eW+C1*eR)+kcl*gamma*ICL_term;
    
    % Update the estimate value
    Theta_hat_sys = Theta_hat_sys_prev+Theta_hat_dot_sys*dt;
    Theta_hat_sys_prev = Theta_hat_sys;
    record_theta_hat(:,i) = Theta_hat_sys;
    % Controller and Control-input
    M = -Kr*eR-Kw*eW - Y_sys*Theta_hat_sys;
    
   %M = [0,0,0]';
   % M_p = [M;0];
   % M = -Kr*eR-Kw*eW - Y_J*Theta_J+Y_CoG*CoG;
                    
    % Get R.W Motor Speed.
   % Omega_dot_mo = -(inv(H_w)/RW_MOI )*M_p;
   % Omega_mo =  Omega_mo_prev + Omega_dot_mo*dt;

    % Record real control input to the system
    
    
    %
    ext_Torque = Y_CoG*CoG;
    
    %ext_Torque=0;
    % System dynamic
   % Omega_sys_dot = inv(MOI)*...
                       %(ext_Torque-(A_w*RW_MOI*Omega_dot_mo)-...
                       % cross(Omega_sys_prev,A_w*RW_MOI*Omega_mo)- ...
                       % cross(Omega_sys_prev,MOI*Omega_sys_prev));
    Omega_sys_dot = inv(MOI)*...
                       (M-Y_CoG*CoG...
                        -cross(Omega_sys_prev, MOI*Omega_sys_prev));
    
    Omega_sys = Omega_sys_prev + Omega_sys_dot*dt;
    
    %% Get the Attitude using quaternion integral
    omega_x =Omega_sys(1);
    omega_y =Omega_sys(2);
    omega_z =Omega_sys(3);
    omega_Matrix = [0, -omega_x, -omega_y, -omega_z;...
                    omega_x, 0, omega_z, -omega_y; ...
                    omega_y, -omega_z, 0, omega_x;...
                    omega_z, omega_y, -omega_x, 0];
    omega_Abs = norm(Omega_sys);


    if omega_Abs == 0  %prevent devided by 0 occur when omega is 0
       Rq = q0;
    else
        I4 = eye(4);
        Quaternion = (cos((omega_Abs*dt)/2)*I4+((1/omega_Abs)*sin((omega_Abs*dt)/2)*omega_Matrix))*q0;
        q_norm = norm(Quaternion);
        Rq = (Quaternion/q_norm);
    end 
    q0 = Rq;
    
    r = quat2eul(Rq.');

    %% %% Get the Attitude using Euler angle integral
    

    % Record Attitude,Omega use to plot.Because the return sequence for
    % fun"quat2eul" is "ZYX" so r(3)="X",r(2)="Y",r(1)="Z"
    record_R(i,:) =[r(3),r(2),r(1)];
   
    record_Rd(i,:)=Rd';
    record_Omega(i,:) =[Omega_sys(1),Omega_sys(2),Omega_sys(3)];
    % Record Attitude,Omega use to next controller parameter
    R_prev = [r(3),r(2),r(1)]; % ORDER "XYZ"
    
    R_prev_zyx = [r(1),r(2),r(3)];
    Omega_sys_prev = [Omega_sys(1),Omega_sys(2),Omega_sys(3)]';

end

%% Plot

% Attitude plot

figure;

tiledlayout(3, 1);

ax1 = nexttile;
plot(ax1, ...  
          t, record_R(1:length(record_R),1),'-',...
          t,record_Rd(1:length(record_Rd),1),'--'...
          );
%title("Roll", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("\psi", 'FontSize',13);
legend("x,xd");





ax2 = nexttile;
plot(ax2, ...  
          t, record_R(1:length(record_R),2),'-',...
          t,record_Rd(1:length(record_Rd),2),'--'...
          );
%title("Pitch", FontSize=14);
xlabel("Time(10ms)",'FontSize',13);
ylabel("\theta", 'FontSize',13);
legend("y,yd");


ax3 = nexttile;

plot(ax3, ...  
          t, record_R(1:length(record_R),3),'-',...
          t,record_Rd(1:length(record_Rd),3),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("\phi", 'FontSize',13);
legend("z,zd");
sgtitle('Platform Attitude (rad)', 'FontSize', 16);



%% Estimate plot

figure
tiledlayout(3, 1);
ax4 = nexttile;

plot(ax4, ...  
          t, record_theta(1,1:length(record_theta)),'-',...
          t,record_theta_hat(1,1:length(record_theta_hat)),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("m", 'FontSize',13);
legend("CoG_x(true),CoG_x(est)");
sgtitle('CoG (x)', 'FontSize', 16);



ax5 = nexttile;

plot(ax5, ...  
          t, record_theta(2,1:length(record_theta)),'-',...
          t,record_theta_hat(2,1:length(record_theta_hat)),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("m", 'FontSize',13);
legend("CoG_y(true),CoG_y(est)");
sgtitle('CoG (y)', 'FontSize', 16);


ax6 = nexttile;

plot(ax6, ...  
          t, record_theta(3,1:length(record_theta)),'-',...
          t,record_theta_hat(3,1:length(record_theta_hat)),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("m", 'FontSize',13);
legend("CoG_z(true),CoG_z(est)");
sgtitle('CoG', 'FontSize', 16);



figure
tiledlayout(3, 1);
ax7 = nexttile;

plot(ax7, ...  
          t, record_theta(4,1:length(record_theta)),'-',...
          t,record_theta_hat(4,1:length(record_theta_hat)),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("kg*m^2", 'FontSize',13);
legend("J_x(true),J_x(est)");
sgtitle('J(x)', 'FontSize', 16);



ax8 = nexttile;

plot(ax8, ...  
          t, record_theta(5,1:length(record_theta)),'-',...
          t,record_theta_hat(5,1:length(record_theta_hat)),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("kg*m^2", 'FontSize',13);
legend("J_y(true),J_y(est)");
sgtitle('J(y)', 'FontSize', 16);


ax9 = nexttile;

plot(ax9, ...  
          t, record_theta(6,1:length(record_theta)),'-',...
          t,record_theta_hat(6,1:length(record_theta_hat)),'--'...
          );
%title("Yaw", FontSize=14);
xlabel("Time(10ms)", 'FontSize',13);
ylabel("kg*m^2", 'FontSize',13);
legend("J_z(true),J_z(est)");
sgtitle('MOI', 'FontSize', 16);



