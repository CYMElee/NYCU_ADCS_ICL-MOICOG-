clear all, close all;
addpath('./tools/')
initialize;
%% Simulate

for i=1:length(t)
    % Trajectory
    Rd = amplitude*[sin(i*dt),cos(i*dt),-sin(i*dt)]';
    Rd_zyx = [Rd(3),Rd(2),Rd(1)];
    

    Euler_Matrix = [1,0,-sin(R_prev(2));...
                    0,cos(R_prev(1)),sin(R_prev(1))*cos(R_prev(2));...
                    0,-sin(R_prev(1)),cos(R_prev(1))*cos(R_prev(2))];
   
    Omegad = Euler_Matrix*amplitude*[cos(i*dt),-sin(i*dt),-cos(i*dt)]';
    Omegad_dot = Euler_Matrix*amplitude*[-sin(i*dt),-cos(i*dt),sin(i*dt)]';


    % Tracking Error
    eR = Attitude_Error(R_prev_zyx,Rd_zyx);
    eW = Omega_Error(R_prev_zyx,Rd_zyx,Omegad,Omega_sys_prev);


    % The g vector in body frame will change
    g_body = Euler_Matrix*g;
    
    
    % Regression matrix
    g_skew = hat_map(g_body);
    Y_CoG = m_sys *g_skew;
    Omega_hat_map = hat_map(Omega_sys_prev);
    Omega_bar = Omega_hat_map* (eul2rotm(R_prev_zyx))'*eul2rotm(Rd_zyx)-(eul2rotm(R_prev_zyx))'*eul2rotm(Rd_zyx)*Omegad_dot;...
    Y_J =[Omega_bar(1),-Omega_sys_prev(2)*Omega_sys_prev(3),Omega_sys_prev*Omega_sys_prev;...
          -Omega_sys_prev(1)*Omega_sys_prev(3),Omega_bar(2),Omega_sys_prev(1)*Omega_sys_prev(3);...
            Omega_sys_prev(1)*Omega_sys_prev(2),-Omega_sys_prev(1)*Omega_sys_prev(2),Omega_bar(3)];
    Y_sys=[Y_CoG,Y_J];

    
   %


    % Regression matrix use for Estimate
    Y_J_cl = [Omega_sys_dot_prev(1),Omega_sys_prev(2)*Omega_sys_prev(3),-Omega_sys_prev(2)*Omega_sys_prev(3) ;...
              Omega_sys_prev(1)*Omega_sys_prev(3) ,Omega_sys_dot_prev(2),-Omega_sys_prev(1)*Omega_sys_prev(3);...
              -Omega_sys_prev(1)*Omega_sys_prev(2) ,Omega_sys_prev(1)*Omega_sys_prev(2),Omega_sys_dot_prev(3)];
    Y_sys_cl = [Y_CoG,Y_J_cl];
    record_y_cl(:,:,i) = Y_sys_cl*dt;

    if i*dt>=0.2
        ICL_term = [0,0,0,0,0,0]';
    else
        ICL_term = integral_concurrent_learning(record_m,record_y_cl,Theta_hat_sys,i);
    end
    Theta_hat_dot_sys = -gamma*Y_sys'*(eW+C1*eR)+gamma*kcl*ICL_term;

    % Update the estimate value
    Theta_hat_sys = Theta_sys_prev+Theta_hat_dot_sys*dt;
      
    % Controller and Control-input
    M = -Kr*eR-Kw*eW + Y_sys*Theta_hat_sys;
    M_p = [M;0];
                    
    % Get R.W Motor Speed.
    Omega_dot_mo = -(inv(H_w)/J_RW_testbed)*M_p;
    Omega_mo =  Omega_mo_prev + Omega_dot_mo*dt;

    % Record real control input to the system
    record_m(i,:) = (- A_w*RW_MOI*Omega_dot_mo -cross(Omega_sys_prev,A_w*RW_MOI*Omega_mo))*dt;

    %
    ext_Torque = cross(CoG,m*g_body);

    % System dynamic
    Omegad_dot = inv(RW_MOI)*...
                       (ext_Torque-(A_w*RW_MOI*Omega_dot_mo)-...
                        cross(Omega_sys_prev,A_w*RW_MOI*Omega_mo)- ...
                        cross(Omega_sys_prev, RW_MOI*Omega_sys_prev));

    Omega_sys = Omega_sys_prev + Omega_sys_dot*dt;
    
    % Get the Attitude using quaternion integral

    if norm(Omega_sys) == 0  %prevent devided by 0 occur when omega is 0
       Rq = q0;
    else
        I4 = eye(4);
        Quaternion = (cos((omega_Abs*dt)/2)*I4+((1/omega_Abs)*sin((omega_Abs*dt)/2)*omega_Matrix))*q0;
        q_norm = norm(Quaternion);
        Rq = (Quaternion/q_norm);
    end 
    q0 = Rq;
    
    r = quat2eul(Rq.');
    % Record Attitude,Omega use to plot.Because the return sequence for
    % fun"quat2eul" is "ZYX" so r(3)="X",r(2)="Y",r(1)="Z"
    record_R(i,:) =[r(3),r(2),r(1)];
    record_Omega(i,:) =[Omega_sys(1),Omega_sys(2),Omega_sys(3)];
    % Record Attitude,Omega use to next controller parameter
    R_prev = [r(3),r(2),r(1)]; % ORDER "XYZ"
    R_prev_zyx = [r(1),r(2),r(3)];
    Omega_sys_prev = record_Omega(i,:)';

end

%% Plot
