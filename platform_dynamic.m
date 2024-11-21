classdef platform_dynamic
    %PLATFORM_DYNAMIC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % simulation time
        dt = 0.001;
        sim_t = 5;     %total simulation duration
        t;              %ever time steps
        iter
        % parameters
        m = 74.33;
        J = eye(3)
        J_diag = eye(3)
        pc_2_mc
        g = 9.81;
        allocation_matrix_AW
        allocation_matrix_HW
        allocation_matrix_HW_inv
        % states
        R
        R_Euler;
        Rd_Euler;
        W
        W_dot
        
        % errors
        eR
        eW
        % control input
        Euler_Matrix = zeros(3,3);
        y_sys_icl_singular_value 
        y_sys_icl_left_singular_value   %U
        y_sys_icl_right_singular_value  %V


        icl_term 

        % R.W.

        Omega_dot
        Omega
        J_RW = 2.824e-3;


    end
    
    methods
        function dX = dynamics(obj , X, F,T)
            dX = zeros(12, 1);  %delta of pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,Rotation_matrix*9, w_x,w_y,w_z
            R_now = reshape(X(1:9), 3, 3);
            W_now = X(10:12);
    
            M = F(1:3);            
           
            dR = R_now*hat_map(W_now);
            dW = obj.J\(-cross(W_now, obj.J*W_now) + M +T);
            
            dX(1:9) = reshape(dR, 9, 1);
            dX(10:12) = dW;
        end

        function Euler_Matrix = get_euler_matrix(obj,iteration)
            R_now = reshape(obj.R(:,iteration-1), 3, 3);
            R_now_euler = rotm2eul(R_now,"XYZ");
            Euler_Matrix = [1,0,-sin(R_now_euler(2));...
                    0,cos(R_now_euler(1)),sin(R_now_euler(1))*cos(R_now_euler(2));...
                    0,-sin(R_now_euler(1)),cos(R_now_euler(1))*cos(R_now_euler(2))];
        
        end

    end
end

