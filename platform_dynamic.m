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
        W
        W_dot
        
        % errors
        eR
        eW
        % control input

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
        

    end
end

