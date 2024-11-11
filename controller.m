classdef controller
    properties
  
         kR = diag([10,10,10]);
         kW = diag([8,8,8]);
         
         M = [0;0;0];
         M_RW =[0;0;0];
         %% adaptive

         theta = [0;0;0;0;0;0;0;0;0];

         gamma =  diag([0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,0.0000001]);

         c2 = 6.5
        %% ICL
        Y_icl_last = zeros(3,9,90);
        M_icl_last = zeros(3,90);
       
        last_W = [0;0;0];

        
        last_R = [1 0 0;0 1 0;0 0 1]

        k_icl =  diag([5000000000,5000000000,5000000000,5000000000,5000000000,5000000000,50000,50000,50000]);

        N = 90;      
        
        theta_hat_dot= [0;0;0;0;0;0;0;0;0];
        
        sigma_y_array;
        
        sigma_M_hat_array;
        y_icl_temp = zeros(3,9);
        singular_value_y_sys_icl = zeros(3,9);
         
        end
   methods

       
       function [control,eR,eW,obj, singular_value_y_sys_icl] = geometric_tracking_ctrl(obj,iteration,platform,desired,type)

                control = zeros(3,1);

                % platform 
                
                R_now = reshape(platform.R(:,iteration-1), 3, 3);
                W_now = platform.W(:,iteration-1);
                

                R_d = eul2rotm(desired(:,1)',"XYZ");

                W_d = desired(:,2);
                W_d_dot = desired(:,3);


                W_hat = hat(W_now);  % hat map for omega
                eR = 0.5*vee((R_d'*R_now-R_now'*R_d));     % error R
                eW = W_now-R_now'*R_d*W_d;
                  
                g_body = platform.Euler_Matrix*[0;0;-9.81];

                % Reaction Wheels

                W_now_hat = hat(W_now);
                Omega_now = platform.Omega(:,iteration-1);
                Omega_dot_now=platform.Omega_dot(:,iteration-1);
                AW = platform.allocation_matrix_AW;
                HW = platform.allocation_matrix_HW;
                HW_inv = inv(HW);
                J_RW = platform.J_RW;


                if type == "adaptive"
                    o_b = W_hat*R_now'*R_d*W_d - R_now'*R_d*W_d_dot; %omega_bar
                    
                    
                    
                    Y1 = hat(platform.m*g_body);
                
                
                
                   
                   Y2 = [ -o_b(1) ,-W_now(2)*W_now(3) ,W_now(2)*W_now(3) ,-o_b(2)-W_now(1)*W_now(3) ,-o_b(3)-W_now(1)*W_now(2), -W_now(3)^2 + W_now(2)^2 ;...
                           W_now(1)*W_now(3), -o_b(2) ,-W_now(1)*W_now(3) ,-o_b(1)-W_now(2)*W_now(3) ,-W_now(1)^2 + W_now(3)^2 , -o_b(3)-W_now(1)*W_now(2);...
                           -W_now(1)*W_now(2) ,W_now(1)*W_now(2), -o_b(3) ,-W_now(2)^2 + W_now(1)^2  ,-o_b(1)-W_now(2)*W_now(3), -o_b(2)-W_now(1)*W_now(3)];
                    Y = [Y2,Y1];
                       
                       obj.theta_hat_dot = -obj.gamma*Y'*(eW+obj.c2*eR);  

                       obj.theta = obj.theta + obj.theta_hat_dot*platform.dt;

                       obj.M = -obj.kR * eR - obj.kW*eW + Y*obj.theta;


                elseif type == "ICL"
                   

                    o_b = W_hat*R_now'*R_d*W_d - R_now'*R_d*W_d_dot; 

                   Y1 = [   0   -platform.m*g_body(3)  platform.m*g_body(2);...
                    platform.m*g_body(3)   0               -platform.m*g_body(1);...
                    -platform.m*g_body(2)   platform.m*g_body(1)   0];


                    Y2 = [ -o_b(1) ,-W_now(2)*W_now(3) ,W_now(2)*W_now(3) ,-o_b(2)-W_now(1)*W_now(3) ,-o_b(3)+W_now(1)*W_now(2), -W_now(3)^2 + W_now(2)^2 ;...
                           W_now(1)*W_now(3), -o_b(2) ,-W_now(1)*W_now(3) ,-o_b(1)+W_now(2)*W_now(3) ,-W_now(1)^2 + W_now(3)^2 , -o_b(3)-W_now(1)*W_now(2);...
                           -W_now(1)*W_now(2) ,W_now(1)*W_now(2), -o_b(3) ,-W_now(2)^2 + W_now(1)^2  ,-o_b(1)-W_now(2)*W_now(3), -o_b(2)+W_now(1)*W_now(3)];

                   Y = [Y2,Y1];

                   Y_omega = [ 0                   ,-W_now(2)*W_now(3) ,W_now(2)*W_now(3)             ,-W_now(1)*W_now(3)      ,W_now(1)*W_now(2)          , -W_now(3)^2 + W_now(2)^2 ;...
                                W_now(1)*W_now(3)   , 0                 ,-W_now(1)*W_now(3)            , W_now(2)*W_now(3)      ,-W_now(1)^2 + W_now(3)^2   , -W_now(1)*W_now(2);...
                                -W_now(1)*W_now(2)  ,W_now(1)*W_now(2)  , 0                            ,-W_now(2)^2 + W_now(1)^2,-W_now(2)*W_now(3)         , W_now(1)*W_now(3)];

                   W_dot = (W_now-obj.last_W);
                  
                   Y_CoG_icl = [   0   -platform.m*g_body(3)  platform.m*g_body(2);...
                   platform.m*g_body(3)   0               -platform.m*g_body(1);...
                   -platform.m*g_body(2)   platform.m*g_body(1)   0];
                   
                   W_dot_matrix = [W_dot(1)     ,0        ,0            ,W_dot(2) ,W_dot(3),0       ;...
                                       0        , W_dot(2),0            ,W_dot(1) ,0       ,W_dot(3);...
                                       0        , 0       ,    W_dot(3) ,0        ,W_dot(1),W_dot(2)];
                                   

                    M_bar = obj.M*platform.dt;


                    y_W = Y_omega*platform.dt + W_dot_matrix;


                    y_cl = [y_W,Y_CoG_icl*platform.dt];
                    

                integral_num = 50;
                for i = integral_num-1:-1:1
                    obj.Y_icl_last(:,:,i+1) = obj.Y_icl_last(:,:,i);
                    obj.M_icl_last(:,i+1) = obj.M_icl_last(:,i);
                end
                        obj.Y_icl_last(:,:,1) = y_cl;
                        obj.M_icl_last(:,1) = M_bar;
                        M_bar_int = zeros(3,1);
                        y_icl_int = zeros(3,9);
                   for i= 1:integral_num
                        y_icl_int = y_icl_int + obj.Y_icl_last(:,:,i);
                        M_bar_int = M_bar_int + obj.M_icl_last(:,i);
                   end
                    

                    if iteration > obj.N
                        for i= 1:obj.N-1
                            obj.sigma_M_hat_array(:,i) = obj.sigma_M_hat_array(:,i+1);
                            obj.sigma_y_array(:,:,i) = obj.sigma_y_array(:,:,i+1);
                        end
                        obj.sigma_M_hat_array(:,obj.N) = M_bar_int;
                        obj.sigma_y_array(:,:,obj.N) = y_icl_int;
                        



                        x = zeros(9,1);
                        for i=2:obj.N
                                x = x + obj.sigma_y_array(:,:,i)'*(obj.sigma_M_hat_array(:,i) - obj.sigma_y_array(:,:,i)*obj.theta );
                                obj.y_icl_temp = obj.y_icl_temp+obj.sigma_y_array(:,:,i);
                        end
                         obj.theta_hat_dot = -obj.gamma*Y'*(eW+obj.c2*eR) + obj.gamma*obj.k_icl * x;
                         [U,S,V] = svd(obj.y_icl_temp);
                         singular_value_y_sys_icl = S;
                         obj.y_icl_temp = zeros(3,9);
                    else
                        for i= 1:obj.N-1
                            obj.sigma_M_hat_array(:,i) = obj.sigma_M_hat_array(:,i+1);
                            obj.sigma_y_array(:,:,i) = obj.sigma_y_array(:,:,i+1);
                        end
                        
                        %if the step not reach N set the singular vector as
                        %0
                        singular_value_y_sys_icl = zeros(3,9);

                        obj.sigma_M_hat_array(:,obj.N) = M_bar;
                        obj.sigma_y_array(:,:,obj.N) = y_cl;
                        obj.theta_hat_dot = -obj.gamma*Y'*(eW+obj.c2*eR);  
                    end

                    obj.last_W = W_now;
                
                    obj.last_R = R_now;
                    obj.theta = obj.theta + obj.theta_hat_dot*platform.dt;
                    obj.M = -obj.kR * eR - obj.kW*eW + Y*obj.theta;



                elseif type == "ICL_RW"
        
                   o_b = W_hat*R_now'*R_d*W_d - R_now'*R_d*W_d_dot; 

                   Y1 = [   0   -platform.m*g_body(3)  platform.m*g_body(2);...
                    platform.m*g_body(3)   0               -platform.m*g_body(1);...
                    -platform.m*g_body(2)   platform.m*g_body(1)   0];


                    Y2 = [ -o_b(1) ,-W_now(2)*W_now(3) ,W_now(2)*W_now(3) ,-o_b(2)-W_now(1)*W_now(3) ,-o_b(3)+W_now(1)*W_now(2), -W_now(3)^2 + W_now(2)^2 ;...
                           W_now(1)*W_now(3), -o_b(2) ,-W_now(1)*W_now(3) ,-o_b(1)+W_now(2)*W_now(3) ,-W_now(1)^2 + W_now(3)^2 , -o_b(3)-W_now(1)*W_now(2);...
                           -W_now(1)*W_now(2) ,W_now(1)*W_now(2), -o_b(3) ,-W_now(2)^2 + W_now(1)^2  ,-o_b(1)-W_now(2)*W_now(3), -o_b(2)+W_now(1)*W_now(3)];

                    Y = [Y2,Y1];

                    Y_omega = [ 0                   ,-W_now(2)*W_now(3) ,W_now(2)*W_now(3)             ,-W_now(1)*W_now(3)      ,W_now(1)*W_now(2)          , -W_now(3)^2 + W_now(2)^2 ;...
                                W_now(1)*W_now(3)   , 0                 ,-W_now(1)*W_now(3)            , W_now(2)*W_now(3)      ,-W_now(1)^2 + W_now(3)^2   , -W_now(1)*W_now(2);...
                                -W_now(1)*W_now(2)  ,W_now(1)*W_now(2)  , 0                            ,-W_now(2)^2 + W_now(1)^2,-W_now(2)*W_now(3)         , W_now(1)*W_now(3)];

                    W_dot = (W_now-obj.last_W);
                  
                    Y_CoG_icl = [   0   -platform.m*g_body(3)  platform.m*g_body(2);...
                    platform.m*g_body(3)   0               -platform.m*g_body(1);...
                    -platform.m*g_body(2)   platform.m*g_body(1)   0];
                   
                    W_dot_matrix = [W_dot(1)     ,0        ,0            ,W_dot(2) ,W_dot(3),0       ;...
                                       0        , W_dot(2),0            ,W_dot(1) ,0       ,W_dot(3);...
                                       0        , 0       ,    W_dot(3) ,0        ,W_dot(1),W_dot(2)];
                                   

                    M_bar = obj.M*platform.dt;


                    y_W = Y_omega*platform.dt + W_dot_matrix;


                    y_cl = [y_W,Y_CoG_icl*platform.dt];
                    

                integral_num = 90;
                for i = integral_num-1:-1:1
                    obj.Y_icl_last(:,:,i+1) = obj.Y_icl_last(:,:,i);
                    obj.M_icl_last(:,i+1) = obj.M_icl_last(:,i);
                end
                        obj.Y_icl_last(:,:,1) = y_cl;
                        obj.M_icl_last(:,1) = M_bar;
                        M_bar_int = zeros(3,1);
                        y_icl_int = zeros(3,9);
                   for i= 1:integral_num
                        y_icl_int = y_icl_int + obj.Y_icl_last(:,:,i);
                        M_bar_int = M_bar_int + obj.M_icl_last(:,i);
                   end
                    

                    if iteration > obj.N
                        for i= 1:obj.N-1
                            obj.sigma_M_hat_array(:,i) = obj.sigma_M_hat_array(:,i+1);
                            obj.sigma_y_array(:,:,i) = obj.sigma_y_array(:,:,i+1);
                        end
                        obj.sigma_M_hat_array(:,obj.N) = M_bar_int;
                        obj.sigma_y_array(:,:,obj.N) = y_icl_int;
                        



                        x = zeros(9,1);
                        for i=2:obj.N
                                x = x + obj.sigma_y_array(:,:,i)'*(obj.sigma_M_hat_array(:,i) - obj.sigma_y_array(:,:,i)*obj.theta );
                                obj.y_icl_temp = obj.y_icl_temp+obj.sigma_y_array(:,:,i);
                        end
                         obj.theta_hat_dot = -obj.gamma*Y'*(eW+obj.c2*eR) + obj.gamma*obj.k_icl * x;
                         [U,S,V] = svd(obj.y_icl_temp);
                         singular_value_y_sys_icl = S;
                         obj.y_icl_temp = zeros(3,9);
                    else
                        for i= 1:obj.N-1
                            obj.sigma_M_hat_array(:,i) = obj.sigma_M_hat_array(:,i+1);
                            obj.sigma_y_array(:,:,i) = obj.sigma_y_array(:,:,i+1);
                        end
                        
                        %if the step not reach N set the singular vector as
                        %0
                        singular_value_y_sys_icl = zeros(3,9);

                        obj.sigma_M_hat_array(:,obj.N) = M_bar;
                        obj.sigma_y_array(:,:,obj.N) = y_cl;
                        obj.theta_hat_dot = -obj.gamma*Y'*(eW+obj.c2*eR);  
                    end

                    obj.last_W = W_now;
                
                    obj.last_R = R_now;
                    obj.theta = obj.theta + obj.theta_hat_dot*platform.dt;


                    %Add the R.W Feedback term



                    RW_Feedback = W_now_hat*AW*J_RW*Omega_now;


                    obj.M = -obj.kR * eR - obj.kW*eW + Y*obj.theta+RW_Feedback; 

                    platform.Omega_dot(:,iteration) = -(HW_inv/J_RW)*[obj.M;0]; %renwe the R.W angular accelerate

                    Omega_dot_now = platform.Omega_dot(:,iteration);
                    Omega_now = Omega_now+Omega_dot_now*platform.dt;  %renwe the R.W angular velocity

                    obj.M_RW = -(AW*J_RW*Omega_dot_now+W_now_hat*AW*J_RW*Omega_now);
                    obj.M = obj.M_RW;


                end
             
                control(1:3) = obj.M;

              end
   end
end
