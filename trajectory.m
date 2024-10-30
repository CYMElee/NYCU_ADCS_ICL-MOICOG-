classdef trajectory
   methods
       function desired = traj_generate(obj, t,type,platform)
        
       R_now = reshape(platform.R(:,t-1), 3, 3);

       R_now_euler = rotm2eul(R_now);
       Euler_Matrix = [1,0,-sin(R_now_euler(2));...
                    0,cos(R_now_euler(1)),sin(R_now_euler(1))*cos(R_now_euler(2));...
                    0,-sin(R_now_euler(1)),cos(R_now_euler(1))*cos(R_now_euler(2))];
        
      frequency_x =0.2;
      frequency_y = 0.6;
      frequency_z = 0.5;
      amplitude = 0.6;

        if type == "twist"
%            % xd, vd, b1d
           % xd
         
           desired_attitude = zeros(3,1);

           desired_attitude  = amplitude*[sin(frequency_x*t)...
                    ,cos(frequency_y*t),...
                    -sin(frequency_z*t)]';
%            % vd
           desired_angular_velocity = zeros(3,1);

           omegad =amplitude*[    frequency_x*cos(frequency_x*t);...    
                         -frequency_y*sin(frequency_y*t);...
                         - frequency_z*cos(frequency_z*t)];


           desired_angular_velocity = Euler_Matrix*omegad;
%            % ad
           desired_angular_acceleration = zeros(3,1);

           omegad_dot =amplitude*[ -(frequency_x^2)*sin(frequency_x*t);...    
                           -(frequency_y^2)*cos(frequency_y*t);...
                         (frequency_z^2)*sin(frequency_z*t)];            

           desired_angular_acceleration = Euler_Matrix*omegad_dot;
           
 
            %% fix pose
        elseif type == "origion"
            desired_attitude = zeros(3,1);
            desired_attitude(1) = 0;
            desired_attitude(2) = 0;
            desired_attitude(3) = 0;
            desired_angular_velocity = zeros(3,1);
            desired_angular_velocity(1) = 0;
            desired_angular_velocity(2) = 0;
            desired_angular_velocity(3) = 0;
            desired_angular_acceleration = zeros(3,1);
            desired_angular_acceleration(1) = 0;
            desired_angular_acceleration(2) = 0;
            desired_angular_acceleration(3) = 0;
   
        end
%% return
            desired = [desired_attitude,desired_angular_velocity,desired_angular_acceleration];
       end
   end
end