classdef trajectory
   methods
       function desired = traj_generate(obj, t,iteration,type,platform)
        

      frequency_x =0.2;
      frequency_y = 0.6;
      frequency_z = 0.5;
      amplitude = 0.6;

        if type == "twist"
%            % xd, vd, b1d
           % xd
         
        

           desired_attitude  = amplitude*[sin(frequency_x*t)...
                    ,cos(frequency_y*t),...
                    -sin(frequency_z*t)]';
%            % vd
           

           omegad =amplitude*[    frequency_x*cos(frequency_x*t);...    
                         -frequency_y*sin(frequency_y*t);...
                         - frequency_z*cos(frequency_z*t)];


           desired_angular_velocity = platform.Euler_Matrix*omegad;
%            % ad
           

           omegad_dot =amplitude*[ -(frequency_x^2)*sin(frequency_x*t);...    
                           -(frequency_y^2)*cos(frequency_y*t);...
                         (frequency_z^2)*sin(frequency_z*t)];            

           desired_angular_acceleration = platform.Euler_Matrix*omegad_dot;
           
 
            %% fix pose
        elseif type == "origion"
           
            desired_attitude = [0;0;0];
            desired_angular_velocity =[0;0;0];
            desired_angular_acceleration = [0;0;0];
   
        end
%% return
            desired = [desired_attitude,desired_angular_velocity,desired_angular_acceleration];
       end
   end
end