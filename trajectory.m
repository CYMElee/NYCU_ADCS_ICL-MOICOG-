classdef trajectory
   methods
       function desired = traj_generate(obj, t,iteration,type,platform)
        

      frequency_x = 0.8;
      frequency_y = 0.8;
      frequency_z = 0.5;
      amplitude_x = 0;
      amplitude_y = 0;
      amplitude_z = 0.6;

        if type == "twist"
%            % xd, vd, b1d
           % xd
         
        

           desired_attitude  =[ amplitude_x*sin(frequency_x*t)...
                    ,amplitude_y*cos(frequency_y*t),...
                    amplitude_z*(-sin(frequency_z*t))]';

     


           

           omegad =[    amplitude_x*frequency_x*cos(frequency_x*t);...    
                        amplitude_y*(-frequency_y*sin(frequency_y*t));...
                         amplitude_z*(-frequency_z*cos(frequency_z*t))];

       

     


           desired_angular_velocity = platform.Euler_Matrix*omegad;

           

           omegad_dot =[ -(frequency_x^2)*amplitude_x*sin(frequency_x*t);...    
                           -(frequency_y^2)*amplitude_y*cos(frequency_y*t);...
                         (frequency_z^2)*amplitude_z*sin(frequency_z*t)]; 

          
              

              

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