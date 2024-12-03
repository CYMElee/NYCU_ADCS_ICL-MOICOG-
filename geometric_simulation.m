 clear;
close all;

addpath('geometry-toolbox')
%% set drone parameters
% simulation time
dt = 1/1000;
sim_t =180;

platform1 = platform_dynamic;
platform1.dt = dt;            %delta t
platform1.sim_t = sim_t;      %whole time 
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


%% create constraint array
platform1.Attitude_upper_limit = zeros(3,length(platform1.t));
platform1.Attitude_lower_limit =  zeros(3,length(platform1.t));

platform1.omega_upper_limit = zeros(3,length(platform1.t));
platform1.omega_lower_limit = zeros(3,length(platform1.t));

platform1.omega_dot_upper_limit = zeros(3,length(platform1.t));
platform1.omega_dot_lower_limit = zeros(3,length(platform1.t));






%% create states arra(R.W.)
platform1.Omega_dot = zeros(4, length(platform1.t));
platform1.Omega = zeros(4, length(platform1.t));





%% create attitude array use to plot
platform1.R_Euler = zeros(3, length(platform1.t));
platform1.Rd_Euler = zeros(3, length(platform1.t));


%% create icl_term 
platform1.icl_term = zeros(3,length(platform1.t));


%% create singular velue matrix

platform1.y_sys_icl_singular_value = zeros(3,9,length(platform1.t));
y_sys_icl_11 =  zeros(length(platform1.t),1);
y_sys_icl_12 =  zeros(length(platform1.t),1);
y_sys_icl_13 =  zeros(length(platform1.t),1);
y_sys_icl_14 =  zeros(length(platform1.t),1);
y_sys_icl_15 =  zeros(length(platform1.t),1);
y_sys_icl_16 =  zeros(length(platform1.t),1);
y_sys_icl_17 =  zeros(length(platform1.t),1);
y_sys_icl_18 =  zeros(length(platform1.t),1);
y_sys_icl_19 =  zeros(length(platform1.t),1);

y_sys_icl_21 =  zeros(length(platform1.t),1);
y_sys_icl_22 =  zeros(length(platform1.t),1);
y_sys_icl_23 =  zeros(length(platform1.t),1);
y_sys_icl_24 =  zeros(length(platform1.t),1);
y_sys_icl_25 =  zeros(length(platform1.t),1);
y_sys_icl_26 =  zeros(length(platform1.t),1);
y_sys_icl_27 =  zeros(length(platform1.t),1);
y_sys_icl_28 =  zeros(length(platform1.t),1);
y_sys_icl_29 =  zeros(length(platform1.t),1);


y_sys_icl_31 =  zeros(length(platform1.t),1);
y_sys_icl_32 =  zeros(length(platform1.t),1);
y_sys_icl_33 =  zeros(length(platform1.t),1);
y_sys_icl_34 =  zeros(length(platform1.t),1);
y_sys_icl_35 =  zeros(length(platform1.t),1);
y_sys_icl_36 =  zeros(length(platform1.t),1);
y_sys_icl_37 =  zeros(length(platform1.t),1);
y_sys_icl_38 =  zeros(length(platform1.t),1);
y_sys_icl_39 =  zeros(length(platform1.t),1);
y_sys_icl_rank = zeros(length(platform1.t),1);

%% create  Left singular vectors
platform1.y_sys_icl_left_singular_value = zeros(3,3,length(platform1.t));
y_sys_icl_left_11 =  zeros(length(platform1.t),1);
y_sys_icl_left_12 =  zeros(length(platform1.t),1);
y_sys_icl_left_13 =  zeros(length(platform1.t),1);


y_sys_icl_left_21 =  zeros(length(platform1.t),1);
y_sys_icl_left_22 =  zeros(length(platform1.t),1);
y_sys_icl_left_23 =  zeros(length(platform1.t),1);


y_sys_icl_left_31 =  zeros(length(platform1.t),1);
y_sys_icl_left_32 =  zeros(length(platform1.t),1);
y_sys_icl_left_33 =  zeros(length(platform1.t),1);




%% create  Right singular vectors
platform1.y_sys_icl_right_singular_value = zeros(9,9,length(platform1.t));
y_sys_icl_right_11 =  zeros(length(platform1.t),1);
y_sys_icl_right_12 =  zeros(length(platform1.t),1);
y_sys_icl_right_13 =  zeros(length(platform1.t),1);
y_sys_icl_right_14 =  zeros(length(platform1.t),1);
y_sys_icl_right_15 =  zeros(length(platform1.t),1);
y_sys_icl_right_16 =  zeros(length(platform1.t),1);
y_sys_icl_right_17 =  zeros(length(platform1.t),1);
y_sys_icl_right_18 =  zeros(length(platform1.t),1);
y_sys_icl_right_19 =  zeros(length(platform1.t),1);

y_sys_icl_right_21 =  zeros(length(platform1.t),1);
y_sys_icl_right_22 =  zeros(length(platform1.t),1);
y_sys_icl_right_23 =  zeros(length(platform1.t),1);
y_sys_icl_right_24 =  zeros(length(platform1.t),1);
y_sys_icl_right_25 =  zeros(length(platform1.t),1);
y_sys_icl_right_26 =  zeros(length(platform1.t),1);
y_sys_icl_right_27 =  zeros(length(platform1.t),1);
y_sys_icl_right_28 =  zeros(length(platform1.t),1);
y_sys_icl_right_29 =  zeros(length(platform1.t),1);

y_sys_icl_right_31 =  zeros(length(platform1.t),1);
y_sys_icl_right_32 =  zeros(length(platform1.t),1);
y_sys_icl_right_33 =  zeros(length(platform1.t),1);
y_sys_icl_right_34 =  zeros(length(platform1.t),1);
y_sys_icl_right_35 =  zeros(length(platform1.t),1);
y_sys_icl_right_36 =  zeros(length(platform1.t),1);
y_sys_icl_right_37 =  zeros(length(platform1.t),1);
y_sys_icl_right_38 =  zeros(length(platform1.t),1);
y_sys_icl_right_39 =  zeros(length(platform1.t),1);


y_sys_icl_right_41 =  zeros(length(platform1.t),1);
y_sys_icl_right_42 =  zeros(length(platform1.t),1);
y_sys_icl_right_43 =  zeros(length(platform1.t),1);
y_sys_icl_right_44 =  zeros(length(platform1.t),1);
y_sys_icl_right_45 =  zeros(length(platform1.t),1);
y_sys_icl_right_46 =  zeros(length(platform1.t),1);
y_sys_icl_right_47 =  zeros(length(platform1.t),1);
y_sys_icl_right_48 =  zeros(length(platform1.t),1);
y_sys_icl_right_49 =  zeros(length(platform1.t),1);




y_sys_icl_right_51 =  zeros(length(platform1.t),1);
y_sys_icl_right_52 =  zeros(length(platform1.t),1);
y_sys_icl_right_53 =  zeros(length(platform1.t),1);
y_sys_icl_right_54 =  zeros(length(platform1.t),1);
y_sys_icl_right_55 =  zeros(length(platform1.t),1);
y_sys_icl_right_56 =  zeros(length(platform1.t),1);
y_sys_icl_right_57 =  zeros(length(platform1.t),1);
y_sys_icl_right_58 =  zeros(length(platform1.t),1);
y_sys_icl_right_59 =  zeros(length(platform1.t),1);


y_sys_icl_right_61 =  zeros(length(platform1.t),1);
y_sys_icl_right_62 =  zeros(length(platform1.t),1);
y_sys_icl_right_63 =  zeros(length(platform1.t),1);
y_sys_icl_right_64 =  zeros(length(platform1.t),1);
y_sys_icl_right_65 =  zeros(length(platform1.t),1);
y_sys_icl_right_66 =  zeros(length(platform1.t),1);
y_sys_icl_right_67 =  zeros(length(platform1.t),1);
y_sys_icl_right_68 =  zeros(length(platform1.t),1);
y_sys_icl_right_69 =  zeros(length(platform1.t),1);


y_sys_icl_right_71 =  zeros(length(platform1.t),1);
y_sys_icl_right_72 =  zeros(length(platform1.t),1);
y_sys_icl_right_73 =  zeros(length(platform1.t),1);
y_sys_icl_right_74 =  zeros(length(platform1.t),1);
y_sys_icl_right_75 =  zeros(length(platform1.t),1);
y_sys_icl_right_76 =  zeros(length(platform1.t),1);
y_sys_icl_right_77 =  zeros(length(platform1.t),1);
y_sys_icl_right_78 =  zeros(length(platform1.t),1);
y_sys_icl_right_79 =  zeros(length(platform1.t),1);

y_sys_icl_right_81 =  zeros(length(platform1.t),1);
y_sys_icl_right_82 =  zeros(length(platform1.t),1);
y_sys_icl_right_83 =  zeros(length(platform1.t),1);
y_sys_icl_right_84 =  zeros(length(platform1.t),1);
y_sys_icl_right_85 =  zeros(length(platform1.t),1);
y_sys_icl_right_86 =  zeros(length(platform1.t),1);
y_sys_icl_right_87 =  zeros(length(platform1.t),1);
y_sys_icl_right_88 =  zeros(length(platform1.t),1);
y_sys_icl_right_89 =  zeros(length(platform1.t),1);



y_sys_icl_right_91 =  zeros(length(platform1.t),1);
y_sys_icl_right_92 =  zeros(length(platform1.t),1);
y_sys_icl_right_93 =  zeros(length(platform1.t),1);
y_sys_icl_right_94 =  zeros(length(platform1.t),1);
y_sys_icl_right_95 =  zeros(length(platform1.t),1);
y_sys_icl_right_96 =  zeros(length(platform1.t),1);
y_sys_icl_right_97 =  zeros(length(platform1.t),1);
y_sys_icl_right_98 =  zeros(length(platform1.t),1);
y_sys_icl_right_99 =  zeros(length(platform1.t),1);




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

traj_type = "twist";   %"twist","exp"
controller_type = "ICL_RW";   %"origin","EMK","adaptive","ICL"

control_output_platform1  = zeros(3,1);
control_platform1.theta = 0.9*[platform1.J(1,1);platform1.J(2,2);platform1.J(3,3);platform1.J(1,2);platform1.J(1,3);platform1.J(2,3);platform1.pc_2_mc(1);platform1.pc_2_mc(2);platform1.pc_2_mc(3)];
%control_platform1.theta = 0.9*[platform1.pc_2_mc(1);platform1.pc_2_mc(2);platform1.pc_2_mc(3);platform1.J(1,1);platform1.J(2,2);platform1.J(3,3);platform1.J(1,2);platform1.J(1,3);platform1.J(2,3)];


for i = 2:length(platform1.t)
    disp(i)
    t_now = platform1.t(i);
    platform1.Euler_Matrix = platform1.get_euler_matrix(i);
    desired = traj.traj_generate(t_now,i,traj_type,platform1);
    platform1.Rd_Euler(:,i)=desired(:,1);
    % calculate control force
    [control_output_platform1, platform1.eR(:, i), platform1.eW(:, i),control_platform1,platform1.y_sys_icl_singular_value(:,:,i),platform1.y_sys_icl_left_singular_value(:,:,i),platform1.y_sys_icl_right_singular_value(:,:,i),platform1.icl_term(:,i),platform1.Omega_dot(:,i),platform1.Omega(:,i)] = control_platform1.geometric_tracking_ctrl(i,platform1,desired,controller_type);
    
    %store the y_sys_icl to y_sys_icl use to plot
    y_sys_icl_11(i) = platform1.y_sys_icl_singular_value(1,1,i);
    y_sys_icl_12(i) = platform1.y_sys_icl_singular_value(1,2,i);
    y_sys_icl_13(i) = platform1.y_sys_icl_singular_value(1,3,i);
    y_sys_icl_14(i) = platform1.y_sys_icl_singular_value(1,4,i);
    y_sys_icl_15(i) = platform1.y_sys_icl_singular_value(1,5,i);
    y_sys_icl_16(i) = platform1.y_sys_icl_singular_value(1,6,i);
    y_sys_icl_17(i) = platform1.y_sys_icl_singular_value(1,7,i);
    y_sys_icl_18(i) = platform1.y_sys_icl_singular_value(1,8,i);
    y_sys_icl_19(i) = platform1.y_sys_icl_singular_value(1,9,i);


    y_sys_icl_21(i) = platform1.y_sys_icl_singular_value(2,1,i);
    y_sys_icl_22(i) = platform1.y_sys_icl_singular_value(2,2,i);
    y_sys_icl_23(i) = platform1.y_sys_icl_singular_value(2,3,i);
    y_sys_icl_24(i) = platform1.y_sys_icl_singular_value(2,4,i);
    y_sys_icl_25(i) = platform1.y_sys_icl_singular_value(2,5,i);
    y_sys_icl_26(i) = platform1.y_sys_icl_singular_value(2,6,i);
    y_sys_icl_27(i) = platform1.y_sys_icl_singular_value(2,7,i);
    y_sys_icl_28(i) = platform1.y_sys_icl_singular_value(2,8,i);
    y_sys_icl_29(i) = platform1.y_sys_icl_singular_value(2,9,i);


    y_sys_icl_31(i) = platform1.y_sys_icl_singular_value(3,1,i);
    y_sys_icl_32(i) = platform1.y_sys_icl_singular_value(3,2,i);
    y_sys_icl_33(i) = platform1.y_sys_icl_singular_value(3,3,i);
    y_sys_icl_34(i) = platform1.y_sys_icl_singular_value(3,4,i);
    y_sys_icl_35(i) = platform1.y_sys_icl_singular_value(3,5,i);
    y_sys_icl_36(i) = platform1.y_sys_icl_singular_value(3,6,i);
    y_sys_icl_37(i) = platform1.y_sys_icl_singular_value(3,7,i);
    y_sys_icl_38(i) = platform1.y_sys_icl_singular_value(3,8,i);
    y_sys_icl_39(i) = platform1.y_sys_icl_singular_value(3,9,i);


   
    y_sys_icl_rank(i)=rank(platform1.y_sys_icl_singular_value(:,:,i));
    %store the y_sys_left_icl to y_sys_icl_left use to plot
    y_sys_icl_left_11(i) =  platform1.y_sys_icl_left_singular_value(1,1,i);
    y_sys_icl_left_12(i) =  platform1.y_sys_icl_left_singular_value(1,2,i);
    y_sys_icl_left_13(i) =  platform1.y_sys_icl_left_singular_value(1,3,i);


    y_sys_icl_left_21(i) =  platform1.y_sys_icl_left_singular_value(2,1,i);
    y_sys_icl_left_22(i) =  platform1.y_sys_icl_left_singular_value(2,2,i);
    y_sys_icl_left_23(i) =  platform1.y_sys_icl_left_singular_value(2,3,i);


    y_sys_icl_left_31(i) =  platform1.y_sys_icl_left_singular_value(3,1,i);
    y_sys_icl_left_32(i) =  platform1.y_sys_icl_left_singular_value(3,2,i);
    y_sys_icl_left_33(i) =  platform1.y_sys_icl_left_singular_value(3,3,i); 
    %store the y_sys_left_icl to y_sys_icl_right use to plot
    y_sys_icl_right_11(i) =  platform1.y_sys_icl_right_singular_value(1,1,i);
    y_sys_icl_right_12(i) =  platform1.y_sys_icl_right_singular_value(1,2,i);
    y_sys_icl_right_13(i) =  platform1.y_sys_icl_right_singular_value(1,3,i);
    y_sys_icl_right_14(i) =  platform1.y_sys_icl_right_singular_value(1,4,i);
    y_sys_icl_right_15(i) =  platform1.y_sys_icl_right_singular_value(1,5,i);
    y_sys_icl_right_16(i) =  platform1.y_sys_icl_right_singular_value(1,6,i);
    y_sys_icl_right_17(i) =  platform1.y_sys_icl_right_singular_value(1,7,i);
    y_sys_icl_right_18(i) =  platform1.y_sys_icl_right_singular_value(1,8,i);
    y_sys_icl_right_19(i) =  platform1.y_sys_icl_right_singular_value(1,9,i);

    y_sys_icl_right_21(i) =  platform1.y_sys_icl_right_singular_value(2,1,i);
    y_sys_icl_right_22(i) =  platform1.y_sys_icl_right_singular_value(2,2,i);
    y_sys_icl_right_23(i) =  platform1.y_sys_icl_right_singular_value(2,3,i);
    y_sys_icl_right_24(i) =  platform1.y_sys_icl_right_singular_value(2,4,i);
    y_sys_icl_right_25(i) =  platform1.y_sys_icl_right_singular_value(2,5,i);
    y_sys_icl_right_26(i) =  platform1.y_sys_icl_right_singular_value(2,6,i);
    y_sys_icl_right_27(i) =  platform1.y_sys_icl_right_singular_value(2,7,i);
    y_sys_icl_right_28(i) =  platform1.y_sys_icl_right_singular_value(2,8,i);
    y_sys_icl_right_29(i) =  platform1.y_sys_icl_right_singular_value(2,9,i);

    y_sys_icl_right_31(i) =  platform1.y_sys_icl_right_singular_value(3,1,i);
    y_sys_icl_right_32(i) =  platform1.y_sys_icl_right_singular_value(3,2,i);
    y_sys_icl_right_33(i) =  platform1.y_sys_icl_right_singular_value(3,3,i);
    y_sys_icl_right_34(i) =  platform1.y_sys_icl_right_singular_value(3,4,i);
    y_sys_icl_right_35(i) =  platform1.y_sys_icl_right_singular_value(3,5,i);
    y_sys_icl_right_36(i) =  platform1.y_sys_icl_right_singular_value(3,6,i);
    y_sys_icl_right_37(i) =  platform1.y_sys_icl_right_singular_value(3,7,i);
    y_sys_icl_right_38(i) =  platform1.y_sys_icl_right_singular_value(3,8,i);
    y_sys_icl_right_39(i) =  platform1.y_sys_icl_right_singular_value(3,9,i);


    y_sys_icl_right_41(i) =  platform1.y_sys_icl_right_singular_value(4,1,i);
    y_sys_icl_right_42(i) =  platform1.y_sys_icl_right_singular_value(4,2,i);
    y_sys_icl_right_43(i) =  platform1.y_sys_icl_right_singular_value(4,3,i);
    y_sys_icl_right_44(i) =  platform1.y_sys_icl_right_singular_value(4,4,i);
    y_sys_icl_right_45(i) =  platform1.y_sys_icl_right_singular_value(4,5,i);
    y_sys_icl_right_46(i) =  platform1.y_sys_icl_right_singular_value(4,6,i);
    y_sys_icl_right_47(i) =  platform1.y_sys_icl_right_singular_value(4,7,i);
    y_sys_icl_right_48(i) =  platform1.y_sys_icl_right_singular_value(4,8,i);
    y_sys_icl_right_49(i) =  platform1.y_sys_icl_right_singular_value(4,9,i);




    y_sys_icl_right_51(i) =  platform1.y_sys_icl_right_singular_value(5,1,i);
    y_sys_icl_right_52(i) =  platform1.y_sys_icl_right_singular_value(5,2,i);
    y_sys_icl_right_53(i) =  platform1.y_sys_icl_right_singular_value(5,3,i);
    y_sys_icl_right_54(i) =  platform1.y_sys_icl_right_singular_value(5,4,i);
    y_sys_icl_right_55(i) =  platform1.y_sys_icl_right_singular_value(5,5,i);
    y_sys_icl_right_56(i) =  platform1.y_sys_icl_right_singular_value(5,6,i);
    y_sys_icl_right_57(i) =  platform1.y_sys_icl_right_singular_value(5,7,i);
    y_sys_icl_right_58(i) =  platform1.y_sys_icl_right_singular_value(5,8,i);
    y_sys_icl_right_59(i) =  platform1.y_sys_icl_right_singular_value(5,9,i);


    y_sys_icl_right_61(i) =  platform1.y_sys_icl_right_singular_value(6,1,i);
    y_sys_icl_right_62(i) =  platform1.y_sys_icl_right_singular_value(6,2,i);
    y_sys_icl_right_63(i) =  platform1.y_sys_icl_right_singular_value(6,3,i);
    y_sys_icl_right_64(i) =  platform1.y_sys_icl_right_singular_value(6,4,i);
    y_sys_icl_right_65(i) =  platform1.y_sys_icl_right_singular_value(6,5,i);
    y_sys_icl_right_66(i) =  platform1.y_sys_icl_right_singular_value(6,6,i);
    y_sys_icl_right_67(i) =  platform1.y_sys_icl_right_singular_value(6,7,i);
    y_sys_icl_right_68(i) =  platform1.y_sys_icl_right_singular_value(6,8,i);
    y_sys_icl_right_69(i) =  platform1.y_sys_icl_right_singular_value(6,9,i);


    y_sys_icl_right_71(i) =  platform1.y_sys_icl_right_singular_value(7,1,i);
    y_sys_icl_right_72(i) =  platform1.y_sys_icl_right_singular_value(7,2,i);
    y_sys_icl_right_73(i) =  platform1.y_sys_icl_right_singular_value(7,3,i);
    y_sys_icl_right_74(i) =  platform1.y_sys_icl_right_singular_value(7,4,i);
    y_sys_icl_right_75(i) =  platform1.y_sys_icl_right_singular_value(7,5,i);
    y_sys_icl_right_76(i) =  platform1.y_sys_icl_right_singular_value(7,6,i);
    y_sys_icl_right_77(i) =  platform1.y_sys_icl_right_singular_value(7,7,i);
    y_sys_icl_right_78(i) =  platform1.y_sys_icl_right_singular_value(7,8,i);
    y_sys_icl_right_79(i) =  platform1.y_sys_icl_right_singular_value(7,9,i);

    y_sys_icl_right_81(i) =  platform1.y_sys_icl_right_singular_value(8,1,i);
    y_sys_icl_right_82(i) =  platform1.y_sys_icl_right_singular_value(8,2,i);
    y_sys_icl_right_83(i) =  platform1.y_sys_icl_right_singular_value(8,3,i);
    y_sys_icl_right_84(i) =  platform1.y_sys_icl_right_singular_value(8,4,i);
    y_sys_icl_right_85(i) =  platform1.y_sys_icl_right_singular_value(8,5,i);
    y_sys_icl_right_86(i) =  platform1.y_sys_icl_right_singular_value(8,6,i);
    y_sys_icl_right_87(i) =  platform1.y_sys_icl_right_singular_value(8,7,i);
    y_sys_icl_right_88(i) =  platform1.y_sys_icl_right_singular_value(8,8,i);
    y_sys_icl_right_89(i) =  platform1.y_sys_icl_right_singular_value(8,9,i);



    y_sys_icl_right_91(i) =  platform1.y_sys_icl_right_singular_value(9,1,i);
    y_sys_icl_right_92(i) =  platform1.y_sys_icl_right_singular_value(9,2,i);
    y_sys_icl_right_93(i) =  platform1.y_sys_icl_right_singular_value(9,3,i);
    y_sys_icl_right_94(i) =  platform1.y_sys_icl_right_singular_value(9,4,i);
    y_sys_icl_right_95(i) =  platform1.y_sys_icl_right_singular_value(9,5,i);
    y_sys_icl_right_96(i) =  platform1.y_sys_icl_right_singular_value(9,6,i);
    y_sys_icl_right_97(i) =  platform1.y_sys_icl_right_singular_value(9,7,i);
    y_sys_icl_right_98(i) =  platform1.y_sys_icl_right_singular_value(9,8,i);
    y_sys_icl_right_99(i) =  platform1.y_sys_icl_right_singular_value(9,9,i);




    
    real_theta_array_platform1(:,i) = [platform1.J(1,1),platform1.J(2,2),platform1.J(3,3),platform1.J(1,2),platform1.J(1,3),platform1.J(2,3),platform1.pc_2_mc(1),platform1.pc_2_mc(2),platform1.pc_2_mc(3)];
    %real_theta_array_platform1(:,i) = [platform1.pc_2_mc(1),platform1.pc_2_mc(2),platform1.pc_2_mc(3),platform1.J(1,1),platform1.J(2,2),platform1.J(3,3),platform1.J(1,2),platform1.J(1,3),platform1.J(2,3)];
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


    % Save the Platform constraint
    platform1.Attitude_upper_limit(:,i) = [0.785;0.785;3.14];
    platform1.Attitude_lower_limit(:,i) = [-0.785;-0.785;-3.14];

    platform1.omega_upper_limit(:,i) = [0.1745;0.1745;0.1745];
    platform1.omega_lower_limit(:,i) = [-0.1745;-0.1745;-0.1745];

    platform1.omega_dot_upper_limit(:,i) = [0.0375;0.0375;0.0375];
    platform1.omega_dot_lower_limit(:,i) = [-0.0375;-0.0375;-0.0375];
    
    
end






