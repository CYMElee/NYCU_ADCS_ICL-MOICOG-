function [Rd,Wd,Wd_dot] = singular_value_trajectory(U,S,V,J_num,R,W,W_dot,platform)
%This function use to update the singular value that return by controller

R_euler = rotm2eul(R,"ZYX");
R_euler_col = [R_euler(3),R_euler(2),R_euler(1)]';
euler_matrix_inv = [1,sin(R_euler(3))*tan(R_euler(2)),cos(R_euler(3))*tan(R_euler(2));...
               0,cos(R_euler(3)),-sin(R_euler(3));...
               0,sin(R_euler(3))*sec(R_euler(2)),cos(R_euler(3))*sec(R_euler(2))];


disp(J_num);
U_d = U;
S_d = S;
V_d = V;
V_d(:,3) = [0,0,0,0,0,0,1,1,1]';
delta_v= V-V_d;
desire_delta = J_num*reshape((U_d*S_d*(delta_v')),27,1);
%disp(desire_delta);
delta_Rd =desire_delta(1:3,1);

delta_Wd =desire_delta(4:6,1);
delta_Wd_dot=desire_delta(7:9,1);


Wd =W-delta_Wd;
Wd_dot =W_dot-delta_Wd_dot;

Rd=-(euler_matrix_inv*Wd*platform.dt-R_euler_col);






end
