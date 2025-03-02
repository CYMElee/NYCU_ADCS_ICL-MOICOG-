function [Rd,Wd,Wd_dot] = singular_value_trajectory(U,S,V,J_num,R,W,W_dot)
%This function use to update the singular value that return by controller

euler



U_d = U;
S_d = S;
V_d = V;
V_d(:,3) = [0,0,0,0,0,0,5e-3,5e-3,5e-3]';
delta_v= V_d-V;
desire_delta = J_num*reshape((U_d*S_d*(delta_v')),27,1);
delta_Rd =desire_delta(1:3,1);
delta_Wd =desire_delta(4:6,1);
delta_Wd_dot=desire_delta(7:9,1);

delta_Rd_hat = hat(delta_Rd);
Rd =R*(eye(3)+2*delta_Rd_hat);
Wd =Rd'*R*(W-delta_Wd);
Wd_dot =Rd'*R*(W_dot-delta_Wd_dot);
Rd = reshape(rotm2eul(Rd,"XYZ"),3,1);


end
