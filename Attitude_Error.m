function [er] = Attitude_Error(r,rd)
%use desire Attitude and measure Attitude get Attitude Error eR
er = [0,0,0]';

R = eul2rotm(r);
Rd = eul2rotm(rd);
%using vee map covert eR from SO(3) to R^3
er_temp = 0.5*(Rd'*R - R'*Rd);
er = [er_temp(3,2);er_temp(1,3);er_temp(2,1)];

end
