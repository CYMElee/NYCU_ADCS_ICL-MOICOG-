function [ew] = Omega_Error(R_measure,R_desire,omega_d,omega)
%OMEGA_ERROR Summary of this function goes here
%   Detailed explanation goes here
ew = [0,0,0]';

R = eul2rotm(R_measure);
Rd = eul2rotm(R_desire);
ew = omega - R'*Rd*omega_d;
end
