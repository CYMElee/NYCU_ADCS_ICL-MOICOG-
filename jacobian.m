syms theta1 theta2 theta3 omega1 omega2 omega3 omega_dot1 omega_dot2 omega_dot3 m g;




Y_COG=[0,-m*g*cos(theta1)*cos(theta2),m*g*sin(theta1)*cos(theta2);...
       m*g*cos(theta1)*cos(theta2),0,-m*g*sin(theta2);...
       -m*g*sin(theta1)*cos(theta2),m*g*sin(theta2),0];



Y_J=[omega_dot1,omega1*omega3,-omega1*omega2;...
     -omega2*omega3,omega_dot2,omega1*omega2;...
     omega2*omega3,-omega1*omega3,omega_dot3;...
     omega_dot2-omega1*omega3,omega_dot1+omega2*omega3,-omega2^2+omega1^2;...
    omega_dot3+omega1*omega3,-omega1^2+omega3^2, -omega2*omega3;...
    omega2^2-omega3^2,omega_dot3-omega1*omega2,omega_dot2+omega1*omega3]';

Y_icl_sys=[Y_J,Y_COG];

y_icl_sys_vec=Y_icl_sys(:);

vars=[theta1 theta2 theta3 omega1 omega2 omega3 omega_dot1 omega_dot2 omega_dot3];
J = jacobian(y_icl_sys_vec, vars);
disp(J);
