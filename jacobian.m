
% Define the symbolic
syms theta1 theta2 theta3 omega1 omega2 omega3 omega_dot1 omega_dot2 omega_dot3 m g;



% Define the regression matrix
Y_COG=[0,                               -m*g*cos(theta1)*cos(theta2),       m*g*sin(theta1)*cos(theta2);...
       m*g*cos(theta1)*cos(theta2),     0,                                  m*g*sin(theta2);...
       -m*g*sin(theta1)*cos(theta2),    -m*g*sin(theta2),                    0];          







Y_J=[omega_dot1,                omega1*omega3,              -omega1*omega2;...
     -omega2*omega3,            omega_dot2,                 omega1*omega2;...
     omega2*omega3,             -omega1*omega3,             omega_dot3;...
     omega_dot2-omega1*omega3,  omega_dot1+omega2*omega3,   -omega2^2+omega1^2;...
     omega_dot3+omega1*omega2,  -omega1^2+omega3^2,         -omega2*omega3;...
     omega2^2-omega3^2,          omega_dot3-omega1*omega2,   omega_dot2+omega1*omega3]';

Y_icl_sys=[Y_J,Y_COG];



y_icl_sys_vec=Y_icl_sys(:);

vars=[theta1 theta2 theta3 omega1 omega2 omega3 omega_dot1 omega_dot2 omega_dot3];
J = jacobian(y_icl_sys_vec, vars);

% Eigenvalue of J'*J
E = eig(J'*J);  % 計算特徵值，可能是 symbolic 類型


E_numeric = vpa(E, 3); 
fileID = fopen('eigenvalues.txt', 'w');  % 開啟檔案
fprintf(fileID, '%s\n', char(E_numeric));  % 轉換為字串並寫入
fclose(fileID);  % 關閉檔案
Det=det(J'*J);




disp("display the eigenvalue of J'*J");
disp(E(1));



disp("display the jacobian matrix");

disp(J);

% Psudo inverse of Jacobian matrix
J_inv = inv(J'*J)*J';
disp("display the inverse jacobian matrix");
disp(J_inv);
