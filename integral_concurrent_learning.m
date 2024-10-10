function [icl_term] = integral_concurrent_learning(record_m,record_y,theta_hat,i)
%ICL Summary of this function goes here
%   Detailed explanation goes here
icl_term = [0,0,0,0,0,0]';

icl_term_prev = icl_term;


for j = 1:10
     icl_term = icl_term_prev + ((record_y(:,:,i-j))'*(record_m(:,i-j)-record_y(i-j)*theta_hat));
     icl_term_prev = icl_term;
end

end


