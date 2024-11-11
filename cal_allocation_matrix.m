function M = cal_allocation_matrix(d, c_tau)
    cos_45 = cosd(45);
    M = [     1/2,     -1/2,      -1/2,     1/2;
              1/2,      1/2,      -1/2,     -1/2;
              1/cos_45, 1/cos_45,1/cos_45,  1/cos_45];
end