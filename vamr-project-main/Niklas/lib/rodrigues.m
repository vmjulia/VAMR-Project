function R = rodrigues(w)
    theta = norm(w);
    k = w / theta;
    k_cross = cross_matrix_3(k);
    R = eye(3) + sin(theta) * k_cross + (1 - cos(theta)) * k_cross * k_cross;
    %assert(det(R) == 1);
end