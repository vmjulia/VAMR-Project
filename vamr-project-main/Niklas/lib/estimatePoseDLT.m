function M = estimatePoseDLT(p, P, K)
    [p_dim, N_points] = size(p);
    [P_dim, N_landmarks] = size(P);
    assert(N_points == N_landmarks, "There must be exactly as many landmarks as keypoints!");
    assert(N_points >= 6, "DLT needs at least 12 points!");
    assert(p_dim == 2 || p_dim == 3, "The first argument are 2D points!");
    assert(P_dim == 4, "3D Points need to be homogeneous!");

    p_normalized = (K \ p);

    Q = build_Q(p_normalized, P);
    [~, ~, V] = svd(Q);
    M_vector = V(:, end);
    M_est = M_to_matrix(M_vector);

    R_est = M_est(:, 1:3);
    t_est = M_est(:, 4);

    [U, ~, V] = svd(R_est);
    R = U * V';
    alpha = norm(R) / norm(R_est);
    t = alpha * t_est;
    
    M = [R, t];
end

function M_matrix = M_to_matrix(M_vector)
    M_matrix = [M_vector(1:4)';
                M_vector(5:8)';
                M_vector(9:12)'];

    if M_matrix(3, 4) < 0
        M_matrix = -1 * M_matrix;
    end
end

function Q = build_Q(p, P)
    n_points = size(p, 2);
    Q = zeros(2 * n_points, 12);
    helper = [1, 0;
              0, 1];
    
    for i = 1:n_points
        P_i = P(:, i)';
        p_i = p(1:2, i);
        Q(2*i - 1 : 2*i, :) = [kron(helper, P_i), kron(-1 * p_i, P_i)];
    end
end