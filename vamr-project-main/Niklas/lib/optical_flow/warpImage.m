function I = warpImage(I_R, W)
    [m,n] = size(I_R);
    I = zeros(1, m*n);
%     R_inv = W(1:2, 1:2)';
%     t_inv = - R_inv * W(1:2,3);
%     W_inv = [R_inv, t_inv];

    [X,Y] = meshgrid(1:n,1:m);
    [indices_R, valid_mask] = getWarpedCoordinates(W, [X(:)'; Y(:)'], m, n);
    indices_I = sub2ind([m,n], Y(valid_mask), X(valid_mask));

    I_R_linearized = reshape(I_R, [1,m*n]);
    I(indices_I) = I_R_linearized(indices_R);
    I = reshape(I, m,n);
end