function patch = getWarpedPatch(I, W, x_T, r_T)
% x_T is 1x2 and contains [x_T y_T] as defined in the statement. patch is
% (2*r_T+1)x(2*r_T+1) and arranged consistently with the input image I.
patch_size = [2*r_T + 1, 2*r_T + 1];
n_patch_pixels = patch_size(1) * patch_size(2);
[m, n] = size(I);

[X,Y] = meshgrid(1:patch_size(1), 1:patch_size(2));
u_patch = X(:)';
v_patch = Y(:)';
patch_coordinates = [u_patch; v_patch];
patch_image_coordinates = patch_coordinates + repmat(x_T' - [r_T; r_T], 1, n_patch_pixels);

W_prime = W + [zeros(2), W(1:2, 1:2) * (-x_T') + x_T'];
[indices_I, mask] = getWarpedCoordinates(W_prime, patch_image_coordinates, m, n);
indices_patch = sub2ind(patch_size, v_patch(mask), u_patch(mask));

patch = zeros(1, patch_size(1) * patch_size(2));
I_linear = reshape(I, [1,m*n]);
patch(indices_patch) = I_linear(indices_I);
patch = reshape(patch, patch_size(1), patch_size(2));

end