function [Sigma_Ix2, Sigma_Iy2, Sigma_Ixy] = moment_matrix_coeffs(img, patch_size)
    sobel_x = [-1, 0, 1;
                -2, 0, 2;
                -1, 0, -1];
    sobel_y = [-1, -2, -1;
                0, 0, 0;
                1, 2, 1];

    Ix = conv2(img, sobel_x, 'valid');
    Iy = conv2(img, sobel_y, 'valid');

    Ix2 = Ix .* Ix;
    Iy2 = Iy .* Iy;
    Ixy = Ix .* Iy;

    window_sum_filter = ones(patch_size);

    Sigma_Ix2 = conv2(Ix2, window_sum_filter, 'valid');
    Sigma_Iy2 = conv2(Iy2, window_sum_filter, 'valid');
    Sigma_Ixy = conv2(Ixy, window_sum_filter, 'valid');


end