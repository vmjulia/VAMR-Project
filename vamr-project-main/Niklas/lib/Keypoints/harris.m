function scores = harris(img, patch_size, kappa)
    [Sxx, Syy, Sxy] = moment_matrix(img, patch_size);
    determinant = Sxx .* Syy - (Sxy .* Sxy);
    trace = (Sxx + Syy);
    
    R = determinant - (kappa * (trace .* trace));
    scores = padarray(R, floor(patch_size ./2 + 1), 0);
    scores = max(scores, 0);
end
