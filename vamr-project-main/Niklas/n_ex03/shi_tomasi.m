function scores = shi_tomasi(img, patch_size)

    [Sxx, Syy, Sxy] = moment_matrix(img, patch_size);

    neg_p = (Sxx + Syy);
    neg_q = (Sxy .^2) - neg_p;
    neg_p_2 = neg_p ./ 2.;

    root_part = ( (neg_p_2 .^2) + neg_q ) .^ .5;

    lambda1 = neg_p_2 + root_part;
    lambda2 = neg_p_2 - root_part;


    R = min(lambda1, lambda2);
    scores = padarray(R, floor(patch_size ./2 + 1), 0);
    scores = max(scores, 0);
    
end
