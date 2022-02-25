function sigma = scaled_sigma(s, S, sigma_0)
    sigma = 2^(s/S) * sigma_0;
end
