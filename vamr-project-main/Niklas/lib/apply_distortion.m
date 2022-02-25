function location = apply_distortion(point, D, K)
    u0v0 = K(1:2, 3);
    diff = point - u0v0;
    r2 = diff' * diff;
    location = (1 + D(1) * r2 + D(2) * r2^2) * diff + u0v0;
end