function [indices, mask] = getWarpedCoordinates(W, targets, y_max, x_max)
    n_targets = size(targets, 2);
    I_uv =  floor(W * [targets; ones(1, n_targets)]);
    us = I_uv(1,:);
    vs = I_uv(2,:);
    valid_mask = (us > 1) .* (us < x_max) .* (vs > 1) .* (vs < y_max);
    mask = valid_mask == 1;

    indices = sub2ind([y_max,x_max], vs(mask), us(mask));
end