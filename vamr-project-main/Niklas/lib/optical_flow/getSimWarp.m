function W = getSimWarp(dx, dy, alpha_deg, lambda)
% alpha given in degrees, as indicated
    alpha_rad = 2 * pi * alpha_deg / 360;
    W = lambda *   [cos(alpha_rad), -sin(alpha_rad), dx;
                    sin(alpha_rad),  cos(alpha_rad), dy];
end