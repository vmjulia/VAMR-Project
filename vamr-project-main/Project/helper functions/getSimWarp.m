function W = getSimWarp(dx, dy, alpha_deg, lambda)
% alpha given in degrees, as indicated
% W(x, y, p) = lambda([cos(alpha) -sin(alpha); [x; + [dx;)
%                      sin(alpha)  cos(alpha)]  y]    dy]

alpha_rad = alpha_deg * pi / 180;
c = cos(alpha_rad);
s = sin(alpha_rad);
R = [c -s; s c];
W = lambda * [R [dx; dy]];

end

