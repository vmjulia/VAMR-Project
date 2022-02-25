function inverted = invertTransform(T)
%INVERTTRANSFORMATION Summary of this function goes here
%   Detailed explanation goes here
R = T(1:3, 1:3);
t = T(1:3, 4);
inverted = [R', -R' * t];
end

