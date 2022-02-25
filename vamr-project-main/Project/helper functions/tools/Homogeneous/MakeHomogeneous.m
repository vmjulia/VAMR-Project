function pts_hom = MakeHomogeneous(pts)
% Converts each point to homogeneous coordinates by appending a 1.
% Argument:
%   pts -  KxN array of K-D non-homogeneous coordinates
% Returns:
%   pts_hom - (K+1)xN array of K-D homogeneous coordinates
N_pts = size(pts, 2);
pts_hom = [pts; ones(1, N_pts)];
end

