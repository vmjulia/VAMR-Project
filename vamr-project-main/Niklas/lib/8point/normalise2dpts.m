function [pts_tilda, T] = normalise2dpts(pts)
% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points
% so that their centroid is at the origin and their mean distance from
% the origin is sqrt(2).
%
% Usage:   [pts_tilda, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   pts_tilda -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, pts_tilda = T*pts
%

N = size(pts, 2);

pts_2D = HNormalize(pts, false);

mu = mean(pts_2D, 2);
D = pts_2D - repmat(mu, 1, N);

sigma = sqrt(mean(sum(D.^2, 1)));

s = sqrt(2) / sigma;

T = [s, 0, -s * mu(1);
     0, s, -s * mu(2);
     0, 0, 1];

pts_tilda = T * pts;

end