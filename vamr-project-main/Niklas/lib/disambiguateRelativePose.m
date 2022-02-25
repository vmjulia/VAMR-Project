function [R,T] = disambiguateRelativePose(Rots,u3,points0_h,points1_h,K1,K2)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_C1 = T_C2_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

M1 = K1 * eye(3,4);

R_options = {Rots(:, :, 1), Rots(:, :, 2)};
T_options = {u3, -u3};

num_in_front = zeros(4);

for i = 1:4
    [r, t] = ind2sub(i, [2,2]);
    R = R_options{r};
    T = T_options{t};
    M2 = K2 * [R, T];
    P = linearTriangulation(points0_h, points1_h, M1, M2);
    num_in_front(i) = sum(P(3, :) > 0);
end

[~, argmax] = max(num_in_front);
R = R_options{argmax};
T = T_options{argmax};
