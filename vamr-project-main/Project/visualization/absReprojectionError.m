function error = absReprojectionError(points3d, points2d, cameraMatrix)
%REPROJECTIONERROR compute mean squared euclidean error between 
%                   2d points and projected 3d points
%   points3d - [3xn] coordinates of the 3d points in the world frame
%   points2d - [2xN] corresponding 2d points from the camera
%   cameraMatrix - [3x4] full projection matrix of the camera (K*P)
%   Returns:
%       error - [1xN] array of (non-squared) reprojection errors
[dim3, n_points3d] = size(points3d);
[dim2, n_points2d] = size(points2d);

assert(n_points2d == n_points3d, "3D and 2D points should be exact correspndences! (sizes mismatch)");
assert(dim3 == 3, "Argument 1 should be non-homogeneous 3d points.");
assert(dim2 == 2, "Argument 2 should be non-homogeneous 2d points.");


reprojected = (cameraMatrix * MakeHomogeneous(points3d));
reprojected = HNormalize(reprojected, false);

%plotReprojectionError(points2d, reprojected);

d = sum((reprojected - points2d), 1);
error = double(d);

end

