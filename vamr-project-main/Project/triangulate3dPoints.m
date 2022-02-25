function [points3d, errors] = triangulate3dPoints(params, points1, points2, P1, P2)
%triangulate 3D points from 2d correspondences
%   Arguments:
%       points1 - [2xN] 2d points observed by camera 1
%       points2 - [2xN] 2d points observed by camera 2
%       T1      - [3x4] transformation matrix for camera 1
%       T2      - [3x4] transformation matrix for camera 2
%  Returns:
%      points3d - [3xN] triangulated 3d points
%      errors    - [2xN] reprojection errors of each point 
%                       (into first and second camera)

    M1 = params.cameraParameters.IntrinsicMatrix' * P1;
    M2 = params.cameraParameters.IntrinsicMatrix' * P2;

    % I3.5 triangulate homogeneous 3D points from 2D correspondences and extrinsics
    points3d = triangulate(points1', points2', M1', M2')';

    errors_c1 = reprojectionError(points3d, points1, M1);
    errors_c2 = reprojectionError(points3d, points2, M2);

    fprintf('mean reprojection error (cam 1): %f\n', mean(errors_c1, 2));
    fprintf('mean reprojection error (cam 2): %f\n', mean(errors_c2, 2));

    if params.init_optimizeTriangulation
        error_terms = @(landmarks) absReprojectionError(landmarks, points1, M1)...
                           + absReprojectionError(landmarks, points2, M2);
        options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt',...
                                        'MaxIter', 30);
        points3d = lsqnonlin(error_terms, double(points3d), [], [], options);
    end

    errors_c1 = reprojectionError(points3d, points1, M1);
    errors_c2 = reprojectionError(points3d, points2, M2);

    fprintf('error after optimization (cam 1): %f\n', mean(errors_c1, 2));
    fprintf('error after optimization(cam 2): %f\n', mean(errors_c2, 2));

    errors = vertcat(errors_c1, errors_c2);
end

