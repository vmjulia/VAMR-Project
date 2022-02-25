function [R_C_W, t_C_W, best_inlier_mask, max_num_inliers_history, num_iteration_history] ...
    = ransacLocalization(matched_query_keypoints, corresponding_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% best_inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.

N_points = size(matched_query_keypoints, 2);
N_landmarks = size(corresponding_landmarks, 2);

assert(N_points == N_landmarks, "Number of keypoints and landmarks must be equal!");
assert(N_points >= 12, "At least 12 points are needed to run DLT!")

num_iterations = 1000;
iteration_count = 1;
use_p3p = true;

max_reprojection_error = 10;

max_num_inliers = 0;
num_iteration_history = zeros(1, num_iterations);
max_num_inliers_history = zeros(1, num_iterations);
best_inlier_mask = zeros(1, num_iterations);
best_R = zeros(3,3);
best_t = zeros(3,1);

p_2D_hom = MakeHomogeneous(matched_query_keypoints);
p_3D_hom = MakeHomogeneous(corresponding_landmarks);

while iteration_count <= num_iterations
    if use_p3p
        sample_indices = datasample(1:N_points, 3, 2, 'Replace', false);
    else
        sample_indices = datasample(1:N_points, 6, 2, 'Replace', false);
    end

    point_sample = matched_query_keypoints(:, sample_indices);
    landmark_sample = corresponding_landmarks(:, sample_indices);

    if use_p3p
        M = p3p(landmark_sample', point_sample');
    else
        M = estimatePoseDLT(point_sample', landmark_sample', K);
    end
    landmarks_projected = HNormalize(K * M * p_3D_hom, false);
    
    error = sqrt(sum((matched_query_keypoints - landmarks_projected) .^ 2, 1));
    inlier_mask = error < max_reprojection_error;
    num_inliers = sum(inlier_mask);
    if num_inliers > max_num_inliers
        max_num_inliers = num_inliers;
        best_R = M(1:3, 1:3);
        best_t = M(1:3, 4);
        best_inlier_mask = inlier_mask;
    end
    
    max_num_inliers_history(:, iteration_count) = max_num_inliers;

    iteration_count = iteration_count + 1;
end

R_C_W = best_R;
t_C_W = best_t;

end