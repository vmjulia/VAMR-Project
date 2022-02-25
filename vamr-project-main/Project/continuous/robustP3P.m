function [cam_transform, best_inliers] = robustP3P(K, points2d, landmarks)
%ROBUSTP3P 
% Estimates the pose of a camera given the given the intrinsic camera
% matrix and 2D-3D point correspondences
% Input:    intrinsic matrix K [3x3] 
%           points2d [Nx2]: 2D points on image
%           landmarks [Nx3]: 3D world points corresponding to points2d
% Output:   cam_transform [3x4]: pose of camera in homogeneous coordinates, [R, t]
%           best_inliers [Nx1]: binary array indicating indices which are inliers

    num_iterations = 200;
    pixel_tolerance = 10;
    k = 3; % number of samples for P3P
    
    max_num_inliers = 0;
    % max_num_inliers_history = [];
    best_inliers = zeros(size(landmarks,1), 1); % [Nx1]
    points2d_hom = MakeHomogeneous(points2d')'; % [Nx3] with the last column entry being 1

    for i = 1:num_iterations
        %build model from k samples with p3p
        
        [landmark_sample, idx_sample] = datasample(landmarks, k, 1, 'Replace', false); 
        % gets samples along dimension 1 of landmarks, e.g. samples k rows
        % from landmarks. dim of landmark_sample is [kx3]
        keypoint_sample = points2d_hom(idx_sample,:); % [kx3]
        
        % Backproject keypoints to unit bearing vectors. % need camera matrix K!
        normalized_bearings = K\(keypoint_sample');
        for ii = 1:3
            normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
                norm(normalized_bearings(:, ii), 2);
        end
        
        poses = p3p(landmark_sample', normalized_bearings); % [3x16]
        
        % Decode p3p output
        R_C_W_guess = zeros(3, 3, 4);
        t_C_W_guess = zeros(3, 1, 4);
        for ii = 0:3
            R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
            t_W_C_ii = real(poses(:, (1+ii*4)));
            R_C_W_guess(:,:,ii+1) = R_W_C_ii'; % [3x3x4]
            t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii; % [3x1x4]
        end
        
        % Count inliers:
        projected_points = projectPoints((R_C_W_guess(:,:,1) * landmarks') + ...
            t_C_W_guess(:,:,1), K)'; % [Nx2]
        difference = points2d - projected_points; % [Nx2]
        errors = sum(difference.^2, 2); % [Nx1]
        is_inlier = errors < pixel_tolerance^2; % [Nx1]
        R_C_W = R_C_W_guess(:,:,1); 
        t_C_W = t_C_W_guess(:,:,1);
        
        % If we use p3p, also consider inliers for the alternative solutions.
        for alt_idx=1:3
            projected_points = projectPoints((R_C_W_guess(:,:,1+alt_idx) * landmarks') + ...
                t_C_W_guess(:,:,1+alt_idx), K)'; % [Nx2]
            difference = points2d - projected_points;
            errors = sum(difference.^2, 2);
            alternative_is_inlier = errors < pixel_tolerance^2;
            if nnz(alternative_is_inlier) > nnz(is_inlier)
                is_inlier = alternative_is_inlier;
                R_C_W = R_C_W_guess(:,:,alt_idx);
                t_C_W = t_C_W_guess(:,:,alt_idx); 
            end
        end
        
        min_inlier_count = 6; % taken from ex7
        if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= min_inlier_count
            max_num_inliers = nnz(is_inlier);  
            best_inliers = is_inlier;
        end
        % max_num_inliers_history(i) = max_num_inliers;
    end
            
    assert(max_num_inliers > 0, 'There was an error in pose estimation');

    R_W_C = R_C_W';
    t_W_C = -R_C_W'*t_C_W;
    cam_transform = [R_W_C, t_W_C];
end

