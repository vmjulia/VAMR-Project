function new_state = triangulateCandidates(params, state, cam_transform)
% Triangulates all candidates that are ready for it
%   This function checks all candidates for whether they are ready
%   to be triangulated based on the bearing angle condition
%
%   It then triangulates all valid ones and moves them from candidates
%   to triangulated points
%
%   Arguments:
%       state - the state containing candidates to be processed
%           note: a state consists of P, X, C, F, Tau 
%                 with sizes 2xK, 3xK, 2xM, 2xM, 12xM
%       cam_transform - the camera transform corresponding to that state

    new_state = struct();
    num_c = size(state.C, 2);

    points3d = zeros(3, num_c);
    candidate_filter = false(num_c, 1);

    K = params.cameraParameters.IntrinsicMatrix';
    M_current = K * cam_transform;
    M_reference = zeros(12, num_c);

    for kp_idx = 1:num_c % iterate over all keypoint indices
        % reshape [1x12] vector to [3x4] camera matrix
        kp = state.C(:, kp_idx);
        reference_point = state.F(:, kp_idx);
        transform_ref = reshape(state.Tau(:, kp_idx), 3, 4); 
        M_ref = K * transform_ref;
        M_reference(:, kp_idx) = M_ref(:);

        if checkRequirement(params, kp, reference_point, cam_transform, transform_ref)
            
            [point_3d, error] = triangulate(kp', reference_point', ...
                                             M_current', M_ref');

            landmark_in_cam = cam_transform * MakeHomogeneous(point_3d');
            in_front = landmark_in_cam(3) > 0;

            if in_front && error < params.cont_maxTriangulationErrorInitial
                points3d(:, kp_idx) = point_3d;
                candidate_filter(kp_idx) = true;
            end 
        end
    end
    
    % if any points passed the angle condition
    num_remaining_candidates = sum(candidate_filter);
    if num_remaining_candidates > 0

        t_candidates = state.C(:, candidate_filter);
        t_references = state.F(:, candidate_filter);
        t_M_ref = M_reference(:, candidate_filter);
        t_points3d = points3d(:, candidate_filter);

        if params.cont_optimizeTriangulation
            fprintf('Optimizing %d points...', num_remaining_candidates)
            t_points3d = optimize3dPoints(t_points3d, t_candidates, t_references, M_current, t_M_ref);
            
            final_error = Inf(num_c, 1);
            final_error(candidate_filter) = reprojectionError(t_points3d, t_candidates, M_current);
            candidate_filter = final_error < params.cont_maxTriangulationErrorFinal;
        end 
    end

        fprintf('Converted %d out of %d candidates to keypoints.\n', sum(candidate_filter), num_c);
        new_state.P = [state.P, state.C(:, candidate_filter)];
        new_state.X = [state.X, points3d(:, candidate_filter)];
    
        candidates_to_keep = ~candidate_filter;
        new_state.C = state.C(:, candidates_to_keep);
        new_state.F = state.F(:, candidates_to_keep);
        new_state.Tau = state.Tau(:, candidates_to_keep);

end

function optimized_points = optimize3dPoints(points3d, obs_current, obs_reference, M_current, M_reference)
    error_terms = @(landmarks) absReprojectionError(landmarks, obs_current, M_current);

    options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt',...
                                        'MaxIter', 30);
    optimized_points = lsqnonlin(error_terms, double(points3d), [], [], options);
end

function errors = multiViewRefprojectionError(points3d, observations, M_matrices)
    % This function computes the all reprojection errors for multiple
    % independent views
    %   Arguments
    %       points3d - [3xN]
    %       observations - [3xN]
    %       M_matrices - [12xN] projection matrix for camera in which the
    %                           point was observed

    n_points = size(points3d, 2);
    points3d_hom = MakeHomogeneous(points3d);
    reprojected = arrayfun(@(i) (reshape(M_matrices(:, i), 3, 4) * points3d_hom(:, i)),...
                                 1:n_points,...
                                 'UniformOutput', false);
    reprojected = cell2mat(reprojected);
    reprojected = HNormalize(reprojected, false);

    %plotReprojectionError(points2d, reprojected);

    d = sum((reprojected - observations) .^ 2, 1);
    errors = double(sqrt(d));
end

function can_be_triangulated = checkRequirement(params, p1, p2, T1, T2)
    K = params.cameraParameters.IntrinsicMatrix';
    R1 = T1(1:3, 1:3);
    R2 = T2(1:3, 1:3);

    % compute the ray directions for each of the cameras (in world space)
    l1 =  R1' * HNormalize(K \ [p1; 1], true);
    l2 =  R2' * HNormalize(K \ [p2; 1], true);
    
    can_be_triangulated = bearingAngle(l1, l2) > params.cont_triangulationBearingAngle;
end

function angle = bearingAngle(l1, l2)
    dot_product = l1' * l2 / (norm(l1) * norm(l2));
    angle = acos(dot_product);
end



