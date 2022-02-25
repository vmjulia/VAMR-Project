function [state_new, cam_transform, prev_or, prev_loc] = processFrame(params, img_new, img_prev, state_prev)
%PROCESSFRAME Processes a new frame during continuous improvement
%   inputs:
%       img_new
%       img_prev
%       state_prev
%
%   outputs:
%       state_new
%       cam_transform
    % extract instrinsic matrix from parameters
    state_new = state_prev;
    

    klt_pointsTracker = vision.PointTracker('NumPyramidLevels', params.cont_NumPyramidLevels, ...
                                'MaxBidirectionalError', params.cont_MaxBidirectionalError, ...
                                'BlockSize', params.cont_BlockSize, ...
                                'MaxIterations', params.cont_MaxIterations);  


    klt_candidateKeypointsTracker = vision.PointTracker('NumPyramidLevels', params.cont_NumPyramidLevels, ...
                               'MaxBidirectionalError', params.cont_MaxBidirectionalError, ...
                                'BlockSize', params.cont_BlockSize, ...
                                'MaxIterations', params.cont_MaxIterations);      

    % Associate keypoints in the current frame 
    % to previously triangulated landmarks.

    initialize(klt_pointsTracker, transpose(state_prev.P), img_prev);   

    [kp, kp_validity] = step(klt_pointsTracker, img_new);
    fprintf('kpT tracked: %d out of %d kp.\n',sum(kp_validity),length(kp_validity));
    
    % input to p3p
    points = kp(kp_validity,:); % [Nx2]
    landmarks = state_prev.X(:,transpose(kp_validity)); % [3xN]
    
    
    %% TODO CO2 - Based on this, estimate the current camera pose.
    
    % Alternative estimation of the camera pose with estimateWorldCameraPose:
     [cameraRotation, cameraTranslation, inlierIdx] = estimateWorldCameraPose(double(points), double(landmarks)',...
                                                        params.cameraParameters,...
                                                        'Confidence', params.p3pConfidence,...
                                                        'MaxReprojectionError', params.p3pMaxReprojectionError);
     [R, t] = cameraPoseToExtrinsics(cameraRotation, cameraTranslation);
     cam_transform = [R', t'];
     prev_or = cameraRotation;
     prev_loc =  cameraTranslation;
    
     
     K = params.cameraParameters.IntrinsicMatrix';
     if params.p3pOptimizePoseEstimation
         est_pose = [R', t'];
         pose_error = @(est_pose) reprojectionError(landmarks, points', K*est_pose);
         options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt','MaxIter', 30);
         est_pose = lsqnonlin(pose_error, double(est_pose), [], [], options);
         R = est_pose(:,1:3);
         t = est_pose(:,4);
         cam_transform = [R, t];
     end

     fprintf('reprojection error after p3p: %d\n', mean(reprojectionError(landmarks, points', K * cam_transform), 2));
     
     fprintf('Number of inliers is %d\n', sum(inlierIdx))
     points = points(inlierIdx, :); % [Nx2]
     landmarks = landmarks(:, inlierIdx); % [3xN]
    
    
    %% TODO CO3 - Regularly triangulate new landmarks using keypoints 
    % not associated to previously triangulated landmarks.
    % check if c can be moved to x in the new state. If the angle exceeds threshold, 
    % you can remove (c,f(c),τ(c)) from Ci,Fi,T i and append (c,x(c)) to Pi,Xi
    % ........, so update p, x, c, f, t in the new state
    
    % new points
    
    [~, ~, newCPoints] = bootstrap_continuous(params, img_prev, img_new);
     if (size(state_new.C, 1) == 0)
         ckp_validity = ones(size(newCPoints,1), 1);
         ckp = newCPoints;
         F = newCPoints;
         Tau = repmat(cam_transform(:)', [size(F,1),1]);
         initialize(klt_candidateKeypointsTracker, ckp, img_prev);  
        
     else 
          % Associate candidate keypoints in the current frame 
          % to previously triangulated landmarks.
          % add newly bootstrapped points
        
        initialize(klt_candidateKeypointsTracker, transpose(state_prev.C), img_prev);  
        [ckp,ckp_validity] = step(klt_candidateKeypointsTracker,img_new);
        [ckp,ckp_validity, F, Tau] = combineCkp(ckp,ckp_validity, newCPoints, state_prev, cam_transform, params.cont_exact, params.cont_delta);
        
     end 
     
    if (params.cont_exact == 1) 
    ckp_validity = adjustValidity(ckp_validity, kp, ckp);
    else
        ckp_validity = adjustValidity2(ckp_validity, kp, ckp, params.cont_delta);
    end
        
    assert(length(ckp_validity) == length(F));
    assert(length(ckp_validity) == length(ckp));
    assert(length(ckp_validity) == length(Tau));
    
    fprintf('CkpT tracked: %d out of %d Ckp.\n',sum(ckp_validity),length(ckp_validity));
    
    %update state
    state_new.P = transpose(points);
    state_new.X = landmarks;
    state_new.C = transpose(ckp(ckp_validity,:)); 
    state_new.F = transpose(F(ckp_validity,:));
    state_new.Tau = transpose(Tau(ckp_validity,:));

    % triangulate all canditates fulfilling the angle requirement
    state_new = triangulateCandidates(params, state_new,cam_transform);

end
