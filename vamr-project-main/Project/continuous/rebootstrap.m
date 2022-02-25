function [state_new, cam_transform, flag] = rebootstrap(params, img_new, img_prev,  prev_loc, prev_orient)
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
    
    % TODO I2 - compute keypoint matches (keypoint correspondences) between img0 and img1
    % TODO I3 - reconstruct 3D keypoints from the computed matches
  
  initialization_error = Inf(1);
  state_new = struct();
  cam_transform = 0;
  i = 0;
  while((initialization_error > .5)&&(i<3))
    i = i+1;
    % TODO I2 - compute keypoint matches (keypoint correspondences) between img0 and img1
    % TODO I3 - reconstruct 3D keypoints from the computed matches
    [fRANSAC, inlier_matches_1, inlier_matches_2] = bootstrap(params, img_prev, img_new);

    
    [relativeOrientation, relativeLocation, validPointsFraction] = relativeCameraPose(fRANSAC, params.cameraParameters, inlier_matches_1, inlier_matches_2);
    % validPointsFraction returns the fraction of the inlier points that project in front of both
    % cameras. If this fraction is too small (e. g. less than 0.9), that can indicate that 
    % the input matrix fRANSAC is unreliable.
    % The function 'relativeCameraPose' returns the orientation [3x3xN] and location [Nx3] (unit vector) of camera 2 relative to
    % camera 1.
    if size(relativeLocation, 1) > 1
        % something went wrong in relativeCameraPose
        continue
    end
   
    %% segment for testing triangulation stuff
    % I3.4 recover extrinsics (rotation and translation) for the
    % triangulate3dPoints function
   
    
    loc = prev_loc + relativeLocation*prev_orient;
    orient = relativeOrientation*prev_orient;
  
    [rotationMatrix, translationVector] = cameraPoseToExtrinsics(orient, loc);
    K = params.cameraParameters.IntrinsicMatrix';
    
    [R1, t1] = cameraPoseToExtrinsics(prev_orient, prev_loc);

    % we set camera 1 as our world coordinate center
    T1 = [R1', t1']
    T2 = [rotationMatrix',  translationVector'];

    [points3d, errors] = triangulate3dPoints(params, inlier_matches_1', inlier_matches_2', T1, T2);
    initialization_error = mean(errors, "all");
  end
  
  
% if finished because number of attempts finished  
if i == 3
        flag = 0;
    
else
        cam_transform = T2;  
   
        state_new.P = inlier_matches_2';
        state_new.X = points3d;
        % for the second frame, candidate keypoints are its matched observations from frame 1
        state_new.C =  zeros(0);
        state_new.F =  zeros(0);
        % copy and repeat camera 1 transformation M times
        state_new.Tau = zeros(0);
        flag = 1
end

end

