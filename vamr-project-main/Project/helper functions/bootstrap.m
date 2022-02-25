function [fRANSAC, inlier_matches_1, inlier_matches_2] = bootstrap(params, img0, img1)

% TODO I2 - compute keypoint matches (keypoint correspondences) between img0 and img1
% I2.1 detect harris features (example with kitti params)
harris_features = detectHarrisFeatures(img0, 'MinQuality', params.MinQuality, ...
'FilterSize', params.FilterSize); % from the first image

% I2.2 select n strongest harris features
harris_features = harris_features.selectStrongest(params.strongest).Location; % harris_features: [Mx2] locations in img0

% I2.3 track keypoints using KLT algorithm (example with kitti params)
% tracker object: klt_tracker
klt_tracker = vision.PointTracker('NumPyramidLevels', params.NumPyramidLevels, ...
   'MaxBidirectionalError', params.MaxBidirectionalError, 'BlockSize', ...
   params.BlockSize, 'MaxIterations', params.MaxIterations);

% I2.4 initializes points to track and sets the initial video frame 
initialize(klt_tracker, harris_features, img0) % initializes the points to track and sets the initial frame

% I2.5 track the points to the next image
[points, validity] = step(klt_tracker, img1);
% points: [Mx2] new locations in img1 (new image)
% validity: [Mx1] logical array indicating whether the point has been
% reliably tracked

% I2.6 extract the matches
matches_1 = harris_features(validity, :); % [Mx2] location of points in img0
matches_2 = points(validity, :); % [Mx2] location of points in img1

% I2.7 Testing: plot the 2D point correspondences
%figure(1);
%plotInitialization(matches_1, matches_2, img0, img1)

% TODO I3 - reconstruct 3D keypoints from the computed matches:
% Estimate the relative pose between the frames and triangulate a point
% cloud of 3D landmarks.
% I3.1 estimate the fundamental matrix
[fRANSAC, inliers] = estimateFundamentalMatrix(matches_1, matches_2, ...
    'Method', params.Method, 'OutputClass', params.OutputClass, ...
    'NumTrials', params.NumTrials, 'DistanceType', params.DistanceType, ...
    'DistanceThreshold', params.DistanceThreshold, 'Confidence', params.Confidence, ...
    'InlierPercentage', params.InlierPercentage); % fRANSAC: [3x3] fundamental matrix; inliers: [Mx1] logical inliers

% fRANSAC: [3x3] fundamental matrix; inliers: [Mx1] logical inliers
fprintf('Inliers: %d vs points tracked: %d\n', sum(inliers), length(points))

% I3.2 eliminate outliers (RANSAC) on the matches
inlier_matches_1 = matches_1(inliers, :);  % [Mx2] location of points in img0
inlier_matches_2 = matches_2(inliers, :);  % [Mx2] location of points in img0

% I3.3 Testing: coompare results of outlier removal
%figure(2);
%plotInitialization(inlier_matches_1, inlier_matches_2, img0, img1);
%subplot(2,1,1);
%showMatchedFeatures(img0, img1, matches_1, matches_2, 'montage', 'PlotOptions', {'ro','go','y--'});
%title('Matches containing outliers')

%subplot(2,1,2);
%showMatchedFeatures(img0, img1, inlier_matches_1, inlier_matches_2, 'montage', 'PlotOptions', {'ro','go','y--'});
%title('Matches with outliers removed')
end