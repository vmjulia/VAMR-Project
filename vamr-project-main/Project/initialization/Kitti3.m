% Set initialization parameters for the Kitti dataset
%----------------------------------------------------
ds = 0;    

% detectHarrisFeatures() for initialization default values
params.MinQuality = 1e-6; % a scalar value in the range [0,1]
params.FilterSize = 5; % an odd integer value in the range [3, min(size(I))].
params.strongest = 800; % selectStrongest Features
    
% vision.PointTracker() (for KLT) default values
params.NumPyramidLevels = 3; 
params.MaxBidirectionalError = 0.3; % Recommended values are between 0 and 3 pixels. Forward-backward error threshold.
params.BlockSize = [31 31]; % size of the neighborhood           
params.MaxIterations = 2000;
    
% estimateFundamentalMatrix()
params.Method = 'RANSAC';
params.OutputClass = 'double';
params.NumTrials = 500;
params.DistanceType = 'Sampson'; % for faster computation 'Algebraic'
params.DistanceThreshold = 0.2;
params.Confidence = 99;
params.InlierPercentage = 70;

% Set continuous operations parameters for the Kitti dataset
%-----------------------------------------------------------
% detectHarrisFeatures() for finding new candidate keypoints
params.cont_MinQuality = 1e-7; % a scalar value in the range [0,1]
params.cont_FilterSize = 13; % an odd integer value in the range [3, min(size(I))].
params.cont_strongest = 800; % selectStrongest Features

% vision.PointTracker() (for KLT)
params.cont_NumPyramidLevels = 5; 
params.cont_MaxBidirectionalError = 0.3; % Recommended values are between 0 and 3 pixels. Forward-backward error threshold.
params.cont_BlockSize = [31 31]; % size of the neighborhood           
params.cont_MaxIterations = 2000;
%kp and ckp comparison (for KLT)
params.cont_exact = 0;
params.cont_delta =3;

% estimateFundamentalMatrix()
params.cont_Method = 'RANSAC';
params.cont_OutputClass = 'double';
params.cont_NumTrials = 500;
params.cont_DistanceType = 'Sampson'; % for faster computation 'Algebraic'
params.cont_DistanceThreshold = 0.2;
params.cont_Confidence = 99;
params.cont_InlierPercentage = 70;

% triangulation
params.cont_maxTriangulationErrorInitial = 3.; % reprojection error a point can have before it is discarded
params.cont_maxTriangulationErrorFinal = .8; % reprojection error a point can have before it is discarded
params.cont_triangulationBearingAngle = deg2rad(2);
params.cont_optimizeTriangulation = false;
    