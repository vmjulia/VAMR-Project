% Set initialization parameters for the parking dataset
%------------------------------------------------------
ds = 1;

rng(120)

% detectHarrisFeatures() for initialization default values
params.MinQuality = 1e-6; % a scalar value in the range [0,1]
params.FilterSize = 5; % an odd integer value in the range [3, min(size(I))].
params.strongest = 800; % selectStrongest Features
    
% vision.PointTracker() (for KLT) default values
params.NumPyramidLevels = 4; 
params.MaxBidirectionalError = .3; % Recommended values are between 0 and 3 pixels. Forward-backward error threshold.
params.BlockSize = [25 25]; % size of the neighborhood           
params.MaxIterations = 50;
    
% estimateFundamentalMatrix()
params.Method = 'RANSAC';
params.OutputClass = 'double';
params.NumTrials = 500;
params.DistanceType = 'Sampson'; % for faster computation 'Algebraic'
params.DistanceThreshold = 2;
params.Confidence = 99;
params.InlierPercentage = 70;  

% Set continuous operations parameters for the parking dataset
%-------------------------------------------------------------
% detectHarrisFeatures() for finding new candidate keypoints
params.cont_MinQuality = 1e-6; % a scalar value in the range [0,1]
params.cont_FilterSize = 5; % an odd integer value in the range [3, min(size(I))].
params.cont_strongest = 800; % selectStrongest Features
%kp and ckp comparison (for KLT)
params.cont_exact = 0;
params.cont_delta =10;


% estimateFundamentalMatrix()
params.cont_Method = 'RANSAC';
params.cont_OutputClass = 'double';
params.cont_NumTrials = 500;
params.cont_DistanceType = 'Sampson'; % for faster computation 'Algebraic'
params.cont_DistanceThreshold = 0.01;
params.cont_Confidence = 99;
params.cont_InlierPercentage = 50;

% P3P pose estimation
params.p3pConfidence = 99;
params.p3pMaxReprojectionError = .6;
params.p3pOptimizePoseEstimation = false;

% triangulation
params.cont_maxTriangulationErrorInitial = 1.; % reprojection error a point can have before it is discarded
params.cont_maxTriangulationErrorFinal = .5; % reprojection error a point can have before it is discarded
params.cont_triangulationBearingAngle = deg2rad(0.5);
params.cont_optimizeTriangulation = false;

params.init_optimizeTriangulation = false;

% for re-initialization
params.reboot = true;
params.NmbOfPoints = 100;
