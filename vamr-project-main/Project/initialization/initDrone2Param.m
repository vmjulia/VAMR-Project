% Set initialization parameters for the Drone 2 dataset
%----------------------------------------------------
ds = 4;   

rng(137);
% detectHarrisFeatures() for initialization default values
params.MinQuality = 1e-6; % a scalar value in the range [0,1]
params.FilterSize = 5; % an odd integer value in the range [3, min(size(I))].
params.strongest = 1000; % selectStrongest Features
    
% vision.PointTracker() (for KLT) default values
params.NumPyramidLevels = 5; 
params.MaxBidirectionalError = inf; % Recommended values are between 0 and 3 pixels. Forward-backward error threshold.
params.BlockSize = [31 31]; % size of the neighborhood           
params.MaxIterations = 30;
    
% estimateFundamentalMatrix()
params.Method = 'RANSAC';
params.OutputClass = 'double';
params.NumTrials = 2000;
params.DistanceType = 'Sampson'; % for faster computation 'Algebraic'
params.DistanceThreshold = 0.1;
params.Confidence = 99;
params.InlierPercentage = 50;

% Set continuous operations parameters for the Drone 2 dataset
%-----------------------------------------------------------
% detectHarrisFeatures() for finding new candidate keypoints
params.cont_MinQuality = 1e-6; % a scalar value in the range [0,1]
params.cont_FilterSize = 13; % an odd integer value in the range [3, min(size(I))].
params.cont_strongest = 1000; % selectStrongest Features

% vision.PointTracker() (for KLT)
params.cont_NumPyramidLevels = 5; 
params.cont_MaxBidirectionalError = inf; % Recommended values are between 0 and 3 pixels. Forward-backward error threshold.
params.cont_BlockSize = [31 31]; % size of the neighborhood           
params.cont_MaxIterations = 30;
%kp and ckp comparison (for KLT)
params.cont_exact = 0;
params.cont_delta = 10.;

% estimateFundamentalMatrix()
params.cont_Method = 'RANSAC';
params.cont_OutputClass = 'double';
params.cont_NumTrials = 2000;
params.cont_DistanceType = 'Sampson'; % for faster computation 'Algebraic'
params.cont_DistanceThreshold = 1.;
params.cont_Confidence = 99;
params.cont_InlierPercentage = 50;

% triangulation
params.cont_maxTriangulationErrorInitial = 1.; % reprojection error a point can have before it is discarded
params.cont_maxTriangulationErrorFinal = .8; % reprojection error a point can have before it is discarded
params.cont_triangulationBearingAngle = deg2rad(1);
params.cont_optimizeTriangulation = false;

% P3P pose estimation
%params.p3pConfidence = 70;
%params.p3pMaxReprojectionError = 1.5;
    