%% Setup
clear all; 
close all;

addpath('helper functions/plot')
addpath('helper functions/triangulation')
addpath('helper functions')
addpath('helper functions/tools')
addpath('helper functions/tools/Homogeneous')
addpath('visualization')
addpath('initialization')
addpath('continuous')

run('setDefaultParams.m')
% optionally change some parameters using a script
run('Kitti3.m')
%run('malaga2.m')

kitti_path = 'datasets/kitti';
malaga_path = 'datasets/malaga-urban-dataset-extract-07';
parking_path = 'datasets/parking';
drone_1_path = 'datasets/drone_1/flight/video_2'; % video_2 from (1:30 - 4:27)
% the full drone_1 videos: https://uzh-my.sharepoint.com/:f:/g/personal/adriana_mohap_zzm_uzh_ch/EpZGyQRav95Au9ghXE6p7KEBLc4ayGlicyA_0bPa0ve-aw?e=w6BxG4
drone_2_path = 'datasets/drone_2/flight/video_3'; % video_3 (0:07 - 2:06)
% the full drone_2 videos: https://uzh-my.sharepoint.com/:f:/g/personal/adriana_mohap_zzm_uzh_ch/EpFmVYtOwh5Kg7rijs1DhhkB_1rDYRN9dkwh6OqF9Vp90w?e=Vebws1

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    %run('initKittiParam.m') not tuned yet
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    %run('initMalagaParam.m') not tuned yet
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    % run('initParkingParam.m')
elseif ds == 3
    % path containing images from the drone_1 dataset
    assert(exist('drone_1_path', 'var') ~= 0);
    last_frame = 5876;
    K = [930.4879  0 484.7689
        0 931.3611 338.3585
        0 0 1];
    %run('initDrone1Param.m') not tuned yet
elseif ds == 4
    % path containing images from the drone_2 dataset
    assert(exist('drone_2_path', 'var') ~= 0);
    last_frame = 3597;
    K = [1851.3 0 1318.7
        0 1852.6 946.0
        0 0 1];
    %run('initDrone2Param.m') not tuned yet
else
    assert(false);
end

% camera parameters intrinsics
params.cameraParameters = cameraParameters('IntrinsicMatrix', K');

%% Bootstrap
if ds == 0
    bootstrap_frames = [1,3]; % as written in the project statement
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    
elseif ds == 1
    bootstrap_frames = [1,3]; % needs to be adapted based on the dataset (should have large baseline)
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    
elseif ds == 2
    bootstrap_frames = [1,3]; % needs to be adapted based on the dataset (should have large baseline)
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    
elseif ds == 3
    bootstrap_frames = [1,10]; % needs to be adapted based on the dataset (should have large baseline)
    img0 = rgb2gray(imread([drone_1_path ...
        sprintf('/images/%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([drone_1_path ...
        sprintf('/images/%05d.png',bootstrap_frames(2))]));
    
elseif ds == 4
    bootstrap_frames = [1,10]; % needs to be adapted based on the dataset (should have large baseline)
    img0 = rgb2gray(imread([drone_2_path ...
        sprintf('/images/%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([drone_2_path ...
        sprintf('/images/%05d.png',bootstrap_frames(2))]));
    
else
    assert(false);
end

initialization_error = Inf(1);
while(initialization_error > .5)
    % TODO I2 - compute keypoint matches (keypoint correspondences) between img0 and img1
    % TODO I3 - reconstruct 3D keypoints from the computed matches
    [fRANSAC, inlier_matches_1, inlier_matches_2] = bootstrap(params, img0, img1);

    
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
     
    [rotationMatrix, translationVector] = cameraPoseToExtrinsics(relativeOrientation, relativeLocation);
    K = params.cameraParameters.IntrinsicMatrix';

    % we set camera 1 as our world coordinate center
    T1 = eye(3,4);
    T2 = [rotationMatrix', translationVector'];
    
    [points3d, errors] = triangulate3dPoints(params, inlier_matches_1', inlier_matches_2', T1, T2);
    initialization_error = mean(errors, "all");
end

% initialize cell arrays for states and camera transforms
% state... a collection of P, X, C, F, Tau 
% type(state)     | struct of 2xK, 3xK, 2xM, 2xM, 12xM
%       with K... number of keypoints
%            M... number of tracks
%
% transform... the 3x4 homogeneous transform from cam to world coordinates
% type(transform) | 3x4 matrix
states = cell(5000, 1);
cam_transforms = cell(5000, 1);
centers = zeros(3, 5000);
frame_indices = cell(5000, 1);

% fill in the transformation matrices computed with relativeCameraPose
cam_transforms{1} = T1;
cam_transforms{2} = T2;

% fill in the first two states and transforms from T_1, T_2, matches, 3D_keypoints

states{1} = struct();
states{1}.P = inlier_matches_1';
states{1}.X = points3d;
% for the first frame there are tracks from previous frames, so M = 0
% (i.e. there are no candidate keypoints, so all matrices are empty)
states{1}.C = zeros(0);
states{1}.F = zeros(0);
states{1}.Tau = zeros(0);
centers(:, 1) = getCameraCenter(cam_transforms{1});
frame_indices{1} = bootstrap_frames(1);


states{2} = struct();
states{2}.P = inlier_matches_2';
states{2}.X = points3d;
% for the second frame, candidate keypoints are its matched observations from frame 1
states{2}.C =  zeros(0);
states{2}.F =  zeros(0);
% copy and repeat camera 1 transformation M times
states{2}.Tau = zeros(0);
centers(:, 2) = getCameraCenter(cam_transforms{2});
frame_indices{2} = bootstrap_frames(2);

prev_img = img1;
stateNumber = 2;

% for plotting the number of landmarks
num_tracked_landmark = zeros(1,20);
trajectory = zeros(3, last_frame);

%num_tracked_landmark = plotAll(img1, bootstrap_frames(2), stateNumber, states, cam_transforms, centers, num_tracked_landmark);
saveState(stateNumber, frame_indices, states, cam_transforms);

%% Continuous operation
 
range = (bootstrap_frames(2)+1):last_frame;

for frame_idx = range
    stateNumber = stateNumber + 1;
    
    % process frame
    
    % if too little of key points
    if params.reboot && length(states{stateNumber-1}.P)<params.NmbOfPoints
       flag = 0;
       i = 1;
       idx = frame_idx;
       while i<3
            idx =  idx+i;
            i = i+1;
            fprintf('\n\nProcessing frame %d\n=====================\n', idx);
            img = loadImage(ds, idx);
            [state, cam, flag ] = rebootstrap(params, img, prev_img, prev_loc, prev_or);
            if flag == 1
                frame_idx = idx;
                states{stateNumber} = state;
                cam_transforms{stateNumber} = cam;
                break
            end
       end   
       if flag == 0
         img = loadImage(ds, frame_idx);
         [states{stateNumber}, cam_transforms{stateNumber}, prev_or, prev_loc] = processFrame(params, img, prev_img, states{stateNumber-1});  
       end
    else
        fprintf('\n\nProcessing frame %d\n=====================\n', frame_idx);
        img = loadImage(ds, frame_idx);
       [states{stateNumber}, cam_transforms{stateNumber}, prev_or, prev_loc] = processFrame(params, img, prev_img, states{stateNumber-1});
    end
    frame_indices{stateNumber} = frame_idx;
    saveState(stateNumber, frame_indices, states, cam_transforms);

    prev_img = img;
end
