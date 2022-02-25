
num_states = 5000;
centers = zeros(3, num_states);
num_tracked_landmark = zeros(1,20);

states = load("states1.mat").states;
cam_transforms = load("transforms1.mat").transforms;
frames = load("frames1.mat").frames;

ds = 0;

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

for stateNumber = 1:num_states
    frame_idx = frames{stateNumber};
    img = loadImage(ds, frame_idx);

    centers(:, stateNumber) = getCameraCenter(cam_transforms{stateNumber});
    num_tracked_landmark = plotAll(img, frame_idx, stateNumber, states, cam_transforms, centers, num_tracked_landmark);

    % Makes sure that plots refresh.
    pause(0.01);

    prev_img = img;
end
