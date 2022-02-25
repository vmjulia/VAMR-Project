clear all
close all
clc

num_scales = 3; % Scales per octave.
num_octaves = 5; % Number of octaves.
sigma = 1.6;
contrast_threshold = 0.04;
image_file_1 = 'images/img_1.jpg';
image_file_2 = 'images/img_2.jpg';
rescale_factor = 0.2; % Rescaling of the original image for speed.

images = {getImage(image_file_1, rescale_factor),...
    getImage(image_file_2, rescale_factor)};

kpt_locations = cell(1, 2);
descriptors = cell(1, 2);

for img_idx = 1:2
    image = images{img_idx};
    imshow(image);
    waitforbuttonpress();

    DoG_octaves = cell(1, num_octaves);
    DoG_octaves{1} = compute_DoG(image, num_scales, sigma);
    for o = 2:num_octaves
        image = image(1:2:end,1:2:end,:); % downsample by 2
        DoG_octaves{o} = compute_DoG(image, num_scales, sigma);
    end

    kp_locations = extractKeypoints(DoG_octaves, contrast_threshold);
    descriptors = computeDescriptors(DoG_octaves, kp_locations, num_scales);

    % 5)    Given the blurred images and keypoints, compute the
    %       descriptors. Discard keypoints/descriptors that are too close
    %       to the boundary of the image. Hence, you will most likely
    %       lose some keypoints that you have computed earlier.
end

% Finally, match the descriptors using the function 'matchFeatures' and
% visualize the matches with the function 'showMatchedFeatures'.
% If you want, you can also implement the matching procedure yourself using
% 'knnsearch'.