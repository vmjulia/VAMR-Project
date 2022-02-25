function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".
    num_keypoints = size(keypoints, 2);
    descriptors = zeros([((2 * r) + 1)^2, num_keypoints]);

    padded_image = padarray(img, [r,r], 0, 'both');

    for i = 1:num_keypoints
        uv = keypoints(:, i);
        patch = padded_image(uv(1) : uv(1) + 2*r, uv(2) : uv(2) + 2*r);
        descriptors(:, i) = patch(:);
    end
end
