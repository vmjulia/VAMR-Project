function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2 x N matrix of image patch vectors based on image
% img (=d) and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".

% Return a d x k matrix, where d is the descriptor dimension 
% (total amount of pixels in patch) and k the amount of keypoints. 
% The ith column of the matrix should contain the patch intensities 
% around the ith keypoint, stored in column-wise order 

N = size(keypoints, 2); % amount of keypoints 200
descriptors = uint8(zeros((2*r+1) ^ 2, N)); % descriptor dimension: total amount of pixels in patch x amount of keypoints [289x200]
padded = padarray(img, [r, r]);
for i = 1:N
    kp = keypoints(:, i) + r;
    descriptors(:,i) = reshape(...
        padded(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r), [], 1); % returns [] x 1 matrix with elements from padded(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r)
end

% The ith column of the matrix should contain the patch intensities around the ith keypoint, stored in column-wise order 

end