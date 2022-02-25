function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.
% num = 200
% r = 8

keypoints = zeros(2, num);
temp_scores = padarray(scores, [r r]); % add r elements of padding to the 1st dim.; 
% add r elements of padding to the 2nd dim
for i = 1:num % 1 - 200 keypoints
    [~, kp] = max(temp_scores(:)); % returns the index of the max. from entire array
    [row, col] = ind2sub(size(temp_scores), kp); % returns row and column subscript corresponding to the index matrix kp for a matrix of size temp scores
    kp = [row;col];
    keypoints(:, i) = kp - r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = 0; % points kp_x -+r, points kp_y -+r: set all points around to 0
        %zeros(2*r + 1, 2*r + 1);
end

end
