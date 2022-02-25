function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.
    keypoints = zeros([2, num]);

    padded_scores = padarray(scores, [r,r]);
    score_size = size(padded_scores);

    for i = 1:num
        [~, max_loc] = max(padded_scores(:));
        [v,u] = ind2sub(score_size, max_loc);
        kp = [v;u];
        kp = kp - r;
        keypoints(:, i) = kp;
        padded_scores(kp(1):kp(1) + 2*r, kp(2):kp(2) + 2*r) = zeros(2*r+1, 2*r+1);
    end
end

