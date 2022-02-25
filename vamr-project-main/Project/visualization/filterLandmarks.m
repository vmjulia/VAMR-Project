function l_filtered = filterLandmarks(reference_point, landmarks, std_factor)
%FILTERLANDMARKS Summary of this function goes here
%   Detailed explanation goes here
    reference_dist = landmarks - reference_point;
    reference_dist = sum(reference_dist .^ 2, 1);
    mean_dist = mean(reference_dist, 2);
    l_std = std(reference_dist);
    to_keep = reference_dist < mean_dist + std_factor * l_std;
    l_filtered = landmarks(:, to_keep);
end

