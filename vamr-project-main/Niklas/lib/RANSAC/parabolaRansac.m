function [best_guess_history, max_num_inliers_history] = ...
    parabolaRansac(data, max_noise)
% data is 2xN with the data points given column-wise, 
% best_guess_history is 3xnum_iterations with the polynome coefficients 
%   from polyfit of the BEST GUESS SO FAR at each iteration columnwise and
% max_num_inliers_history is 1xnum_iterations, with the inlier count of the
%   BEST GUESS SO FAR at each iteration.
num_iterations = 100;
iteration_count = 1;

max_num_inliers = 0;
max_num_inliers_history = zeros(1, num_iterations);
best_guess_history = zeros(3, num_iterations);

while iteration_count <= num_iterations
    samples = datasample(data, 3, 2, 'Replace', false);

    polynomial = polyfit(samples(1,:), samples(2,:), 2);
    x_test = data(1, :);
    y_true = data(2, :);
    y_projected = polyval(polynomial, x_test);
    
    error = abs(y_true - y_projected);
    inliers = data(:, error < max_noise);
    num_inliers = size(inliers, 2);
    if num_inliers > max_num_inliers
        max_num_inliers = num_inliers;
        full_inlier_fit = polyfit(inliers(1,:), inliers(2,:), 2);
        best_guess_history(:, iteration_count) = full_inlier_fit;
    else
        best_guess_history(:, iteration_count) = best_guess_history(:, iteration_count-1);
    end

    max_num_inliers_history(:, iteration_count) = max_num_inliers;

    iteration_count = iteration_count + 1;
end