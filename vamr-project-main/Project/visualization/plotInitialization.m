function plotInitialization(matches_1, matches_2, img0, img1)
% Initialization plot of matched features

subplot(2,2,1);
imshow(img0);
hold on
plot(matches_1(:, 1)', matches_1(:, 2)', 'bx');

subplot(2,2,2);
imshow(img1);
hold on
plot(matches_2(:, 1)', matches_2(:, 2)', 'rx');

subplot(2,2,[3,4]);
imshow(img1);
hold on;
plot(matches_2(:, 1)', matches_2(:, 2)', 'rx');
hold on;
plot(matches_1(:, 1)', matches_1(:, 2)', 'bx');
hold on;
plot([matches_1(:, 1)'; matches_2(:, 1)'], ...
     [matches_1(:, 2)'; matches_2(:, 2)'], 'g-', 'Linewidth', 1);
end