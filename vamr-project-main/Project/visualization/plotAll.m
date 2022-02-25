function num_tracked_landmark = plotAll(img, frame_idx, state_number, states_history, cam_transform_history, camera_centers, num_tracked_landmark)
% plots the 4 different frames
% 1. Current Image: candidate keypoints and keypoints
% 2. # tracked landmarks over last 20 frames
% 3. full trajectory
% 4. Trajectory of last 20 frames and landmarks

current_states = states_history{state_number};

% 1. Current image
set(gcf,'WindowState','fullscreen')
figure(1);

subplot(2,4,[1,2])
imshow(img), hold on
if ~isempty(current_states.C)
    plot(current_states.C(1,:)', current_states.C(2,:)', 'rx'), hold on
end
plot(current_states.P(1,:)', current_states.P(2,:)', 'gx'), hold off
title(sprintf('Frame %d', frame_idx))
legend('Candidate Keypoints', 'Actual Keypoints')


% 2. # tracked landmarks over last 20 frames
subplot(2,4,5)
x_axis = (state_number-19):state_number;
num_tracked_landmark = [num_tracked_landmark(2:end) length(current_states.X)];
plot(x_axis, num_tracked_landmark, '-xb')
xlim([x_axis(1) x_axis(end)])
title('# tracked landmarks over last 20 frames')


% 3. full trajectory
% subplot(2,4,6)
% position = getCameraCenter(cam_transform_history{state_number});
% trajectory(:, state_number) = position;

% % plot 3D
% plot3(trajectory(1,1:state_number), trajectory(2, 1:state_number), trajectory(3,1:state_number), '.-')

ax = subplot(2, 4, 6);
hold(ax, 'on')
plotSimpleTrajectory(camera_centers, 1:state_number, ax);
xlabel('X')
ylabel('Z')
axis equal
%xlim([-10, 10]) % for now (parking dataset)
%ylim([-10, 10]) % for now (parking dataset)
title('Full trajectory')
set(gcf, 'GraphicsSmoothing', 'on')


% 4. Trajectory of last 20 frames and landmarks
ax = subplot(2, 4, [3 4 7 8]);
current_cam_pos = getCameraCenter(cam_transform_history{state_number});
plotLandmarks(states_history, max(1,state_number-4):state_number, current_cam_pos, ax);
hold(ax, 'on')
plotCameraTrajectory(cam_transform_history, max(1,state_number-19):state_number, true, false, ax);
axis equal
hold(ax, 'off')
title('Trajectory of last 20 frames and landmarks')

% pause(.2)
% saveas(gcf, sprintf("recording/%05d.png", state_number));
% pause(.2)

end

