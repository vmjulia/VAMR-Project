function plotLandmarks(states, indices, camera_center, ax)
%plotCameraLandmarks Plots the landmarks of the specified states
%   Arguments:
%       states  - cell array of state structs
%       indices - 1xK matrix of indices to plot
%       ax - the axis (figure or subplot) to plot onto
%            When no axis is provided, this plots to figure 1

    if nargin >= 3
        axes(ax);
    else
        figure(1);
    end

    buffer = 0;
    
    last_state = max(indices);
    first_state = min(indices);
    for idx = indices
        color = getColor(idx, first_state, last_state);

        landmarks = filterLandmarks(camera_center, states{idx}.X, .8);
        
        if ~isempty(landmarks)
            scatter(landmarks(1, :), landmarks(3, :), 'x', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
            hold on
        end
    end
    % zoom in to current landmarks
    %current_landmarks = states{last_state}.X;
    %axis([min(current_landmarks(1, :)) - buffer, max(current_landmarks(1, :)) + buffer, min(current_landmarks(3, :)) - buffer, max(current_landmarks(3, :)) + buffer])
end

function color = getColor(idx, firstIdx, lastIdx)
    state_difference = lastIdx - firstIdx + 1;
    if idx == lastIdx
        color = [0, 1, 0];
    else
        age = 1. - ((idx - firstIdx + 1) / state_difference);
        color = [age, age, age];
    end
end