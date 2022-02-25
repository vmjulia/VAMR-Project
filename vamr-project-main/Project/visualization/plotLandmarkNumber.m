function plotLandmarkNumber(states, current_state, num_states, ax)
%PLOTLANDMARKNUMBER Summary of this function goes here
%   Detailed explanation goes here
    if nargin >= 2
        axes(ax);
    else
        figure(1);
    end

    start = max(1, current_state - num_states + 1);
    finish = current_state;
    num_states_available = finish - start + 1;
    padding = num_states - num_states_available;
    num_landmarks = arrayfun(@(i) size(states{i}.X, 2), start:finish);

    xs = current_state - num_states + 1 : finish;
    ys = [zeros(1, padding), num_landmarks];
    plot(xs, ys);
end

