function saveState(i, frames, states, transforms)
    k = mod(i, 2);
    save(sprintf('states%d.mat', k), 'states');
    save(sprintf('transforms%d.mat', k), 'transforms');
    save(sprintf('frames%d.mat', k), 'frames');
end