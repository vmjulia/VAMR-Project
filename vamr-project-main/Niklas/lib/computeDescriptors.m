function descriptors = computeDescriptors(DoGs, locations, num_scales)
    num_octaves = numel(DoGs);
    descriptors = cell(1, num_octaves);
    for o = 1:num_octaves
        current_octave = DoGs{o};
        n_locations = length(locations{o});
        oct_descriptors = cell(1, n_locations);
        for i = 1:n_locations
            location_current = locations{o}(i);
            x, y, scale = location_current(3);
            s = (scale - 1) - num_scales * o;
            I = current_octave{s};
            mag, dir = imgradient(I);
            patch = [x-7 : x+8, y-7:y+8];
            mag_patch = mag(patch);
            dir_patch = dir(patch);
            oct_descriptors{i} = compute_descriptor(mag_patch, dir_patch);
        end
        descriptors{o} = oct_descriptors;
    end
end

function compute_descriptor(patch)
    
end