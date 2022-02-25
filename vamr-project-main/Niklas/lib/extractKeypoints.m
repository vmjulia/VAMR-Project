function kpt_locations = extractKeypoints(DoGs, contrast_threshold)
    num_octaves = numel(DoGs);
    kpt_locations = cell(1, num_octaves);
    for o = 1:num_octaves
        current_octave = DoGs{o};
        imshow(normalize(current_octave(:, :, 3)));
        waitforbuttonpress();
        DoG_max = imdilate(current_octave, true(3, 3, 3));
        imshow(normalize(DoG_max(:, :, 3)));
        waitforbuttonpress();
        is_kpt = (current_octave == DoG_max) & (current_octave >= contrast_threshold);
        
        visualization = zeros(size(current_octave));
        visualization(is_kpt) = 1;
        imshow(normalize(visualization(:, :, 3)));
        waitforbuttonpress();

        is_kpt(:, :, 1) = false;
        is_kpt(:, :, end) = false;
        [x, y, s] = ind2sub(size(is_kpt), find(is_kpt));
        kpt_locations{o} = horzcat(x, y, s);
    end
end