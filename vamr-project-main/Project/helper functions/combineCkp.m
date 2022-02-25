
function  [ckp,ckp_validity, F, Tau] = combineCkp (ckp, ckp_validity, newCPoints, state_prev, cam_transform, exact, delta)
    newCPointsValidity = ones(size(newCPoints,1), 1);

    if (exact == 1) 
        newCPointsValidity = adjustValidity(newCPointsValidity, ckp, newCPoints);
    else 
        newCPointsValidity = adjustValidity2(newCPointsValidity, ckp, newCPoints, delta);
    end

    newCPoints = newCPoints(newCPointsValidity,:);
    ckp = vertcat(ckp, newCPoints);
    F =  vertcat(transpose(state_prev.F), newCPoints);
    ckp_validity = vertcat(ckp_validity, ones(size(newCPoints,1), 1));
    Tau =  vertcat(transpose(state_prev.Tau) , repmat(cam_transform(:)', [size(newCPoints,1),1]));
end

