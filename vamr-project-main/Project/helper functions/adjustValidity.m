
function [Validity] = adjustValidity(ckp_validity, kp, ckp)
%input: key points and candidate key points, ckp validity
% output: adjusted ckp_validity

ckp_different = ones(size(ckp,1), 1);

for i=1:size(ckp,1)
    for j=1:size(kp,1)
        
        if ((ckp(i,1)==kp(j,1)) && (ckp(i,2)==kp(j,2)))
            ckp_different (i) = 0; 
            break; 
        end
        
    end
end
Validity = and(ckp_validity,ckp_different);
end

