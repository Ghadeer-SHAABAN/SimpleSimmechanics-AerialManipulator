
function [eul]=quat2eul(q)


if (size(q,1) ~= 4)
    q = q';
    transposeOutput = true;
else
    transposeOutput = false;
end



seq = 'ZYX';
% Normalize the quaternions
norm_q = sqrt(sum(q.^2, 1));
q = q ./ norm_q;

% invert if scalar is negative
neg_idx = find(q(1,:) < 0);
q(:,neg_idx) = -q(:,neg_idx);

% extract individual quaternion elements
qw = q(1,:);
qx = q(2,:);
qy = q(3,:);
qz = q(4,:);

% The parsed sequence will be in all upper-case letters and validated
switch upper(seq)
    case 'ZYX'
        eul = [ atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 ); ...
            asin( -2*(qx.*qz-qw.*qy) ); ...
            atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 )];
        
        % Convert to rotation matrix instead to get more consistent results
        %R = quat2rotm(q);
        %eul = rotm2eul(R, 'ZYX');        
        
    case 'ZYZ'
        % Need to convert to intermediate rotation matrix here to avoid
        % singularities
        R = quat2rotm(q);
        eul = rotm2eul(R, 'ZYZ');
end

% Check for complex numbers
if ~isreal(eul)
    eul = real(eul);
end

if (transposeOutput)
    eul = eul';
end

end