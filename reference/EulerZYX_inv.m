function [ theta ] = EulerZYX_inv( rot_mat )
%	A matriz rotacional é representada por:
%	R = Rx(t1) * Ry(t2) * Rz(t3)
%   
    theta = [0,0,0];
    
    % Matriz com singularidade
    if abs(rot_mat(3,3)) < eps && abs(rot_mat(2,3)) < eps
        theta(1) = 0;
        theta(2) = atan2(rot_mat(1,3), rot_mat(3,3));
        theta(3) = atan2(rot_mat(2,1), rot_mat(2,2));
    
    % Normal 
    else
        theta(1) = atan2(-rot_mat(2,3), rot_mat(3,3));
        sinr = sin(theta(1));
        cosr = cos(theta(1));
        theta(2) = atan2(rot_mat(1,3), cosr * rot_mat(3,3) - sinr * rot_mat(2,3));
        theta(3) = atan2(-rot_mat(1,2), rot_mat(1,1));
    end
end