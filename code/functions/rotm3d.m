function [ rotm3d ] = rotm3d( ang,axis )
%rotate3d Generalised rotation in 3D along x,y,z axis
%   General rotation matrix computation
%   ang = angle of rotation in radians
%   axis = axis of rotation 1=x,2=y,3=z

    if axis == 1
        rotm3d = [1 0 0; 0 cos(ang) -sin(ang); 0 sin(ang) cos(ang)];
    elseif axis == 2
        rotm3d = [cos(ang) 0 sin(ang); 0 1 0; -sin(ang) 0 cos(ang)];
    elseif axis == 3
        rotm3d = [cos(ang) -sin(ang) 0; sin(ang) cos(ang) 0; 0 0 1];
    end

end
