% theta 1x1, pos 2x1
function[out]=M_O_Om(theta, pos)
cos_theta = cos(theta);
sin_theta = sin(theta);
out = [cos_theta, -sin_theta, pos(1); sin_theta, cos_theta, pos(2); 0, 0, 1];
end