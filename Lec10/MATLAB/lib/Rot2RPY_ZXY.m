function [phi,theta,psi] = Rot2RPY_ZXY(R)
phi = asin(R(2,3));
psi = atan2(-R(2,1)/cos(phi),R(2,2)/cos(phi));
theta = atan2(-R(1,3)/cos(phi),R(3,3)/cos(phi));
end