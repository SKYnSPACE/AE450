function bRi = RPY2Rot(angles)
phi = angles(1);
theta = angles(2);
psi = angles(3);

R_3 = [ cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
R_2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R_1 = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];

bRi = R_1*R_2*R_3;

end