%JINV   Jacobian of the inverse compound operator.
%   J = JINV(XIJ) returns the Jacobian matrix of the inversion XJI of
%   XIJ. All X's are 3x1-vectors.
%
%   See also J1COMP, J2COMP, ICOMPOUND, COMPOUND.

% v.1.0, 30.11.02, Kai Arras, ASL-EPFL
% v.1.1, Nov.2003, Kai Arras, CAS-KTH: Bug fix in j23: +sin!


function J = jinv(xij);

j13 = xij(1)*sin(xij(3)) - xij(2)*cos(xij(3));
j23 = xij(1)*cos(xij(3)) + xij(2)*sin(xij(3));

J = [-cos(xij(3)), -sin(xij(3)), j13;
      sin(xij(3)), -cos(xij(3)), j23;
          0,             0,      -1 ];