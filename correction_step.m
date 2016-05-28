function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);
landmarks = read_world('../data/world.dat');
n = size(landmarks,2);
% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);
K = zeros(2*n+3,2*m);
% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = zeros(2*m,2*n+3);
Hlow = zeros(2,5);
del = zeros(2,1);
for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
	% If the landmark is obeserved for the first time:
	if(observedLandmarks(landmarkId)==false)
		% TODO: Initialize its pose in mu based on the measurement and the current robot pose:
		mu((2*landmarkId+2),1) = mu(1,1)+((z(i).range)*cos(z(i).bearing+mu(3,1)));
    mu((2*landmarkId+3),1) = mu(2,1)+((z(i).range)*sin(z(i).bearing+mu(3,1)));
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
	endif

	% TODO: Add the landmark measurement to the Z vector
	Z((2*(i-1))+1,1) = z(i).range;
  Z((2*(i-1))+2,1) = z(i).bearing;
	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:
  del(1,1) = mu((2*landmarkId+2),1) - mu(1,1);
  del(2,1) = mu((2*landmarkId+3),1) - mu(2,1);
  q = (del')*del;
  expectedZ((2*(i-1))+1,1) = sqrt(q);
  expectedZ((2*(i-1))+2,1) = atan2(del(2,1),del(1,1)) - mu(3,1);
	% TODO: Compute the Jacobian Hi of the measurement function h for this observation
	Hlow(1,1) = -(sqrt(q))*del(1,1)*(1/q);
  Hlow(1,2) = -((sqrt(q))*del(2,1))*(1/q);
  Hlow(1,3) = 0;
  Hlow(1,4) = ((sqrt(q))*del(1,1))*(1/q);
  Hlow(1,5) = ((sqrt(q))*del(2,1))*(1/q);
  Hlow(2,1) = del(2,1)*(1/q);
  Hlow(2,2) = -del(1,1)*(1/q);
  Hlow(2,3) = -q*(1/q);
  Hlow(2,4) = -del(2,1)*(1/q);
  Hlow(2,5) = del(1,1)*(1/q);
	% Augment H with the new Hi
	H((2*(i-1))+1:(2*(i-1))+2,1:3) = Hlow(1:2,1:3);
  H((2*(i-1))+1,2*landmarkId+2:2*landmarkId+3) = Hlow(1,4:5);
  H((2*(i-1))+2,2*landmarkId+2:2*landmarkId+3) = Hlow(2,4:5);	
endfor

% TODO: Construct the sensor noise matrix Q
Q = zeros(2*m);
for l = 1:2*m
Q(l,l) = 0.01;
endfor
% TODO: Compute the Kalman gain
s = ((H*sigma*H')+Q)^-1;
K = sigma*H'*s;
% TODO: Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
diff = (Z - expectedZ);
normalize_all_bearings(diff);
% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.
mu = mu + K*diff;
normalize_angle(mu(3,1));

I = eye(2*n+3,2*n+3);
sigma = (I - (K*H))*sigma;
end
