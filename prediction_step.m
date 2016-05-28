function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
landmarks = read_world('../data/world.dat');
n = size(landmarks,2);
% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
mu(1:3,1) = [mu(1,1) + u.t*cos(mu(3, 1)+u.r1);
             mu(2,1) + u.t*sin(mu(3, 1)+u.r1);
             mu(3,1) + u.r1 + u.r2];
          theta = mu(3,1);
          normalize_angle(theta);

% TODO: Compute the 3x3 Jacobian Gx of the motion model
gx = [1, 0, -u.t*sin(mu(3, :)+u.r1); 
      0, 1, u.t*cos(mu(3, :)+u.r1); 
      0, 0, 1] ;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma(1:3,1:3) = (gx*sigma(1:3,1:3)*gx') + R3 ;
sigma(1:3,4:end) = gx*sigma(1:3,4:end);
sigma(4:end,1:3) = (sigma(1:3,4:end))';

endfunction
