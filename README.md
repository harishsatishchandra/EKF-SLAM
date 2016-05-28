# EKF-SLAM
implementation of EKF based SLAM simulation with known correspondences using octave

This code is trying to implement EKF based SLAM approach to simulate a robot moving around a set of landmarks with known correspondences. Odometry based motion model is used for this project. A sensor with range and bearing information is assumed.

The simulation works well when the motion model is overconfident (i.e when I ignore the motion noise covariance matrix from the main covariance matrix.) but when I add some motion noise to the main covariance matrix, the robot starts acting weird even though, as far as I know, the code is proper.
