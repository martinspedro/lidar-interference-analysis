quat = [0.494777 -0.499970 0.499913 0.505285]

%rotm = rotmat(quat)

% https://www.andre-gaschler.com/rotationconverter/
R = [  0.0002342, -0.9999442, -0.0105638;
       0.0104498,  0.0105656, -0.9998896;
       0.9999453,  0.0001238,  0.0104517 ]
T = [0.059372, -0.075109, -0.272133]'

P_LIDAR = [19.705000, -11.008000, -1.194000, 1]

P_CAMERA = [R, T] * P_LIDAR'


P_CAMERA_ROS = [11.083997, 1.208361, 19.417940]