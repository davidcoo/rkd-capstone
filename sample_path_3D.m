function [] = sample_path_3D()

%% Load log data (allow for sample data as well as arbitrary logs)
log = load('sample_ground_truth_3D.mat');
joint_angles = log.theta;

%% Calculate end effector path through entire log
n = length(joint_angles);
x = zeros(n,1);
y = zeros(n,1);
z = zeros(n,1);
phi = zeros(n,1);
theta = zeros(n,1);
psi = zeros(n,1);

% --------------- BEGIN STUDENT SECTION ----------------------------------
%% Fill in DH parameters
% Instantiate a robot object using DH parameters with 0 link and joint mass
dh = [0 pi/2 56.05 0;
      330.3 0  103.55 0;
      254.10 0 -73.05 0;
      0 pi/2 91 0;
      0 0 213.75 0];
robot = Robot(dh, [0;0;0;0;0],[0;0;0;0;0]);
% loop over columns and set
for col = 1:length(joint_angles)
    ee = robot.ee(joint_angles(:,col));
    x(col) = ee(1);
    y(col) = ee(2);
    z(col) = ee(3);
    psi(col) = ee(4);
    theta(col) = ee(5);
    phi(col) = ee(6);
end


% calculate end effector position and orientation for each timestep of
% joint_angles and store them in x/y/z/phi/theta/psi


% --------------- END STUDENT SECTION ------------------------------------
% Plot actual data
figure();
plot3(x, y, z, 'k-', 'LineWidth', 1);
title('Plot of end effector position over a sample run.');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis equal;

% Plot additional ground truth
hold on;
plot3(log.ground_truth_x, log.ground_truth_y, log.ground_truth_z, 'g--', 'LineWidth', 1);
hold off;
legend('Your Kinematics', 'Correct Kinematics', 'location', 'southOutside');

figure();
plot3(psi, theta, phi, 'k-', 'LineWidth', 1);
title('Plot of end effector orientation over a sample run.');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis equal;

% Plot additional ground truth
hold on;
plot3(log.ground_truth_psi, log.ground_truth_theta, log.ground_truth_phi, 'g--', 'LineWidth', 1);
hold off;
legend('Your Kinematics', 'Correct Kinematics', 'location', 'southOutside');

end