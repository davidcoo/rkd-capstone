function [] = pick_place_sample()
% pick_place_sample
%
% pick and place example code; picks up an object at position "A" on
% the table and moves it to position "B".
%% First, we define a couple of helper functions.  You can break these out into
%% separate files if you wish.
% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
    function [] = command_trajectory(robot, trajectory, frequency)
        %% Setup reusable structures to reduce memory use in loop
        cmd = CommandStruct();
        % Compute the velocity numerically
        trajectory_vel = diff(trajectory, 1, 2);
        % Command the trajectory
        for i = 1:(size(trajectory, 2) - 1)
            % Send command to the robot (the transposes on the trajectory
            % points turns column into row vector for commands).
            cmd.position = trajectory(:,i)';
            cmd.velocity = trajectory_vel(:,i)' * frequency;
            robot.set(cmd);
            % Wait a little bit to send at ~100Hz.
            pause(1 / frequency);
        end
        % Send the last point, with a goal of zero velocity.
        cmd.position = trajectory(:,end)';
        cmd.velocity = zeros(1, size(trajectory, 1));
        robot.set(cmd);
    end
% Convenience function to use to hide the internal logic of starting the suction
    function [] = pick(suction_cup)
        suction_cmd = IoCommandStruct();
        suction_cmd.e2 = 1;
        suction_cup.set(suction_cmd);
    end
% Convenience function to use to hide the internal logic of stopping the suction
    function [] = place(suction_cup)
        suction_cmd = IoCommandStruct();
        suction_cmd.e2 = 0;
        suction_cup.set(suction_cmd);
    end
% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);
% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(1);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
%% ONLY UNCOMMENT THIS TO SAVE GAINS ON ROBOT
gains = load('jenga_gains.mat');
for i = 1:5
    robot.set('gains', gains.jenga_gains);
    pause(.5);
end


%% Connect to gripper, and initialize some settings properly
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);
warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');
%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));
%% command frequency, in Hz
frequency = 100;
% We define our "position 1", "position 2", and a "midpoint" waypoints
% here. We found these points by calling robot.getNextFeedback() from the MATLAB
% command line, and tweaking the results as necessary.
%% Fill in DH parameters
% dh = size(5,4);
%  a_i  alp_i  d_i  th_i
% %Robot left
%             l1 = 406.4;
%             l2 = 330.2;
%             l3 = 101.6;
%             w1 = 101.6;
%             w2 = -69.85; -76
%             w3 = 95.25;
%             h1 = 50.8; 30.8
%             Robot right
l1 = 406.4;
l2 = 330.2;
l3 = 114.3;
w1 = 101.6;
w2 = -76.2;
w3 = 101.6;
h1 = 38.1;
dh = [ 0 pi/2 h1 0;
    l1 0 w1 0;
    l2 0 w2 0;
    0 pi/2 w3 0;
    0 0 l3 0];

% Instantiate a robot object using DH parameters with 0 link and joint mass
bot=Robot(dh,[0;0;0;0;0],[0;0;0;0;0]);
%% Fill in positions
pickup_angles = [0.02 0.79 1.53 .02 0]';
pickup_approach_angles = [0.02 0.84 1.55 .02 0]';
midpoint = [0.4366 1.2449 1.9249 -0.0845 0]';

time1 = 2;
time2 = 4;
approach_to_goal = 2;
goal_to_approach = 2;
%% Moves the robot from the initial position to the first waypoint over 4
%% seconds.  We break this into 3 seconds to make most of the motion, and 1 for
%% the final approach.
trajectory = trajectory_spline([initial_thetas midpoint pickup_approach_angles], [0, time1, time2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([pickup_approach_angles pickup_angles], [0, approach_to_goal], frequency);
command_trajectory(robot, trajectory, frequency);
pick(gripper);
pause(0.75);
trajectory = trajectory_spline([pickup_angles pickup_approach_angles], [0, goal_to_approach], frequency);
command_trajectory(robot, trajectory, frequency);


height = 0;
angle = .29;
approach_offset = 38;
separation = 28;
% Place in loop
for row = 1:3
    block_position = [440 438-((row-1)*separation) height 0 0 angle]';
    approach_position = [440 438-((row-1)*separation) height+approach_offset 0 0 angle]';
    block_angles = bot.robot_IK(block_position);
    approach_angles = bot.robot_IK(approach_position);
    new_approach = approach_angles;
    % We keep the last joint equal to the first to ensure the block does not rotateas we move
    midpoint(5) = pickup_angles(5)*0.5 + block_angles(5)*0.5;
    % go to place position
    trajectory = trajectory_spline([pickup_approach_angles midpoint approach_angles], [0, time1, time2], frequency);
    command_trajectory(robot, trajectory, frequency);
    trajectory = trajectory_spline([approach_angles block_angles], [0, approach_to_goal], frequency);
    command_trajectory(robot, trajectory, frequency);
    % place block
    place(gripper);
    pause(0.75);
    trajectory_1 = trajectory_spline([block_angles new_approach], [0, goal_to_approach], frequency);
    disp(approach_angles')
    command_trajectory(robot, trajectory_1, frequency);
    % pick up new block
    trajectory = trajectory_spline([approach_angles midpoint pickup_approach_angles], [0, time1, time2], frequency);
    command_trajectory(robot, trajectory, frequency);
    trajectory = trajectory_spline([pickup_approach_angles pickup_angles], [0, approach_to_goal], frequency);
    command_trajectory(robot, trajectory, frequency);
    if (row ~= 3)
        pick(gripper);
        pause(0.75);
    end
    trajectory = trajectory_spline([pickup_angles pickup_approach_angles], [0, goal_to_approach], frequency);
    command_trajectory(robot, trajectory, frequency);

end

%% Stop logging, and plot results
robot.stopLog();
hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));
% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');
end