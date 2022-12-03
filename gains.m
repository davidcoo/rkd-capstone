robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
robot.setCommandLifetime(1);
%% ONLY UNCOMMENT TO SAVE GAINS
    %robot_gains = robot.getGains();
    %save('my_gains.mat', "robot_gains");
    %robot_gains = robot.getGains();
    %disp("found gains")
    %disp(robot_gains)
robot_gains = load('my_gains.mat');
disp("uploading gains")
disp(robot_gains.robot_gains)
for i = 1:10
    robot.set('gains', robot_gains.robot_gains);
    pause(.5);
end