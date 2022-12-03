%% Fill in DH parameters
% dh = size(5,4);
%  a_i  alp_i  d_i  th_i
%Robot left
            l1 = 406.4;
            l2 = 330.2;
            l3 = 101.6;
            w1 = 101.6;
            w2 = -69.85;
            w3 = 95.25;
            h1 = 50.8;

%             Robot right
%             l1 = 406.4;
%             l2 = 330.2;
%             l3 = 114.3;
%             w1 = 101.6;
%             w2 = -63.5;
%             w3 = 101.6;
%             h1 = 50.8;

dh = [ 0 pi/2 h1 0;
      l1 0 w1 0;
      l2 0 w2 0;
      0 pi/2 w3 0;
      0 0 l3 0];
% dh = [0 pi/2 56.05 0;
%       330.3 0  103.55 0;
%       254.10 0 -73.05 0;
%       0 pi/2 91 0;
%       0 0 213.75 0];
%   
% Instantiate a robot object using DH parameters with 0 link and joint mass
bot=Robot(dh,[0;0;0;0;0],[0;0;0;0;0]);

%% Fill in positions
log = load('sample_ground_truth_3D.mat');
joint_angles = log.theta;

%[0.9960  0.6034 1.3342 -0.5951 0]
place_1 = [0.9960  0.6034 -1.3342 -0.5951+(pi/2) 0]';
bot.ee([0.9581  0.6144 -1.3294 -0.7571+(pi/2) 0]')
% start = [-330.2, 419.1, -76.2 0 0 0]';
% disp("Trying to go to");
% disp(start');
% joints_1 = bot.inverse_kinematics_analytical(start);
% disp("Joint Angles")
% disp(joints_1')
% disp("Actually got")
% pos2 = bot.ee(joints_1);
% disp(pos2')
% bot.visualize(joints_1)