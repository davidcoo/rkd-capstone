load('jenga_gains.mat') %Loads the .mat file as a struct
%jenga_gains.positionKp %Displays the kP gains for all motors. There are similar fields for Kp, velocity gains, etc. Pretty sure the order is base to tip, but I'm not certain.
%jenga_gains.positionKp = [1.8, 2.2, 2.2, 2.2,2.2] %Changes the kP gains in the struct. (I just made these numbers up; THESE ARE NOT NECESSARILY GOOD GAINS.)
% jenga_gains.positionKp = [1.8, 2.2, 2.2, 2.2,2.2]
jenga_gains.positionKi = [0 0 0 0 0]
jenga_gains.positionKd = [0 0 0 0 0]

save('jenga_gains.mat', 'jenga_gains') %Save the struct back to the file so it'll be used in your code