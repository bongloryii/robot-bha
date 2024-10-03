l=1
inverse_kinematics = [sqrt(3)/2     -1/2    -l;...
                      -sqrt(3)/2    -1/2    -l;...
                      0             1       -l]
% question_b: 
xdotR = 1;
ydotR = 0;
thetadotR = 0;

inverse_kinematics * [xdotR; ydotR; thetadotR];

% question_c: 
xdotR = 1;
ydotR = 1;
thetadotR = 0;

inverse_kinematics * [xdotR; ydotR; thetadotR]