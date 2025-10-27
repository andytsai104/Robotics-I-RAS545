clc; clear; close all;

% Define modified DH parameters for myCobot Pro 600
% [theta (variable), d, a, alpha] (radians and mm)
syms q1 q2 q3 q4 q5 q6 % Joint angles (symbolic)
% 
q1 = 0;
q2 = 0.05;
q3 = 0.3;
q4 = 0.5;
q5 = 0.2;
q6 = 0;

DH = [q1, 219.34,    0,     0;
      q2,      0,    0, -pi/2;
      q3,      0,  250,     0;
      q4,  109.1,  250,     0;
      q5,    108,    0, -pi/2;
      q6,  75.86,    0,  pi/2];

% Compute transformation matrices
T = eye(4);
for i = 1:size(DH,1)
    theta = DH(i,1);
    d = DH(i,2);
    a = DH(i,3);
    alpha = DH(i,4);
    
    % A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
    %      sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    %      0, sin(alpha), cos(alpha), d;
    %      0, 0, 0, 1];
    A = [cos(theta) -sin(theta) 0 a;
        sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
        sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) d*cos(alpha);
        0 0 0 1];
    
    % Multiply to get forward kinematics
    T = T * A;
    
    % Display transformation matrix for each joint
    % fprintf("Transformation Matrix T_%d:\n", i);
    % try 
    %     disp(simplify(A));
    % catch exception
    %     disp(A);
    % end

end

% Final transformation matrix (End-effector position)
disp("Final Transformation Matrix (End-Effector Pose):");

try
    disp(simplify(T));
catch exception
    disp(T);
end
