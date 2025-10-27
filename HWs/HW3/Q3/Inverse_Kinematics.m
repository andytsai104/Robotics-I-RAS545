clc; clear all

% theta
syms th1 th2 th3;

% DH-table
DH = [[th1 0.2 0.4 0];
    [th2 0.25 0.3 0];
    [th3 0.15 0.15 0]];

len = length(DH);

for i=1:len-1
    if i==1
        T_final = createDH_T(DH(1, 1), DH(1, 2), DH(1, 3), DH(1, 4));
    else
        T_final = T_final * createDH_T(DH(i, 1), DH(i, 2), DH(i, 3), DH(i, 4));
    end
end

T_final = simplify(T_final);

% Define a random position of E.E.
% r = -0.85+1.7*rand();
% theta = 2*pi*rand();
% x = r*cos(theta);
% y = r*sin(theta);

x = 0.0280;
y = -0.0796;
z = 0.6;

% Solve the equations
solutions = solve([T_final(1, 4)==x, T_final(2, 4)==y, T_final(3, 4)==z], [th1, th2, th3], 'Real', true);
th1_sol = rad2deg(double(solutions.th1(1)));
th2_sol = rad2deg(double(solutions.th2(1)));
th3_sol = rad2deg(double(solutions.th3(1)));


% Display the Results
fprintf('The given position of E.E. is (%.4f, %.4f, %.4f)\n', x, y, z);
fprintf('One of the solutions from inverse kinematics:\n');
fprintf('th1 = %.4f°\n', th1_sol);
fprintf('th2 = %.4f°\n', th2_sol);
fprintf('th3 = %.4f°\n', th3_sol);

