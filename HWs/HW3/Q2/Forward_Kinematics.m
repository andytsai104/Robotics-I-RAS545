clear all; clc

% Define theta
th1 = 0;
th2 = 0;
th3 = 0;

th1 = deg2rad(th1);
th2 = deg2rad(th2);
th3 = deg2rad(th3);

% Run the simulator
Ts = 0.001;
[SCARA, ArmInfo] = importrobot("SCARA_robot.slx");
stop_time = 1.0;
load_system("move_SCARA.slx");

out = sim("move_SCARA.slx", 'StopTime', num2str(stop_time));

% Get the result
TrVec = out.TransformationMatrix.Data;
x = TrVec(1);
y = TrVec(2);
z = TrVec(3);

fprintf('When theta_1=%.1f°, theta_2=%.1f°, theta_3=%.1f°,\n', rad2deg(th1), rad2deg(th2), rad2deg(th3))
fprintf('the final position of the end effector (Px, Py, Pz) is (%.4f, %.4f, %.4f)\n', x, y, z)
