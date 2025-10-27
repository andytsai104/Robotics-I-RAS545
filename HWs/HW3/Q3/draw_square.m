clear all, clc
Ts = 0.001;
[SCARA, ArmInfo] = importrobot("SCARA_robot.slx");

% Run the simulator
model_name = "move_SCARA_square.slx";
stop_time = 13.0;
load_system(model_name);

sim(model_name, 'StopTime', num2str(stop_time));