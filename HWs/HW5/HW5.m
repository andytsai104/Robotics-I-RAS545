clear all, clc

Ts = 0.001;
%% b: Open-loop
% run the simulink model
simOut = sim("HW5_b.slx");
r_b = simOut.r_b;
y_b = simOut.y_b;
% plot the results
figure(1)
plot(r_b.Time, r_b.Data, 'Color', 'blue', 'LineWidth', 3)
hold on
plot(y_b.Time, y_b.Data, 'Color', 'red', 'LineWidth', 1)
hold off
grid on
legend('Input Signal r(t)', 'Response y(t)')
title('Open-loop system response')

%% c: Close-loop
% skip

%% d: Close-loop with PID 
Kp = 0.5;
Ki = 3;
Kd = 0;
% run the simulink model
simOut = sim("HW5_d.slx");
r_d = simOut.r_d;
y_d = simOut.y_d;
% plot the results
figure(2)
plot(r_d.Time, r_d.Data, 'Color', 'blue', 'LineWidth', 3)
hold on
plot(y_d.Time, y_d.Data, 'Color', 'red', 'LineWidth', 1)
hold off
grid on
legend('Input Signal r(t)', 'Response y(t)')
title('Close-loop system Response with PID controller')


%% e: Open-loop vs Close-loop vs Error
simOut = sim("HW5_e.slx");
y_o = simOut.y_OpenLoop;
y_c = simOut.y_CloseLoop;
e = simOut.e;
% plot the results
figure(3)
plot(y_o.Time, y_o.Data, 'Color', 'blue', 'LineWidth', 1)
hold on
plot(y_d.Time, y_d.Data, 'Color', 'red', 'LineWidth', 1)
plot(e.Time, e.Data, 'Color', 'green', 'LineWidth', 1)
hold off
grid on
legend('Open-loop output', 'Closed-loop output ', 'Error signal')
title('Comparisons')