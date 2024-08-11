clc;
clear;
close all;

t = linspace(0, 10, 303);
x = str2double(readlines("x"));
y = str2double(readlines("y"));
r0 = str2double(readlines("r0"));

smoothed_x = smoothdata(x, 'gaussian', 25);

figure;
plot(t, x, 'b', 'LineWidth', 1.5); hold on;
plot(t, smoothed_x, 'r', 'LineWidth', 1.5);
legend('Original Data', 'Smoothed Data');
title('Comparison of Original and Smoothed Data');
xlabel('time');
ylabel('X');
grid on;

smoothed_y = smoothdata(y, 'gaussian', 20); 

figure;
plot(t, y, 'o', 'LineWidth', 1.5); hold on;
plot(t, smoothed_y, 'p', 'LineWidth', 1.5);
legend('Original Data', 'Smoothed Data');
title('Comparison of Original and Smoothed Data');
xlabel('time');
ylabel('Y');
grid on;



smoothed_r0 = smoothdata(r0, 'gaussian', 15); 

figure;
plot(t, r0, 'g', 'LineWidth', 1.5); hold on;
plot(t, smoothed_r0, 's', 'LineWidth', 1);
legend('Original Data', 'Smoothed Data');
title('Comparison of Original and Smoothed Data');
xlabel('time');
ylabel('r0');
grid on;