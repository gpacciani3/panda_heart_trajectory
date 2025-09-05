clc
clear all
close all

%% Read data .csv (joint states) 
data1 = readmatrix('joint_states.csv');
disp(data1);

% Remove columns 2 to 11
data1_final = data1(:, [1, 12:end]);

% Save the cleaned data back to CSV
writematrix(data1_final, 'trajectory_cleaned.csv');

% Columns extraction
time = data1_final(:,1); %first column

positions = data1_final(:,2:8); %7 columns for 7 joints

velocities = data1_final(:,9:15); %velocities

efforts = data1_final(:, 16:22); %efforts

% Joints name
joint_names = {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'};

% Plot positions
figure('Name', 'Joint Positions');
for i = 1:7
    subplot(4, 2, i);
    plot(time, positions(:, i), 'LineWidth', 1.5);
    title(['Position - ' joint_names{i}]);
    hold on
    grid on;
end
xlabel('Time [s]');
ylabel('Position [rad]');
hold off

% Plot velocities
figure('Name', 'Joint Velocities');
for i = 1:7
    subplot(4, 2, i);
    plot(time, velocities(:, i), 'LineWidth', 1.5, 'Color', [0.85, 0.33, 0.10]);
    title(['Velocity - ' joint_names{i}]);
    xlabel('Time [s]');
    ylabel('Velocity [rad/s]');
    grid on;
end

% Plot efforts
figure('Name', 'Joint Efforts');
for i = 1:7
    subplot(4, 2, i);
    plot(time, efforts(:, i), 'LineWidth', 1.5, 'Color', [0.49, 0.18, 0.56]);
    title(['Effort - ' joint_names{i}]);
    xlabel('Time [s]');
    ylabel('Effort [Nm]');
    grid on;
end

