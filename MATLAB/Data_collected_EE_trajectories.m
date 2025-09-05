clear all
close all
clc
csv_filename = "franka_states.csv"
output_filename = "EE_pose.csv"

% Read the CSV file
data = readmatrix(csv_filename);
  
% Extract time 
time = data(:, 1);
  
% O_T_EE matrix
O_T_EE_data = data(:, 190:205); % 16 values for the 4x4 matrix
  
pos_in_time = O_T_EE_data(:,13:15);
% Initialize arrays for position and orientation
num_samples = length(time);
position = zeros(num_samples, 3);
orientation_quat = zeros(num_samples, 4);
orientation_rpy = zeros(num_samples, 3);
  
% Process each sample
for i = 1:num_samples
   % Reshape the 16-element vector into a 4x4 matrix 
   T = reshape(O_T_EE_data(i, :), [4, 4])';
      
   % Extract position (translation) from the matrix
   position(i, :) = T(1:3, 4)';
      
   % Extract rotation matrix
   R = T(1:3, 1:3);
      
   % Convert rotation matrix to quaternion
   orientation_quat(i, :) = rotm2quat(R);
      
   % Convert rotation matrix to Roll-Pitch-Yaw angles 
   orientation_rpy(i, :) = rotm2eul(R, 'ZYX');
end
  
% Create output table
output_table = table(time, ...
                    position(:, 1), position(:, 2), position(:, 3), ...
                    orientation_quat(:, 1), orientation_quat(:, 2), ...
                    orientation_quat(:, 3), orientation_quat(:, 4), ...
                    orientation_rpy(:, 1), orientation_rpy(:, 2), ...
                    orientation_rpy(:, 3), ...
                    'VariableNames', {'Time', ...
                    'Pos_X', 'Pos_Y', 'Pos_Z', ...
                    'Quat_W', 'Quat_X', 'Quat_Y', 'Quat_Z', ...
                    'Roll', 'Pitch', 'Yaw'});
  
% Save to CSV
writetable(output_table, output_filename);

% Parameters for the Heart-Shaped Trajectory
num_points = 100;
t = linspace(0, 2*pi, num_points);
        
% Parametric equations on XZ plane
x = 0.1 * (sin(t)).^3;
z = 0.1 * (13.0 * cos(t) - 5.0 * cos(2*t) - 2.0 * cos(3*t) - cos(4*t)) / 16.0;
    
% Offsets
x = x + 0.4;
z = z + 0.6;
y = zeros(size(x));
  
% Plot the results
plot_ee_trajectory(time, pos_in_time, orientation_rpy, x, y, z);
  
fprintf('Data extracted and saved to: %s\n', output_filename);
function plot_ee_trajectory(time, position, orientation_rpy, x, y, z)
   % Position plot
   figure('Name', 'End Effector Position', 'Position', [100, 100, 1200, 400]);
   subplot(1, 3, 1);
   plot(time, position(:, 1), 'r-', 'LineWidth', 2);
   title('X Position');
   xlabel('Time [s]');
   ylabel('Position [m]');
   grid on;
  
   subplot(1, 3, 2);
   plot(time, position(:, 2), 'g-', 'LineWidth', 2);
   title('Y Position');
   xlabel('Time [s]');
   ylabel('Position [m]');
   grid on;
  
   subplot(1, 3, 3);
   plot(time, position(:, 3), 'b-', 'LineWidth', 2);
   title('Z Position');
   xlabel('Time [s]');
   ylabel('Position [m]');
   grid on;
   sgtitle('End-effector positions');
  
   % Orientation plot (RPY angles)
   figure('Name', 'End Effector Orientation', 'Position', [100, 100, 1200, 400]);
   subplot(1, 3, 1);
   plot(time, rad2deg(orientation_rpy(:, 1)), 'r-', 'LineWidth', 2);
   title('Roll');
   xlabel('Time [s]');
   ylabel('Angle [deg]');
   grid on;
  
   subplot(1, 3, 2);
   plot(time, rad2deg(orientation_rpy(:, 2)), 'g-', 'LineWidth', 2);
   title('Pitch');
   xlabel('Time [s]');
   ylabel('Angle [deg]');
   grid on;
  
   subplot(1, 3, 3);
   plot(time, rad2deg(orientation_rpy(:, 3)), 'b-', 'LineWidth', 2);
   title('Yaw');
   xlabel('Time [s]');
   ylabel('Angle [deg]');
   grid on;
   sgtitle('End-effector orientations');
  
   % 3D trajectory plot
   figure('Name', '3D End Effector Trajectory');
   h = plot3(position(:, 1), position(:, 2), position(:, 3), 'b-', 'LineWidth', 2);
   hold on;
   scatter3(position(1, 1), position(1, 2), position(1, 3), 100, 'g', 'filled'); %start point
   scatter3(position(end, 1), position(end, 2), position(end, 3), 100, 'yellow', 'filled'); %end point
   title('3D End Effector Trajectory');
   xlabel('X [m]');
   ylabel('Y [m]');
   zlabel('Z [m]');
   grid on;
   axis equal;
   hold on
   h1 = plot3(x, y, z, 'r--','LineWidth', 2); %plot of the desired trajectory
   axis equal; grid on;
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   title('Heart-Shaped Trajectory in XZ-plane');
   legend('Executed trajectory by the robot', 'Start', 'End','Desired trajectory');
end