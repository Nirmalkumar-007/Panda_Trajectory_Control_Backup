%%
% ROBOT PLAYBACK - Animates the Franka Panda robot from logged joint positions
% Requires: Robotics System Toolbox + Franka Panda robot model
%% 
if ~exist('joint_positions','var') || ~exist('ee_positions','var')
    error('Run Load_Data.m first.');
end

% set(0, 'DefaultFigureWindowStyle', 'normal');

%% Individual Joint Plot  (edit dataToPlot and joints as needed)
dataToPlot = joint_torques;   % joint_positions / joint_velocities / joint_torques
joints     = 4;               % joint index 1-7

JOINT_TO_PLOT = 6;     % Individual joint detail plot (1-7)
                       % joint_positions / joint_velocities / joint_torques - together with subplots

%% Section 1 — Data Plots  
figure('Name','Joint Positions');
plot(time_rel, joint_positions,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Joint Position [rad]');
title('Joint Positions vs Time');
legend('J1','J2','J3','J4','J5','J6','J7'); grid on;

figure('Name','Joint Velocities');
plot(time_rel, joint_velocities,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Joint Velocity [rad/s]');
title('Joint Velocities vs Time');
legend('J1','J2','J3','J4','J5','J6','J7'); grid on;

figure('Name','Joint Torques');
plot(time_rel, joint_torques,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Torque [Nm]');
title('Joint Torques vs Time');
legend('J1','J2','J3','J4','J5','J6','J7'); grid on;

figure('Name','EE Positions');
plot(time_rel, ee_positions,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('EE Position [m]');
title('End-Effector Position vs Time');
legend('X','Y','Z'); grid on;

figure('Name','EE Orientations');
plot(time_rel, ee_orientations,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('EE Orientation (Quaternion)');
title('End-Effector Orientation vs Time');
legend('q_x','q_y','q_z','q_w'); grid on;

%%
figure('Name','Individual Joint Plot ');
plot(time_rel, dataToPlot(:,joints),'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Value');
title('joint\_torques');
legend("J" + string(joints)); grid on;

%%  12. Individual Joint Detail
% figure('Name',sprintf('Joint %d Details', JOINT_TO_PLOT),'NumberTitle','off');
figure('Name',sprintf('Joint %d Pos, Vel, Tor', JOINT_TO_PLOT));
subplot(3,1,1);
plot(time_rel, joint_positions(:,JOINT_TO_PLOT), 'b','LineWidth',1.5);
ylabel('Position [rad]');
title(sprintf('Joint %d - Position / Velocity / Torque', JOINT_TO_PLOT));
grid on;

subplot(3,1,2);
plot(time_rel, joint_velocities(:,JOINT_TO_PLOT), 'g','LineWidth',1.5);
ylabel('Velocity [rad/s]');
grid on;

subplot(3,1,3);
plot(time_rel, joint_torques(:,JOINT_TO_PLOT), 'r','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Torque [Nm]');
grid on;

%%  4. Commanded vs Actual EE Position (X / Y / Z over time)
figure('Name','Commanded vs Actual EE Position');

subplot(3,1,1);
plot(time_rel, cmd_positions(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time_rel, ee_positions(:,1),  'r--', 'LineWidth', 1.2);
ylabel('X [m]'); title('Commanded vs Actual End-Effector Position');
legend('Commanded','Actual','Location','best'); grid on;

subplot(3,1,2);
plot(time_rel, cmd_positions(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(time_rel, ee_positions(:,2),  'r--', 'LineWidth', 1.2);
ylabel('Y [m]'); grid on;

subplot(3,1,3);
plot(time_rel, cmd_positions(:,3), 'b', 'LineWidth', 1.5); hold on;
plot(time_rel, ee_positions(:,3),  'r--', 'LineWidth', 1.2);
ylabel('Z [m]'); xlabel('Time [s]'); grid on;
drawnow;
%%
figure('Name','3D EE Trajectory'); hold on; grid on; axis equal; view(45,30);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D End-Effector Trajectory');
plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3),'b','LineWidth',1.5);
scatter3(ee_positions(1,1),   ee_positions(1,2),   ee_positions(1,3),   80,'g','filled');
scatter3(ee_positions(end,1), ee_positions(end,2), ee_positions(end,3), 80,'r','filled');
%%
figure('Name','Robot Trajectory Motion Playback'); hold on; grid on; axis equal; view(45,30);
% %% Section 2 — Robot Playback  (Transparent)
% 
% % PLAYBACK SETTINGS
% fps         = 60;    % frames per second
% speedFactor = 10;    % >1 = faster, <1 = slower
% 
% % FIX JOINT VECTOR  (logged 7 joints → 9 needed by MATLAB Panda model)
% joint_positions9 = zeros(N, 9);
% joint_positions9(:,1:7) = joint_positions;   % cols 8-9 stay zero (fingers)
% 
% % INTERPOLATE for smooth playback at chosen speed
% t_interp     = linspace(time_rel(1), time_rel(end)/speedFactor, ...
%                         round(fps*(time_rel(end)-time_rel(1))/speedFactor));
% joint_interp = interp1(time_rel, joint_positions9, t_interp*speedFactor);
% ee_interp    = interp1(time_rel, ee_positions,     t_interp*speedFactor);
% colors       = jet(length(t_interp));
% 
% % LOAD ROBOT MODEL
% robot = loadrobot('frankaEmikaPanda','DataFormat','row');
% robot.Gravity = [0 0 -9.81];
% 
% fig = figure('Name','Robot Motion Playback','Position',[1000 100 1200 700]);
% ax  = axes('Parent',fig);
% hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
% view(ax,45,30);
% xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
% title(ax,'Franka Panda Robot Playback');
% camlight; lighting gouraud;
% 
% scatter3(ax, ee_interp(1,1),   ee_interp(1,2),   ee_interp(1,3),   100,'g','filled');
% scatter3(ax, ee_interp(end,1), ee_interp(end,2), ee_interp(end,3), 100,'r','filled');
% 
% h_point = plot3(ax, ee_interp(1,1), ee_interp(1,2), ee_interp(1,3), ...
%                 'ro','MarkerFaceColor','r','MarkerSize',8);
% 
% for i = 2:length(t_interp)
%     show(robot, joint_interp(i,:), 'PreservePlot',false, 'Frames','off', ...
%          'Parent',ax, 'Visuals','on');
%     patches = findobj(ax,'Type','Patch');
%     for p = 1:length(patches)
%         patches(p).FaceAlpha = 0.4;
%     end
%     line(ax, [ee_interp(i-1,1), ee_interp(i,1)], ...
%              [ee_interp(i-1,2), ee_interp(i,2)], ...
%              [ee_interp(i-1,3), ee_interp(i,3)], ...
%          'Color',colors(i,:),'LineWidth',2);
%     set(h_point,'XData',ee_interp(i,1),'YData',ee_interp(i,2),'ZData',ee_interp(i,3));
%     drawnow;
%     pause(1/fps);
% end

% %% Section 3 — Robot Playback  (Non-Transparent) 
% 
% fig = figure('Name','Franka Robot Motion Playback','Position',[1000 100 1200 700]);
% ax  = axes('Parent',fig);
% hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
% view(ax,45,30);
% xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
% title(ax,'Franka Panda Robot Playback');
% camlight; lighting gouraud;
% 
% show(robot, joint_interp(1,:), 'PreservePlot',false, 'Frames','off', 'Parent',ax);
% 
% h_trace = plot3(ax, ee_interp(1,1), ee_interp(1,2), ee_interp(1,3),'b','LineWidth',2);
% h_point = plot3(ax, ee_interp(1,1), ee_interp(1,2), ee_interp(1,3), ...
%                 'ro','MarkerFaceColor','r','MarkerSize',8);
% 
% for i = 2:length(t_interp)
%     show(robot, joint_interp(i,:), 'PreservePlot',false, 'Frames','off', 'Parent',ax);
%     set(h_trace,'XData',ee_interp(1:i,1),'YData',ee_interp(1:i,2),'ZData',ee_interp(1:i,3));
%     set(h_point,'XData',ee_interp(i,1),'YData',ee_interp(i,2),'ZData',ee_interp(i,3));
%     drawnow;
%     pause(1/fps);
% end

