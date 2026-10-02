%% PERFORMANCE PLOTS
%%
if ~exist('cmd_positions','var')
    error('Run Load_Data.m first.');
end
if ~exist('cmd_error_norm','var')
    error('Run EE_Error_Analysis.m first.');
end

% Force figures to open as separate undocked windows
% Docked figures (tabbed inside MATLAB desktop) often do not render plots
% set(0, 'DefaultFigureWindowStyle', 'normal');

joint_labels = {'J1','J2','J3','J4','J5','J6','J7'};

%%  1. 3D Commanded vs Actual (Overlaid)
figure('Name','Commanded vs Actual Trajectory');
plot3(cmd_positions(:,1), cmd_positions(:,2), cmd_positions(:,3), 'k--','LineWidth',1.5); hold on;
plot3(ee_positions(:,1),  ee_positions(:,2),  ee_positions(:,3),  'b',  'LineWidth',1.5);
scatter3(cmd_positions(1,1),   cmd_positions(1,2),   cmd_positions(1,3),   80,'g','filled');
scatter3(cmd_positions(end,1), cmd_positions(end,2), cmd_positions(end,3), 80,'r','filled');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Commanded vs Actual End-Effector Trajectory');
legend('Commanded','Actual','Start','End','Location','best');
grid on; axis equal; view(45,30);
drawnow;

% %% 3. Deviation Tube (Shaded Region)
% figure('Name','Deviation Shadded');
% hold on;
% % Commanded trajectory
% plot3(cmd_positions(:,1), cmd_positions(:,2), cmd_positions(:,3), ...
%       'k--','LineWidth',1.5);
% % Actual trajectory
% plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), ...
%       'b','LineWidth',1.5);
% % Shaded deviation region
% for i = 1:length(cmd_positions)-1
%     X = [cmd_positions(i,1), cmd_positions(i+1,1), ee_positions(i+1,1), ee_positions(i,1)];
%     Y = [cmd_positions(i,2), cmd_positions(i+1,2), ee_positions(i+1,2), ee_positions(i,2)];
%     Z = [cmd_positions(i,3), cmd_positions(i+1,3), ee_positions(i+1,3), ee_positions(i,3)];
% 
%     fill3(X, Y, Z, [0.6 0.8 1], ...   % light blue
%           'FaceAlpha',0.25, ...
%           'EdgeColor','none');
% end
% xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
% title('Deviation Between Commanded and Actual Trajectories');
% legend('Commanded','Actual','Deviation Region','Location','best');
% grid on; axis equal; view(45,30);

% %%  2. 3D Separated Comparison (Commanded offset, Actual in place)
% figure('Name','Commanded vs Actual Separated');
% z_gap = max(ee_positions(:,3)) - min(cmd_positions(:,3)) + 0.05;
% x_gap = max(ee_positions(:,1)) - min(cmd_positions(:,1)) + 0.05;
% 
% xd = cmd_positions(:,1)+x_gap;  yd = cmd_positions(:,2);  zd = cmd_positions(:,3)+z_gap;
% xa = ee_positions(:,1);          ya = ee_positions(:,2);   za = ee_positions(:,3);
% 
% plot3(xd, yd, zd, 'k--','LineWidth',1.5); hold on;
% plot3(xa, ya, za, 'b',  'LineWidth',1.5);
% plot3([xd(1) xa(1)],     [yd(1) ya(1)],     [zd(1) za(1)],     'g--','LineWidth',1.0);
% plot3([xd(end) xa(end)], [yd(end) ya(end)], [zd(end) za(end)], 'r--','LineWidth',1.0);
% scatter3([xd(1) xa(1)],     [yd(1) ya(1)],     [zd(1) za(1)],     80,'g','filled');
% scatter3([xd(end) xa(end)], [yd(end) ya(end)], [zd(end) za(end)], 80,'r','filled');
% xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
% title('Commanded (offset) vs Actual (in place) EE Trajectory');
% legend('Commanded','Actual','Start connector','End connector','Location','best');
% grid on; axis equal; view(0,90);
drawnow;
%%  3. EE Position Error Distribution 
figure('Name','EE Position Error Distribution');
scatter3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), ...
         10, cmd_error_norm*500, 'filled');

green_red = [linspace(0,1,128)', linspace(1,0,128)', zeros(128,1)];
colormap(green_red);

cb = colorbar;
cb.Label.String = 'Position Error [mm]';
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('EE Position Error Distribution');
grid on; axis equal; view(45,30);
drawnow;

%% 3A. EE Deviation Heatmap (Commanded vs Actual)
figure('Name','EE Deviation Heatmap');
% Plot commanded trajectory (reference)
plot3(cmd_positions(:,1), cmd_positions(:,2), cmd_positions(:,3), ...
      'k--','LineWidth',1.5); hold on;
% Plot actual trajectory with error heatmap
scatter3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), ...
         10, cmd_error_norm*1000, 'filled');
% Colormap: green → yellow → red
green_red = [linspace(0,1,128)', linspace(1,0,128)', zeros(128,1)];
colormap(green_red);
cb = colorbar;
cb.Label.String = 'Deviation from Commanded Path [mm]';
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Deviation Heatmap (Actual vs Commanded Path)');
legend('Commanded Path','Actual (colored by deviation)','Location','best');
grid on; axis equal; view(45,30);

% %%  4. Position Tracking Error
% figure('Name','Position Tracking Error');
% plot(time_rel, cmd_error_norm, 'b', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Error [m]');
% title('End-Effector Position Tracking Error (3D Euclidean)');
% grid on;
% 
% %%  5. Orientation Tracking Error
% figure('Name','Orientation Tracking Error');
% plot(time_rel, ori_error, 'r', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Orientation Error [rad]');
% title('End-Effector Orientation Tracking Error');
% grid on;

%%  6. EE Speed Profile
figure('Name','EE Speed Profile');
plot(time_vel, vel_norm, 'm', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('End-Effector Speed Profile');
grid on;

%%  7. Homing Error Bar
figure('Name','Homing Error');
bar([delta(1), delta(2), delta(3), homing_error], ...
    'FaceColor','flat','CData',[0 0.4 1; 1 0.5 0; 0.2 0.7 0.2; 0.8 0.1 0.1]);
xticklabels({'dX','dY','dZ','3D Distance'});
ylabel('Error [m]');
title('Homing Error - Start vs End Position');
yline(0,'k--','LineWidth',1.0);
grid on;

%%  RMS Position per Joint
figure('Name','RMS Joint Position','Color','w');

bar(rms_pos_joint * (180/pi), 'FaceColor',[0.3 0.7 0.4]);

xticklabels(joint_labels);
ylabel('RMS Position [deg]');
title('RMS Joint Position');
grid on;

%%  8. RMS Torque per Joint
figure('Name','RMS Torque per Joint');
bar(rms_torque,'FaceColor',[0.2 0.5 0.8]);
xticklabels(joint_labels);
ylabel('RMS Torque [Nm]');
title('RMS Torque per Joint');
grid on;

%% RMS Velocity per Joint
figure('Name','RMS Joint Velocity','Color','w');

bar(rms_vel_joint, 'FaceColor',[0.8 0.4 0.2]);

xticklabels(joint_labels);
ylabel('RMS Velocity [rad/s]');
title('RMS Joint Velocity');
grid on;
%%  9. Joint Jerk Metrics (Grouped Bar)
figure('Name','Joint Jerk Metrics');
bar_data = [rms_jerk; mean_abs_jerk]';
b = bar(bar_data,'grouped');
b(1).FaceColor = [0.2 0.5 0.8];
b(2).FaceColor = [0.8 0.4 0.1];
xticklabels(joint_labels);
ylabel('Jerk [rad/s^3]');
title('Joint Jerk per Joint (RMS vs Mean Absolute)');
legend('RMS Jerk','Mean Abs Jerk','Location','best');
grid on;

%%  13. Joint Range of Motion Bar
figure('Name','Joint Range of Motion','Color','w');
rom_deg = range_pos_joint * (180/pi);
b = bar(rom_deg, 'FaceColor','flat', 'BarWidth', 0.6);

% Softer, consistent color palette
b.CData = lines(7);

xticks(1:7);
xticklabels(joint_labels);
xtickangle(20);

xlabel('Joint','FontWeight','bold');
ylabel('Range of Motion [deg]','FontWeight','bold');
title('Joint Range of Motion','FontWeight','bold');

grid on;

% Improve limits
ylim([0, max(rom_deg)*1.15]);

% Add value labels (cleaner placement)
for j = 1:7
    text(j, rom_deg(j) + max(rom_deg)*0.03, ...
        sprintf('%.1f°', rom_deg(j)), ...
        'HorizontalAlignment','center', ...
        'FontSize',10, ...
        'FontWeight','bold');
end


%% Trajectory Progress Visualization (s_val)
% This plot visualizes the normalized trajectory progress signal (s_val)
figure('Name','Trajectory Progress (s_{val})');

plot(time_rel, s_val, 'b', 'LineWidth', 1.6);
grid on;
hold on;

xlabel('Time [s]');
ylabel('s_{val}');
title('Normalized Trajectory Progress with Segment Labels');

ylim([0 1]);

% segment boundaries (trajectory resets)
seg_reset = find(diff(s_val) < -0.05) + 1;

% full segment indices
seg_starts = [1; seg_reset];
seg_ends   = [seg_reset-1; length(s_val)];

% draw boundaries + labels
for i = 1:length(seg_starts)

    % vertical boundary line (except first segment start)
    if i > 1
        xline(time_rel(seg_starts(i)), '--r', 'LineWidth', 1.2);
    end

    % mid-point of segment for label placement
    mid_idx = round((seg_starts(i) + seg_ends(i)) / 2);

    % label text (from YAML if available)
    if exist('seg_labels','var') && i <= length(seg_labels)
        lbl = seg_labels{i};
    else
        lbl = sprintf('Seg %d', i);
    end

    % place label slightly above curve
    text(time_rel(mid_idx), 0.85, lbl, ...
        'HorizontalAlignment','center', ...
        'FontSize',9, ...
        'FontWeight','bold', ...
        'Color',[0.2 0.2 0.2], ...
        'BackgroundColor','w');
end

%% JOINT PERFORMANCE DASHBOARD

joint_labels = {'J1','J2','J3','J4','J5','J6','J7'};

rom_deg = range_pos_joint * (180/pi);
rms_pos_deg = rms_pos_joint * (180/pi);

figure('Name','Joint Performance Dashboard','Color','w');

tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% 1. RMS Torque
nexttile;
bar(rms_torque,'FaceColor',[0.2 0.5 0.8]);
xticklabels(joint_labels);
title('RMS Torque [Nm]');
grid on; box on;

% 2. RMS Velocity
nexttile;
bar(rms_vel_joint,'FaceColor',[0.8 0.4 0.2]);
xticklabels(joint_labels);
title('RMS Velocity [rad/s]');
grid on; box on;

% 3. RMS Jerk
nexttile;
bar(rms_jerk,'FaceColor',[0.6 0.3 0.8]);
xticklabels(joint_labels);
title('RMS Jerk [rad/s^3]');
grid on; box on;

% 4. Range of Motion
nexttile;
bar(rom_deg,'FaceColor',[0.2 0.7 0.4]);
xticklabels(joint_labels);
title('Range of Motion [deg]');
ylabel('deg');
grid on; box on;


% %% JERK TIME-DOMAIN PLOT  (add after the existing bar chart)
% figure('Name','Joint Jerk vs Time');
% plot(time_jerk, joint_jerk, 'LineWidth', 1.2);
% xlabel('Time [s]');
% ylabel('Jerk [rad/s^3]');
% title('Joint Jerk vs Time');
% legend('J1','J2','J3','J4','J5','J6','J7','Location','best');
% grid on;