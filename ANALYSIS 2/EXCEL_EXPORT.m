%%
% EXPORT RESULTS
% Run after Load_Data.m + EE_Error_Analysis.m + Joint_Analysis.m
% Exports all analysis metrics to a numbered Excel file

%% GUARD
if ~exist('CSV_FILE','var') || ~exist('rms_pos_error','var') || ~exist('rms_torque','var')
    error('Run Load_Data.m, EE_Error_Analysis.m and Joint_Analysis.m first.');
end

%% OUTPUT FILE — auto-incremented to avoid overwriting previous exports
EXPORT_FOLDER = 'C:\Users\nravikumar\Desktop\ANALYSIS 2';   % <- SET THIS

[~, csv_name, ~] = fileparts(CSV_FILE);
base_name   = sprintf('Panda_Analysis_%s', csv_name);
OUTPUT_FILE = fullfile(EXPORT_FOLDER, [base_name '.xlsx']);

counter = 1;
while exist(OUTPUT_FILE, 'file')
    OUTPUT_FILE = fullfile(EXPORT_FOLDER, sprintf('%s (%d).xlsx', base_name, counter));
    counter = counter + 1;
end

%% SHEET 1 — EE Position Tracking  (cmd vs actual)
writecell({
    'EE POSITION TRACKING (cmd vs actual)', '',              '';
    'Metric',                               'Value',         'Unit';
    'RMS Error',                            rms_pos_error,   'm';
    'Mean Error',                           mean_pos_error,  'm';
    'Max Error',                            max_pos_error,   'm';
    'Time of Max',                          time_of_max_pos, 's';
}, OUTPUT_FILE, 'Sheet', '1 - EE Position');
fprintf('Sheet 1 — EE Position Tracking\n');

%% SHEET 2 — EE Orientation Tracking
writecell({
    'EE ORIENTATION TRACKING METRICS', '',              '';
    'Metric',                          'Value',         'Unit';
    'RMS Error',                       rms_ori_error,   'rad';
    'Mean Error',                      mean_ori_error,  'rad';
    'Max Error',                       max_ori_error,   'rad';
    'Time of Max',                     time_of_max_ori, 's';
}, OUTPUT_FILE, 'Sheet', '2 - EE Orientation');
fprintf('Sheet 2 — EE Orientation Tracking\n');

%% SHEET 3 — EE Homing Error
writecell({
    'EE HOMING ERROR', '',             '';
    'Metric',          'Value',        'Unit';
    'Start X',         start_point(1), 'm';
    'Start Y',         start_point(2), 'm';
    'Start Z',         start_point(3), 'm';
    'End X',           end_point(1),   'm';
    'End Y',           end_point(2),   'm';
    'End Z',           end_point(3),   'm';
    'Delta X',         delta(1),       'm';
    'Delta Y',         delta(2),       'm';
    'Delta Z',         delta(3),       'm';
    '3D Distance',     homing_error,   'm';
}, OUTPUT_FILE, 'Sheet', '3 - EE Homing Error');
fprintf('Sheet 3 — Homing Error\n');

%% SHEET 4 — Per-Segment Tracking Error
seg_header = {'Segment', 'Label', 'RMS [m]', 'Mean [m]', 'Max [m]'};
seg_data   = cell(n_segs_det, 5);
for s = 1:n_segs_det
    seg_data{s,1} = s;
    if s <= length(seg_labels)
        seg_data{s,2} = seg_labels{s};
    else
        seg_data{s,2} = sprintf('Seg %d', s);
    end
    seg_data{s,3} = seg_rms(s);
    seg_data{s,4} = seg_mean(s);
    seg_data{s,5} = seg_max(s);
end
writecell([seg_header; seg_data], OUTPUT_FILE, 'Sheet', '4 - Per-Segment Error');
fprintf('Sheet 4 — Per-Segment Error\n');

%% SHEET 5 — Joint Torque Metrics
torque_header = {'Joint', 'RMS [Nm]', 'Mean [Nm]', 'Max [Nm]'};
torque_data   = cell(7, 4);
for j = 1:7
    torque_data{j,1} = sprintf('J%d', j);
    torque_data{j,2} = rms_torque(j);
    torque_data{j,3} = mean_torque(j);
    torque_data{j,4} = max_torque(j);
end
writecell([torque_header; torque_data], OUTPUT_FILE, 'Sheet', '5 - Joint Torque');
fprintf('Sheet 5 — Joint Torque\n');

%% SHEET 6 — Joint Velocity Metrics
vel_header = {'Joint', 'RMS [rad/s]', 'MeanAbs [rad/s]', 'Max [rad/s]'};
vel_data   = cell(7, 4);
for j = 1:7
    vel_data{j,1} = sprintf('J%d', j);
    vel_data{j,2} = rms_vel_joint(j);
    vel_data{j,3} = mean_abs_vel_joint(j);
    vel_data{j,4} = max_vel_joint(j);
end
writecell([vel_header; vel_data], OUTPUT_FILE, 'Sheet', '6 - Joint Velocity');
fprintf('Sheet 6 — Joint Velocity\n');

%% SHEET 7 — Joint Jerk Metrics
jerk_header = {'Joint', 'RMS [rad/s3]', 'MeanAbs [rad/s3]', 'Max [rad/s3]'};
jerk_data   = cell(7, 4);
for j = 1:7
    jerk_data{j,1} = sprintf('J%d', j);
    jerk_data{j,2} = rms_jerk(j);
    jerk_data{j,3} = mean_abs_jerk(j);
    jerk_data{j,4} = max_jerk(j);
end
writecell([jerk_header; jerk_data], OUTPUT_FILE, 'Sheet', '7 - Joint Jerk');
fprintf('Sheet 7 — Joint Jerk\n');

%% SHEET 8 — Joint Position Absolute Stats
abs_header = {'Joint', 'RMS [rad]', 'MeanAbs [rad]', 'MaxAbs [rad]'};
abs_data   = cell(7, 4);
for j = 1:7
    abs_data{j,1} = sprintf('J%d', j);
    abs_data{j,2} = rms_pos_joint(j);
    abs_data{j,3} = mean_abs_pos_joint(j);
    abs_data{j,4} = max_abs_pos_joint(j);
end
writecell([abs_header; abs_data], OUTPUT_FILE, 'Sheet', '8 - Joint Pos Absolute');
fprintf('Sheet 8 — Joint Position Absolute Stats\n');

%% SHEET 9 — Joint Position Range of Motion
rom_header = {'Joint', 'Min [rad]', 'Max [rad]', 'Range [rad]', 'Range [deg]'};
rom_data   = cell(7, 5);
for j = 1:7
    rom_data{j,1} = sprintf('J%d', j);
    rom_data{j,2} = min_pos_joint(j);
    rom_data{j,3} = max_pos_joint(j);
    rom_data{j,4} = range_pos_joint(j);
    rom_data{j,5} = range_pos_joint(j) * (180/pi);
end
writecell([rom_header; rom_data], OUTPUT_FILE, 'Sheet', '9 - Joint Pos Range');
fprintf('Sheet 9 — Joint Position Range of Motion\n');

fprintf('\nExport complete: %s\n', OUTPUT_FILE);


%% SHEET 10 — Per-Segment Steady-State Force & Error
ss_header = {'Segment','Label','SS Force [N]','SS Error [mm]'};
ss_data   = cell(n_segs, 4);
for s = 1:n_segs
    ss_data{s,1} = s;
    ss_data{s,2} = seg_labels{min(s,end)};
    ss_data{s,3} = seg_ss_force(s);
    ss_data{s,4} = seg_ss_error(s) * 1000;
end
writecell([ss_header; ss_data], OUTPUT_FILE, 'Sheet','10 - SS Force Error');