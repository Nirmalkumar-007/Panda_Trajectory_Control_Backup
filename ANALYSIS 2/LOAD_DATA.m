%% Robot Log Loader for Analysis (CSV + Yaml)
% Loads Panda trajectory logs and YAML waypoint config, for computations,
% and prepares data for visualization, and diagnostics. 
%
% INPUTS:
%   - CSV: joint states, EE pose, forces, commands, time
%   - YAML: waypoints, segment definitions, tilt and pause configuration
%
% OUTPUTS:
%   - Waypoint structure and segment labels
%   - Diagnostic summaries (tilt, pauses, trajectory stats)

%% USER CONFIGURATION 
% File paths for trajectory log (CSV) and task definition (YAML).
% External libraries for YAML parsing and colored console output.
CSV_FILE      = 'C:\Users\nravikumar\Desktop\EXPERIMENTS MAY\CSV_logs\BRISTLE BLASTER\EXP_BRISTLE_2.csv';
YAML_FILE     = 'C:\Users\nravikumar\Desktop\EXPERIMENTS MAY\ss_bristleblaster_3.yaml';
YAML_LIB_PATH = 'C:\Users\nravikumar\Desktop\MATLAB SCRIPTS\Libraries\yamlmatlab';
CPRINTF_PATH  = 'C:\Users\nravikumar\Desktop\MATLAB SCRIPTS\Libraries\Cprintf';

addpath(genpath(YAML_LIB_PATH));
addpath(genpath(CPRINTF_PATH));

%% LOAD CSV DATA
% Reads raw logged data and separates it into:
% - time stamps
% - joint states (positions, velocities, torques)
% - end-effector (position, orientation)
% - force/torque measurements
% - commanded trajectory references
raw  = readcell(CSV_FILE);
data = cell2mat(raw(3:end, :));   % skip both header rows

time_log         = data(:,  1);              % raw timestamps - absolute values (unix)
time_rel         = time_log - time_log(1);   % start from t = 0 at the first sample 

joint_positions  = data(:,  2:8);    % joint positions   [rad]
joint_velocities = data(:,  9:15);   % joint velocities  [rad/s] 
joint_torques    = data(:, 16:22);   % joint torques     [Nm]
ee_positions     = data(:, 23:25);   % actual EE position [m]
ee_orientations  = data(:, 26:29);   % actual EE orientation (quaternion), [qx qy qz qw]  — no reorder needed
ee_ft            = data(:, 30:35);   % [fx fy fz tx ty tz]  at EE [N, Nm]
cmd_positions    = data(:, 36:38);   % commanded EE position [m]
s_val            = data(:, 39);      % normalised profile progress (trapezoidal)

N  = size(ee_positions, 1);          % total logged samples
dt = mean(diff(time_rel));           % average sampling time
                                     % mean sample interval [s]

%% EE VELOCITY ESTIMATION
% Computes end-effector velocity from differences of position.
% Velocity is derived per time step using actual logged sampling intervals.
ee_velocity = diff(ee_positions) ./diff(time_rel);  % v=Δp/Δt​
vel_norm    = vecnorm(ee_velocity, 2, 2);
time_vel    = time_rel(1:end-1); % diff() reduces data size by 1: original time: [t1, t2, ..., tN], velocity time: [t1, t2, ..., tN-1]

%% NORMALISED MOTION PROFILE
% Computes rate of progression along trajectory (ds/dt)
ds_dt = gradient(s_val, time_rel); % s_val = normalized path progress (0 → 1)
                                   % ds_dt = speed along the trajectory path

%% LOAD YAML
% Used only for waypoint names, offsets, and segment labels.
YamlStruct = yaml.ReadYaml(YAML_FILE);
wp_cell    = YamlStruct.waypoints;
num_wp     = numel(wp_cell);

WAYPOINT_OFFSETS = zeros(num_wp, 3);
WAYPOINT_NAMES   = cell(num_wp, 1);

for k = 1:num_wp
    if iscell(wp_cell{k}.offset)
        WAYPOINT_OFFSETS(k,:) = cell2mat(wp_cell{k}.offset);
    else
        WAYPOINT_OFFSETS(k,:) = wp_cell{k}.offset;
    end
    WAYPOINT_NAMES{k} = wp_cell{k}.name;
end

num_segments = num_wp - 1;

% Absolute waypoint positions (for plot markers only)
home_pos  = cmd_positions(1,:);
waypoints = home_pos + WAYPOINT_OFFSETS;

% Segment labels e.g. "HOME->A1", "A1->A2" ...
seg_labels = cell(num_segments, 1);
for s = 1:num_segments
    seg_labels{s} = sprintf('%s->%s', WAYPOINT_NAMES{s}, WAYPOINT_NAMES{s+1});
end

%% PAUSE INDICES (from YAML)
pause_indices = [];
if isfield(YamlStruct, 'pause_behavior') && ...
   isfield(YamlStruct.pause_behavior, 'pause_at_indices')
    raw_pause = YamlStruct.pause_behavior.pause_at_indices;
    if iscell(raw_pause)
        pause_indices = cell2mat(raw_pause);
    else
        pause_indices = raw_pause;
    end
end

%%
fprintf('\n');
cprintf('blue', '  LOGGED DATA SUMMARY\n');
fprintf('  Samples         : %d\n',       N);
fprintf('  Duration        : %.2f s\n',   time_rel(end));
fprintf('  Mean dt         : %.4f s  \n', dt);
fprintf('  Segments        : %d\n',       num_segments);

%% LOG SUMMARY - Displays:
% - dataset size
% - duration and sampling rate
% - waypoint sequence
% - pause configuration
% - tilt configuration
fprintf('\n  Waypoint Sequence:   ');
for k = 1:num_wp
    cprintf([0.1 0.6 0.1], '%d (%s)', k-1, WAYPOINT_NAMES{k});   % WP0=HOME, WP1=A1 ...
    if k < num_wp
        fprintf(' -> ');
    end
end
fprintf('\n\n');

if isempty(pause_indices)
    fprintf('  Pause Indices  : none\n');
else
    fprintf('  Pause Indices  : ');
    for k = 1:length(pause_indices)
        yaml_idx  = pause_indices(k);          % 0-based from YAML
        matlab_idx = yaml_idx + 1;             % shift to MATLAB 1-based
        if matlab_idx <= num_wp
            cprintf([0.1 0.6 0.1], '%d (%s)', yaml_idx, WAYPOINT_NAMES{matlab_idx});
        else
            cprintf([0.1 0.6 0.1], '%d', yaml_idx);
        end
        if k < length(pause_indices); fprintf(', '); end
    end
    fprintf('\n');
end

%% TILT STATUS -  displays the tilt status 
% Reports initial and post-waypoint tilt settings applied
% during trajectory execution for process validation.
init = YamlStruct.orientation;

init_vals = [init.tilt_sideways_deg, init.tilt_forward_deg, init.tilt_roll_deg];
init_flag = any(abs(init_vals) > 1e-6);

if init_flag
    fprintf('\n INITIAL TILT: YES\n');
else
    fprintf('\n INITIAL TILT: NO\n');
end

% Printing the initial Tilt applied during the trajectory
fprintf('  Y (sideways): %.2f deg\n', init_vals(1));
fprintf('  X (forward) : %.2f deg\n', init_vals(2));
fprintf('  Z (roll)    : %.2f deg\n', init_vals(3));

post = [];
for k = 1:num_wp
    wp = wp_cell{k};

    % flat format
    if isfield(wp,'post_tilt_sideways_deg')
        post(end+1,:) = [wp.post_tilt_sideways_deg, ...
                         wp.post_tilt_forward_deg, ...
                         wp.post_tilt_roll_deg];
    end

    % nested format
    if isfield(wp,'post_tilt')
        pt = wp.post_tilt;

        if isfield(pt,'sideways_deg')
            post(end+1,:) = [pt.sideways_deg, pt.forward_deg, pt.roll_deg];
        end
    end
end

post_flag = ~isempty(post);

% Printing the post Tilt applied Mid-trajectory
if post_flag
    fprintf('\n POST TILT: YES\n');
    fprintf('  Y (sideways): %s deg\n', mat2str(post(:,1)'));
    fprintf('  X (forward) : %s deg\n', mat2str(post(:,2)'));
    fprintf('  Z (roll)    : %s deg\n', mat2str(post(:,3)'));
else
    fprintf('\n POST TILT: NO\n');
end



