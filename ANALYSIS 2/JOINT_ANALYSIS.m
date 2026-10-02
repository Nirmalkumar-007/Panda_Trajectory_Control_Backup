%% JOINT PERFORMANCE ANALYSIS 
% This section evaluates robot joint-level behavior during trajectory execution
% using torque, velocity, jerk, and position statistics.
%
% The analysis includes:
%
% 1. TORQUE ANALYSIS
%    - Computes RMS, mean absolute, and maximum torque per joint.
%    
% 2. VELOCITY ANALYSIS
%    - Evaluates joint motion speed using RMS, mean absolute, and max values.
%  
% 3. JERK ANALYSIS (MOTION SMOOTHNESS)
%    - Approximates jerk (third derivative of position) using finite differences.
%   
% 4. POSITION / RANGE OF MOTION (ROM)
%    - Computes absolute position statistics (RMS, mean, max).
%    
% OUTPUT
%    - Per-joint performance tables (7 DOF robot)
%    - Torque, velocity, jerk, and ROM diagnostics
%    - Indicators for controller aggressiveness and mechanical loading

%% TORQUE PER JOINT
rms_torque  = sqrt(mean(joint_torques .^ 2, 1));   % [1x7]
mean_torque = mean(abs(joint_torques),      1);
max_torque  = max (abs(joint_torques),  [], 1);

%% VELOCITY PER JOINT
rms_vel_joint      = sqrt(mean(joint_velocities .^ 2, 1));
mean_abs_vel_joint = mean(abs(joint_velocities),      1);
max_vel_joint      = max (abs(joint_velocities),  [], 1);

%% JERK PER JOINT
% Jerk = d³q/dt³  computed by double-differencing velocity
dt_vec      = diff(time_rel);  % [N-1 x 1]
joint_accel = diff(joint_velocities) ./ dt_vec;   % [rad/s²]
joint_jerk  = diff(joint_accel)      ./ dt_vec(1:end-1);   % [rad/s³]
time_jerk   = time_rel(1:end-2);

rms_jerk      = sqrt(mean(joint_jerk .^ 2, 1));
mean_abs_jerk = mean(abs(joint_jerk),      1);
max_jerk      = max (abs(joint_jerk),  [], 1);

%% JOINT POSITION — Absolute stats + Range of Motion
rms_pos_joint      = sqrt(mean(joint_positions .^ 2, 1));
mean_abs_pos_joint = mean(abs(joint_positions),      1);
max_abs_pos_joint  = max (abs(joint_positions),  [], 1);

range_pos_joint   = max(joint_positions) - min(joint_positions);   % [rad]
min_pos_joint     = min(joint_positions);
max_pos_joint     = max(joint_positions);

%% CONSOLE REPORT

cprintf('red', 'TRAJECTORY — JOINT PERFORMANCE ANALYSIS:\n');

fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '      TORQUE METRICS PER JOINT [Nm]\n');
fprintf('══════════════════════════════════════════════════\n');
fprintf('  JOINT |   RMS   |   Mean  |   Max\n');
fprintf('  -----------------------------------\n');
for j = 1:7
    fprintf('  J%d    | %7.4f | %7.4f | %7.4f\n', ...
            j, rms_torque(j), mean_torque(j), max_torque(j));
end
fprintf('══════════════════════════════════════════════════\n\n');

fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '    VELOCITY METRICS PER JOINT [rad/s]\n');
fprintf('══════════════════════════════════════════════════\n');
fprintf('  JOINT |   RMS   |  MeanAbs |   Max\n');
fprintf('  -----------------------------------\n');
for j = 1:7
    fprintf('  J%d    | %7.4f | %7.4f  | %7.4f\n', ...
            j, rms_vel_joint(j), mean_abs_vel_joint(j), max_vel_joint(j));
end
fprintf('══════════════════════════════════════════════════\n\n');

% fprintf('══════════════════════════════════════════════════\n');
% cprintf('blue', '     JERK METRICS PER JOINT [rad/s^3]\n');
% fprintf('══════════════════════════════════════════════════\n');
% fprintf('  JOINT |    RMS     |  MeanAbs   |   Max\n');
% fprintf('  ----------------------------------------\n');
% for j = 1:7
%     fprintf('  J%d    | %8.4f  | %8.4f  | %8.4f\n', ...
%             j, rms_jerk(j), mean_abs_jerk(j), max_jerk(j));
% end

fprintf('══════════════════════════════════════\n');
cprintf('blue', '     JERK METRICS PER JOINT [rad/s^3]\n');
fprintf('══════════════════════════════════════\n');
fprintf('  JOINT |    RMS     |  MeanAbs\n');
fprintf('  ----------------------------------\n');

for j = 1:7
    % Removed max_jerk(j) and the third formatting placeholder
    fprintf('  J%d    | %8.4f  | %8.4f\n', ...
            j, rms_jerk(j), mean_abs_jerk(j));
end

fprintf('══════════════════════════════════════════════════\n\n');

fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '  JOINT POSITION — ABSOLUTE STATS [rad]\n');
fprintf('══════════════════════════════════════════════════\n');
fprintf('  JOINT |   RMS   |  MeanAbs |   Max\n');
fprintf('  -----------------------------------\n');
for j = 1:7
    fprintf('  J%d    | %7.4f | %7.4f  | %7.4f\n', ...
            j, rms_pos_joint(j), mean_abs_pos_joint(j), max_abs_pos_joint(j));
end
fprintf('══════════════════════════════════════════════════\n\n');

fprintf('══════════════════════════════════════════════════════════════\n');
cprintf('blue', '   JOINT POSITION — RANGE OF MOTION\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  JOINT |  Min [rad] |  Max [rad] | Range[rad] | Range [deg]\n');
fprintf('  ------------------------------------------------------------\n');
for j = 1:7
    fprintf('  J%d    |   %7.4f  |   %7.4f  |   %7.4f  |   %7.3f\n', ...
            j, min_pos_joint(j), max_pos_joint(j), ...
            range_pos_joint(j), range_pos_joint(j)*(180/pi));
end
fprintf('══════════════════════════════════════════════════════════════\n\n');

% Torque  RMS >> Mean  → torque spikes, check stiffness or fast transitions
% Jerk    high values  → abrupt velocity changes, reduce s_ddot_max in YAML
% ROM     small range  → joint barely moved (verify waypoint offsets)


 