%% End-Effector Tracking Error Analysis
% This section evaluates the performance of the robot trajectory execution
% by comparing commanded references with measured end-effector states.
%
% The analysis includes:
%
% 1. POSITION TRACKING ERROR
%    - Computes deviation between commanded and actual EE position.
%
% 2. ORIENTATION ERROR (QUATERNION-BASED)
%    - Computes angular deviation between actual EE orientation and a reference pose.   
%
% 3. HOMING ERROR
%    - Measures final drift between start and end EE positions.
%
% 4. SEGMENT-WISE TRACKING ERROR
%    - Divides trajectory into motion segments using profile progress (s_val).
%    - Computes RMS, mean, and maximum position error per segment.
%
% 5. PER-AXIS ERROR STATISTICS
%    - Provides detailed X/Y/Z error breakdown for diagnostic insight.
%
% OUTPUT
%    - Console summary of tracking performance
%    - Segment-wise error table
%    - Key metrics for validation and tuning of control performance

%% POSITION ERROR  (commanded vs actual)
cmd_error      = cmd_positions - ee_positions;   % [N x 3]  per-axis [m], e(t) = Pcmd​(t) − Pactual​(t)
cmd_error_norm = vecnorm(cmd_error, 2, 2);       % [N x 1]  norm [m] 

rms_pos_error              = sqrt(mean(cmd_error_norm .^ 2));
mean_pos_error             = mean(cmd_error_norm);
[max_pos_error, idx_max_p] = max(cmd_error_norm);
% max_pos_error -  the largest value in the vector  
% idx_max_p     -  the position (index) where that max occurs 

time_of_max_pos            = time_rel(idx_max_p);



%% EE ORIENTATION DEVIATION FROM HOME

ee_q_ml = ee_orientations(:, [4 1 2 3]);   % [qx qy qz qw] -> [qw qx qy qz]

q_home = ee_q_ml(1,:);   % reference = HOME orientation

ori_error = zeros(N,1);

for i = 1:N

    q_err = quatmultiply(q_home, quatinv(ee_q_ml(i,:)));

    qw = min(max(q_err(1), -1.0), 1.0);

    ori_error(i) = 2 * acos(abs(qw));   % [rad]

end

rms_ori_error              = sqrt(mean(ori_error.^2));
mean_ori_error             = mean(ori_error);

[max_ori_error, idx_max_o] = max(ori_error);
time_of_max_ori            = time_rel(idx_max_o);

% % EE ORIENTATION DEVIATION (vs home)
% % CSV stores [qx qy qz qw].
% % MATLAB quatmultiply / quatinv use [qw qx qy qz] → reorder first.
% 
% ee_q_ml  = ee_orientations(:, [4, 1, 2, 3]);   % [qx qy qz qw] -> [qw qx qy qz]
% qd_fixed = ee_q_ml(1, :);                      % reference = first sample (home)
% qd_rep   = repmat(qd_fixed, N, 1);
% 
% fprintf('  Reference      : initial EE pose (home orientation)\n');
% 
% ori_error = zeros(N, 1);
% % Compute relative rotation - computes how much rotation from current pose to reference
% for i = 1:N
%     q_err        = quatmultiply(qd_rep(i,:), quatinv(ee_q_ml(i,:))); % qerr ​= qref ​⊗ qcurrent−1​
%     qw_clamped   = min(max(q_err(1), -1.0), 1.0);   % clamp for acos safety
%     ori_error(i) = 2 * acos(abs(qw_clamped));       % [rad]
% end
% 
% rms_ori_error              = sqrt(mean(ori_error .^ 2));
% mean_ori_error             = mean(ori_error);
% [max_ori_error, idx_max_o] = max(ori_error);
% time_of_max_ori            = time_rel(idx_max_o);

%% HOMING ERROR  (start vs end actual EE position)
start_point  = ee_positions(1, :);
end_point    = ee_positions(end, :);
delta        = end_point - start_point;
homing_error = norm(delta);

%% PER-SEGMENT TRACKING ERROR
% Segment boundaries detected from s_val resets (accurate, profile-aware).
% Each segment = one motion leg from the YAML waypoint list.

seg_reset    = find(diff(s_val) < -0.05) + 1;    % sample indices where s resets
seg_starts   = [1; seg_reset];                   % start sample of each segment
seg_ends     = [seg_reset - 1; N];               % end   sample of each segment
n_segs_det   = length(seg_starts);               % detected from data

% Use detected count (may differ from YAML if run was partial)
seg_rms  = zeros(n_segs_det, 1);
seg_mean = zeros(n_segs_det, 1);
seg_max  = zeros(n_segs_det, 1);

for s = 1:n_segs_det
    e           = cmd_error_norm(seg_starts(s):seg_ends(s));
    seg_rms(s)  = sqrt(mean(e .^ 2));
    seg_mean(s) = mean(e);
    seg_max(s)  = max(e);
end

% Safe label lookup — fall back to "Seg N" if YAML had fewer entries
% seg_labels comes from YAML (planned trajectory)
% n_segs_det comes from actual data (what really happened)
get_label = @(s) ...
    ternary(s <= length(seg_labels), seg_labels{s}, sprintf('Seg %d', s));


%%
rms_x = sqrt(mean(cmd_error(:,1).^2));
rms_y = sqrt(mean(cmd_error(:,2).^2));
rms_z = sqrt(mean(cmd_error(:,3).^2));

mean_x = mean(cmd_error(:,1));
mean_y = mean(cmd_error(:,2));
mean_z = mean(cmd_error(:,3));

max_x = max(abs(cmd_error(:,1)));
max_y = max(abs(cmd_error(:,2)));
max_z = max(abs(cmd_error(:,3)));

%% CONSOLE REPORT
fprintf('\n');
cprintf('red', '     TRAJECTORY ERROR ANALYSIS:\n');

fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '    EE POSITION TRACKING METRICS\n');
fprintf('══════════════════════════════════════════════════\n');
fprintf('  Reference      : cmd_positions (logged setpoint)\n');
fprintf('  RMS Error      : %8.6f  m\n', rms_pos_error);
fprintf('  Mean Error     : %8.6f  m\n', mean_pos_error);
fprintf('  Max Error      : %8.6f  m\n', max_pos_error);
fprintf('  Time of Max    : %8.3f  s\n', time_of_max_pos);
fprintf('══════════════════════════════════════════════════\n\n');

fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '    EE ORIENTATION TRACKING METRICS\n');
fprintf('══════════════════════════════════════════════════\n');
fprintf('  Reference      : initial EE pose (home)\n');
fprintf('  RMS Error      : %8.6f  rad\n', rms_ori_error);
fprintf('  Mean Error     : %8.6f  rad\n', mean_ori_error);
fprintf('  Max Error      : %8.6f  rad\n', max_ori_error);
fprintf('  Time of Max    : %8.3f  s\n',   time_of_max_ori);
fprintf('══════════════════════════════════════════════════\n\n');

fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '        EE HOMING ERROR\n');
fprintf('══════════════════════════════════════════════════\n');
fprintf('  Start          : [%.4f, %.4f, %.4f] m\n', start_point);
fprintf('  End            : [%.4f, %.4f, %.4f] m\n', end_point);
fprintf('  dX             : %.6f m\n', delta(1));
fprintf('  dY             : %.6f m\n', delta(2));
fprintf('  dZ             : %.6f m\n', delta(3));
fprintf('  3D Distance    : %.6f m\n', homing_error);
fprintf('══════════════════════════════════════════════════\n\n');

fprintf('══════════════════════════════════════════════════════════════════\n');
cprintf('blue', '          PER-SEGMENT TRACKING ERROR\n');
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('  Seg | Waypoints            |  RMS [m]  |  Mean [m] |  Max [m]\n');
fprintf('  ------------------------------------------------------------\n');
for s = 1:n_segs_det
    lbl = get_label(s);
    fprintf('  %3d | %-20s | %9.6f | %9.6f | %9.6f\n', ...
            s, lbl, seg_rms(s), seg_mean(s), seg_max(s));
end
fprintf('══════════════════════════════════════════════════════════════════\n\n');

%%
fprintf('══════════════════════════════════════════════════\n');
cprintf('blue', '    PER-AXIS POSITION ERROR\n');
fprintf('══════════════════════════════════════════════════\n');

fprintf('        |     X          Y          Z\n');
fprintf('  RMS   | %8.6f %8.6f   %8.6f m\n',   rms_x, rms_y, rms_z);
fprintf('  Mean  | %8.6f %8.6f   %8.6f m\n',  mean_x, mean_y, mean_z);
fprintf('  Max   | %8.6f %8.6f   %8.6f m\n',   max_x, max_y, max_z);

fprintf('══════════════════════════════════════════════════\n\n');

%%
% RMS  ≈ Mean  → smooth consistent impedance lag, no spikes
% RMS  >> Mean → transient spikes present (check stiffness / acceleration)
% Max  ≈ RMS   → no dangerous single peaks
% Max  >> RMS  → single large overshoot event (check segment transitions)

%%   LOCAL HELPER 
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end