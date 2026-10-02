%% Force / Torque Analysis and Contact Diagnostics
% This module analyzes external forces and torques measured at the robot
% end-effector during trajectory execution.
%
% It provides a full physical and task-level evaluation of interaction
% behavior, including contact detection, disturbance events, and
% steady-state performance per trajectory segment.
%
% The analysis includes:
%
% 1. FORCE / TORQUE OVERVIEW
%    - Computes per-axis and norm-based statistics (RMS, mean, max)
%    
% 2. CONTACT DETECTION
%    - Detects physical contact using a force norm threshold
%    
% 3. DISTURBANCE / SPIKE DETECTION
%    - Identifies sudden force peaks exceeding a higher threshold
%    
% 4. PER-SEGMENT STEADY-STATE ANALYSIS
%    - Divides trajectory into motion segments using profile progress (s_val)
%    
% 5. VISUALIZATION
%    - Force/torque time series
%    - Contact and spike event highlighting
%    - Per-segment steady-state comparison plots
%
% OUTPUTS
%    - Force and torque statistics (per axis and norm-based)
%    - Contact event timeline and duration
%    - Disturbance/spike detection summary
%    - Steady-state performance per trajectory segment

%% 
if ~exist('ee_ft','var') || ~exist('cmd_error_norm','var')
    error('Run Load_Data.m and EE_Error_Analysis.m first.');
end


%% SETTINGS  
CONTACT_FORCE_THRESHOLD  = 10.0;    % [N]   force norm above = in contact
SPIKE_FORCE_THRESHOLD    = 15.0;   % [N]   force norm above = disturbance spike (must be > CONTACT_FORCE_THRESHOLD)
SPIKE_MIN_SEPARATION_S   = 0.5;    % [s]   min time between separate spike events
SS_FRACTION              = 0.20;   % last 20% of segment = steady-state window

%% DATA SANITY CHECK
fprintf('\n');
cprintf('red', 'TRAJECTORY FORCE ANALYSIS:\n');
fprintf('\n');
cprintf('blue',' F/T data check:\n');
fprintf('  ee_ft size : %d x %d\n', size(ee_ft,1), size(ee_ft,2));
fprintf('  Fx  range  : [%.4f,  %.4f] N\n',  min(ee_ft(:,1)), max(ee_ft(:,1)));
fprintf('  Fy  range  : [%.4f,  %.4f] N\n',  min(ee_ft(:,2)), max(ee_ft(:,2)));
fprintf('  Fz  range  : [%.4f,  %.4f] N\n',  min(ee_ft(:,3)), max(ee_ft(:,3)));
fprintf('  Tx  range  : [%.4f,  %.4f] Nm\n', min(ee_ft(:,4)), max(ee_ft(:,4)));
fprintf('  Ty  range  : [%.4f,  %.4f] Nm\n', min(ee_ft(:,5)), max(ee_ft(:,5)));
fprintf('  Tz  range  : [%.4f,  %.4f] Nm\n', min(ee_ft(:,6)), max(ee_ft(:,6)));
fprintf('\n');
%% 
force_xyz  = ee_ft(:, 1:3);
torque_xyz = ee_ft(:, 4:6);

force_norm  = vecnorm(force_xyz,  2, 2);
torque_norm = vecnorm(torque_xyz, 2, 2);

%% CONTACT DETECTION
in_contact   = force_norm >= CONTACT_FORCE_THRESHOLD;
contact_on   = find(diff([0;        in_contact]) ==  1);
contact_off  = find(diff([in_contact; 0       ]) == -1);

n_contact_events   = length(contact_on);
contact_durations  = time_rel(contact_off) - time_rel(contact_on);
total_contact_time = sum(contact_durations);

%% SPIKE DETECTION
is_spike  = force_norm >= SPIKE_FORCE_THRESHOLD;
spike_on  = find(diff([0;       is_spike]) ==  1);
spike_off = find(diff([is_spike; 0      ]) == -1);

% Merge spikes closer than SPIKE_MIN_SEPARATION_S
if length(spike_on) > 1
    gap   = time_rel(spike_on(2:end)) - time_rel(spike_off(1:end-1));
    merge = gap < SPIKE_MIN_SEPARATION_S;
    spike_on( [false; merge]) = [];
    spike_off([merge; false]) = [];
end

% Assign count AFTER merge so it reflects the final spike list
n_spikes = length(spike_on);

%% PER-SEGMENT STEADY-STATE FORCE & ERROR
seg_reset  = find(diff(s_val) < -0.05) + 1;
seg_starts = [1; seg_reset];
seg_ends   = [seg_reset - 1; N];
n_segs     = length(seg_starts);

seg_ss_force = zeros(n_segs, 1);
seg_ss_error = zeros(n_segs, 1);

for s = 1:n_segs
    idx             = seg_starts(s):seg_ends(s);
    ss              = round((1 - SS_FRACTION) * length(idx));
    seg_ss_force(s) = mean(force_norm(idx(ss:end)));
    seg_ss_error(s) = mean(cmd_error_norm(idx(ss:end)));
end

%% PER-AXIS STATS
f_max_ax  = max(abs(force_xyz),  [], 1);   % [Fx Fy Fz] peak absolute [N]
t_max_ax  = max(abs(torque_xyz), [], 1);   % [Tx Ty Tz] peak absolute [Nm]
f_rms_ax  = sqrt(mean(force_xyz  .^ 2, 1));
t_rms_ax  = sqrt(mean(torque_xyz .^ 2, 1));
f_mean_ax = mean(abs(force_xyz),  1);
t_mean_ax = mean(abs(torque_xyz), 1);

ax_labels  = {'X', 'Y', 'Z'};
[~, f_dom] = max(f_max_ax);
[~, t_dom] = max(t_max_ax);


%% CONSOLE REPORT
fprintf('════════════════════════════════\n');
cprintf('blue','    FORCE / TORQUE OVERVIEW\n');
fprintf('════════════════════════════════\n');
fprintf('  NORM SUMMARY\n');
fprintf('  Max Force Norm   : %.4f N\n',  max(force_norm));
fprintf('  Mean Force Norm  : %.4f N\n',  mean(force_norm));
fprintf('  RMS Force Norm   : %.4f N\n',  sqrt(mean(force_norm.^2)));
fprintf('  Max Torque Norm  : %.4f Nm\n', max(torque_norm));
fprintf('  Mean Torque Norm : %.4f Nm\n', mean(torque_norm));
fprintf('  RMS Torque Norm  : %.4f Nm\n', sqrt(mean(torque_norm.^2)));
fprintf('════════════════════════════════\n');
fprintf('\n');
%%
fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  PER-AXIS FORCE  [N]\n');
fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  Axis |   Peak   |   RMS    |   Mean\n');
fprintf('  ------------------------------------\n');
for k = 1:3
    if k == f_dom
        fprintf('  %s    | %8.4f | %8.4f | %8.4f   <- dominant\n', ...
                ax_labels{k}, f_max_ax(k), f_rms_ax(k), f_mean_ax(k));
    else
        fprintf('  %s    | %8.4f | %8.4f | %8.4f\n', ...
                ax_labels{k}, f_max_ax(k), f_rms_ax(k), f_mean_ax(k));
    end
end
fprintf('═════════════════════════════════════════════════════════\n');

fprintf('\n');

%%
fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  PER-AXIS TORQUE  [Nm]\n');
fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  Axis |   Peak   |   RMS    |   Mean\n');
fprintf('  ------------------------------------\n');
for k = 1:3
    if k == t_dom
        fprintf('  %s    | %8.4f | %8.4f | %8.4f   <- dominant\n', ...
                ax_labels{k}, t_max_ax(k), t_rms_ax(k), t_mean_ax(k));
    else
        fprintf('  %s    | %8.4f | %8.4f | %8.4f\n', ...
                ax_labels{k}, t_max_ax(k), t_rms_ax(k), t_mean_ax(k));
    end
end
fprintf('═════════════════════════════════════════════════════════\n\n');
fprintf('\n');

%%
fprintf('═════════════════════════════════════════════════════════\n');
cprintf('blue','    CONTACT DETECTION  (threshold = %.1f N)\n', CONTACT_FORCE_THRESHOLD);
fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  Contact Events   : %d\n',     n_contact_events);
fprintf('  Total Contact    : %.3f s\n', total_contact_time);
if n_contact_events > 0
    fprintf('  Event | t_start [s] | t_end [s] | Duration [s]\n');
    fprintf('  -----------------------------------------------\n');
    for k = 1:n_contact_events
        fprintf('  %5d | %10.3f | %9.3f | %12.3f\n', k, ...
                time_rel(contact_on(k)), time_rel(contact_off(k)), contact_durations(k));
    end
end
fprintf('═════════════════════════════════════════════════════\n\n');
fprintf('\n');

%%
fprintf('═════════════════════════════════════════════════════\n');
cprintf('blue','    DISTURBANCE EVENTS  (threshold = %.1f N)\n', SPIKE_FORCE_THRESHOLD);
fprintf('═════════════════════════════════════════════════════\n');
fprintf('  Spike Events     : %d\n', n_spikes);
if n_spikes > 0
    fprintf('  Event | t_start [s] | Peak Force [N]\n');
    fprintf('  ------------------------------------\n');
    for k = 1:n_spikes
        idx_win = spike_on(k):spike_off(k);
        pk      = max(force_norm(idx_win));
        fprintf('  %5d | %10.3f  | %14.4f\n', k, time_rel(spike_on(k)), pk);
    end
end
fprintf('═════════════════════════════════════════════════════\n\n');
fprintf('\n');

%%
fprintf('════════════════════════════════════════════════════════════\n');
cprintf('blue','    PER-SEGMENT STEADY-STATE  (last %.0f%% of each segment)\n', SS_FRACTION*100);
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  Seg | Label                | SS Force [N] | SS Error [mm]\n');
fprintf('  ----------------------------------------------------------\n');
for s = 1:n_segs
    lbl = '';
    if s <= length(seg_labels); lbl = seg_labels{s}; end
    fprintf('  %3d | %-20s | %12.4f | %13.4f\n', ...
            s, lbl, seg_ss_force(s), seg_ss_error(s)*1000);
end
fprintf('════════════════════════════════════════════════════════════\n\n');

% SS Error small and stable             -  holding well, stiffness appropriate
% SS Error large, SS Force also large   -  expected impedance compliance under contact
% SS Error large, SS Force near zero    -  controller lag or segment too short or the robot never actually reached the waypoint
% SS Error varies across segments       -  load condition changes between waypoints
% Impedance law: e_ss = F / K

%%  LOCAL HELPER — shade time regions (compatible with all MATLAB versions)

function shade_regions(ax, t_on, t_off, time_rel, ylims, fc, alpha)
% Draws filled patches between t_on and t_off on axes ax.
    for k = 1:length(t_on)
        t1 = time_rel(t_on(k));
        t2 = time_rel(t_off(k));
        patch(ax, [t1 t2 t2 t1], [ylims(1) ylims(1) ylims(2) ylims(2)], ...
              fc, 'FaceAlpha', alpha, 'EdgeColor','none', 'HandleVisibility','off');
    end
end

%% PLOTS
% Fig 1 — Force & Torque channels over time
figure('Name','Force Torque Overview');
ft_lbl   = {'Fx [N]','Fy [N]','Fz [N]','Tx [Nm]','Ty [Nm]','Tz [Nm]'};
ft_color = {[0.8 0.1 0.1],[0.1 0.7 0.1],[0.1 0.1 0.8], ...
            [0.8 0.5 0.1],[0.5 0.1 0.8],[0.1 0.6 0.7]};
for k = 1:6
    subplot(6,1,k);
    plot(time_rel, ee_ft(:,k),'Color',ft_color{k},'LineWidth',1.1); hold on;
    yline(0,'k--','LineWidth',0.7,'HandleVisibility','off');
    ylabel(ft_lbl{k}); grid on; xlim([0 time_rel(end)]);
    if k == 1; title('External Force / Torque at End-Effector'); end
end
xlabel('Time [s]');

%% Fig 2 — Force Norm + Contact Detection
figure('Name','Contact Detection');

ax1 = subplot(2,1,1);
plot(time_rel, force_norm,'Color',[0.15 0.40 0.80],'LineWidth',1.4); hold on;
yline(CONTACT_FORCE_THRESHOLD,'r--','LineWidth',1.2, ...
      'DisplayName',sprintf('Threshold (%.1f N)',CONTACT_FORCE_THRESHOLD));
ylim('auto'); yl = ylim;
shade_regions(ax1, contact_on, contact_off, time_rel, yl, [1 0.6 0], 0.25);
ylabel('Force Norm [N]'); title('Force Norm and Contact Detection');
legend('Location','best'); grid on; xlim([0 time_rel(end)]);

ax2 = subplot(2,1,2);
plot(time_rel, double(in_contact),'Color',[1 0.6 0],'LineWidth',1.5);
yl2 = [-0.1 1.4];
shade_regions(ax2, contact_on, contact_off, time_rel, yl2, [1 0.6 0], 0.30);
ylabel('In Contact'); ylim(yl2);
yticks([0 1]); yticklabels({'No','Yes'});
xlabel('Time [s]'); grid on; xlim([0 time_rel(end)]);
title(sprintf('Contact Flag  (%d events  |  %.2f s total)', ...
      n_contact_events, total_contact_time));

%% Fig 3 — Disturbance / Spike Events
figure('Name','Disturbance Events');
ax3 = axes;
plot(time_rel, force_norm,'Color',[0.15 0.40 0.80],'LineWidth',1.3); hold on;
yline(SPIKE_FORCE_THRESHOLD,'r--','LineWidth',1.2, ...
      'DisplayName',sprintf('Spike threshold (%.1f N)',SPIKE_FORCE_THRESHOLD));
ylim('auto'); yl3 = ylim;
shade_regions(ax3, spike_on, spike_off, time_rel, yl3, [0.9 0.1 0.1], 0.25);
for k = 1:n_spikes
    idx_win    = spike_on(k):spike_off(k);
    [pk, pk_i] = max(force_norm(idx_win));
    pk_t       = time_rel(spike_on(k) + pk_i - 1);
    text(pk_t, pk+0.3, sprintf('%.1f N',pk),'FontSize',8,'Color',[0.7 0 0], ...
         'HorizontalAlignment','center','HandleVisibility','off');
end
ylabel('Force Norm [N]');
title(sprintf('Disturbance Events  (%d spikes detected)', n_spikes));
legend('Location','best'); grid on; xlim([0 time_rel(end)]);
xlabel('Time [s]');

%% Fig 4 — Per-Segment Steady-State Force & Error
figure('Name','Per-Segment Steady-State');
x = 1:n_segs;
yyaxis left;
bar(x-0.2, seg_ss_force, 0.35,'FaceColor',[0.15 0.40 0.80],'DisplayName','SS Force [N]');
ylabel('Steady-State Force [N]');
yyaxis right;
bar(x+0.2, seg_ss_error*1000, 0.35,'FaceColor',[0.85 0.20 0.20],'DisplayName','SS Error [mm]');
ylabel('Steady-State Position Error [mm]');
if n_segs <= length(seg_labels)
    set(gca,'XTick',x,'XTickLabel',seg_labels(1:n_segs),'XTickLabelRotation',15);
end
title('Per-Segment Steady-State Force and Position Error');
legend('Location','best'); grid on;

% SS_Error is small and stable                  -   Robot is holding well, stiffness is appropriate
% SS Error is large but SS Force is also large  -   Expected — robot is pushing against something, impedance is doing its job
% SS Error is large but SS Force is near zero   -   Problem — controller lag, wrong gains, or the robot never actually reached the waypoint
% SS Error varies a lot across segments         -   Load condition changes
% impedance law e = F/K:

