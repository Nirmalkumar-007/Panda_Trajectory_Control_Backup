%% SY_EVALUATION.m
% Additional visualization for detailed evaluation
% Fixes applied:
%   - B14: updatePlotsV2 typo corrected (was updaCtePlots)
%   - Contact replaced with EE Speed (vel_norm) in both dashboards
%   - runPlayback correctly calls updatePlotsV2
%   - Spike merge guard added (B12)
%   - Both dashboards are fully independent

%% DASHBOARD 1: TIME-SYNC (slider only) 

fig1 = figure('Name','Robot + Force/Torque Dashboard','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

%% PLOTS 

% Force
ax1 = nexttile;
hF = plot(nan,nan,'b','LineWidth',1.2);
ylabel('Force [N]');
grid on; title('Force Norm');

% EE Speed  (replaces Contact)
ax2 = nexttile;
hC = plot(nan,nan,'m','LineWidth',1.2);
ylabel('Speed [m/s]');
grid on; title('EE Speed');

% Torque
ax3 = nexttile;
hT = plot(nan,nan,'r','LineWidth',1.2);
ylabel('Torque [Nm]');
grid on; title('Torque Norm');

% Robot (4th tile)
ax4 = nexttile;
hold(ax4,'on'); grid(ax4,'on'); axis(ax4,'equal');
view(ax4,45,30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Playback');

robot = loadrobot('frankaEmikaPanda','DataFormat','row');
robot.Gravity = [0 0 -9.81];

% Fix joints (7 → 9)
joint_positions9 = zeros(length(time_rel),9);
joint_positions9(:,1:7) = joint_positions;

% Initial robot pose
show(robot, joint_positions9(1,:), ...
    'PreservePlot',false,'Frames','off','Visuals','on','Parent',ax4);
camlight; lighting gouraud;

% EE point marker
h_point = plot3(ax4, ...
    ee_positions(1,1), ee_positions(1,2), ee_positions(1,3), ...
    'ro','MarkerFaceColor','r','MarkerSize',8);

% Trajectory trail
h_traj = plot3(ax4, nan,nan,nan, 'b','LineWidth',1.5);

%%  SLIDER 

sld1 = uicontrol('Style','slider', ...
    'Min',  time_rel(1), ...
    'Max',  time_rel(end), ...
    'Value',time_rel(min(5,end)), ...
    'Units','normalized', ...
    'Position',[0.2 0.01 0.55 0.03]);

uicontrol('Style','text', ...
    'Units','normalized', ...
    'Position',[0.2 0.045 0.55 0.03], ...
    'String','Time [s]', ...
    'BackgroundColor','w');

txt_time1 = uicontrol('Style','text', ...
    'Units','normalized', ...
    'Position',[0.77 0.01 0.2 0.035], ...
    'String',sprintf('t = %.2f s', time_rel(1)), ...
    'BackgroundColor','w', ...
    'FontWeight','bold');

%%  STORE 

h1.time_rel       = time_rel;
h1.force_norm     = force_norm;
h1.torque_norm    = torque_norm;
h1.vel_norm       = vel_norm;      % EE speed  [N-1 x 1]
h1.time_vel       = time_vel;      % matching time axis [N-1 x 1]
h1.robot          = robot;
h1.joint_positions9 = joint_positions9;
h1.ee_positions   = ee_positions;
h1.ax4            = ax4;
h1.hF             = hF;
h1.hC             = hC;
h1.hT             = hT;
h1.h_point        = h_point;
h1.h_traj         = h_traj;
h1.txt_time       = txt_time1;

guidata(fig1, h1);

%%  CALLBACK 

sld1.Callback = @(src,~) updatePlots(fig1, src.Value);

%%  UPDATE FUNCTION (Dashboard 1) 

function updatePlots(fig, tmax)

    h = guidata(fig);
    t = h.time_rel;

    idx = find(t <= tmax);
    if isempty(idx); return; end
    i = idx(end);

    % Force and Torque
    set(h.hF, 'XData', t(idx),         'YData', h.force_norm(idx));
    set(h.hT, 'XData', t(idx),         'YData', h.torque_norm(idx));

    % EE Speed — uses time_vel (N-1 length), separate index
    idx_vel = find(h.time_vel <= tmax);
    if ~isempty(idx_vel)
        set(h.hC, 'XData', h.time_vel(idx_vel), 'YData', h.vel_norm(idx_vel));
    end

    % Robot pose
    show(h.robot, h.joint_positions9(i,:), ...
        'PreservePlot',false,'Frames','off','Visuals','on','Parent',h.ax4);

    % Transparency
    patches = findobj(h.ax4,'Type','Patch');
    for p = 1:length(patches)
        patches(p).FaceAlpha = 0.4;
    end

    % EE moving point
    set(h.h_point, ...
        'XData', h.ee_positions(i,1), ...
        'YData', h.ee_positions(i,2), ...
        'ZData', h.ee_positions(i,3));

    % Trajectory trail
    set(h.h_traj, ...
        'XData', h.ee_positions(idx,1), ...
        'YData', h.ee_positions(idx,2), ...
        'ZData', h.ee_positions(idx,3));

    % Time display
    h.txt_time.String = sprintf('t = %.2f s', tmax);

    drawnow;
end


%%  DASHBOARD 2: FULL PLAYBACK (play/pause + speed + auto-stop) 

fig2 = figure('Name','Robot Replay Dashboard','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

%%  PLOTS 

% Force
ax1 = nexttile;
hF2 = plot(nan,nan,'b','LineWidth',1.2);
ylabel('Force [N]');
grid on; title('Force Norm');

% EE Speed  (replaces Contact)
ax2 = nexttile;
hC2 = plot(nan,nan,'m','LineWidth',1.2);
ylabel('Speed [m/s]');
grid on; title('EE Speed');

% Torque
ax3 = nexttile;
hT2 = plot(nan,nan,'r','LineWidth',1.2);
ylabel('Torque [Nm]');
grid on; title('Torque Norm');

% Robot
ax4b = nexttile;
hold(ax4b,'on'); grid(ax4b,'on'); axis(ax4b,'equal');
view(ax4b,45,30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Playback');

robot2 = loadrobot('frankaEmikaPanda','DataFormat','row');
robot2.Gravity = [0 0 -9.81];

joint_positions9b = zeros(length(time_rel),9);
joint_positions9b(:,1:7) = joint_positions;

show(robot2, joint_positions9b(1,:), ...
    'PreservePlot',false,'Frames','off','Visuals','on','Parent',ax4b);
camlight; lighting gouraud;

h_point2 = plot3(ax4b, ...
    ee_positions(1,1), ee_positions(1,2), ee_positions(1,3), ...
    'ro','MarkerFaceColor','r','MarkerSize',8);

h_traj2 = plot3(ax4b, nan,nan,nan, 'b','LineWidth',1.5);

%%  SLIDER 

sld2 = uicontrol('Style','slider', ...
    'Min',  time_rel(1), ...
    'Max',  time_rel(end), ...
    'Value',time_rel(min(5,end)), ...
    'Units','normalized', ...
    'Position',[0.2 0.01 0.55 0.03]);

uicontrol('Style','text', ...
    'Units','normalized', ...
    'Position',[0.2 0.045 0.55 0.03], ...
    'String','Time [s]', ...
    'BackgroundColor','w');

txt_time2 = uicontrol('Style','text', ...
    'Units','normalized', ...
    'Position',[0.77 0.01 0.2 0.035], ...
    'String',sprintf('t = %.2f s', time_rel(1)), ...
    'BackgroundColor','w', ...
    'FontWeight','bold');

%%  CONTROLS 

btn_play = uicontrol('Style','togglebutton', ...
    'String','▶ Play / ‖ Pause', ...
    'Units','normalized', ...
    'Position',[0.05 0.01 0.1 0.04]);

spd = uicontrol('Style','slider', ...
    'Min',0.1,'Max',3,'Value',1, ...
    'Units','normalized', ...
    'Position',[0.05 0.07 0.1 0.02]);

uicontrol('Style','text', ...
    'Units','normalized', ...
    'Position',[0.05 0.09 0.1 0.02], ...
    'String','Speed', ...
    'BackgroundColor','w');

chk_spike = uicontrol('Style','checkbox', ...
    'String','Auto-stop spike', ...
    'Value',0, ...
    'Units','normalized', ...
    'Position',[0.87 0.05 0.12 0.03], ...
    'BackgroundColor','w');

%%  STORE 

h2.time_rel         = time_rel;
h2.force_norm       = force_norm;
h2.torque_norm      = torque_norm;
h2.vel_norm         = vel_norm;     % EE speed  [N-1 x 1]
h2.time_vel         = time_vel;     % matching time axis [N-1 x 1]
h2.robot            = robot2;
h2.joint_positions9 = joint_positions9b;
h2.ee_positions     = ee_positions;
h2.ax4              = ax4b;
h2.hF               = hF2;
h2.hC               = hC2;
h2.hT               = hT2;
h2.h_point          = h_point2;
h2.h_traj           = h_traj2;
h2.txt_time         = txt_time2;
h2.btn_play         = btn_play;
h2.spd              = spd;
h2.chk_spike        = chk_spike;

guidata(fig2, h2);

%%  CALLBACKS 

sld2.Callback      = @(src,~) updatePlotsV2(fig2, src.Value);
btn_play.Callback  = @(src,~) runPlayback(fig2, sld2);

%%  UPDATE FUNCTION (Dashboard 2) — B14 fix: was updaCtePlots 

function updatePlotsV2(fig, tmax)

    h = guidata(fig);
    t = h.time_rel;

    idx = find(t <= tmax);
    if isempty(idx); return; end
    i = idx(end);

    % Force and Torque
    set(h.hF, 'XData', t(idx),         'YData', h.force_norm(idx));
    set(h.hT, 'XData', t(idx),         'YData', h.torque_norm(idx));

    % EE Speed — uses time_vel (N-1 length), separate index
    idx_vel = find(h.time_vel <= tmax);
    if ~isempty(idx_vel)
        set(h.hC, 'XData', h.time_vel(idx_vel), 'YData', h.vel_norm(idx_vel));
    end

    % Robot pose
    show(h.robot, h.joint_positions9(i,:), ...
        'PreservePlot',false,'Frames','off','Visuals','on','Parent',h.ax4);

    % Transparency
    patches = findobj(h.ax4,'Type','Patch');
    for p = 1:length(patches)
        patches(p).FaceAlpha = 0.4;
    end

    % EE moving point
    set(h.h_point, ...
        'XData', h.ee_positions(i,1), ...
        'YData', h.ee_positions(i,2), ...
        'ZData', h.ee_positions(i,3));

    % Trajectory trail
    set(h.h_traj, ...
        'XData', h.ee_positions(idx,1), ...
        'YData', h.ee_positions(idx,2), ...
        'ZData', h.ee_positions(idx,3));

    % Time display
    h.txt_time.String = sprintf('t = %.2f s', tmax);

    drawnow;
end

%%  PLAYBACK ENGINE 

function runPlayback(fig, sld)

    h = guidata(fig);

    while h.btn_play.Value == 1

        tnow  = sld.Value;
        speed = h.spd.Value;

        dt_step = (h.time_rel(end) - h.time_rel(1)) / 500;
        tnew    = tnow + dt_step * speed;

        if tnew >= h.time_rel(end)
            tnew = h.time_rel(end);
            h.btn_play.Value = 0;
        end

        sld.Value = tnew;
        updatePlotsV2(fig, tnew);   % B14 fix: was updatePlots (called wrong dashboard)

        % Auto-stop on force spike
        if h.chk_spike.Value == 1
            idx = find(h.time_rel <= tnew, 1, 'last');
            if h.force_norm(idx) > 15
                h.btn_play.Value = 0;
                break;
            end
        end

        pause(0.02);
        h = guidata(fig);   % refresh handles each loop (speed/checkbox may change)
    end
end