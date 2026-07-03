%% =========================================================================
%  pid_step_response_demo.m — PID Tuning Step Response Demo
%  =========================================================================
%  HOW TO RUN:
%    >> pid_step_response_demo
%
%  Standalone script — no other files needed.
%  Shows 4 subplots:
%    1. P vs PD vs PID comparison
%    2. Tracking error over time
%    3. Kp gain sweep
%    4. Performance metrics bar chart
% =========================================================================

clc; close all;
fprintf('=== PID Step Response Demo ===\n\n');

%% ── Setup ─────────────────────────────────────────────────────────────────
dt  = 0.01;
t   = (0 : dt : 10)';
N   = length(t);
r   = ones(N, 1);
tau = 0.8;

%% ── Simulate each controller ──────────────────────────────────────────────
yP    = sim_pid(1.5, 0,    0,    N, dt, tau, r);
yPD   = sim_pid(2.0, 0,    0.6,  N, dt, tau, r);
yPID  = sim_pid(2.4, 0.15, 0.8,  N, dt, tau, r);
yOver = sim_pid(5.0, 0.15, 0.8,  N, dt, tau, r);

%% ── Kp sweep ──────────────────────────────────────────────────────────────
kp_vals = [0.8, 1.5, 2.4, 4.0, 6.0];
kp_ys   = zeros(N, length(kp_vals));
for ii = 1:length(kp_vals)
    kp_ys(:,ii) = sim_pid(kp_vals(ii), 0.15, 0.8, N, dt, tau, r);
end

%% ── Performance metrics ───────────────────────────────────────────────────
configs   = {'P only','PD','PID tuned','Over-tuned'};
resps     = {yP, yPD, yPID, yOver};
ss_err    = zeros(1,4);
overshoot = zeros(1,4);
settle_t  = zeros(1,4);

for ii = 1:4
    yi = resps{ii};
    ss_err(ii)    = abs(1 - yi(end)) * 100;
    overshoot(ii) = max(0, max(yi) - 1) * 100;
    idx_s = find(abs(yi - 1) < 0.02, 1);
    if ~isempty(idx_s)
        settle_t(ii) = t(idx_s);
    else
        settle_t(ii) = t(end);
    end
end

%% ── Print table ───────────────────────────────────────────────────────────
fprintf('%-12s  %14s  %13s  %12s\n','Controller','SS Error [%]','Overshoot [%]','Settle [s]');
fprintf('%s\n', repmat('-',1,55));
for ii = 1:4
    fprintf('%-12s  %14.2f  %13.2f  %12.2f\n', ...
            configs{ii}, ss_err(ii), overshoot(ii), settle_t(ii));
end
fprintf('\n');

%% ── Plot ──────────────────────────────────────────────────────────────────
figure('Name','PID Step Response Demo','Color','w','Position',[100 80 1100 750]);

% ── Subplot 1: controller comparison ──────────────────────────────────────
subplot(2,2,1);
plot(t, r,    'k--','LineWidth',1.5,'DisplayName','Setpoint'); hold on;
plot(t, yP,   'r-', 'LineWidth',2,  'DisplayName','P only  (Kp=1.5)');
plot(t, yPD,  'm-', 'LineWidth',2,  'DisplayName','PD      (Kp=2.0, Kd=0.6)');
plot(t, yPID, 'b-', 'LineWidth',2.5,'DisplayName','PID tuned (Kp=2.4, Ki=0.15, Kd=0.8)');
idx_pid = find(abs(yPID-1) < 0.02, 1);
if ~isempty(idx_pid)
    xline(t(idx_pid),'--b',sprintf('Settle %.1f s',t(idx_pid)), ...
          'FontSize',8,'LabelHorizontalAlignment','right');
end
xlabel('Time [s]'); ylabel('Response');
legend('Location','southeast','FontSize',9);
grid on; ylim([-0.05 1.4]);
title('P vs PD vs PID — Step Response','FontWeight','bold');
set(gca,'FontSize',10);

% ── Subplot 2: error over time ─────────────────────────────────────────────
subplot(2,2,2);
plot(t, r-yP,   'r-','LineWidth',1.8,'DisplayName','Error — P only'); hold on;
plot(t, r-yPID, 'b-','LineWidth',1.8,'DisplayName','Error — PID tuned');
yline(0,'k-','LineWidth',0.8);
fill([t;flipud(t)],[r-yP;zeros(N,1)],'r','FaceAlpha',0.10,'EdgeColor','none');
fill([t;flipud(t)],[r-yPID;zeros(N,1)],'b','FaceAlpha',0.10,'EdgeColor','none');
xlabel('Time [s]'); ylabel('Error  r(t) - y(t)');
legend('Location','northeast','FontSize',9);
grid on;
title('Tracking Error e(t) = r(t) - y(t)','FontWeight','bold');
set(gca,'FontSize',10);

% ── Subplot 3: Kp sweep ────────────────────────────────────────────────────
subplot(2,2,3);
plot(t, r,'k--','LineWidth',1.5,'DisplayName','Setpoint'); hold on;
kp_colors = {'#7B5EA7','#D0021B','#2196A8','#F5A623','#888800'};
for ii = 1:length(kp_vals)
    lbl = sprintf('Kp=%.1f', kp_vals(ii));
    if kp_vals(ii) == 2.4, lbl = [lbl '  <- optimal']; end
    lw = 1.3;
    if kp_vals(ii) == 2.4, lw = 2.5; end
    plot(t, kp_ys(:,ii), 'Color', kp_colors{ii}, ...
         'LineWidth', lw, 'DisplayName', lbl);
end
xlabel('Time [s]'); ylabel('Response');
legend('Location','southeast','FontSize',8);
grid on; ylim([-0.05 1.8]);
title('Kp Gain Sweep (Ki=0.15, Kd=0.8)','FontWeight','bold');
set(gca,'FontSize',10);

% ── Subplot 4: performance bar chart ──────────────────────────────────────
subplot(2,2,4);
xp = 1:4;
b1 = bar(xp-0.25, ss_err,    0.22,'FaceColor','#D0021B','FaceAlpha',0.85); hold on;
b2 = bar(xp,      overshoot, 0.22,'FaceColor','#F5A623','FaceAlpha',0.85);
b3 = bar(xp+0.25, settle_t,  0.22,'FaceColor','#2196A8','FaceAlpha',0.85);
for ii = 1:4
    text(ii-0.25, ss_err(ii)+0.2,   sprintf('%.1f',ss_err(ii)),   'HorizontalAlignment','center','FontSize',7.5,'FontWeight','bold');
    text(ii,      overshoot(ii)+0.2, sprintf('%.1f',overshoot(ii)),'HorizontalAlignment','center','FontSize',7.5,'FontWeight','bold');
    text(ii+0.25, settle_t(ii)+0.2,  sprintf('%.1f',settle_t(ii)), 'HorizontalAlignment','center','FontSize',7.5,'FontWeight','bold');
end
set(gca,'XTick',1:4,'XTickLabel',configs,'FontSize',9);
legend([b1,b2,b3],'SS Error [%]','Overshoot [%]','Settling time [s]', ...
       'Location','northwest','FontSize',9);
grid on; ylabel('Value');
title('Performance Metrics Comparison','FontWeight','bold');

sgtitle('PID Controller Tuning — Step Response Analysis', ...
        'FontWeight','bold','FontSize',13);

fprintf('Done — 4 subplots shown.\n');

%% =========================================================================
%  LOCAL FUNCTION — defined at bottom of script (MATLAB R2016b+)
%  All variables passed explicitly — no workspace sharing issue
%% =========================================================================
function y = sim_pid(Kp, Ki, Kd, N, dt, tau, r)
    y  = zeros(N, 1);
    ig = 0;
    ep = 0;
    for k = 2 : N
        e  = r(k) - y(k-1);
        ig = max(-5, min(5, ig + e * dt));
        d  = (e - ep) / dt;
        u  = max(-10, min(10, Kp*e + Ki*ig + Kd*d));
        y(k) = y(k-1) + (dt/tau) * (-y(k-1) + u);
        ep = e;
    end
end