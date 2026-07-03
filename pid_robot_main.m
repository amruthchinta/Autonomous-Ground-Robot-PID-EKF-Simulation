%% =========================================================================
%  pid_robot_main.m — Autonomous Ground Robot PID Control Simulation
%  =========================================================================
%  SELF-CONTAINED: all helper functions embedded at the bottom of this file.
%  Just run:  >> pid_robot_main
%
%  Project : Autonomous Inspection Robot — PID Control + EKF Navigation
%  Author  : [Your Name]  |  Version 1.0  |  2025
%
%  HOW TO RUN
%  ----------
%  1. Open MATLAB (R2020b or newer)
%  2. cd to the folder containing this file
%  3. Type:  pid_robot_main
%  4. All figures appear automatically. No toolboxes required.
%
%  OPTIONAL EXPORT
%  ---------------
%  pid_robot_main('export')   % saves simulation_results.csv
% =========================================================================

function pid_robot_main(varargin)

clc; close all;
fprintf('=============================================================\n');
fprintf('  Autonomous Robot — PID Control + EKF  |  MATLAB Simulation\n');
fprintf('=============================================================\n\n');

mode = 'simulate';
if nargin > 0, mode = varargin{1}; end

%% ── 1. Simulation parameters ─────────────────────────────────────────────
dt   = 0.01;          % [s]  100 Hz control loop
T    = 60.0;          % [s]  mission duration
N    = round(T / dt); % number of steps (bounded loop)

%% ── 2. Robot physical parameters ─────────────────────────────────────────
wheel_base  = 0.28;   % [m]
wheel_r     = 0.065;  % [m]
tau_motor   = 0.08;   % [s]  first-order motor lag
Kv          = 1.5;    % [m/s per unit PWM]
max_speed   = 1.5;    % [m/s]
max_omega   = 2.5;    % [rad/s]

%% ── 3. PID gains (Ziegler-Nichols tuned, then refined) ───────────────────
% Position PID
Kp_pos = 2.4;  Ki_pos = 0.15;  Kd_pos = 0.80;  Imax_pos = 2.0;
% Heading PID
Kp_yaw = 3.1;  Ki_yaw = 0.08;  Kd_yaw = 1.20;  Imax_yaw = 3.0;
% Velocity PID
Kp_vel = 1.8;  Ki_vel = 0.22;  Kd_vel = 0.40;  Imax_vel = 5.0;

%% ── 4. Sensor noise ──────────────────────────────────────────────────────
gps_std  = 0.25;    % [m]     GPS position noise (1-sigma)
gps_rate = round(1.0 / dt);  % GPS fires every 100 steps = 1 Hz
enc_std  = 0.005;   % [m/s]
gyr_std  = 0.03;    % [rad/s]

%% ── 5. Waypoints ─────────────────────────────────────────────────────────
waypoints = [
     0.0,  0.0;
     3.0,  0.0;
     3.0,  2.0;
     0.0,  2.0;
    -2.0,  1.0;
     0.0,  0.0;
];
wp_radius = 0.15;   % [m]  capture radius
n_wp      = size(waypoints, 1);

%% ── 6. Pre-allocate arrays (no dynamic allocation) ───────────────────────
x_true = zeros(N, 5);   % [x, y, yaw, v, omega] true state
x_est  = zeros(N, 5);   % EKF estimated state
ref_xy = zeros(N, 2);   % reference [x_ref, y_ref]
pwm_L  = zeros(N, 1);
pwm_R  = zeros(N, 1);
u_pos_log = zeros(N, 1);
u_yaw_log = zeros(N, 1);

% PID integrators and previous errors
int_pos = 0;  int_yaw = 0;  int_vel = 0;
ep_pos  = 0;  ep_yaw  = 0;  ep_vel  = 0;

% EKF covariance (5x5)
P_ekf = diag([0.1, 0.1, 0.05, 0.2, 0.1]);
Q_ekf = diag([0.001, 0.001, 0.002, 0.01, 0.02]);
R_gps = diag([gps_std^2, gps_std^2]);
R_enc = diag([enc_std^2, gyr_std^2]);

current_wp = 1;
rng(42);   % reproducible noise

%% ── 7. Main simulation loop ──────────────────────────────────────────────
fprintf('Running 60 s mission...\n');

for k = 1 : N-1

    %% 7.1 Current EKF state
    xh   = x_est(k,1);
    yh   = x_est(k,2);
    yawh = x_est(k,3);
    vh   = x_est(k,4);

    %% 7.2 Waypoint logic
    if current_wp <= n_wp
        xr = waypoints(current_wp, 1);
        yr = waypoints(current_wp, 2);
    else
        xr = waypoints(n_wp, 1);
        yr = waypoints(n_wp, 2);
    end
    dist_wp = sqrt((xr-xh)^2 + (yr-yh)^2);
    if dist_wp < wp_radius && current_wp <= n_wp
        fprintf('  WP%d reached at t = %.1f s\n', current_wp, (k-1)*dt);
        current_wp = min(current_wp + 1, n_wp);
    end
    yaw_ref = atan2(yr - yh, xr - xh);
    v_ref   = min(dist_wp * 0.8, max_speed);
    ref_xy(k,:) = [xr, yr];

    %% 7.3 Position PID
    e_pos    = dist_wp;
    int_pos  = int_pos + e_pos * dt;
    int_pos  = max(-Imax_pos, min(Imax_pos, int_pos));
    d_pos    = (e_pos - ep_pos) / dt;
    u_pos    = Kp_pos*e_pos + Ki_pos*int_pos + Kd_pos*d_pos;
    u_pos    = max(0, min(max_speed, u_pos));
    ep_pos   = e_pos;

    %% 7.4 Heading PID
    e_yaw    = yaw_ref - yawh;
    e_yaw    = atan2(sin(e_yaw), cos(e_yaw));   % wrap to [-pi, pi]
    int_yaw  = int_yaw + e_yaw * dt;
    int_yaw  = max(-Imax_yaw, min(Imax_yaw, int_yaw));
    d_yaw    = (e_yaw - ep_yaw) / dt;
    u_yaw    = Kp_yaw*e_yaw + Ki_yaw*int_yaw + Kd_yaw*d_yaw;
    u_yaw    = max(-max_omega, min(max_omega, u_yaw));
    ep_yaw   = e_yaw;

    %% 7.5 Velocity PID
    e_vel    = u_pos - vh;
    int_vel  = int_vel + e_vel * dt;
    int_vel  = max(-Imax_vel, min(Imax_vel, int_vel));
    d_vel    = (e_vel - ep_vel) / dt;
    u_vel    = Kp_vel*e_vel + Ki_vel*int_vel + Kd_vel*d_vel;
    u_vel    = max(-1, min(1, u_vel));
    ep_vel   = e_vel;

    %% 7.6 Differential drive mixing
    pL = u_vel - (u_yaw * wheel_base / 2) / max_speed;
    pR = u_vel + (u_yaw * wheel_base / 2) / max_speed;
    pL = max(-1, min(1, pL));
    pR = max(-1, min(1, pR));
    pwm_L(k) = pL * 100;
    pwm_R(k) = pR * 100;
    u_pos_log(k) = u_pos;
    u_yaw_log(k) = u_yaw;

    %% 7.7 Robot plant — propagate true state
    v_cmd  = (pL + pR) / 2  * Kv;
    om_cmd = (pR - pL) / wheel_base * Kv;

    yaw_k  = x_true(k,3);
    v_k    = x_true(k,4);
    om_k   = x_true(k,5);

    v_new  = v_k  + dt/tau_motor * (-v_k  + v_cmd);
    om_new = om_k + dt/tau_motor * (-om_k + om_cmd);
    yaw_n  = yaw_k + om_new * dt;
    x_n    = x_true(k,1) + v_new * cos(yaw_k) * dt;
    y_n    = x_true(k,2) + v_new * sin(yaw_k) * dt;

    x_true(k+1,:) = [x_n, y_n, yaw_n, v_new, om_new];

    %% 7.8 Simulated sensor readings
    v_enc  = v_new  + randn() * enc_std;
    om_enc = om_new + randn() * gyr_std;

    gps_meas = [];
    if mod(k, gps_rate) == 0
        gps_meas = [x_n + randn()*gps_std;
                    y_n + randn()*gps_std];
    end

    %% 7.9 EKF predict + update
    [x_hat, P_ekf] = ekf_step(x_est(k,:)', P_ekf, ...
                               v_enc, om_enc, gps_meas, ...
                               Q_ekf, R_gps, R_enc, dt);
    x_est(k+1,:) = x_hat';
end

fprintf('Simulation complete.\n\n');

%% ── 8. KPI summary ───────────────────────────────────────────────────────
t_vec   = (0:N-1)' * dt;
pos_err = sqrt((x_true(:,1)-x_est(:,1)).^2 + (x_true(:,2)-x_est(:,2)).^2);
yaw_err = abs(x_true(:,3) - x_est(:,3));

fprintf('──────────────────────────────────────────────\n');
fprintf('  KPI Results\n');
fprintf('──────────────────────────────────────────────\n');
fprintf('  Position RMSE (EKF)  : %.2f cm\n', sqrt(mean(pos_err.^2))*100);
fprintf('  Heading RMSE         : %.2f deg\n', sqrt(mean(yaw_err.^2))*180/pi);
fprintf('  Max position error   : %.2f cm\n', max(pos_err)*100);
fprintf('  Waypoints reached    : %d / %d\n', min(current_wp-1, n_wp), n_wp);
fprintf('──────────────────────────────────────────────\n\n');

%% ── 9. Export ────────────────────────────────────────────────────────────
if strcmp(mode, 'export')
    T_out = table(t_vec, x_true(:,1), x_true(:,2), x_true(:,3), ...
                  x_est(:,1),  x_est(:,2),  x_est(:,3), pos_err, ...
                  pwm_L, pwm_R, ...
        'VariableNames', {'t','x_true','y_true','yaw_true', ...
                          'x_est','y_est','yaw_est','pos_err', ...
                          'pwm_L_pct','pwm_R_pct'});
    writetable(T_out, 'simulation_results.csv');
    fprintf('Exported to simulation_results.csv\n');
end

%% ── 10. Plot all results ─────────────────────────────────────────────────
plot_all_results(t_vec, x_true, x_est, ref_xy, pwm_L, pwm_R, ...
                 u_pos_log, u_yaw_log, waypoints, pos_err);

end  % end main function


%% =========================================================================
%  LOCAL FUNCTION: ekf_step
%  Extended Kalman Filter — predict + optional GPS update
%  State: [x, y, yaw, v, omega]
%% =========================================================================
function [x_new, P_new] = ekf_step(x, P, v_enc, om_enc, gps, ...
                                    Q, R_gps, R_enc, dt)
    yaw = x(3);
    v   = x(4);

    %% Predict
    x_p    = x;
    x_p(1) = x(1) + v * cos(yaw) * dt;
    x_p(2) = x(2) + v * sin(yaw) * dt;
    x_p(3) = x(3) + om_enc * dt;
    x_p(4) = v_enc;
    x_p(5) = om_enc;

    F      = eye(5);
    F(1,3) = -v * sin(yaw) * dt;
    F(1,4) =  cos(yaw) * dt;
    F(2,3) =  v * cos(yaw) * dt;
    F(2,5) =  sin(yaw) * dt;

    P_p = F * P * F' + Q;

    %% Encoder update (velocity + omega)
    H_enc = zeros(2,5);
    H_enc(1,4) = 1;  % v
    H_enc(2,5) = 1;  % omega
    z_enc  = [v_enc; om_enc];
    S_enc  = H_enc * P_p * H_enc' + R_enc;
    K_enc  = P_p * H_enc' / S_enc;
    innov  = z_enc - H_enc * x_p;
    x_p    = x_p + K_enc * innov;
    P_p    = (eye(5) - K_enc * H_enc) * P_p;

    %% GPS update (position x, y) — only when measurement available
    if ~isempty(gps)
        H_gps      = zeros(2,5);
        H_gps(1,1) = 1;   % x
        H_gps(2,2) = 1;   % y
        S_gps = H_gps * P_p * H_gps' + R_gps;
        K_gps = P_p * H_gps' / S_gps;
        x_p   = x_p + K_gps * (gps - H_gps * x_p);
        P_p   = (eye(5) - K_gps * H_gps) * P_p;
    end

    x_p(3) = atan2(sin(x_p(3)), cos(x_p(3)));  % wrap yaw to [-pi, pi]
    x_new  = x_p;
    P_new  = P_p;
end


%% =========================================================================
%  LOCAL FUNCTION: plot_all_results
%  Generates all result figures — embedded here so no external file needed
%% =========================================================================
function plot_all_results(t, x_true, x_est, ref_xy, pwm_L, pwm_R, ...
                           u_pos, u_yaw, waypoints, pos_err)

    C1 = [0.102 0.227 0.361];  % navy
    C2 = [0.129 0.588 0.659];  % teal
    C3 = [0.961 0.651 0.137];  % amber
    C4 = [0.816 0.008 0.106];  % red
    C5 = [0.231 0.549 0.231];  % green

    %% ── Figure 1: XY Trajectory ─────────────────────────────────────────
    figure('Name','Trajectory','Color','w','Position',[50 50 680 580]);
    plot(x_true(:,1), x_true(:,2), '--', 'Color', C1, ...
         'LineWidth', 2, 'DisplayName','True path');
    hold on;
    plot(x_est(:,1), x_est(:,2), '-', 'Color', C2, ...
         'LineWidth', 1.8, 'DisplayName','EKF estimate');
    % Waypoints
    for i = 1:size(waypoints,1)
        plot(waypoints(i,1), waypoints(i,2), 'r*', ...
             'MarkerSize',14, 'LineWidth',2, 'HandleVisibility','off');
        text(waypoints(i,1)+0.08, waypoints(i,2)+0.1, ...
             sprintf('WP%d',i-1), 'FontSize',9, 'Color',C4, ...
             'FontWeight','bold');
    end
    xlabel('X [m]'); ylabel('Y [m]');
    grid on; axis equal;
    legend('Location','best','FontSize',10);
    title('Autonomous Navigation — XY Trajectory', ...
          'FontWeight','bold','FontSize',13);
    set(gca,'FontSize',10);

    %% ── Figure 2: State time histories ──────────────────────────────────
    figure('Name','States over time','Color','w','Position',[80 30 1100 700]);

    subplot(3,2,1);
    plot(t, x_true(:,1), '-', 'Color',C1, 'LineWidth',1.8); hold on;
    plot(t, x_est(:,1),  '--','Color',C2, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('x [m]');
    legend('True','EKF','Location','best','FontSize',9);
    title('X Position'); grid on; set(gca,'FontSize',9);

    subplot(3,2,2);
    plot(t, x_true(:,2), '-', 'Color',C1, 'LineWidth',1.8); hold on;
    plot(t, x_est(:,2),  '--','Color',C2, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('y [m]');
    legend('True','EKF','Location','best','FontSize',9);
    title('Y Position'); grid on; set(gca,'FontSize',9);

    subplot(3,2,3);
    plot(t, rad2deg(x_true(:,3)), '-', 'Color',C1, 'LineWidth',1.8); hold on;
    plot(t, rad2deg(x_est(:,3)),  '--','Color',C2, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Yaw [deg]');
    legend('True','EKF','Location','best','FontSize',9);
    title('Heading (Yaw)'); grid on; set(gca,'FontSize',9);

    subplot(3,2,4);
    plot(t, pos_err*100, '-', 'Color',C4, 'LineWidth',1.5);
    yline(5, '--k', '5 cm limit', 'FontSize',9);
    xlabel('Time [s]'); ylabel('Error [cm]');
    title('Position Estimation Error (EKF)'); grid on; set(gca,'FontSize',9);

    subplot(3,2,5);
    plot(t, pwm_L, '-', 'Color',C2, 'LineWidth',1.2); hold on;
    plot(t, pwm_R, '--','Color',C3, 'LineWidth',1.2);
    xlabel('Time [s]'); ylabel('PWM [%]');
    legend('Left motor','Right motor','Location','best','FontSize',9);
    title('Motor PWM Commands'); grid on; set(gca,'FontSize',9);

    subplot(3,2,6);
    plot(t, u_pos, '-', 'Color',C5, 'LineWidth',1.5); hold on;
    plot(t, rad2deg(u_yaw), '--', 'Color',C3, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Command value');
    legend('u_{pos} [m/s]','u_{yaw} [deg/s]','Location','best','FontSize',9);
    title('PID Controller Outputs'); grid on; set(gca,'FontSize',9);

    sgtitle('Simulation Results — Autonomous Ground Robot PID Control', ...
            'FontWeight','bold','FontSize',13);

    %% ── Figure 3: PID step response comparison ───────────────────────────
    figure('Name','PID Step Response','Color','w','Position',[120 60 900 500]);
    dt_s = 0.01; t_s = (0:dt_s:10)'; N_s = length(t_s);
    r_s  = ones(N_s,1);

    function y = sim_pid_local(Kp_, Ki_, Kd_)
        y_ = zeros(N_s,1); ig = 0; ep = 0;
        for ii = 2:N_s
            e_ = r_s(ii) - y_(ii-1);
            ig = max(-5, min(5, ig + e_*dt_s));
            d_ = (e_ - ep)/dt_s;
            u_ = max(-10, min(10, Kp_*e_ + Ki_*ig + Kd_*d_));
            y_(ii) = y_(ii-1) + dt_s/0.8*(-y_(ii-1) + u_);
            ep = e_;
        end
        y = y_;
    end

    yP   = sim_pid_local(1.5, 0,    0   );
    yPD  = sim_pid_local(2.0, 0,    0.6 );
    yPID = sim_pid_local(2.4, 0.15, 0.8 );

    plot(t_s, r_s,   'k--', 'LineWidth',1.5, 'DisplayName','Setpoint');
    hold on;
    plot(t_s, yP,   'Color',C4, 'LineWidth',2,   'DisplayName','P only  (Kp=1.5)');
    plot(t_s, yPD,  'Color',C3, 'LineWidth',2,   'DisplayName','PD      (Kp=2.0, Kd=0.6)');
    plot(t_s, yPID, 'Color',C2, 'LineWidth',2.5, 'DisplayName','PID tuned  Kp=2.4, Ki=0.15, Kd=0.8');
    xlabel('Time [s]'); ylabel('Response');
    legend('Location','southeast','FontSize',10);
    grid on;
    title('PID Tuning — Step Response Comparison','FontWeight','bold','FontSize',13);
    set(gca,'FontSize',10);

    % Annotate settling time
    tol_s = 0.02;
    idx_s = find(abs(yPID - 1) < tol_s, 1);
    if ~isempty(idx_s)
        xline(t_s(idx_s), '--b', sprintf('Settle %.1f s', t_s(idx_s)), ...
              'FontSize',9,'LabelHorizontalAlignment','right');
    end

    fprintf('Figures generated. Check the Figure windows.\n');
end
