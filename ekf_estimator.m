%% =========================================================================
%  ekf_estimator.m — Extended Kalman Filter for Robot State Estimation
%  =========================================================================
%  HOW TO RUN:
%    >> ekf_estimator.demo()         % standalone demo — run this first
%
%  HOW TO USE AS A CLASS:
%    >> ekf = ekf_estimator()        % create with defaults
%    >> ekf = ekf_estimator(0.01)    % create with dt = 0.01 s
%    >> ekf.predict(v_enc, om_enc)   % predict step (every 10 ms)
%    >> ekf.update_gps(gps_x, gps_y)% GPS update (every 1 s)
%    >> s = ekf.get_state()          % returns [x; y; yaw; v; omega]
%    >> ekf.info()                   % print current estimate
%    >> ekf.reset()                  % reset to origin
%
%  State vector: [x_pos, y_pos, yaw, v_forward, omega]
% =========================================================================

classdef ekf_estimator < handle

    properties (Access = public)
        dt      = 0.01    % [s] prediction timestep

        % Process noise covariance Q (tuned for 3.2 kg robot)
        Q = diag([0.001, 0.001, 0.002, 0.01, 0.02])

        % Measurement noise — encoders
        R_enc = diag([0.005^2, 0.01^2])

        % Measurement noise — GPS
        R_gps = diag([0.25^2, 0.25^2])
    end

    properties (Access = private)
        state   % [x; y; yaw; v; omega]  estimated state (5x1)
        P       % Error covariance matrix (5x5)
    end

    methods

        function obj = ekf_estimator(dt, x0, P0)
            % Constructor — all arguments optional
            %
            % Examples:
            %   ekf = ekf_estimator()
            %   ekf = ekf_estimator(0.01)
            %   ekf = ekf_estimator(0.01, [1;0;0;0;0], eye(5)*0.1)

            if nargin >= 1 && ~isempty(dt), obj.dt = dt; end

            if nargin >= 2 && ~isempty(x0)
                obj.state = x0(:);
            else
                obj.state = zeros(5, 1);
            end

            if nargin >= 3 && ~isempty(P0)
                obj.P = P0;
            else
                obj.P = diag([0.1, 0.1, 0.05, 0.2, 0.1]);
            end
        end

        function predict(obj, v_enc, om_enc)
            % EKF predict step — call every control loop tick (100 Hz)
            %
            % Inputs:
            %   v_enc  : encoder forward velocity estimate [m/s]
            %   om_enc : encoder angular velocity estimate [rad/s]

            yaw = obj.state(3);
            v   = obj.state(4);
            dt  = obj.dt;

            % State prediction (unicycle kinematics)
            x_pred    = obj.state;
            x_pred(1) = obj.state(1) + v * cos(yaw) * dt;
            x_pred(2) = obj.state(2) + v * sin(yaw) * dt;
            x_pred(3) = obj.state(3) + om_enc * dt;
            x_pred(4) = v_enc;
            x_pred(5) = om_enc;

            % Wrap yaw to [-pi, pi]
            x_pred(3) = atan2(sin(x_pred(3)), cos(x_pred(3)));

            % State Jacobian F = d(f)/d(x)
            F      = eye(5);
            F(1,3) = -v * sin(yaw) * dt;
            F(1,4) =  cos(yaw) * dt;
            F(2,3) =  v * cos(yaw) * dt;
            F(2,5) =  sin(yaw) * dt;

            % Propagate covariance
            obj.state = x_pred;
            obj.P     = F * obj.P * F' + obj.Q;
        end

        function update_encoders(obj, v_enc, om_enc)
            % EKF update from wheel encoder measurements
            %
            % Inputs:
            %   v_enc  : measured forward velocity [m/s]
            %   om_enc : measured angular velocity [rad/s]

            H = zeros(2, 5);
            H(1, 4) = 1;   % observes v
            H(2, 5) = 1;   % observes omega

            z     = [v_enc; om_enc];
            innov = z - H * obj.state;
            S     = H * obj.P * H' + obj.R_enc;
            K     = obj.P * H' / S;

            obj.state = obj.state + K * innov;
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
            obj.P = (eye(5) - K * H) * obj.P;
        end

        function update_gps(obj, gps_x, gps_y)
            % EKF update from GPS measurement — call at 1 Hz
            %
            % Inputs:
            %   gps_x : GPS measured x position [m]
            %   gps_y : GPS measured y position [m]

            H = zeros(2, 5);
            H(1, 1) = 1;   % observes x
            H(2, 2) = 1;   % observes y

            z     = [gps_x; gps_y];
            innov = z - H * obj.state;
            S     = H * obj.P * H' + obj.R_gps;
            K     = obj.P * H' / S;

            obj.state = obj.state + K * innov;
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
            obj.P = (eye(5) - K * H) * obj.P;
        end

        function s = get_state(obj)
            % Return current estimated state [x; y; yaw; v; omega]
            s = obj.state;
        end

        function p = get_covariance(obj)
            % Return current error covariance matrix P (5x5)
            p = obj.P;
        end

        function reset(obj, x0)
            % Reset state to origin (or to x0 if provided)
            if nargin < 2 || isempty(x0)
                obj.state = zeros(5, 1);
            else
                obj.state = x0(:);
            end
            obj.P = diag([0.1, 0.1, 0.05, 0.2, 0.1]);
        end

        function info(obj)
            % Print current EKF estimate to command window
            s = obj.state;
            fprintf('\n--- ekf_estimator state ---\n');
            fprintf('  x_pos  : %.4f m\n',     s(1));
            fprintf('  y_pos  : %.4f m\n',     s(2));
            fprintf('  yaw    : %.4f rad  (%.1f deg)\n', s(3), rad2deg(s(3)));
            fprintf('  speed  : %.4f m/s\n',   s(4));
            fprintf('  omega  : %.4f rad/s\n', s(5));
            fprintf('  P diag : [%.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
                    obj.P(1,1), obj.P(2,2), obj.P(3,3), obj.P(4,4), obj.P(5,5));
            fprintf('---------------------------\n\n');
        end

    end  % instance methods

    methods (Static)

        function demo()
            % Standalone EKF demo — compares dead-reckoning vs EKF vs truth
            % Run with:  ekf_estimator.demo()

            fprintf('\n=== ekf_estimator.demo() ===\n');
            fprintf('Comparing: True path vs Dead-reckoning vs EKF estimate\n\n');

            rng(42);
            dt      = 0.01;
            T       = 30.0;
            N       = round(T / dt);
            t_vec   = (0:N-1)' * dt;

            % ── Simulate true robot path (figure-8 motion) ────────────────
            true_x   = zeros(N,1);
            true_y   = zeros(N,1);
            true_yaw = zeros(N,1);
            true_v   = zeros(N,1);

            for k = 2:N
                % Sinusoidal speed and heading commands
                v_true  = 0.6 + 0.3*sin(0.2*t_vec(k));
                om_true = 0.4*cos(0.15*t_vec(k));

                true_yaw(k) = true_yaw(k-1) + om_true * dt;
                true_yaw(k) = atan2(sin(true_yaw(k)), cos(true_yaw(k)));
                true_x(k)   = true_x(k-1) + v_true * cos(true_yaw(k-1)) * dt;
                true_y(k)   = true_y(k-1) + v_true * sin(true_yaw(k-1)) * dt;
                true_v(k)   = v_true;
            end

            % ── Sensor noise parameters ───────────────────────────────────
            enc_std = 0.005;    % [m/s]  encoder noise
            gyr_std = 0.03;     % [rad/s] gyro noise
            gps_std = 0.25;     % [m]    GPS noise
            gps_rate = round(1.0 / dt);  % GPS every 100 steps = 1 Hz

            % ── Dead-reckoning (encoder only, no correction) ──────────────
            dr_x   = zeros(N,1);
            dr_y   = zeros(N,1);
            dr_yaw = zeros(N,1);

            % ── EKF ──────────────────────────────────────────────────────
            ekf = ekf_estimator(dt);
            ekf_x   = zeros(N,1);
            ekf_y   = zeros(N,1);
            ekf_yaw = zeros(N,1);
            P_trace = zeros(N,1);

            for k = 2:N
                % Simulated encoder readings (noisy)
                v_enc  = true_v(k)   + randn()*enc_std;
                om_enc_meas = (true_yaw(k)-true_yaw(k-1))/dt + randn()*gyr_std;

                % Dead-reckoning update
                dr_yaw(k) = dr_yaw(k-1) + om_enc_meas * dt;
                dr_x(k)   = dr_x(k-1)   + v_enc * cos(dr_yaw(k-1)) * dt;
                dr_y(k)   = dr_y(k-1)   + v_enc * sin(dr_yaw(k-1)) * dt;

                % EKF predict
                ekf.predict(v_enc, om_enc_meas);
                ekf.update_encoders(v_enc, om_enc_meas);

                % EKF GPS update (1 Hz only)
                if mod(k, gps_rate) == 0
                    gps_x = true_x(k) + randn()*gps_std;
                    gps_y = true_y(k) + randn()*gps_std;
                    ekf.update_gps(gps_x, gps_y);
                end

                % Log EKF state
                s = ekf.get_state();
                ekf_x(k)   = s(1);
                ekf_y(k)   = s(2);
                ekf_yaw(k) = s(3);
                P_trace(k) = trace(ekf.get_covariance());
            end

            % ── Compute errors ────────────────────────────────────────────
            err_dr  = sqrt((true_x - dr_x).^2  + (true_y - dr_y).^2);
            err_ekf = sqrt((true_x - ekf_x).^2 + (true_y - ekf_y).^2);

            % ── Plot ──────────────────────────────────────────────────────
            figure('Name','ekf_estimator Demo','Color','w', ...
                   'Position',[100 80 1050 520]);

            subplot(1,2,1);
            plot(true_x, true_y, 'k--', 'LineWidth',2.5, ...
                 'DisplayName','True path');
            hold on;
            plot(dr_x, dr_y, 'r-', 'LineWidth',1.5, ...
                 'DisplayName', sprintf('Dead-reckoning  (mean err=%.0f cm)', ...
                 mean(err_dr)*100));
            plot(ekf_x, ekf_y, 'b-', 'LineWidth',2, ...
                 'DisplayName', sprintf('EKF estimate    (mean err=%.0f cm)', ...
                 mean(err_ekf)*100));
            plot(true_x(1), true_y(1), 'gs', 'MarkerSize',12, ...
                 'MarkerFaceColor','g', 'HandleVisibility','off');
            xlabel('X [m]'); ylabel('Y [m]');
            grid on; axis equal;
            legend('Location','best','FontSize',9);
            title('EKF vs Dead-Reckoning — XY Path', ...
                  'FontWeight','bold','FontSize',12);
            set(gca,'FontSize',10);

            subplot(1,2,2);
            plot(t_vec, err_dr*100,  'r-', 'LineWidth',1.5, ...
                 'DisplayName','Dead-reckoning error [cm]');
            hold on;
            plot(t_vec, err_ekf*100, 'b-', 'LineWidth',2, ...
                 'DisplayName','EKF error [cm]');
            yline(5, 'k--', '5 cm limit', 'FontSize',9);
            xlabel('Time [s]'); ylabel('Position error [cm]');
            grid on;
            legend('Location','best','FontSize',9);
            title('Position Estimation Error over Time', ...
                  'FontWeight','bold','FontSize',12);
            set(gca,'FontSize',10);

            % ── Print summary ─────────────────────────────────────────────
            fprintf('Results after %.0f seconds:\n', T);
            fprintf('  Dead-reckoning mean error : %.1f cm\n', mean(err_dr)*100);
            fprintf('  EKF mean error            : %.1f cm\n', mean(err_ekf)*100);
            fprintf('  Improvement               : %.1fx better\n', ...
                    mean(err_dr)/mean(err_ekf));
            fprintf('\nFinal EKF state:\n');
            ekf.info();
        end

    end  % static methods

end  % classdef ekf_estimator