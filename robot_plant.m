%% =========================================================================
%  robot_plant.m — Differential Drive Robot Dynamics Model
%  =========================================================================
%  HOW TO RUN:
%    >> robot_plant.demo()          % standalone demo — run this first
%
%  HOW TO USE AS A CLASS:
%    >> r = robot_plant()           % create robot with default params
%    >> r = robot_plant(0.01)       % create with dt = 0.01 s
%    >> x = r.step(0.8, 0.8)       % step with left/right PWM [-1, 1]
%    >> r.info()                    % print current state
%    >> r.reset()                   % reset to origin
%
%  State vector returned: [x_pos, y_pos, yaw, v, omega]
% =========================================================================

classdef robot_plant < handle

    properties (Access = public)
        % Physical parameters — change these to match your robot
        mass        = 3.2     % [kg]   total mass
        wheel_r     = 0.065   % [m]    wheel radius
        wheel_base  = 0.28    % [m]    distance between wheels
        I_z         = 0.05    % [kg.m2] moment of inertia about Z axis
        tau_motor   = 0.08    % [s]    first-order motor time constant
        Kv          = 1.5     % [m/s]  speed per unit PWM input
        max_speed   = 1.5     % [m/s]  maximum forward speed
        max_omega   = 2.5     % [rad/s] maximum angular velocity
        dt          = 0.01    % [s]    simulation timestep
    end

    properties (Access = private)
        state   % [x, y, yaw, v, omega] — current state vector
    end

    methods

        function obj = robot_plant(dt, x0)
            % Constructor — all arguments optional
            %
            % Examples:
            %   r = robot_plant()           % default dt=0.01, start at origin
            %   r = robot_plant(0.01)       % specify dt
            %   r = robot_plant(0.01, [1; 0; 0; 0; 0])  % start at x=1

            if nargin >= 1 && ~isempty(dt)
                obj.dt = dt;
            end
            if nargin >= 2 && ~isempty(x0)
                obj.state = x0(:);
            else
                obj.state = zeros(5, 1);
            end
        end

        function x = step(obj, pwm_L, pwm_R)
            % Propagate robot dynamics one timestep
            %
            % Inputs:
            %   pwm_L : left  wheel duty cycle [-1, 1]
            %   pwm_R : right wheel duty cycle [-1, 1]
            %
            % Output:
            %   x : new state [x_pos; y_pos; yaw; v; omega]

            % Clamp inputs
            pwm_L = max(-1, min(1, pwm_L));
            pwm_R = max(-1, min(1, pwm_R));

            % Current state
            x_k   = obj.state(1);
            y_k   = obj.state(2);
            yaw_k = obj.state(3);
            v_k   = obj.state(4);
            om_k  = obj.state(5);

            % Speed commands from PWM
            v_cmd  = (pwm_L + pwm_R) / 2.0 * obj.Kv;
            om_cmd = (pwm_R - pwm_L) / obj.wheel_base * obj.Kv;

            % First-order motor dynamics (Euler integration)
            v_new  = v_k  + obj.dt / obj.tau_motor * (-v_k  + v_cmd);
            om_new = om_k + obj.dt / obj.tau_motor * (-om_k + om_cmd);

            % Clamp speeds
            v_new  = max(-obj.max_speed, min(obj.max_speed,  v_new));
            om_new = max(-obj.max_omega, min(obj.max_omega, om_new));

            % Kinematic integration
            yaw_new = yaw_k + om_new * obj.dt;
            x_new   = x_k   + v_new * cos(yaw_k) * obj.dt;
            y_new   = y_k   + v_new * sin(yaw_k) * obj.dt;

            % Wrap yaw to [-pi, pi]
            yaw_new = atan2(sin(yaw_new), cos(yaw_new));

            obj.state = [x_new; y_new; yaw_new; v_new; om_new];
            x = obj.state;
        end

        function s = get_state(obj)
            % Return current state vector [x; y; yaw; v; omega]
            s = obj.state;
        end

        function reset(obj, x0)
            % Reset state to origin (or to x0 if provided)
            if nargin < 2 || isempty(x0)
                obj.state = zeros(5, 1);
            else
                obj.state = x0(:);
            end
        end

        function [v_L, v_R] = inverse_kinematics(obj, v, omega)
            % Convert (v, omega) commands to individual wheel speeds [m/s]
            v_L = v - omega * obj.wheel_base / 2.0;
            v_R = v + omega * obj.wheel_base / 2.0;
        end

        function info(obj)
            % Print current robot state to command window
            s = obj.state;
            fprintf('\n--- robot_plant state ---\n');
            fprintf('  x_pos  : %.4f m\n',     s(1));
            fprintf('  y_pos  : %.4f m\n',     s(2));
            fprintf('  yaw    : %.4f rad  (%.1f deg)\n', s(3), rad2deg(s(3)));
            fprintf('  speed  : %.4f m/s\n',   s(4));
            fprintf('  omega  : %.4f rad/s\n', s(5));
            fprintf('  dt     : %.4f s\n',     obj.dt);
            fprintf('-------------------------\n\n');
        end

    end  % instance methods

    methods (Static)

        function demo()
            % Standalone demo — drives the robot in a square pattern
            % Run with:  robot_plant.demo()

            fprintf('\n=== robot_plant.demo() ===\n');
            fprintf('Driving robot in a square: Forward -> Turn -> x4\n\n');

            dt  = 0.01;
            r   = robot_plant(dt);

            T_straight = 3.0;   % [s] drive straight
            T_turn     = 1.57;  % [s] 90-degree turn (~pi/2 rad at 1 rad/s)
            T_total    = 4 * (T_straight + T_turn);
            N          = round(T_total / dt);

            % Pre-allocate log arrays
            log_x   = zeros(N, 1);
            log_y   = zeros(N, 1);
            log_yaw = zeros(N, 1);
            log_v   = zeros(N, 1);
            t_vec   = (0:N-1)' * dt;

            for k = 1 : N
                % Time within current square segment
                t_seg = mod((k-1)*dt, T_straight + T_turn);

                if t_seg < T_straight
                    % Drive straight — both wheels equal
                    pwm_L = 0.6;
                    pwm_R = 0.6;
                else
                    % Turn right — right wheel slower
                    pwm_L =  0.4;
                    pwm_R = -0.4;
                end

                x = r.step(pwm_L, pwm_R);
                log_x(k)   = x(1);
                log_y(k)   = x(2);
                log_yaw(k) = x(3);
                log_v(k)   = x(4);
            end

            % ── Plot results ──────────────────────────────────────────────
            figure('Name','robot_plant Demo','Color','w', ...
                   'Position',[150 100 1000 450]);

            subplot(1,2,1);
            plot(log_x, log_y, 'b-', 'LineWidth', 2);
            hold on;
            plot(log_x(1), log_y(1), 'gs', 'MarkerSize',12, ...
                 'MarkerFaceColor','g', 'DisplayName','Start');
            plot(log_x(end), log_y(end), 'rs', 'MarkerSize',12, ...
                 'MarkerFaceColor','r', 'DisplayName','End');
            xlabel('X [m]'); ylabel('Y [m]');
            grid on; axis equal;
            legend('Path','Start','End','Location','best');
            title('robot\_plant.demo()  —  Square Path', ...
                  'FontWeight','bold','FontSize',12);
            set(gca,'FontSize',10);

            subplot(1,2,2);
            yyaxis left;
            plot(t_vec, log_v, 'b-', 'LineWidth', 1.8);
            ylabel('Speed [m/s]');
            yyaxis right;
            plot(t_vec, rad2deg(log_yaw), 'r-', 'LineWidth', 1.8);
            ylabel('Yaw [deg]');
            xlabel('Time [s]');
            grid on;
            title('Speed and Heading over Time', ...
                  'FontWeight','bold','FontSize',12);
            legend('Speed','Yaw','Location','best');
            set(gca,'FontSize',10);

            % Print final state
            r.info();
            fprintf('Path length : %.2f m\n', ...
                    sum(sqrt(diff(log_x).^2 + diff(log_y).^2)));
        end

    end  % static methods

end  % classdef robot_plant