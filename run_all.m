classdef pid_controller < handle
% pid_controller  Discrete-time PID controller with anti-windup
%
% HOW TO RUN:
%   pid_controller.demo()              % run standalone demo
%
% HOW TO CREATE:
%   c = pid_controller(2.4, 0.15, 0.8, 0.01)
%   u = c.step(error)
%   c.info()
%   c.reset()
%
% STATIC METHODS (no object needed):
%   pid_controller.demo()
%   [Kp,Ki,Kd] = pid_controller.ziegler_nichols(4.8, 1.8)

    properties (Access = public)
        Kp          = 1.0
        Ki          = 0.0
        Kd          = 0.0
        dt          = 0.01
        N           = 100
        u_min       = -inf
        u_max       =  inf
        anti_windup =  inf
    end

    properties (Access = private)
        integrator     = 0
        differentiator = 0
        e_prev         = 0
        u_prev         = 0
    end

    methods

        function obj = pid_controller(Kp, Ki, Kd, dt, varargin)
            if nargin >= 1, obj.Kp = Kp; end
            if nargin >= 2, obj.Ki = Ki; end
            if nargin >= 3, obj.Kd = Kd; end
            if nargin >= 4, obj.dt = dt; end
            i = 1;
            while i <= length(varargin) - 1
                switch lower(varargin{i})
                    case 'n',          obj.N           = varargin{i+1};
                    case 'saturation', obj.u_min       = varargin{i+1}(1);
                                       obj.u_max       = varargin{i+1}(2);
                    case 'antiwindup', obj.anti_windup = varargin{i+1};
                end
                i = i + 2;
            end
            obj.reset();
        end

        function u = step(obj, error)
            P = obj.Kp * error;

            obj.integrator = obj.integrator + obj.Ki * error * obj.dt;
            obj.integrator = max(-obj.anti_windup, min(obj.anti_windup, obj.integrator));
            I = obj.integrator;

            d_raw = (error - obj.e_prev) / obj.dt;
            obj.differentiator = (1.0 - obj.N*obj.dt)*obj.differentiator + obj.N*obj.Kd*d_raw;
            D = obj.differentiator;
            obj.e_prev = error;

            u_raw = P + I + D;
            u_sat = max(obj.u_min, min(obj.u_max, u_raw));

            if obj.Ki ~= 0.0
                obj.integrator = obj.integrator + (u_sat - u_raw)*obj.dt;
            end
            obj.u_prev = u_sat;
            u = u_sat;
        end

        function reset(obj)
            obj.integrator     = 0;
            obj.differentiator = 0;
            obj.e_prev         = 0;
            obj.u_prev         = 0;
        end

        function set_gains(obj, Kp, Ki, Kd)
            obj.Kp = Kp; obj.Ki = Ki; obj.Kd = Kd;
        end

        function info(obj)
            fprintf('\n--- pid_controller ---\n');
            fprintf('  Kp=%.4f  Ki=%.4f  Kd=%.4f\n', obj.Kp, obj.Ki, obj.Kd);
            fprintf('  dt=%.4f s   N=%.0f\n', obj.dt, obj.N);
            fprintf('  Saturation  : [%.3f, %.3f]\n', obj.u_min, obj.u_max);
            fprintf('  Anti-windup : %.3f\n', obj.anti_windup);
            fprintf('  Integrator  : %.6f\n', obj.integrator);
            fprintf('----------------------\n\n');
        end

    end % instance methods

    methods (Static)

        function [Kp, Ki, Kd] = ziegler_nichols(Ku, Tu)
            if nargin < 2
                error('Usage: pid_controller.ziegler_nichols(Ku, Tu)');
            end
            Kp = 0.6 * Ku;
            Ki = 2.0 * Kp / Tu;
            Kd = Kp  * Tu  / 8.0;
            fprintf('\nZiegler-Nichols: Kp=%.3f  Ki=%.3f  Kd=%.3f\n\n', Kp, Ki, Kd);
        end

        function demo()
            fprintf('\n=== pid_controller.demo() ===\n');
            dt  = 0.01;
            t   = (0:dt:10)';
            N   = length(t);
            r   = ones(N,1);
            tau = 0.8;

            cP   = pid_controller(1.5, 0,    0,    dt);
            cPD  = pid_controller(2.0, 0,    0.6,  dt);
            cPID = pid_controller(2.4, 0.15, 0.8,  dt, 'Saturation',[-10 10],'AntiWindup',5.0);

            yP   = zeros(N,1);
            yPD  = zeros(N,1);
            yPID = zeros(N,1);

            for k = 2:N
                yP(k)   = yP(k-1)   + dt/tau*(-yP(k-1)   + cP.step(  r(k)-yP(k-1)));
                yPD(k)  = yPD(k-1)  + dt/tau*(-yPD(k-1)  + cPD.step( r(k)-yPD(k-1)));
                yPID(k) = yPID(k-1) + dt/tau*(-yPID(k-1) + cPID.step(r(k)-yPID(k-1)));
            end

            figure('Name','pid_controller.demo()','Color','w','Position',[200 150 860 500]);
            plot(t, r,    'k--','LineWidth',1.5,'DisplayName','Setpoint'); hold on;
            plot(t, yP,   'r-', 'LineWidth',2,  'DisplayName','P only  (Kp=1.5)');
            plot(t, yPD,  'm-', 'LineWidth',2,  'DisplayName','PD      (Kp=2.0, Kd=0.6)');
            plot(t, yPID, 'b-', 'LineWidth',2.5,'DisplayName','PID tuned (Kp=2.4, Ki=0.15, Kd=0.8)');
            xlabel('Time [s]'); ylabel('Plant output');
            legend('Location','southeast','FontSize',10);
            grid on;
            title('pid\_controller.demo() — Step Response Comparison','FontWeight','bold','FontSize',13);
            set(gca,'FontSize',10);

            idx = find(abs(yPID-1.0) < 0.02, 1);
            if ~isempty(idx)
                xline(t(idx),'--b',sprintf('Settle: %.1f s',t(idx)),...
                      'FontSize',9,'LabelHorizontalAlignment','right');
            end

            fprintf('Settling time : %.2f s\n', t(idx));
            fprintf('Overshoot     : %.1f %%\n', max(0,max(yPID)-1)*100);
            fprintf('SS error      : %.3f %%\n\n', abs(1-yPID(end))*100);
        end

    end % static methods

end % classdef pid_controller