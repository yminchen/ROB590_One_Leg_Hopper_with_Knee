function tau = flightController(x,t_prev_stance)
tau = zeros(5,1);

%% parameters
Yparam = yumingParameters();
lH = Yparam.lH;
lL2 = Yparam.lL2;
lL3 = Yparam.lL3;
beta_0 = Yparam.beta_eq;

%% Get the desired touch-down angle
% Position controller parameters
kp_pos = Yparam.k_f(1);
kd_pos = Yparam.k_f(2);
% Position controller (PD control)
dx_des = -kp_pos*(x(1)-Yparam.target_pos) - kd_pos*x(6); % target_pos is fixed.
if dx_des>Yparam.max_dx_des
    dx_des = Yparam.max_dx_des;
elseif dx_des<-Yparam.max_dx_des
    dx_des = -Yparam.max_dx_des;
end

dx_des = 1; % for tuning kp_rai
% dx_des = 0; % for testing

% Raibert style controller parameters
kp_rai = Yparam.k_f(3);
max_theta_tar = 50*pi/180;
% Raibert style controller
x_des = x(6)*t_prev_stance/2 + kp_rai*(x(6)-dx_des);
theta_tar = asin(x_des/Yparam.L_sp0);
if theta_tar > max_theta_tar
    theta_tar = max_theta_tar;
elseif theta_tar < -max_theta_tar
    theta_tar = -max_theta_tar;
end

% theta_tar = 1; % for tuning the next PD controller
% theta_tar = 0; % for debugging

%% Hip joint
% When the spring is between hip and foot: 
alpha_tar = -x(3) + theta_tar + Yparam.theta2;
%------
% When the spring is between body and foot:
% (use Newton's method to solve for alpha_tar numerically)
% epsilon = 0.000001;
% delta = 1;
% phi = pi/4;
% loop = 1;
% while (delta>epsilon) && (loop<20)
%     A = lH*sin(-x(3)+theta_tar);
%     B = lL3*sin(pi-phi+beta_0);
%     phi = asin((A+B)*lH*sin(-x(3)+theta_tar)/(A*lL2));
%     loop = loop +1;
% end
% alpha_tar = -x(3)+theta_tar+phi;
%------
% PD controller parameters
kp = 10;    % 10
kd = 0.5;   % 0.5
max_f = 1000;    % maximum torque that can be applied
% PD controller for desired phi.
err = x(4) - alpha_tar;
derr =  x(9);     
        %%% TODO: theta_target is dynamic, so I should change derr.
tau_hip = -kp*err - kd*derr;
if tau_hip > max_f
    tau_hip = max_f;
elseif tau_hip < -max_f
    tau_hip = -max_f;
end

% Assignment
tau(4) = tau_hip;
    
%% Knee joint
% PD controller parameters
kp = 10;    % 10
kd = 0.5;   % 0.5
max_f = 1000;    % maximum torque that can be applied
% PD controller for desired phi.
err = x(5) - Yparam.beta_eq;
derr =  x(10);      
tau_knee = -kp*err - kd*derr;
if tau_knee > max_f
    tau_knee = max_f;
elseif tau_knee < -max_f
    tau_knee = -max_f;
end

% Assignment
tau(5) = tau_knee;

%% testing
% tau = zeros(5,1);

end