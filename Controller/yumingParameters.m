function param = yumingParameters()

simParam = simParameters();

%% System parameters
param.ter_i = 0;          % terrain label (0 is flag terrain)

%% Joint mass, inertia and length parameters
% Main body segment
param.m1 = simParam(1);   %[kg]         Mass of main body segment 
param.J1 = simParam(2);   %[kgm^2]      Rotational inertia of main body segment 
param.lH = simParam(3);   %[m]          Distance from main body CoM to hip center

% Thigh segments
param.m2 = simParam(4);   %[kg]         Mass of thigh segment
param.J2 = simParam(5);   %[kgm^2]      Rotational inertia of thigh segment
param.l2 = simParam(6);   %[m]          Distance from hip center to thigh CoM
param.lL2 = simParam(7);  %[m]          Distance from hip center to knee center

% Shank segments
param.m3 = simParam(8);   %[kg]         Mass of shank segment
param.J3 = simParam(9);   %[kgm^2]      Rotational inertia of thigh segment
param.l3 = simParam(10);  %[m]          Distance from knee center to shank CoM
param.lL3 = simParam(11); %[m]          Distance from knee center to foot center

% Environmental constants
param.g = 9.81;           %[m/s^2]      Accelerations due to gravity

%% Joint and actuator constraint parameters
% Hip constraint
param.alpha_min = -pi/2;  %[rad]        Minimum hip angle
param.alpha_max = pi/2;   %[rad]        Maximum hip angle

% Knee constraint
param.beta_min = -3*pi/4; %[rad]        Minimum knee angle
param.beta_max = 0;       %[rad]        Maximum knee angle

% Motor constraints
param.T_rot_max = 0.568;  %[Nm]         Maximum rotor torque
param.du_rot_max = 838;   %[rad/s]      Maximum rotor velocity torque

%% Virtual Component (a spring between body and foot)
param.k = 5000;            % spring constant   
param.d = 10;             % spring damping 
param.beta_eq = -20*pi/180;
                          % knee relax angle

param.L_sp0 = simParam(3) + (simParam(7)^2+simParam(11)^2 ...
               -2*simParam(7)*simParam(11)*cos(pi+param.beta_eq))^0.5;      
                          % spring original length (m)
% When the spring is between hip and foot: (L_sp0 need to be adjusted)
param.L_sp0_HF = (simParam(7)^2+simParam(11)^2 ...
               -2*simParam(7)*simParam(11)*cos(pi+param.beta_eq))^0.5; 
param.theta2 = asin(simParam(11)*sin(-param.beta_eq)/param.L_sp0_HF);

%% controller parameters
param.target_pos = 10;
param.t_prev_stance = 0.2/(param.k/100);
param.H = 0.5;            % desired height (effecting kp_rai and kp_pos!)
param.max_dx_des = 1;     % maximum of desired speed (not real speed)
        % max_dx_des can go to 8 or higher, but then it also jumps higher.
% param.dx_des = 0;         % desired speed (initialized to be 0)
% param.E_low = 0;          % energy at lowest point (initialized to be 0)
% param.E_des = 0;          % desired energy (initialized to be 0)
% param.L_sp_low = 0;       % spring length when mass reaches lowest height 
% param.k_des = 0;          % desired spring constant during the thrust phase
% param.x_td = 0;           % state vector at previous touchdown 
% param.prev_t = 0;

% position controller parameters
kp_pos = 2;       
        % kp_pos depends on max_dx_des.
kd_pos = 1.5;
% Raibert controller parameter
kp_rai = 0.04;      % Raibert sytle controller
        % kp_rai depends on H (the height) and k (the stiffness).
        % For larger desired speed, you need higher kp_rai.
        % But the larger kp_rai is, the more unstable when changing speed.
        % When it's stable enough, you can try to increase max_dx_des.
param.k_f = [kp_pos kd_pos kp_rai];  % f stands for flight


%% finite state machine
param.lLeg_flag = 0;      % 0 is right leg, and 1 is left leg.
param.phase = 0;          % 0 is flight phase, and 1 is stance phase
param.postApex_flag = 0;  % Flight phase: 0 is before Apex, and 1 is after Apex.
param.thrust_flag = 0;    % 0 is in thrust phase, 1 is not.
              
%% plotting parameters
param.line_height = 7;

end
