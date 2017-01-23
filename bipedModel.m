%% Some minor housekeeping 
clear; clc; clf; close all;
addpath('Functions');

%% Flags 
F_PLOT = 1;             % Plot system response
F_ANIMATE = 0;          % Animate system response
F_SAVEPLOT = 1;         % Save generated plot
F_SAVEVID = 0;          % Save generated animation   
%% Simulation control
relTol  = 1e-6;         % Relative tolerance: Relative tolerance for ode45 numerical integration
absTol  = 1e-6;         % Absolute tolerance: Absolute tolerance for ode45 numerical integration 
dt      = 0.1; %[s]    % Max time step: Maximum time step for numerica integration 
tFinal  = 1;   %[s]    % Simulation end time

%% Simulation parameters
x0 = 0;         %[m]    % initial X position 
y0 = 0.6;         %[m]    % initial Y position
phi0 = 0;       %[rad]  % initial angle between vertical and hip
alpha0 = 0/4;     %[rad]  % iniial angle between hip and thigh
beta0 = -0/4;      %[rad]  % initial angle between thigh and shank
vx0 = 0;        %[m/s]  % initial X velociy 
vy0 = 0;        %[m/s]  % initial Y velociy
vphi0 = 0;      %[rad/s]% initial phi angular velocity
valpha0 = 0;    %[rad/s]% initial alpha angular  velocity
vbeta0 = 0;     %[rad/s]% initial beta angular  velocity

%% Simulation 
%Setting up simulation
gndSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,'Events',@groundEvent,'MaxStep',dt);
fltSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,'Events',@flightEvent,'MaxStep',dt);    
param = simParameters();
T(1) = 0;
S(1,:) = [x0;y0;phi0;alpha0;beta0;vx0;vy0;vphi0;valpha0;vbeta0];
feet = contPoint(S(1,1:5)',param);
DS(1) = feet(2)>0;
while T(end) < tFinal
    if(DS(end) == 1)
        [Tp,Sp,TEp,SEp,Ie] = ode45(@flightDyn,[T(end),tFinal],S(end,:),fltSimOpts);
        DS = [DS;ones(size(Tp))];
        DS(end) = 0;
        qplus = impactVelUpdate(Sp(end,:)');
        Sp(end,:) = [Sp(end,1:5)'; qplus];
    else
        [Tp,Sp,TEp,SEp,Ie] = ode45(@groundDyn,[T(end),tFinal],S(end,:),gndSimOpts);
        DS = [DS;zeros(size(Tp))];
    end
    S = [S;Sp];
    T = [T;Tp];
end

%% Post processing 
if F_PLOT
    plotResp
else
end
% if F_ANIMATE
%     if F_SAVEVID
%     end
% end