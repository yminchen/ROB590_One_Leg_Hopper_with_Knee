%% Some minor housekeeping 
clear; clc; clf; close all;
addpath('Functions','Visualization','Visualization/Terrain','Dynamics','Events');
%% Flags 
F_PLOT = 0;             % Plot system response
F_ANIMATE = 1;          % Animate system response
F_SAVEPLOT = 0;         % Save generated plot
F_SAVEVID = 1;          % Save generated animation   
%% Simulation control
relTol  = 1e-6;         % Relative tolerance: Relative tolerance for ode45 numerical integration
absTol  = 1e-6;         % Absolute tolerance: Absolute tolerance for ode45 numerical integration 
dt      = 0.01;%[s]    % Max time step: Maximum time step for numerica integration 
tFinal  = 1;   %[s]    % Simulation end time

%% Simulation parameters
x0 = 0;         %[m]    % initial X position 
y0 = 0.6;       %[m]    % initial Y position
phi0 = 0;       %[rad]  % initial angle between vertical and hip
alpha0 = pi/4;     %[rad]  % iniial angle between hip and thigh
beta0 = -pi/2;      %[rad]  % initial angle between thigh and shank
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



feet = posFoot(S(1,1:5)',param);
DS(1) = feet(2)>0;
while T(end) < tFinal
    tspan = T(end):dt:tFinal;
    if(DS(end) == 1)
        [Tp,Sp,TEp,SEp,Ie] = ode45(@flightDyn,[tspan, tspan(end)+dt],S(end,:),fltSimOpts);
        sz = size(Sp,1);
        DS = [DS;ones(sz-1,1)];
        if(isempty(Ie)== 1) % Simulation timed out
            display('Time out');
        elseif(Ie == 1) % Touchdown event
            display('Touchdown');
            DS(end) = 0;
            qplus = impactVelUpdate(Sp(end,:)');
            Sp(end,:) = [Sp(end,1:5)'; qplus];
        elseif(Ie == 2 || Ie == 3)
            S = [S;Sp(2:sz,:)];
            T = [T;Tp(2:sz,:)];
            display('Contact point is not feet');
            break;
        else 
            display('Flight Phase: Invalid event code');
        end
    elseif(DS(end) == 0)
        [Tp,Sp,TEp,SEp,Ie] = ode45(@groundDyn,[tspan, tspan(end)+dt],S(end,:),gndSimOpts);
        sz = size(Sp,1);
        DS = [DS;zeros(sz-1,1)];
        if(isempty(Ie) == 1) % Simulation timed out
            display('Time out');
        elseif(Ie == 1) % Takeoff event
            display('Takeoff');
            DS(end) = 1;
        elseif(Ie == 2 || Ie == 3)
            S = [S;Sp(2:sz,:)];
            T = [T;Tp(2:sz,:)];
            display('Contact point is not feet');
            break;
        else 
            display('Ground Phase: Invalid event code');
        end
    end
    S = [S;Sp(2:sz,:)];
    T = [T;Tp(2:sz,:)];
end

%% Post processing 
if F_PLOT || F_SAVEPLOT
    plotResp
else
end


%% Animation
if F_ANIMATE
    Animation(T,S,DS,tFinal,F_SAVEVID);    
end