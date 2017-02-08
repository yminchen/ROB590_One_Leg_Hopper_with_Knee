%% Some minor housekeeping 
clear; clc; clf; close all;
addpath('Functions','Dynamics','Events',...
    'Controller','Controller/Flight','Controller/Stance',...
    'Visualization','Visualization/Animation/','Visualization/Animation/Terrain',...
    'Visualization/yumingPlot');
%% Flags 
F_PLOT = 0;             % Plot system response
F_SAVEPLOT = 0;         % Save generated plot
F_ANIMATE = 1;          % Animate system response
F_SAVEVID = 1;          % Save generated animation   
%% Simulation control
relTol  = 1e-6;         % Relative tolerance: Relative tolerance for ode45 numerical integration
absTol  = 1e-6;         % Absolute tolerance: Absolute tolerance for ode45 numerical integration 
dt      = 0.01; %[s]    % Max time step: Maximum time step for numerica integration 
tFinal  = 10;    %[s]    % Simulation end time

%% Simulation parameters
x0 = 0;         %[m]    % initial X position 
y0 = 0.6;       %[m]    % initial Y position
body_rot = 0;
phi0 = body_rot;       %[rad]  % initial angle between vertical and hip
alpha0 = 0+body_rot;     %[rad]  % iniial angle between hip and thigh
beta0 = 0+body_rot;      %[rad]  % initial angle between thigh and shank
vx0 = 0;        %[m/s]  % initial X velociy 
vy0 = 0;        %[m/s]  % initial Y velociy
vphi0 = 0;      %[rad/s]% initial phi angular velocity
valpha0 = 0;    %[rad/s]% initial alpha angular  velocity
vbeta0 = 0;     %[rad/s]% initial beta angular  velocity

%% Yu-ming's parameters
Yparam = yumingParameters();
t_prev_stance = Yparam.t_prev_stance;   %[s]
t_prev_stance_forPlot = [t_prev_stance 0]; 
prev_t = 0;
dx_des = 0;         % desired speed (initialized to be 0)
dx_des_forPlot = [dx_des 0];
E_low = 0;          % energy at lowest point (initialized to be 0)
E_des = 0;          % desired energy (initialized to be 0)
L_sp_low = 0;       % spring length when mass reaches lowest height 
k_des = 0;          % desired spring constant during the thrust phase
k_des_forPlot = [k_des 0];
x_td = 0;           % state vector at previous touchdown 

%% Simulation 
%Setting up simulation
param = simParameters();

T(1) = 0;
S(1,:) = [x0;y0;phi0;alpha0;beta0;vx0;vy0;vphi0;valpha0;vbeta0];

DS(1) = 1;
while T(end) < tFinal
    tspan = T(end):dt:tFinal;
    if(DS(end) == 1)
        fltSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,'Events',@flightEvent,'MaxStep',dt);    
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) flightDyn(t,x,t_prev_stance),...
            [tspan, tspan(end)+dt],S(end,:),fltSimOpts);
        sz = size(Sp,1);
        DS = [DS;ones(sz-1,1)];
        
%         disp(size(Ie));
        if(isempty(Ie)== 1) % Simulation timed out
            display('Time out');
        elseif(Ie == 1) % Touchdown event
            display('Touchdown');
            DS(end) = 2;
            qplus = impactVelUpdate(Sp(end,:)');
            Sp(end,:) = [Sp(end,1:5)'; qplus];
            % Yu-Ming's parameter
            prev_t = Tp(end);
            % flight controller info
            dx_des = -Yparam.k_f(1)*((Sp(end,1)+Sp(end,6)*t_prev_stance/2)-Yparam.target_pos)...
                     -Yparam.k_f(2)*Sp(end,6);
            if dx_des>Yparam.max_dx_des
                dx_des = Yparam.max_dx_des;
            elseif dx_des<-Yparam.max_dx_des
                dx_des = -Yparam.max_dx_des;
            end
            dx_des_forPlot = [dx_des_forPlot;dx_des Tp(end)]; 
            % state vector at touch down
            x_td = Sp(end,:);
            % length speed at touch down
            dL = abs(dSpringLength(Sp(end,:)',param)); 
        elseif(Ie == 2 || Ie == 3 || Ie == 4)
            S = [S;Sp(2:sz,:)];
            T = [T;Tp(2:sz,:)];
            display('Contact point is not feet');
            break;
        else 
            display('Flight Phase: Invalid event code');
        end
    elseif(DS(end) == 2)||(DS(end) == 3)
        gndSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,'Events',@(t,x) groundEvent(t,x,DS(end),k_des),'MaxStep',dt);
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) groundDyn(t,x,DS(end),...
            t_prev_stance,k_des,dx_des),[tspan, tspan(end)+dt],S(end,:),gndSimOpts);
        sz = size(Sp,1);
        DS = [DS;DS(end)*ones(sz-1,1)];
        
        %disp(size(Ie));
        if(isempty(Ie) == 1) % Simulation timed out
            display('Time out');
        elseif(Ie(end) == 1) % Takeoff event
            display('Takeoff');
            DS(end) = 1;
            % Yu-Ming's parameter
            t_prev_stance = Tp(end) - prev_t;
            t_prev_stance_forPlot = [t_prev_stance_forPlot; t_prev_stance Tp(end)]; 
        elseif(Ie(end) == 5) % Reach the lowest height
            display('thrust begins');
            DS(end) = 3;
            % calculate spring length and energy at lowest height
            L_sp_low = SpringLength(Sp(end,1:5)',param);
            E_low = energy(Sp(end,:)',param)+0.5*Yparam.k*(L_sp_low - Yparam.L_sp0)^2;
            % calculate desired energy (not including swing leg's)
            m_tot = param(1)+param(4)+param(8);
            E_des = m_tot*9.81*(Yparam.H+Terrain(Sp(end,1)+Sp(end,6)*t_prev_stance/2,Yparam.ter_i))...
                    + 0.5*m_tot*dx_des^2;
                    %+ pi/4*Yparam.d*dL*(Yparam.L_sp0-L_sp_low);
            disp(E_des);
            
            k_des = Yparam.k + 2*(E_des-E_low)/(Yparam.L_sp0-L_sp_low)^2;
%             if k_des < Yparam.k
%                 k_des = Yparam.k;
%                 %disp('here');
%             end
            k_des_forPlot = [k_des_forPlot; k_des Tp(end)];
        elseif(Ie(end) == 2 || Ie(end) == 3 || Ie(end) == 4)
            S = [S;Sp(2:sz,:)];
            T = [T;Tp(2:sz,:)];
            display('Contact point is not feet');
            break;
        elseif Ie(end) == 6
            display('ERROR: Contact force was negative');
%             S = [S;Sp(2:sz,:)];
%             T = [T;Tp(2:sz,:)];
%             break;
        else 
            display('Ground Phase: Invalid event code');
        end
    end
    S = [S;Sp(2:sz,:)];
    T = [T;Tp(2:sz,:)];
end

%% Plot 

% if F_PLOT || F_SAVEPLOT
%     plotResp
% else
% end


% plotting settings
F_yuming_plot = 1;  % flag for plotting
plot_flag_index = [3 8 15 16 17 18];
plot_flag_index = [3 8 15 16 ];
plot_flag_index = [1 3 8 ]; % look at phi
% plot_flag_index = [13 14]; % look at energy
%plot_flag_index = [15 16 19 20]; % tune PD controller for theta in flight
%plot_flag_index = [5  10]; % tune PD controller for knee in flight
% plot_flag_index = [11 12]; % look at spring length and speed
% plot_flag_index = [6]; % x velocity
% plot_flag_index = [1 6]; % x
n_plot = 20;
%P = [S, L, dL, E, E_des, tau, F_c, Theta, dTheta]
if F_yuming_plot
    yumingPlot;
end

%% Animation
if F_ANIMATE
    Animation(T,S,DS,T(end),F_SAVEVID);    
end
