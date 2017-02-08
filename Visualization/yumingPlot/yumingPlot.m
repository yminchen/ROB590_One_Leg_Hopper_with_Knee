
%% initialize settings
%P = [S, L, dL, E, E_des, tau, F_c, Theta, dTheta]

% generate flags indicating which plot should be shown.
plot_flag = zeros(1,n_plot);
if size(plot_flag_index,2) == 0
    plot_flag = zeros(1,n_plot);
else    
    for i = 1:n_plot
        for j = 1:size(plot_flag_index,2)
            if plot_flag_index(j) == i
                plot_flag(i) = 1;
            end
        end
    end
end

%% Generate the data we want to use
n = size(T,1);

dx_des = zeros(n,1);
for i = 1:n
    for j = 2:size(dx_des_forPlot,1)+1
        if j==size(dx_des_forPlot,1)+1
            dx_des(i) = dx_des_forPlot(j-1,1);
            break;
        end
        if T(i)<dx_des_forPlot(j,2)
            dx_des(i) = dx_des_forPlot(j-1,1);
            break;
        end
    end
end

%% Generate the data we want to see

% length of the virtual spring
dL = zeros(n,1);
L = zeros(n,1);
if plot_flag(11) || plot_flag(12)
    for i = 1:n
        % length of the virtual spring
        CoG = CoG_tot(S(i,1:5)',param);
        posF = posFoot(S(i,1:5)',param);
        theta = atan2((posF(1)-CoG(1)),(CoG(2)-posF(2)));
        dL(i) = -S(i,6)*sin(theta)+S(i,7)*cos(theta); %TODO: This line of code is only correct during stance phase.
        L(i) = sum((CoG-posF).^2)^0.5;
    end
end

% total energy
E = zeros(n,1);
if plot_flag(13)
    for i = 1:n
        % length of the virtual spring
        E(i) = energy(S(i,:)',param);
    end
end
% desired energy
%TODO: if it's uneven terrain, them this part should be modified.
E_des = zeros(n,1);
if plot_flag(14)
    E_des_temp = m_tot*9.81*(Yparam.H+Terrain(S(1,1)+S(1,6)*Yparam.t_prev_stance/2,Yparam.ter_i))...
                + 0.5*m_tot*dx_des(1)^2;
    for i = 2:n
        if (DS(i)==1) && (DS(i-1)~=DS(i)) 
            for j = 2:size(t_prev_stance_forPlot,1)+1
                if j == size(t_prev_stance_forPlot,1)+1
                    t_prev_stance = t_prev_stance_forPlot(j-1,1);
                    break;
                end
                if T(i)<t_prev_stance_forPlot(j,2)
                    t_prev_stance = t_prev_stance_forPlot(j-1,1);
                    break;
                end
            end
            E_des_temp = m_tot*9.81*(Yparam.H+Terrain(S(i,1)+S(i,6)*t_prev_stance/2,Yparam.ter_i))...
                + 0.5*m_tot*dx_des(i)^2;
        end
        E_des(i) = E_des_temp;
    end
end

% hip/knee torque
tau = zeros(n,2);
if plot_flag(15)||plot_flag(16)
    for i = 1:n
        if DS(i) == 1
            for j = 2:size(t_prev_stance_forPlot,1)+1
                if j == size(t_prev_stance_forPlot,1)+1
                    t_prev_stance = t_prev_stance_forPlot(j-1,1);
                    break;
                end
                if T(i)<t_prev_stance_forPlot(j,2)
                    t_prev_stance = t_prev_stance_forPlot(j-1,1);
                    break;
                end
            end
            tau_temp = flightController(S(i,:)',t_prev_stance);
            tau(i,:) = tau_temp(4:5,1)';
        else
            for j = 2:size(k_des_forPlot,1)+1
                if j == size(k_des_forPlot,1)+1
                    k_des = k_des_forPlot(j-1,1);
                    break;
                end
                if T(i)<k_des_forPlot(j,2)
                    k_des = k_des_forPlot(j-1,1);
                    break;
                end
            end
            tau_temp = groundController(S(i,:)',DS(i),k_des);
            tau(i,:) = tau_temp(4:5,1)';
        end
    end
end

% contact force
F_c = zeros(n,2);   
if plot_flag(17)||plot_flag(18)
    for i = 1:n
        if DS(i) ~= 1
            for j = 2:size(k_des_forPlot,1)+1
                if j == size(k_des_forPlot,1)+1
                    k_des = k_des_forPlot(j-1,1);
                    break;
                end
                if T(i)<k_des_forPlot(j,2)
                    k_des = k_des_forPlot(j-1,1);
                    break;
                end
            end
            tau_temp = groundController(S(i,:)',DS(i),k_des);

            M = MassMatrix(S(i,1:5)',param);
            invM = inverseMassMatrix(S(i,1:5)',param);
            J = JcontPoint(S(i,1:5)',param);
            dJ = dJcontPoint(S(i,:)',param);
            fCG = FCorGrav(S(i,:)',param);
            lamda = -inv(J*invM*J')*(J*invM*(fCG+tau_temp) + dJ*S(i,6:10)');
            F_c(i,:) = lamda';
        end
    end
end

% theta and d_theta
theta = zeros(n,1);
d_theta = zeros(n,1);   
if plot_flag(19)||plot_flag(20)
    for i = 1:n
        theta(i) = Theta(S(i,1:5)',param);
        d_theta(i) = dTheta(S(i,:)',param);
    end
end


% put all the data into P
P = [S L dL E E_des tau F_c theta d_theta];

% height of the phase zone
max_height = 0;
min_height = 0;
for i = 1:size(plot_flag,2)
    if plot_flag(i)
        for j = 1:n
            if P(j,i)>max_height
                max_height = P(j,i);
            elseif P(j,i)<min_height
                min_height = P(j,i);
            end
        end
    end
end
max_height = max_height+0.1*(max_height-min_height);
min_height = min_height-0.1*(max_height-min_height);

%% first plot
figure;
axis([T(1) T(end) min_height max_height]);

% plot states
plotStates;    
legend('show');

% plot phase zone
plotPhaseZone;

% plot states 
plotStates;    

% others
hold on
plot([T(1) T(end)], [0 0], 'k--' ,'LineWidth',1)
hold off

% figure name
title('One-leg Hopper')
xlabel('Time (s)')

%% second plot (trajectory)
figure;
plot(P(:,1),P(:,2),'b')
title('Trayjectory of One-leg Hopper')
xlabel(' (m)')
ylabel(' (m)')
