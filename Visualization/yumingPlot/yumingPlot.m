
%% settings
plot_flag_index = [3 8 14 15 16 17];
n_plot = 17;
%P = [S, L, dL, E, tau, F_c]

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

%% Generate the data we want to see
n = size(T,1);

% length of the virtual spring
dL = zeros(n,1);
L = zeros(n,1);
for i = 1:n
    % length of the virtual spring
    posH = posHip(S(i,1:5)',param);
    posF = posFoot(S(i,1:5)',param);
    theta = atan((posF(1)-posH(1))/(posH(2)-posF(2)));
    dL(i) = -S(i,6)*sin(theta)+S(i,7)*cos(theta); %TODO: This line of code is only correct during stance phase.
    L(i) = sum((posH-posF).^2)^0.5;
end

% total energy
E = zeros(n,1);
for i = 1:n
    % length of the virtual spring
    E(i) = energy(S(i,:)',param);
end

% hip/knee torque
tau = zeros(n,2);
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


% contact force
F_c = zeros(n,2);
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

% put all the data into P
P = [S L dL E tau F_c];

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

%% plot
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
