function tau = groundController(x,phase,k_des,dx_des)
tau = zeros(5,1);

%% parameters
param = simParameters();
Yparam = yumingParameters();


% dx_des = -Yparam.k_f(1)*2*(x(1)-Yparam.target_pos)...
%          -Yparam.k_f(2)*x(6);
% if dx_des>Yparam.max_dx_des
%     dx_des = Yparam.max_dx_des;
% elseif dx_des<-Yparam.max_dx_des
%     dx_des = -Yparam.max_dx_des;
% end


%% Virtual force
F = zeros(3,1);

theta = Theta(x(1:5),param);
L_sp = SpringLength(x(1:5),param);

k = Yparam.k;
if phase == 3
    k = k_des;
    % for testing
%     k = k/3;
%    k = k*30;
end
F(1) = -k*(Yparam.L_sp0-L_sp)*sin(theta);
F(2) =  k*(Yparam.L_sp0-L_sp)*cos(theta);

maxForce = 3000;%300;
if (F(1)^2+F(2)^2)>maxForce^2
    F(1) = F(1)*maxForce/(F(1)^2+F(2)^2)^0.5;
    F(2) = F(2)*maxForce/(F(1)^2+F(2)^2)^0.5;
end

%% Body balance
if phase == 2 
    tar_angle = 0; 

    % PD controller parameters
    kp = 200;    % 10
    kd = 5;   % 0.5
    max_f = 1000;    % maximum torque that can be applied
    % PD controller for desired phi.
    err = x(3) - tar_angle;
    derr =  x(8);     
    tau_balance = -1*(kp*err + kd*derr);
    if tau_balance > max_f
        tau_balance = max_f;
    elseif tau_balance < -max_f
        tau_balance = -max_f;
    end
    
elseif phase == 3
    % I found the robot lean forward too much with tar_vel = 0.
    tar_vel = 1.5*dx_des + 2.5;
                % if dx_des = 1, then tar_vel should be 4
                % if dx_des = 0, then tar_vel should be 2.5
    % Failed try:
    % tar_vel = -((x(8)+x(9))*Yparam.J2+(x(8)+x(9)+x(10))*Yparam.J3)/Yparam.J1;
    
    % P controller parameters
    kp = 10;%*50;       % 2
    % kd = 0.2;   % 0.2
    max_f = 1000;    % maximum torque that can be applied
    % P controller for desired angular velocity.
    err = x(8) - tar_vel;
    % derr =  x(5)-x(6);
    tau_balance = -kp*err;% - kd*derr;
    if tau_balance > max_f
        tau_balance = max_f;
    elseif tau_balance < -max_f
        tau_balance = -max_f;
    end
end

F(3) = tau_balance;
% Just for testing
% F(3) = 0;

%% WAY I: when the virtual spring is between the body and the foot
%% Conversion
phi_f = -(x(3)+x(4)+x(5));
JT = J_virtual_force(phi_f,x(5),x(4),Yparam.lH,Yparam.lL2,Yparam.lL3);
tau_kh = JT*[F(2);F(3)];
%% Hip joint
tau(4) = tau_kh(2);
%% Knee joint
tau(5) = tau_kh(1); 

%% WAY II: when the virtual spring is between the hip and the foot
% %% Conversion
% theta_a = -(x(3)+x(4)+x(5));
% theta_k = x(4);
% theta = [theta_a;theta_k];
% tau_kh = J_virtual_force_HipFoot(theta,[F(2);F(3)]);
% %% Hip joint
% tau(4) = tau_kh(2);
% %% Knee joint
% tau(5) = tau_kh(1);

%% testing

end