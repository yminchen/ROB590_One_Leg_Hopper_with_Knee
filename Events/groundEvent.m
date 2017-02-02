function [position, isterminal, direction] = groundEvent(t,x,phase,k_des)
    param = simParameters();
    n = size(x,1);
    
    tau = groundController(x,phase,k_des);
    
    M = MassMatrix(x(1:5),param);
    invM = inverseMassMatrix(x(1:5),param);
    J = JcontPoint(x(1:5),param);
    dJ = dJcontPoint(x,param);
    fCG = FCorGrav(x,param);
    lamda = -inv(J*invM*J')*(J*invM*(fCG+tau) + dJ*x(n/2+1:n));
    
    hip = posHip(x,param);
    knee = posKnee(x,param);
    
    %% detect take-off
    position(1)     = lamda(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
    
    
    %% detect robot falls onto the ground
    position(2)     = knee(2);
    isterminal(2)   = 1;
    direction(2)    = -1;
    position(3)     = hip(2);
    isterminal(3)   = 1;
    direction(3)    = -1;
    position(4)     = x(2);
    isterminal(4)   = 1;
    direction(4)    = -1;
    
    %% detect lowest point
    posH = posHip(x(1:5),param);
    posF = posFoot(x(1:5),param);
    theta = atan((posF(1)-posH(1))/(posH(2)-posF(2)));
    dL = -x(6)*sin(theta)+x(7)*cos(theta); 
    position(5)     = dL; % time derivative of virtual spring length.
    isterminal(5)   = 1;
    direction(5)    = 1;
    
    %% dectect negative contact force
%     position(6)     = lamda(2);
%     isterminal(6)   = 1;
%     direction(6)    = 1;
end