function [position, isterminal, direction] = groundEvent(t,x)
    param = simParameters();
    n = size(x,1);
    M = MassMatrix(x(1:5),param);
    invM = inverseMassMatrix(x(1:5),param);
    J = JcontPoint(x(1:5),param);
    dJ = dJcontPoint(x,param);
    fCG = FCorGrav(x,param);
    lamda = -inv(J*invM*J')*(J*invM*(fCG) + dJ*x(n/2+1:n));
    
    hip = posHip(x,param);
    knee = posKnee(x,param);
    
    position(1)     = lamda(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
    position(2)     = hip(2);
    isterminal(2)   = 1;
    direction(2)    = -1;
    position(3)     = knee(2);
    isterminal(3)   = 1;
    direction(3)    = -1;
    
end