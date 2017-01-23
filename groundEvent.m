function [position, isterminal, direction] = flightEvent(t,x)
    param = simParameters();
    n = size(x,1);
    M = MassMatrix(x(1:5),param);
    invM = inverseMassMatrix(x(1:5),param);
    J = JcontPoint(x(1:5),param);
    dJ = dJcontPoint(x,param);
    fCG = FCorGrav(x,param);
    lamda = -inv(J*invM*J')*(J*invM*(fCG) + dJ*x(n/2+1:n));
    position(1)     = lamda(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
end