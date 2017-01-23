function dxdt = groundDyn(t,x)
    param = simParameters();
    dxdt = zeros(size(x));
    n = size(x,1);
    dxdt(1:n/2) = x(n/2+1:n);
    %M = MassMatrix(x(1:5),param);
    invM = inverseMassMatrix(x(1:5),param);
    J = JcontPoint(x(1:5),param);
    dJ = dJcontPoint(x,param);
    fCG = FCorGrav(x,param);
    lamda = -inv(J*invM*J')*(J*invM*(fCG) + dJ*x(n/2+1:n));
    dxdt(n/2+1:n) = invM*(fCG + J'*lamda);
end