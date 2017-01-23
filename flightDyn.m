function dxdt = flightDyn(t,x)
    param = simParameters();
    n = size(x,1);
    dxdt = zeros(size(x));
    dxdt(1:n/2) = x(n/2+1:n);
    dxdt(n/2+1:n) = inverseMassMatrix(x(1:5),param)*FCorGrav(x,param);
end