function dxdt = flightDyn(t,x,t_prev_stance)
    param = simParameters();
    n = size(x,1);
    dxdt = zeros(size(x));
    dxdt(1:n/2) = x(n/2+1:n);
    
    %controller
    tau = flightController(x,t_prev_stance);
    
    dxdt(n/2+1:n) = inverseMassMatrix(x(1:5),param)*(FCorGrav(x,param)+tau);
%     dxdt(n/2+1:n) = MassMatrix(x(1:5),param)\FCorGrav(x,param);
    
end