function [position, isterminal, direction] = flightEvent(t,x)
%%
    param = simParameters();
    contP = contPoint(x,param);
    position(1)     = contP(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
end