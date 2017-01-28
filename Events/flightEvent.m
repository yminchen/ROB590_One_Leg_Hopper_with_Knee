%%
function [position, isterminal, direction] = flightEvent(t,x)

    param = simParameters();
    foot = posFoot(x,param);
    hip = posHip(x,param);
    knee = posKnee(x,param);
    
    position(1)     = foot(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
    position(2)     = hip(2);
    isterminal(2)   = 1;
    direction(2)    = -1;
    position(3)     = knee(2);
    isterminal(3)   = 1;
    direction(3)    = -1;
end